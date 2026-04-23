/*
 * DataLogger.c
 *
 * Owns the full data logging pipeline for the rocket flight computer.
 *
 * Responsibilities:
 *   - Poll the BME680 non-blocking state machine every 5 ms.
 *   - Trigger new BME680 measurements every DATALOGGER_BME_TRIGGER_MS (50 ms = 20Hz).
 *   - Note the LMS6DSO32 uses automatic/continuous measurement mode, so this module doesn't have to trigger it, it just reads it
 *   - Read the LSM6DSO32 IMU and write a CSV row every DATALOGGER_CSV_WRITE_MS (10 ms = 100Hz).
 *   - Detect when the SD card is mounted and create the next LOG_XXXX.CSV file.
 *   - Write CSV rows at the data logger rate; BME680 columns repeat the last valid reading
 *     along with the timestamp of when that BME680 reading was actually captured.
 *   - Detect SD card removal and cleanly abandon the open file handle.
 *
 * Internal state machine (DL_State_t):
 *   DL_IDLE    — Sensors polled; SD card not mounted or no file open.
 *   DL_OPENING — SD card just mounted; creating log file.
 *   DL_LOGGING — File open; actively writing CSV rows.
 *   DL_ERROR   — File operation failed; waits for card re-insertion.
 *
 * File naming uses 8.3 format: LOG_0001.CSV through LOG_9999.CSV since FATFS is configured without LFN (Long File Name) support in CubeMX.
 * The root directory is scanned on every mount to find the highest existing
 * log file number and creates the next one incremented by 1.
 *
 * SD card removal: DataLogger does NOT call SD_FileClose when the card is
 * physically removed because the FatFS filesystem is already gone at that
 * point. It simply resets its own state.
 */

#include <bme680_device.h>
#include "DataLogger.h"
#include "SD_Card.h"
#include "lsm6dso32_device.h"
#include <stdio.h>
#include <string.h>
#include "main.h"           /* myprintf() */
#include "stm32h7xx_hal.h"  /* HAL_GetTick() */

/* =========================================================================
 * CSV header — written once at the top of each new log file
 * ========================================================================= */

static const char CSV_HEADER[] =
    "WriteTime_ms,"
    "IMU_Timestamp_ms,Accel_New,Accel_X_mg,Accel_Y_mg,Accel_Z_mg,"
    "Gyro_New,Gyro_X_mdps,Gyro_Y_mdps,Gyro_Z_mdps,"
    "Temp_New,IMU_Temp_C,"
    "BME_Timestamp_ms,Pressure_hPa,BME_Temp_C,Humidity_pctRH\r\n";

/* =========================================================================
 * Module state
 * ========================================================================= */

typedef enum {
    DL_IDLE,
    DL_OPENING,
    DL_LOGGING,
    DL_ERROR
} DL_State_t;

static DL_State_t  dlState              = DL_IDLE;
static SD_State_t  prevSdState          = SD_NOT_PRESENT;  /* for edge detection */

static FIL         logFile;
static uint8_t     isFileOpen           = 0;

static BME680_Data_t bmeCache;
static uint8_t       bmeHasReturnedFirstReading = 0;
static uint32_t      bmeLastTrigger     = 0;
static uint32_t      lastCsvWriteTick   = 0;
static uint32_t      lastSyncTick       = 0;

/* =========================================================================
 * Internal helpers
 * ========================================================================= */

/*
 * Scan the root directory for files matching LOG_XXXX.CSV.
 * Returns the highest XXXX found, or 0 if none exist.
 */
static uint16_t findHighestLogNumber(void)
{
    DIR      dir;
    FILINFO  fno;
    uint16_t maxNum = 0;

    if (SD_DirOpen(&dir, "/") != FR_OK) return 0;

    while (SD_DirRead(&dir, &fno) == FR_OK && fno.fname[0] != '\0') {
        if (strncmp(fno.fname, "LOG_", 4) == 0) {
            uint16_t num = (uint16_t)atoi(fno.fname + 4);
            if (num > maxNum) maxNum = num;
        }
    }
    SD_DirClose(&dir);
    return maxNum;
}

/*
 * Create the next log file, write the CSV header, and leave it open.
 * Returns FR_OK on success.
 */
static FRESULT createLogFile(void)
{
    // Determine file number and file name
    uint16_t nextNum = findHighestLogNumber() + 1;
    // If all 9999 files have been used, just keep overwriting the last one
    if (nextNum > 9999) nextNum = 9999;

    char fileName[13];   /* "LOG_XXXX.CSV" = 12 chars + null */
    snprintf(fileName, sizeof(fileName), "LOG_%04u.CSV", nextNum);

    // Create log file
    // Only overwrite if we're at the max number (9999) and that file already exists —
    // otherwise create new without risking overwriting existing data.
    FRESULT res = (nextNum == 9999 && findHighestLogNumber() == 9999)
                  ? SD_FileOpen(&logFile, fileName, FA_CREATE_ALWAYS | FA_WRITE)    // Overwrite
                  : SD_FileOpen(&logFile, fileName, FA_CREATE_NEW | FA_WRITE);      // Don't allow overwrite
    if (res != FR_OK) {
        myprintf("DL: f_open(%s) failed (%d)\r\n", fileName, res);
        return res;
    }

    // Write header row
    UINT numBytesWritten;
    res = SD_FileWrite(&logFile, CSV_HEADER, strlen(CSV_HEADER), &numBytesWritten);
    if (res != FR_OK || numBytesWritten != strlen(CSV_HEADER)) {
        myprintf("DL: header write failed (%d)\r\n", res);
        SD_FileClose(&logFile);
        return res;
    }

    isFileOpen = 1;
    myprintf("DL: logging to %s\r\n", fileName);
    return FR_OK;
}

/*
 * Format and write one CSV row using the latest IMU data and the cached BME680 data.
 *
 * Each IMU reading carries its own timestamp (set in lsm6_readData at the moment of
 * the SPI read), and the BME cache carries the timestamp from when that BME measurement
 * completed. This allows post-processing to correctly align data from the two sensors.
 *
 * On write failure, closes the file handle and transitions  the DataLogger_StateMachine_Task() state to DL_ERROR.
 *
 * @param writeTimestamp  HAL_GetTick() value at the time this row is being written,
 *                        used as a simple wall-clock reference column.
 */
static void writeCSVRow(uint32_t writeTimestamp)
{
    LSM6DSO_Data_t imu;
    if (lsm6_readData(&imu)) {
        myprintf("DL: failed to read IMU data. Is it initialized?\r\n");
    }

/*    Here's the field-by-field breakdown:                                                                                   
 *                                                              
 *    ┌───────────────────────┬─────────┬───────────────────────────────────────┬──────────────┐                             
 *    │         Field         │ Format  │               Max value               │  Max chars   │                           
 *    ├───────────────────────┼─────────┼───────────────────────────────────────┼──────────────┤                             
 *    │ writeTimestamp        │ %lu     │ 4294967295                            │ 10           │                             
 *    ├───────────────────────┼─────────┼───────────────────────────────────────┼──────────────┤                             
 *    │ imu.timestamp_ms      │ %lu     │ 4294967295                            │ 10           │                             
 *    ├───────────────────────┼─────────┼───────────────────────────────────────┼──────────────┤                           
 *    │ isAccelDataNew        │ %u      │ 1                                     │ 1            │                             
 *    ├───────────────────────┼─────────┼───────────────────────────────────────┼──────────────┤
 *    │ accel_mg[0..2] ×3     │ %.2f    │ ±16000 mg (FS16g)                     │ 9 each = 27  │                             
 *    ├───────────────────────┼─────────┼───────────────────────────────────────┼──────────────┤                             
 *    │ isGyroDataNew         │ %u      │ 1                                     │ 1            │                           
 *    ├───────────────────────┼─────────┼───────────────────────────────────────┼──────────────┤                             
 *    │ gyro_mdps[0..2] ×3    │ %.2f    │ ±2,293,690 mdps (32767 × 70 mdps/LSB) │ 11 each = 33 │
 *    ├───────────────────────┼─────────┼───────────────────────────────────────┼──────────────┤                             
 *    │ isTempDataNew         │ %u      │ 1                                     │ 1            │                           
 *    ├───────────────────────┼─────────┼───────────────────────────────────────┼──────────────┤                             
 *    │ temperature_degC      │ %.2f    │ −40 to +85°C                          │ 6            │
 *    ├───────────────────────┼─────────┼───────────────────────────────────────┼──────────────┤                             
 *    │ bmeCache.timestamp_ms │ %lu     │ 4294967295                            │ 10           │
 *    ├───────────────────────┼─────────┼───────────────────────────────────────┼──────────────┤
 *    │ pressure_hPa          │ %.2f    │ 300–1100 hPa                          │ 7            │                           
 *    ├───────────────────────┼─────────┼───────────────────────────────────────┼──────────────┤                             
 *    │ BME temperature_degC  │ %.2f    │ −40 to +85°C                          │ 6            │
 *    ├───────────────────────┼─────────┼───────────────────────────────────────┼──────────────┤                             
 *    │ humidity_pctRH        │ %.2f    │ 0–100%                                │ 6            │                           
 *    ├───────────────────────┼─────────┼───────────────────────────────────────┼──────────────┤                             
 *    │ 15 commas + \r\n      │ literal │ —                                     │ 17           │
 *    ├───────────────────────┼─────────┼───────────────────────────────────────┼──────────────┤                             
 *    │ null terminator       │ —       │ —                                     │ 1            │                           
 *    ├───────────────────────┼─────────┼───────────────────────────────────────┼──────────────┤                             
 *    │ Total                 │         │                                       │ 136          │
 *    └───────────────────────┴─────────┴───────────────────────────────────────┴──────────────┘      
 */
    char line[160];  /* Max row = 136 bytes (calculated); 160 gives 24-byte margin */

    if (bmeHasReturnedFirstReading) {
        snprintf(line, sizeof(line),
                 "%lu,"
                 "%lu,%u,%.2f,%.2f,%.2f,"
                 "%u,%.2f,%.2f,%.2f,"
                 "%u,%.2f,"
                 "%lu,%.2f,%.2f,%.2f\r\n",
                 (unsigned long)writeTimestamp,
                 (unsigned long)imu.timestamp_ms,
                 imu.isAccelDataNew,
                 imu.accel_mg[0], imu.accel_mg[1], imu.accel_mg[2],
                 imu.isGyroDataNew,
                 imu.gyro_mdps[0], imu.gyro_mdps[1], imu.gyro_mdps[2],
                 imu.isTempDataNew,
                 imu.temperature_degC,
                 (unsigned long)bmeCache.timestamp_ms,
                 bmeCache.pressure_hPa,
                 bmeCache.temperature_degC,
                 bmeCache.humidity_pctRH);
    } else {
        /* BME680 hasn't returned its first reading yet — leave BME columns empty */
        snprintf(line, sizeof(line),
                 "%lu,"
                 "%lu,%u,%.2f,%.2f,%.2f,"
                 "%u,%.2f,%.2f,%.2f,"
                 "%u,%.2f,"
                 ",,,\r\n",
                 (unsigned long)writeTimestamp,
                 (unsigned long)imu.timestamp_ms,
                 imu.isAccelDataNew,
                 imu.accel_mg[0], imu.accel_mg[1], imu.accel_mg[2],
                 imu.isGyroDataNew,
                 imu.gyro_mdps[0], imu.gyro_mdps[1], imu.gyro_mdps[2],
                 imu.isTempDataNew,
                 imu.temperature_degC);
    }

    UINT numBytesWritten;
    FRESULT res = SD_FileWrite(&logFile, line, strlen(line), &numBytesWritten);
    if (res != FR_OK) {
        myprintf("DL: write failed (%d) — entering error state\r\n", res);
        isFileOpen = 0;
        dlState    = DL_ERROR;
        return;
    }
}

/* =========================================================================
 * Public API
 * ========================================================================= */

void DataLogger_Init(void)
{
    bmeHasReturnedFirstReading = 0;
    isFileOpen                 = 0;
    dlState                    = DL_IDLE;
    prevSdState                = SD_GetState();

    bme680_setGasEnabled(0);         /* Gas sensor not needed for rocketry */
    bme680_triggerMeasurement();     /* Kick off first measurement immediately */
    bmeLastTrigger = HAL_GetTick();
}

/*
 * Single update task — register with the scheduler at 5 ms.
 *
 * Every call : run BME680 state machine.
 * Every DATALOGGER_CSV_WRITE_MS : read IMU and write a CSV row.
 */
void DataLogger_StateMachine_Task(void)
{
    uint32_t   now          = HAL_GetTick();
    SD_State_t currentSdState = SD_GetState();

    // Detect SD Card State transitions and transition this state machine accordingly
    /* --- Edge detection: react to SD card state transitions --- */
    if (currentSdState != prevSdState) {
        // SD Card Inserted
        if (currentSdState == SD_MOUNTED && dlState == DL_IDLE) {
            /* Card just became mounted — try to open a log file */
            dlState = DL_OPENING;
        }

        // SD Card Removed
        if (currentSdState != SD_MOUNTED && isFileOpen) {
            /* Card removed or error — abandon the file handle.
               Do NOT call SD_FileClose: the filesystem is already gone. */
            isFileOpen = 0;
            dlState    = DL_IDLE;

            // Reset BME cache so stale data is never written when logging resumes.
            bmeHasReturnedFirstReading = 0;
            memset(&bmeCache, 0, sizeof(bmeCache));
        }
        prevSdState = currentSdState;
    }

    /* --- BME680 state machine (runs every call = every 5 ms) ---
     * Always drive the state machine so any in-progress measurement completes
     * cleanly when the card is removed (avoids leaving the BME mid-measurement).
     * Data is cached whenever it arrives, keeping the cache fresh for logging. */
    bme680_stateMachine();
    if (bme680_isDataReady()) {
        bmeCache = bme680_getData();
        bmeHasReturnedFirstReading = 1;
    }

    /* Trigger a new BME680 measurement every DATALOGGER_BME_TRIGGER_MS.
     * Only trigger when actively logging — no point measuring if no file is open. */
    if (dlState == DL_LOGGING &&
        (now - bmeLastTrigger) >= DATALOGGER_BME_TRIGGER_MS)
    {
        bme680_triggerMeasurement();
        bmeLastTrigger = now;
    }

    /* Note: IMU data is read on demand in writeCSVRow() — the sensor runs
       continuously in the background at its configured ODR. */

    /* --- DataLogger state machine --- */
    switch (dlState)
    {
        case DL_IDLE:
            /* Nothing to do — sensors still polled above */
            break;

        case DL_OPENING:
            if (createLogFile() == FR_OK) {
                lastCsvWriteTick = now;
                lastSyncTick     = now;
                dlState          = DL_LOGGING;
            } else {
                dlState = DL_ERROR;
            }
            break;

        case DL_LOGGING:
            // Note with this kind of subtraction the wrap around case is handled correctly as long as the write interval is less than half a uint32_t ~24.8 days (which it always is in practice)
            if ((now - lastCsvWriteTick) >= DATALOGGER_CSV_WRITE_MS) {
                lastCsvWriteTick = now;
                writeCSVRow(now);
            }

            /* Sync to SD card once per DATALOGGER_SYNC_MS (1 second).
             * Flushing on every write could require 2–3 sector writes per row —
             * several ms each — far exceeding the 10 ms logging period budget.
             * By syncing once per second like this instead, at worst, 1 second 
             * of data is lost on unexpected power-off.
             */
            if ((now - lastSyncTick) >= DATALOGGER_SYNC_MS) {
                lastSyncTick = now;
                FRESULT syncResult = SD_FileSync(&logFile);
                if (syncResult != FR_OK) {
                    myprintf("DL: sync failed (%d) — entering error state\r\n", syncResult);
                    isFileOpen = 0;
                    dlState    = DL_ERROR;
                }
            }
            break;

        case DL_ERROR:
            /* Wait for card removal/re-insertion — edge detection above
               will move us back to DL_IDLE once the card is removed */
            break;
    }
}
