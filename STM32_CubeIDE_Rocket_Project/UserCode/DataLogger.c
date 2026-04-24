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
 *   - Compute an estimated altitude_AGL_ft using the barometric formula and log it, 
 *     using an averaged ground-level pressure as the reference P0.
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
#include <math.h>           /* powf() for barometric altitude computation */

/* =========================================================================
 * CSV header — written once at the top of each new log file
 * ========================================================================= */

static const char CSV_HEADER[] =
    "WriteTime_ms,"
    "IMU_Timestamp_ms,Accel_New,Accel_X_mg,Accel_Y_mg,Accel_Z_mg,"
    "Gyro_New,Gyro_X_mdps,Gyro_Y_mdps,Gyro_Z_mdps,"
    "Temp_New,IMU_Temp_C,"
    "BME_Timestamp_ms,Pressure_hPa,BME_Temp_C,Humidity_pctRH,"
    "Altitude_AGL_ft\r\n";

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

/* Number of BME680 readings to average for the ground-level pressure reference (P0).
 * At 20 Hz, 100 samples = 5 seconds of stationary pad data.
 * At 20 Hz, 60 samples = 3 seconds of stationary pad data. */
#define GROUND_PRESSURE_NUM_SAMPLES  60U

static float         groundPressure_hPa        = 0.0f;  /* averaged ground-level pressure (P0), set after calibration */
static uint8_t       hasGroundPressure         = 0;     /* 1 once GROUND_PRESSURE_NUM_SAMPLES readings have been averaged */
static float         groundPressureSum         = 0.0f;  /* running sum for P0 averaging */
static uint16_t      groundPressureSampleCount = 0;     /* number of readings accumulated so far */
static float         altitude_AGL_ft           = 0.0f;  /* current AGL altitude in feet */

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
 *    │ accel_mg[0..2] ×3     │ %.2f    │ ±32000.00 mg longest-case             │ 9 each = 27  │   // Note: the actual min and max depends on the value of LSM6DSO_ACCEL_FS, with largest case being when set to 32g
 *    ├───────────────────────┼─────────┼───────────────────────────────────────┼──────────────┤
 *    │ isGyroDataNew         │ %u      │ 1                                     │ 1            │
 *    ├───────────────────────┼─────────┼───────────────────────────────────────┼──────────────┤
 *    │ gyro_mdps[0..2] ×3    │ %.2f    │ ±2000000.00 mdps longest-case         │ 11 each = 33 │  // Note: the actual min and max depends on the value of LSM6DSO_GYRO_FS, with largest case being when sest to 2000dps
 *    ├───────────────────────┼─────────┼───────────────────────────────────────┼──────────────┤
 *    │ isTempDataNew         │ %u      │ 1                                     │ 1            │
 *    ├───────────────────────┼─────────┼───────────────────────────────────────┼──────────────┤
 *    │ temperature_degC      │ %.2f    │ −40.00 to +85.99°C                    │ 6            │
 *    ├───────────────────────┼─────────┼───────────────────────────────────────┼──────────────┤
 *    │ bmeCache.timestamp_ms │ %lu     │ 4294967295                            │ 10           │
 *    ├───────────────────────┼─────────┼───────────────────────────────────────┼──────────────┤
 *    │ pressure_hPa          │ %.2f    │ 300.00–1100.00 hPa                    │ 7            │
 *    ├───────────────────────┼─────────┼───────────────────────────────────────┼──────────────┤
 *    │ BME temperature_degC  │ %.2f    │ −40.00 to +85.99°C                    │ 6            │
 *    ├───────────────────────┼─────────┼───────────────────────────────────────┼──────────────┤
 *    │ humidity_pctRH        │ %.2f    │ 0.00–100.00%                          │ 6            │
 *    ├───────────────────────┼─────────┼───────────────────────────────────────┼──────────────┤
 *    │ altitude_AGL_ft       │ %.2f    │ ±99999.00 ft                          │ 9            │
 *    ├───────────────────────┼─────────┼───────────────────────────────────────┼──────────────┤
 *    │ 16 commas + \r\n      │ literal │ —                                     │ 18           │
 *    ├───────────────────────┼─────────┼───────────────────────────────────────┼──────────────┤
 *    │ null terminator       │ —       │ —                                     │ 1            │
 *    ├───────────────────────┼─────────┼───────────────────────────────────────┼──────────────┤
 *    │ Total                 │         │                                       │ 147          │
 *    └───────────────────────┴─────────┴───────────────────────────────────────┴──────────────┘
 */
    char line[160];  /* Max row = 147 bytes (calculated); 160 gives 13-byte margin */

    if (bmeHasReturnedFirstReading) {
        snprintf(line, sizeof(line),
                 "%lu,"
                 "%lu,%u,%.2f,%.2f,%.2f,"
                 "%u,%.2f,%.2f,%.2f,"
                 "%u,%.2f,"
                 "%lu,%.2f,%.2f,%.2f,"
                 "%.2f\r\n",
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
                 bmeCache.humidity_pctRH,
                 altitude_AGL_ft);
    } else {
        /* BME680 hasn't returned its first reading yet — leave BME columns empty */
        snprintf(line, sizeof(line),
                 "%lu,"
                 "%lu,%u,%.2f,%.2f,%.2f,"
                 "%u,%.2f,%.2f,%.2f,"
                 "%u,%.2f,"
                 ",,,,\r\n",
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

// Only compile this code if debug UART echo is enabled to avoid any runtime cost when disabled.
#if DATALOGGER_UART_ECHO
    /* Echo CSV row to UART3 at a reduced rate (DATALOGGER_UART_ECHO_MS).
     * Echoing every row is not possible: myprintf() is blocking and a
     * worst-case 147-byte row takes ~12.8 ms at 115200 baud — longer than the
     * 10 ms write period. 
     * Additionally, 11,520 bytes/sec is the maximum UART throughput, 
     * which is insufficient for echoing every row at 100 Hz.
     * So we echo at a reduced rate. */
    static uint32_t lastUartEchoTick = 0;
    if ((writeTimestamp - lastUartEchoTick) >= DATALOGGER_UART_ECHO_MS) {
        lastUartEchoTick = writeTimestamp;
        myprintf("%s", line);   /* line is already \r\n terminated */
    }
#endif
}

/* =========================================================================
 * Public API
 * ========================================================================= */

uint8_t DataLogger_IsLogging(void)
{
    return (dlState == DL_LOGGING) ? 1U : 0U;
}

void DataLogger_Init(void)
{
    bmeHasReturnedFirstReading = 0;
    isFileOpen                 = 0;
    dlState                    = DL_IDLE;
    prevSdState                = SD_GetState();

    // Initialize ground pressure averaging and altitude data
    hasGroundPressure          = 0;
    groundPressure_hPa         = 0.0f;
    groundPressureSum          = 0.0f;
    groundPressureSampleCount  = 0;
    altitude_AGL_ft            = 0.0f;

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

            // Reset ground pressure so a fresh calibration window runs on the next mount.
            hasGroundPressure         = 0;
            groundPressure_hPa        = 0.0f;
            groundPressureSum         = 0.0f;
            groundPressureSampleCount = 0;
            altitude_AGL_ft           = 0.0f;
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

        /* Accumulate pressure readings to form an averaged ground-level reference (P0).
         * altitude_AGL_ft in the log file stays at 0.0 during this time while gathering 
         * and averaging data for GROUND_PRESSURE_NUM_SAMPLES (afterall, we are assuming
         * the rocket is indeed at 0 ft AGL during this time). */
        if (!hasGroundPressure && bmeCache.pressure_hPa > 0.0f) {
            groundPressureSum += bmeCache.pressure_hPa;
            groundPressureSampleCount++;
            if (groundPressureSampleCount >= GROUND_PRESSURE_NUM_SAMPLES) {
                groundPressure_hPa = groundPressureSum / (float)groundPressureSampleCount;
                hasGroundPressure  = 1;
            }
        }

        /* Compute estimated AGL altitude using the simplified ISA barometric formula:
         *   h_meters = 44330 * (1 - (P / P0)^0.1903)
         *   Then convert meters to feet (* 3.28084).
         * Note: This assumes a standard temperature lapse rate!
         * Post processing could compute a more accurate altitude using the raw pressure
         * and temperature data, and averaging, filtering, and compensating for temerature effects,
         * but this is good enough for a rough estimate and its nice to be able to see this estimated
         * data in the log at a glance with no post-processing, and having this data local could be
         * useful for detecting apogee and deploying a parachute etc., though I suppose you could
         * do that with just raw pressure data as well, and you would still want some filtering/averaging.
         */
        if (hasGroundPressure) {
            altitude_AGL_ft = 44330.0f
                              * (1.0f - powf(bmeCache.pressure_hPa / groundPressure_hPa, 0.1903f))
                              * 3.28084f;
        }
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
