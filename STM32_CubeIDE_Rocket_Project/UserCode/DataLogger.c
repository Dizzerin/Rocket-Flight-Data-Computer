/*
 * DataLogger.c
 *
 * Owns the full data logging pipeline for the rocket flight computer.
 *
 * Responsibilities:
 *   - Poll the BME680 non-blocking state machine every 5 ms.
 *   - Trigger new BME680 measurements every DATALOGGER_BME_TRIGGER_MS (50 ms).
 *   - Read the LSM6DSO32 IMU every DATALOGGER_IMU_POLL_MS (10 ms).
 *   - Detect when the SD card is mounted and create the next LOG_XXXX.CSV file.
 *   - Write CSV rows at the IMU rate; BME680 columns repeat the last valid reading.
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

#include "DataLogger.h"
#include "SD_Card.h"
#include "bme680_spi.h"
#include "lsm6dso32_device.h"
#include <stdio.h>
#include <string.h>
#include "main.h"           /* myprintf() */
#include "stm32h7xx_hal.h"  /* HAL_GetTick() */

/* =========================================================================
 * CSV header — written once at the top of each new log file
 * ========================================================================= */

static const char CSV_HEADER[] =
    "Timestamp_ms,Accel_X_mg,Accel_Y_mg,Accel_Z_mg,"
    "Gyro_X_mdps,Gyro_Y_mdps,Gyro_Z_mdps,"
    "IMU_Temp_C,Pressure_hPa,BME_Temp_C,Humidity_pctRH\r\n";

/* =========================================================================
 * Module state
 * ========================================================================= */

typedef enum {
    DL_IDLE,
    DL_OPENING,
    DL_LOGGING,
    DL_ERROR
} DL_State_t;

static DL_State_t  dlState       = DL_IDLE;
static SD_State_t  prevSdState   = SD_NOT_PRESENT;  /* for edge detection */

static FIL         logFile;
static uint8_t     fileOpen      = 0;

static BME680_Data_t bmeCache;
static uint8_t       bmeValid        = 0;
static uint32_t      bmeLastTrigger  = 0;
static uint32_t      imuLastTick     = 0;

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

    // Create log File
    // Only overwrite if we're at the max number (9999) and that file already exists — otherwise create new without risking overwriting
    FRESULT res = (nextNum == 9999 && findHighestLogNumber() == 9999)
                  ? SD_FileOpen(&logFile, fileName, FA_CREATE_ALWAYS | FA_WRITE)    // Overwrite
                  : SD_FileOpen(&logFile, fileName, FA_CREATE_NEW | FA_WRITE);      // Don't allow overwrite
    if (res != FR_OK) {
        myprintf("DL: f_open(%s) failed (%d)\r\n", fileName, res);
        return res;
    }

    // Write Header Row
    uint8_t written;
    res = SD_FileWrite(&logFile, CSV_HEADER, strlen(CSV_HEADER), &written);
    if (res != FR_OK || written != strlen(CSV_HEADER)) {
        myprintf("DL: header write failed (%d)\r\n", res);
        SD_FileClose(&logFile);
        return res;
    }

    fileOpen = 1;
    myprintf("DL: logging to %s\r\n", fileName);
    return FR_OK;
}

/*
 * Format and write one CSV row using the latest IMU and BME680 data.
 * On failure, closes the file and transitions to DL_ERROR.
 */
static void writeCSVRow(uint32_t timestamp)
{
    // Get latest IMU data
    LSM6DSO_Data_t imu;
    if (!lsm6_readData(&imu)) {
        myprintf("DL: failed to read IMU data. Is it initialized?\r\n");
    }

    char line[192];

    // TODO come back to this and think about the timing, we want the timestamp to be as close to correct as it can be for the IMU data and the BME data, if we are just using cache we have no way of knowing when that data was actually obtained
    // TODO bmeValid should be renamed to something more like bmeHasReturnedFirstReading
    // Write CSV row with all data — if BME680 hasn't returned its first reading yet, leave those columns empty
    if (bmeValid) {
        snprintf(line, sizeof(line),
                 "%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
                 (unsigned long)timestamp,
                 imu.accel_mg[0], imu.accel_mg[1], imu.accel_mg[2],
                 imu.gyro_mdps[0], imu.gyro_mdps[1], imu.gyro_mdps[2],
                 imu.temperature_degC,
                 bmeCache.pressure_hPa,
                 bmeCache.temperature_degC,
                 bmeCache.humidity_pctRH);
    } else {
        /* BME680 hasn't returned its first reading yet — leave columns empty */
        snprintf(line, sizeof(line),
                 "%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,,,\r\n",
                 (unsigned long)timestamp,
                 imu.accel_mg[0], imu.accel_mg[1], imu.accel_mg[2],
                 imu.gyro_mdps[0], imu.gyro_mdps[1], imu.gyro_mdps[2],
                 imu.temperature_degC);
    }

    uint8_t written;
    FRESULT res = SD_FileWrite(&logFile, line, strlen(line), &written);
    if (res != FR_OK) {
        myprintf("DL: write failed (%d) — entering error state\r\n", res);
        fileOpen = 0;   // TODO I like variables like this to be named starting with "is" or "has" to make it more clear that they are booleans, maybe rename to isFileOpen
        dlState = DL_ERROR;
        return;
    }
}

/* =========================================================================
 * Public API
 * ========================================================================= */

void DataLogger_Init(void)
{
    bmeValid    = 0;
    fileOpen    = 0;
    dlState     = DL_IDLE;
    prevSdState = SD_GetState();

    bme680_setGasEnabled(0);         /* Gas sensor not needed for rocketry */
    bme680_triggerMeasurement();     /* Kick off first measurement immediately */
    bmeLastTrigger = HAL_GetTick();
}

// TODO I would like to rename all the task functions to *_StateMachine_Task() instead of *_Update()
/*
 * Single update task — register with the scheduler at 5 ms.
 *
 * Every call : run BME680 state machine.
 * Every DATALOGGER_IMU_POLL_MS : read IMU and write a CSV row.
 */
void DataLogger_Update(void)
{
    uint32_t  now          = HAL_GetTick();
    SD_State_t currentSdState = SD_GetState();

    /* --- Edge detection: react to SD card state transitions --- */
    if (currentSdState != prevSdState) {
        if (currentSdState == SD_MOUNTED && dlState == DL_IDLE) {
            /* Card just became mounted — try to open a log file */
            dlState = DL_OPENING;
        }
        if (currentSdState != SD_MOUNTED && fileOpen) {
            /* Card removed or error — abandon the file handle.
               Do NOT call SD_FileClose: the filesystem is already gone. */
            fileOpen = 0;
            dlState  = DL_IDLE;
        }
        prevSdState = currentSdState;
    }

    /* --- BME680 polling (runs every call = every 5 ms) --- */
    bme680_update();
    if (bme680_isDataReady()) {
        bmeCache = bme680_getData();
        bmeValid = 1;
    }

    // Trigger a new BME680 measurement every DATALOGGER_BME_TRIGGER_MS (50 ms)
    if ((now - bmeLastTrigger) >= DATALOGGER_BME_TRIGGER_MS) {
        bme680_triggerMeasurement();
        bmeLastTrigger = now;
    }

    // Note the IMU data is automatically continuously updated in the background, so we don't need to trigger it like we do for the BME680

    /* --- DataLogger state machine --- */
    switch (dlState)
    {
        case DL_IDLE:
            /* Nothing to do — sensors still polled above */
            break;

        case DL_OPENING:
            if (createLogFile() == FR_OK) {
                imuLastTick = now;
                dlState     = DL_LOGGING;
            } else {
                dlState = DL_ERROR;
            }
            break;

        case DL_LOGGING:
            // TODO DATALOGGER_IMU_POLL_MS should be renamed to something like DATALOGGER_CSV_WRITE_MS since it's not really about polling the IMU, we are just reading the latest data that is being continuously updated in the background by the IMU driver, we are really just writing a new CSV row every DATALOGGER_CSV_WRITE_MS
            // TODO I would like to rename imuLastTick to something like lastCsvWriteTick
            // TODO I feel there is a better way to do the timing here, trigger a measurement at the end of each write and then wait for the next measurement to be ready/valid and then log the time that that measurement was captured.  Perhaps we should have timing values associated with the data structs returned from the sensors instead of here, since we really want the timing associated with the sensors
            if ((now - imuLastTick) >= DATALOGGER_IMU_POLL_MS) {
                imuLastTick = now;
                writeCSVRow(now);
            }
            break;

        case DL_ERROR:
            /* Wait for card removal/re-insertion — edge detection above
               will move us back to DL_IDLE once the card is removed */
            break;
    }
}
