/*
 * SD_Card.c
 *
 * SD card state machine with card-detect interrupt support.
 *
 * States:
 *   NOT_PRESENT      — No card (detect pin HIGH, active-low).
 *   PRESENT_UNMOUNTED— Card just inserted; waiting 500 ms for power settle.
 *  // TODO should we separate this out?  Better separation of concerns so that this is just a generic SD card driver and somewhere else we handle the log file naming and creation etc.  That should probably be done in the DataLogger module instead.
 *   MOUNTING         — Mounting FatFS, scanning for existing log files,
 *                      creating the next LOG_XXXX.CSV, writing the header.
 *   READY            — Log file open and accepting writes.
 *   ERROR            — Mount or file-create failed; stays here until removal.
 *
 * The HAL_GPIO_EXTI_Callback override sets a flag on SD_CARD_DETECT changes.
 * SD_Update() processes the flag and reads the pin to determine direction.
 *
 * File naming uses 8.3 format: LOG_0001.CSV through LOG_9999.CSV since long filename support is not currently enabled in CubeMX.
 * The directory is scanned on every mount to find the highest existing number
 * and create the next one.
 *
 * Write strategy: keep file open, call f_sync() after each write.
 */

#include <stdio.h>
#include <string.h>
#include "SD_Card.h"
#include "fatfs.h"
#include "main.h"           /* SD_CARD_DETECT pin defines, myprintf() */
#include "stm32h7xx_hal.h" /* Provide the low-level HAL functions - GPIO pin for SD_CARD_DETECT*/

/* =========================================================================
 * Module state
 * ========================================================================= */

static SD_State_t  state            = SD_NOT_PRESENT;
static FATFS       fatFs;
static FIL         logFile; // TODO change to generic file handle if we separate out the log file management from the SD card driver
static uint8_t     fileOpen         = 0;

/* Set to 1 from EXTI ISR; cleared after SD_Update processes it */
static volatile uint8_t cardDetectChanged = 0;

/* Tick captured when card is first detected — used for the CARD_SETTLE_MS wait */
static uint32_t mountWaitStartTick = 0;

#define CARD_SETTLE_MS  500U

/* CSV header written once at the top of each new log file */
static const char CSV_HEADER[] =
    "Timestamp_ms,Accel_X_mg,Accel_Y_mg,Accel_Z_mg,"
    "Gyro_X_mdps,Gyro_Y_mdps,Gyro_Z_mdps,"
    "IMU_Temp_C,Pressure_hPa,BME_Temp_C,Humidity_pctRH\r\n";

/* =========================================================================
 * EXTI callback — called from HAL interrupt context
 * ========================================================================= */

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == SD_CARD_DETECT_Pin) {
        cardDetectChanged = 1;
    }
}

/* =========================================================================
 * Internal helpers
 * ========================================================================= */

/* Returns 1 if the SD card is physically present (pin LOW = active low). */
static uint8_t cardIsPresent(void)
{
    return (HAL_GPIO_ReadPin(SD_CARD_DETECT_GPIO_Port, SD_CARD_DETECT_Pin)
            == GPIO_PIN_RESET) ? 1 : 0;
}

/*
 * Scan the root directory for files matching LOG_XXXX.CSV.
 * Returns the highest XXXX found, or 0 if none exist.
 */
static uint16_t findHighestLogNumber(void)
{
    DIR     dir;
    FILINFO fno;
    uint16_t maxNum = 0;

    if (f_opendir(&dir, "/") != FR_OK) return 0;

    while (f_readdir(&dir, &fno) == FR_OK && fno.fname[0] != '\0') {
        /* Match LOG_XXXX.CSV — name must be exactly 12 chars (8.3 + dot) */
        if (strncmp(fno.fname, "LOG_", 4) == 0) {
            /* Extract the 4-digit number after "LOG_" */
            uint16_t num = (uint16_t)atoi(fno.fname + 4);
            if (num > maxNum) maxNum = num;
        }
    }
    f_closedir(&dir);
    return maxNum;
}

/*
 * Mount the filesystem, find the next log file number, create the file,
 * write the CSV header, and leave the file open for appending.
 * Returns FR_OK on success.
 */
static FRESULT mountAndCreateLog(void)
{
    FRESULT res;
    UINT    written;

    res = f_mount(&fatFs, "", 1);
    if (res != FR_OK) {
        myprintf("SD: f_mount failed (%d)\r\n", res);
        return res;
    }

    uint16_t nextNum = findHighestLogNumber() + 1;
    if (nextNum > 9999) nextNum = 9999;   /* clamp — unlikely but safe */

    char fileName[13];   /* "LOG_XXXX.CSV" = 12 chars + null */
    snprintf(fileName, sizeof(fileName), "LOG_%04u.CSV", nextNum);

    res = f_open(&logFile, fileName, FA_CREATE_NEW | FA_WRITE);
    if (res != FR_OK) {
        myprintf("SD: f_open(%s) failed (%d)\r\n", fileName, res);
        f_mount(NULL, "", 0);
        return res;
    }

    res = f_write(&logFile, CSV_HEADER, strlen(CSV_HEADER), &written);
    if (res != FR_OK || written != strlen(CSV_HEADER)) {
        myprintf("SD: header write failed (%d)\r\n", res);
        f_close(&logFile);
        f_mount(NULL, "", 0);
        return (res != FR_OK) ? res : FR_DISK_ERR;
    }

    f_sync(&logFile);
    fileOpen = 1;

    myprintf("SD: ready — %s created\r\n", fileName);
    return FR_OK;
}

/* Close log file and unmount cleanly. */
static void unmount(void)
{
    if (fileOpen) {
        f_close(&logFile);
        fileOpen = 0;
    }
    f_mount(NULL, "", 0);
}

/* =========================================================================
 * Public API
 * ========================================================================= */

void SD_Init(void)
{
    fileOpen = 0;
    cardDetectChanged = 0;

    if (cardIsPresent()) {
        myprintf("SD: card present at boot — starting settle wait\r\n");
        mountWaitStartTick = HAL_GetTick();
        state = SD_PRESENT_UNMOUNTED;
    } else {
        myprintf("SD: no card at boot\r\n");
        state = SD_NOT_PRESENT;
    }
}

void SD_Update(void)
{
    switch (state)
    {
        case SD_NOT_PRESENT:
            if (cardDetectChanged) {
                cardDetectChanged = 0;
                if (cardIsPresent()) {
                    myprintf("SD: card inserted — waiting for settle\r\n");
                    mountWaitStartTick = HAL_GetTick();
                    state = SD_PRESENT_UNMOUNTED;
                }
            }
            break;

        case SD_PRESENT_UNMOUNTED:
            /* Check for removal during the settle wait */
            if (cardDetectChanged) {
                cardDetectChanged = 0;
                if (!cardIsPresent()) {
                    myprintf("SD: card removed during settle\r\n");
                    state = SD_NOT_PRESENT;
                    break;
                }
            }
            if ((HAL_GetTick() - mountWaitStartTick) >= CARD_SETTLE_MS) {
                state = SD_MOUNTING;
            }
            break;

        case SD_MOUNTING:
            if (mountAndCreateLog() == FR_OK) {
                state = SD_READY;
            } else {
                state = SD_ERROR;
            }
            break;

        case SD_READY:
            if (cardDetectChanged) {
                cardDetectChanged = 0;
                if (!cardIsPresent()) {
                    myprintf("SD: card removed — unmounting\r\n");
                    unmount();
                    state = SD_NOT_PRESENT;
                }
            }
            break;

        case SD_ERROR:
            if (cardDetectChanged) {
                cardDetectChanged = 0;
                if (!cardIsPresent()) {
                    myprintf("SD: card removed (was in error state)\r\n");
                    unmount();
                    state = SD_NOT_PRESENT;
                }
            }
            break;
    }
}

SD_State_t SD_GetState(void)
{
    return state;
}

uint8_t SD_IsReady(void)
{
    return (state == SD_READY) ? 1 : 0;
}

FRESULT SD_WriteLine(const char *line)
{
    if (state != SD_READY || !fileOpen) return FR_NOT_READY;

    UINT written;
    FRESULT res = f_write(&logFile, line, strlen(line), &written);
    if (res != FR_OK) {
        myprintf("SD: write failed (%d) — entering error state\r\n", res);
        unmount();
        state = SD_ERROR;
        return res;
    }

    res = f_sync(&logFile);
    if (res != FR_OK) {
        myprintf("SD: f_sync failed (%d) — entering error state\r\n", res);
        unmount();
        state = SD_ERROR;
    }
    return res;
}
