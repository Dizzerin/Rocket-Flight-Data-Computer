/*
 * DataLogger.c
 *
 * Collects data from the LSM6DSO32 IMU and BME680 barometer and writes
 * CSV rows to the SD card.
 *
 * IMU is sampled every 10 ms (100 Hz) by DataLogger_UpdateIMU().
 * BME680 measurements are triggered every DATALOGGER_BME_TRIGGER_MS (50 ms =
 * 20 Hz) by DataLogger_UpdateBME(), which also polls the non-blocking BME680
 * state machine every 5 ms.
 *
 * Each CSV row contains all sensor values. BME680 columns repeat the last
 * valid reading between measurement updates (every ~5 rows at 100 Hz IMU rate).
 *
 * Gas measurement is disabled — irrelevant for rocketry and saves ~100 ms
 * per measurement cycle.
 */

#include "DataLogger.h"
#include "SD_Card.h"
#include "bme680_spi.h"
#include "lsm6dso32_device.h"
#include <stdio.h> // For snprinf
#include "main.h"  // For HAL_GetTick() (TODO is there a better place to get this from?)
#include "stm32h7xx_hal.h"

/* =========================================================================
 * Module state
 * ========================================================================= */

static BME680_Data_t bmeCache;
static uint8_t       bmeValid        = 0;   /* 1 once the first reading arrives */
static uint32_t      bmeLastTrigger  = 0;

/* =========================================================================
 * Public API
 * ========================================================================= */

void DataLogger_Init(void)
{
    bmeValid = 0;
    bme680_setGasEnabled(0);         /* Gas sensor not needed for rocketry */
    bme680_triggerMeasurement();     /* Kick off first measurement immediately */
    bmeLastTrigger = HAL_GetTick();
}

/*
 * Called by the scheduler every 5 ms.
 * Advances the BME680 non-blocking state machine. When data is ready,
 * caches it and triggers the next measurement if the interval has elapsed.
 */
void DataLogger_UpdateBME(void)
{
    bme680_update();

    if (bme680_isDataReady()) {
        bmeCache = bme680_getData();
        bmeValid = 1;

        if ((HAL_GetTick() - bmeLastTrigger) >= DATALOGGER_BME_TRIGGER_MS) {
            bme680_triggerMeasurement();
            bmeLastTrigger = HAL_GetTick();
        }
    }
}

/*
 * Called by the scheduler every 10 ms (100 Hz).
 * Reads the IMU, formats a CSV line with the latest BME680 cache values,
 * and writes it to the SD card.
 */
void DataLogger_UpdateIMU(void)
{
    if (!SD_IsReady()) return;

    LSM6DSO_Data_t imu;
    lsm6_readData(&imu);

    char line[192];

    if (bmeValid) {
        snprintf(line, sizeof(line),
                 "%lu,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\r\n",
                 (unsigned long)HAL_GetTick(),
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
                 (unsigned long)HAL_GetTick(),
                 imu.accel_mg[0], imu.accel_mg[1], imu.accel_mg[2],
                 imu.gyro_mdps[0], imu.gyro_mdps[1], imu.gyro_mdps[2],
                 imu.temperature_degC);
    }

    SD_WriteLine(line);
}
