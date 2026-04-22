/*
 * DataLogger.h
 * TODO create a single dataLogger state machine that handles both the IMU and BME updates and writing to SD card, instead of having the DataLogger_UpdateBME and DataLogger_UpdateIMU functions be separate.
 *
 * Ties the LSM6DSO32 IMU and BME680 barometer together and writes CSV rows
 * to the SD card via SD_WriteLine().
 *
 * Usage:
 *   1. Call DataLogger_Init() once after bme680_init() and SD_Init().
 *   2. Register DataLogger_UpdateBME() with the scheduler at 5 ms.
 *   3. Register DataLogger_UpdateIMU() with the scheduler at 10 ms.
 *
 * Configurable rates:
 *   DATALOGGER_BME_TRIGGER_MS — how often a new BME680 measurement is started
 *                               (default 50 ms = 20 Hz)
 */

#ifndef DATALOGGER_H_
#define DATALOGGER_H_

/* Milliseconds between BME680 measurement triggers (20 Hz = 50 ms) */
#define DATALOGGER_BME_TRIGGER_MS   50U

void DataLogger_Init(void);
void DataLogger_UpdateBME(void);   /* Poll BME680 state machine + trigger at 20 Hz */
void DataLogger_UpdateIMU(void);   /* Read IMU at 100 Hz and write CSV row          */

#endif /* DATALOGGER_H_ */
