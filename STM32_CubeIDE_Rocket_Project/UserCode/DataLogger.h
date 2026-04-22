/*
 * DataLogger.h
 *
 * Owns the full data logging pipeline: sensor polling, CSV formatting,
 * log file lifecycle (naming, creation, header), and SD card writes.
 *
 * Uses SD_Card's generic file API for all storage I/O.
 * Detects SD card mount/unmount by polling SD_GetState().
 *
 * Usage:
 *   1. Call DataLogger_Init() once after lsm6_init() and bme680_init() and SD_Init().
 *   2. Register DataLogger_Update() with the scheduler at 5 ms.
 *
 * Internally, DataLogger_Update() runs BME680 polling every call (5 ms)
 * and IMU reads + CSV writes every DATALOGGER_IMU_POLL_MS (10 ms).
 * 
 * See DataLogger.c for implementation details and comments.
 *
 * Configurable rates:
 *   DATALOGGER_BME_TRIGGER_MS — how often a new BME680 measurement is started
 *                               (default 50 ms = 20 Hz)
 *   DATALOGGER_IMU_POLL_MS    — how often the IMU is read and a CSV row written
 *                               (default 10 ms = 100 Hz)
 */

#ifndef DATALOGGER_H_
#define DATALOGGER_H_

/* Milliseconds between BME680 measurement triggers (20 Hz = 50 ms) */
#define DATALOGGER_BME_TRIGGER_MS   50U

/* Milliseconds between IMU reads and CSV row writes (100 Hz = 10 ms) */
#define DATALOGGER_IMU_POLL_MS      10U

void DataLogger_Init(void);
void DataLogger_Update(void);   /* Single update task — register at 5 ms */

#endif /* DATALOGGER_H_ */
