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
 *   2. Register DataLogger_StateMachine_Task() with the scheduler at 5 ms.
 *
 * Internally, DataLogger_StateMachine_Task() runs the BME680 state machine every
 * call (5 ms) and gets new LSM6 data and writes a CSV row every DATALOGGER_CSV_WRITE_MS (10 ms).
 *
 * See DataLogger.c for implementation details and comments.
 *
 * Configurable rates:
 *   DATALOGGER_BME_TRIGGER_MS  — how often a new BME680 measurement is started
 *                                (default 50 ms = 20 Hz)
 *   DATALOGGER_CSV_WRITE_MS    — how often a CSV row is written - and how often the LMS6DSO is read and logged
 *                                (default 10 ms = 100 Hz)
 */

#ifndef DATALOGGER_H_
#define DATALOGGER_H_

/* Milliseconds between BME680 measurement triggers (20 Hz = 50 ms) */
#define DATALOGGER_BME_TRIGGER_MS   50U

/* Milliseconds between CSV row writes (100 Hz = 10 ms) */
#define DATALOGGER_CSV_WRITE_MS     10U

/* Milliseconds between SD card syncs (1 Hz = 1000 ms).
 * f_sync forces the data sector, FAT, and directory entry to physical media —
 * 2–3 sector writes costing several milliseconds each. Syncing every 1 second
 * instead of every row keeps SD overhead under 1% of CPU time. In the worst
 * case of unexpected power-off, at most 1 second of log data is lost. */
#define DATALOGGER_SYNC_MS          1000U

void DataLogger_Init(void);
void DataLogger_StateMachine_Task(void);   /* Single update task — register at 5 ms */

#endif /* DATALOGGER_H_ */
