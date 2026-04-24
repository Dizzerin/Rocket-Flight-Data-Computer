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

#include <stdint.h>

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

/* Set to 1 to echo live sampled CSV log file rows to UART3 via myprintf() for live testing and debugging;
 * set to 0 to disable (default, no runtime cost when disabled).
 *
 * IMPORTANT: myprintf() is blocking. At 115200 baud a worst-case 147-byte row
 * takes ~12.8 ms to transmit, which exceeds the 10 ms CSV write period.
 * Echoing every row at 100 Hz would also exceed the UART's ~11,520 byte/sec
 * capacity (14,700 bytes/sec needed). For these reasons, UART echo is rate-limited
 * to DATALOGGER_UART_ECHO_MS and should only be used during bench testing. */
#define DATALOGGER_UART_ECHO        0

/* Milliseconds between UART echo prints when DATALOGGER_UART_ECHO is enabled.
 * Default 500 ms (2 Hz): ~294 bytes/sec UART load, ~12.8 ms blocking time
 * every 500 ms — well within the UART bandwidth and scheduler tolerance. */
#define DATALOGGER_UART_ECHO_MS     500U

void DataLogger_Init(void);
void DataLogger_StateMachine_Task(void);   /* Single update task — register at 5 ms */

/* Returns 1 if the DataLogger is actively writing to an open log file, 0 otherwise.
 * Used by the LED task to select the appropriate flash pattern. */
uint8_t DataLogger_IsLogging(void);

#endif /* DATALOGGER_H_ */
