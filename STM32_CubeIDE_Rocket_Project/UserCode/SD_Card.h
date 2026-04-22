/*
 * SD_Card.h
 *
 * SD card driver: state machine with card-detect interrupt, auto-incrementing
 * log file creation (LOG_0001.CSV ... LOG_9999.CSV), and f_sync() after each
 * write for power-loss or unexpected removal protection.
 *
 * Usage:
 *   1. Call SD_Init() once after MX_GPIO_Init() and MX_FATFS_Init().
 *   2. Register SD_Update() task with at task scheduler to run at ~100 ms.
 *   3. Call SD_WriteLine() to write lines to the log file, note they will not be written if the SD card is not in the READY state
 */

#ifndef SD_CARD_H_
#define SD_CARD_H_

#include <stdint.h>
#include "ff.h"

typedef enum {
    SD_NOT_PRESENT,        /* Card detect pin HIGH — no card inserted       */
    SD_PRESENT_UNMOUNTED,  /* Card detected, waiting for settle delay       */
    SD_MOUNTING,           /* Mounting FatFS, scanning files, creating log  */
    SD_READY,              /* Mounted, log file open, ready to write        */
    SD_ERROR               /* Unrecoverable error; retries on re-insertion  */
} SD_State_t;

void       SD_Init(void);
void       SD_Update(void);                /* Run state machine — call every 100 ms */
SD_State_t SD_GetState(void);
uint8_t    SD_IsReady(void);               /* 1 if state == SD_READY                */
FRESULT    SD_WriteLine(const char *line); /* Write + f_sync; no-op if not ready    */

#endif /* SD_CARD_H_ */
