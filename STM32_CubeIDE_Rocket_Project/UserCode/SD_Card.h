/*
 * SD_Card.h
 *
 * Generic SD card driver: card-detect interrupt, FatFS mount/unmount,
 * and thin file/directory wrappers that guard against unmounted state.
 *
 * This module has no knowledge of log files, CSV, or sensors.
 * File lifecycle (naming, creation, headers) belongs in the caller.
 *
 * Usage:
 *   1. Call SD_Init() once after MX_GPIO_Init() and MX_FATFS_Init().
 *   2. Register SD_Update() with the task scheduler to run at ~100 ms.
 *   3. Use SD_FileOpen/Write/Sync/Close for file I/O once SD_IsMounted().
 *   4. Poll SD_GetState() to detect mount/unmount transitions.
 */

#ifndef SD_CARD_H_
#define SD_CARD_H_

#include <stdint.h>
#include "ff.h"

typedef enum {
    SD_NOT_PRESENT,        /* Card detect pin HIGH — no card inserted       */
    SD_PRESENT_UNMOUNTED,  /* Card detected, waiting for settle delay       */
    SD_MOUNTING,           /* Calling f_mount                               */
    SD_MOUNTED,            /* Filesystem mounted, file ops available        */
    SD_ERROR               /* Mount failed; retries on re-insertion         */
} SD_State_t;

void       SD_Init(void);
void       SD_Update(void);                /* Run state machine — call every 100 ms */  // TODO rename it to something that include the words StateMachine
SD_State_t SD_GetState(void);
uint8_t    SD_IsMounted(void);             /* 1 if state == SD_MOUNTED              */

/* Generic file operations — all return FR_NOT_READY if not mounted */
FRESULT    SD_FileOpen(FIL *fp, const char *path, BYTE mode);
FRESULT    SD_FileClose(FIL *fp);
FRESULT    SD_FileWrite(FIL *fp, const void *buff, UINT btw, UINT *bw);

/* Directory operations */
FRESULT    SD_DirOpen(DIR *dp, const char *path);
FRESULT    SD_DirRead(DIR *dp, FILINFO *fno);
FRESULT    SD_DirClose(DIR *dp);

#endif /* SD_CARD_H_ */
