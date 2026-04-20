/*
 * SD_Card.h
 *
 *  Created on: Mar 16, 2026
 *      Author: nicholaschang
 */

#ifndef SD_CARD_H_
#define SD_CARD_H_

#include <stdint.h>
#include "ff.h"

FRESULT SD_initAndMount(void);
FRESULT SD_read(void);
FRESULT SD_createAndOpenFile(const char* fileName);
FRESULT SD_openFileForWriting(const char* fileName);
FRESULT SD_writeToOpenedFile(const char* string);
FRESULT SD_closeFile(void);
FRESULT SD_unmount(void);


#endif /* SD_CARD_H_ */

