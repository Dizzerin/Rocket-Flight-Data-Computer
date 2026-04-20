/*
 * SD_Card.c
 *
 *  Created on: Mar 16, 2026
 *      Author: nicholaschang
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "SD_Card.h"
#include "fatfs.h"
#include "stm32h7xx_hal.h" /* Provide the low-level HAL functions - GPIO pin for SD_CARD_DETECT*/

// TODO implement SD_CARD_DETECT (active low I believe)

// Local globals vars
//some variables for FatFs
FATFS FatFs; 	//Fatfs handle
FIL fil; 		//File handle
FRESULT fres; //Result after operations


FRESULT SD_initAndMount(void)
{
    myprintf("Initializing and Mounting SD Card...\r\n");

    HAL_Delay(1000); //a short delay is important to let the SD card settle

    //Open the file system
    fres = f_mount(&FatFs, "", 1); //1=mount now
    if (fres != FR_OK) {
    	myprintf("f_mount error (%i)\r\n", fres);
    	return 1;
    }

    //Let's get some statistics from the SD card
    DWORD free_clusters, free_sectors, total_sectors;

    FATFS* getFreeFs;

    fres = f_getfree("", &free_clusters, &getFreeFs);
    if (fres != FR_OK) {
    	myprintf("f_getfree error (%i)\r\n", fres);
    	return 1;
    }

    //Formula comes from ChaN's documentation
    total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
    free_sectors = free_clusters * getFreeFs->csize;

    myprintf("Successfully initialized and mounted SD card!\r\n");
    myprintf("SD card stats:\r\n"
    		"%10lu KiB total drive space.\r\n"
    		"%10lu KiB available.\r\n", total_sectors / 2, free_sectors / 2);

    return FR_OK;
}



FRESULT SD_read(void)
{
	//Example code for reading a file

    // ----------- START reading ------------------ //
	myprintf("Attempting to read test.txt file...\r\n");

    //Now let's try to open file "test.txt"
    fres = f_open(&fil, "test.txt", FA_READ);
    if (fres != FR_OK) {
    	myprintf("f_open error (%i)\r\n");
    	while(1);
    }
    myprintf("I was able to open 'test.txt' for reading!\r\n");

    //Read 30 bytes from "test.txt" on the SD card
    BYTE readBuf[30];

    //We can either use f_read OR f_gets to get data out of files
    //f_gets is a wrapper on f_read that does some string formatting for us
    TCHAR* rres = f_gets((TCHAR*)readBuf, 30, &fil);
    if(rres != 0) {
    	myprintf("Read string from 'test.txt' contents: %s\r\n", readBuf);
    } else {
    	myprintf("f_gets error (%i)\r\n", fres);
    }

    //Be tidy - don't forget to close your file!
    f_close(&fil);
    // ----------- END reading ------------------ //

	// Todo return proper value
	return FR_OK;
}

FRESULT SD_createAndOpenFile(const char* fileName)
{
	myprintf("Attempting to create %s\r\n", fileName);

	// Create file
	fres = f_open(&fil, fileName, FA_CREATE_NEW);
	if(fres == FR_OK) {
		myprintf("Created file\r\n");
	} else {
		myprintf("f_open error (%i)\r\n", fres);
	}

	return fres;

}

FRESULT SD_openFileForWriting(const char* fileName)
{
	// TODO make sure the file isn't already open

	// Open file
	fres = f_open(&fil, fileName, FA_OPEN_APPEND | FA_WRITE);
	if(fres == FR_OK) {
		myprintf("Opened file\r\n");
	} else {
		myprintf("f_open error (%i)\r\n", fres);
	}

	return fres;
}

FRESULT SD_writeToOpenedFile(const char* string)
{
	// TODO make sure file is opened!

	// Write the string into the file
	UINT bytesWrote;
	fres = f_write(&fil, string, strlen(string), &bytesWrote);
	if(fres == FR_OK) {
			myprintf("Wrote %i bytes to file\r\n", bytesWrote);
	} else {
		myprintf("f_write error (%i)\r\n");
	}

	return fres;
}

FRESULT SD_closeFile(void)
{
	return f_close(&fil);
}

FRESULT SD_unmount(void)
{
    return f_mount(NULL, "", 0);
}

