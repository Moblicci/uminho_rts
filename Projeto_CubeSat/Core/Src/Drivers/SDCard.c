/*
 * SDCard.c
 *
 *  Created on: May 6, 2025
 *      Author: gabrielmoblicci
 */

#include "fatfs.h"
#include "SDCard.h"
#include <stdio.h>
#include <string.h>

static FATFS FatFs;
static FIL file;

bool SDCard_Init(void)
{
	return (f_mount(&FatFs, "", 1) == FR_OK);
}

bool SDCard_WriteLine(const char* filename, const char* line)
{
	if (f_open(&file, filename, FA_WRITE | FA_OPEN_APPEND) != FR_OK)
		return false;

	UINT bw;
	FRESULT res = f_write(&file, line, strlen(line), &bw);
	f_close(&file);

	if(res == FR_OK)
	{
		return true;
	}
	else
	{
		return false;
	}
}

bool SDCard_ClearFile(const char* filename)
{
    if (f_open(&file, filename, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK)
        return false;

    f_close(&file);
    return true;
}

void SDCard_Deinit(void)
{
	f_mount(NULL, "", 0);
}
