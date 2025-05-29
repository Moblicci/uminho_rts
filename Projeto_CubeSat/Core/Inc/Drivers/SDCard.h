/*
 * SDCard.h
 *
 *  Created on: May 6, 2025
 *      Author: gabrielmoblicci
 */

#ifndef DRIVERS_SDCARD_H
#define DRIVERS_SDCARD_H

#include <stdbool.h>

bool SDCard_Init(void);
bool SDCard_WriteLine(const char* filename, const char* line);
bool SDCard_ClearFile(const char* filename);
void SDCard_Deinit(void);

#endif /* DRIVERS_SDCARD_H */
