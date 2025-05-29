/*
 * LoggerService.h
 *
 *  Created on: May 6, 2025
 *      Author: gabrielmoblicci
 */

#ifndef SERVICE_LOGGERSERVICE_H
#define SERVICE_LOGGERSERVICE_H

#include <stdbool.h>
#include "Barometer.h"
#include "Accelerometer.h"
#include "Gyroscope.h"

bool LoggerService_LogBMPData(const BarometerData data);
bool LoggerService_LogAccelData(const AccelData data);
bool LoggerService_LogGyroData(const GyroData data);
bool LoggerService_ClearBmpLogFile();


#endif /* SERVICE_LOGGERSERVICE_H */
