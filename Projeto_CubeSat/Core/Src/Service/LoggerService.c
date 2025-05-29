/*
 * LoggerService.c
 *
 *  Created on: May 6, 2025
 *      Author: gabrielmoblicci
 */

#include "LoggerService.h"
#include "Barometer.h"
#include "SDCard.h"
#include <stdio.h>
#include "cmsis_os.h"

static void FormatLogLine(char* buffer, size_t len, float p, float t, double a);

bool LoggerService_LogBMPData(const BarometerData data)
{
	char line[128];
	FormatLogLine(line, sizeof(line), data.pressure, data.temperature, data.altitude);

	bool success =  SDCard_WriteLine("bmp_log.txt", line);

	return success;
}

bool LoggerService_LogAccelData(const AccelData data)
{
	printf("Logger: ACC gravando X=%.2f\r\n", data.x);

	char line[128];
	snprintf(line, sizeof(line), "ACC: X=%.2f, Y=%.2f, Z=%.2f\r\n",
			data.x, data.y, data.z);
	return SDCard_WriteLine("accel_log.txt", line);
}

bool LoggerService_LogGyroData(const GyroData data)
{
	char line[128];
	snprintf(line, sizeof(line), "GYRO: X=%.2f, Y=%.2f, Z=%.2f\r\n",
			data.x, data.y, data.z);
	return SDCard_WriteLine("gyro_log.txt", line);
}

bool LoggerService_ClearBmpLogFile()
{
	return SDCard_ClearFile("bmp_log.txt");
}

static void FormatLogLine(char* buffer, size_t len, float p, float t, double a)
{
    snprintf(buffer, len, "P=%.2f,T=%.2f,A=%.2f\r\n", p, t, a);
}
