/*
 * UartService.h
 *
 *  Created on: May 17, 2025
 *      Author: gabrielmoblicci
 */

#ifndef SERVICE_UARTSERVICE_H
#define SERVICE_UARTSERVICE_H

#include <stdint.h>

typedef struct
{
	char message[128];
} UartMessage;

typedef enum {
    MSG_TYPE_BAROMETER,
    MSG_TYPE_ACCEL,
    MSG_TYPE_GYRO,
} CommMessageType;

typedef struct {
    CommMessageType type;

    union {
        BarometerData baro;
        AccelData accel;
        GyroData gyro;
    } payload;

    char message[128];
} CommMessage;


#endif /* SERVICE_UARTSERVICE_H */
