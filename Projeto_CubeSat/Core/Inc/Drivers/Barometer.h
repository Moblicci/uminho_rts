/*
 * Barometer.h
 *
 *  Created on: May 6, 2025
 *      Author: gabrielmoblicci
 */

#ifndef DRIVERS_BAROMETER_H
#define DRIVERS_BAROMETER_H

#include <stdbool.h>
#include "stm32f7xx_hal.h"

typedef struct
{
	float pressure;
	float temperature;
	float humidity;
	double altitude;
} BarometerData;

bool Barometer_Init(I2C_HandleTypeDef* i2c_handle);
bool Barometer_Read(BarometerData* out);

#endif /* DRIVERS_BAROMETER_H */
