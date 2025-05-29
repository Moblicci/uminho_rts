/*
 * Barometer.c
 *
 *  Created on: May 6, 2025
 *      Author: gabrielmoblicci
 */

#include "Barometer.h"
#include "bmp280.h"
#include "cmsis_os.h"
#include <stdio.h>
#include <math.h>

#define SEA_LEVEL_PRESSURE 101500.0f

static BMP280_HandleTypedef bmp280;
static bool initialized = false;
static bool is_bme280 = false;

bool Barometer_Init(I2C_HandleTypeDef* i2c_handle)
{
    printf("Initializing barometer...\r\n");

    bmp280_init_default_params(&bmp280.params);
    bmp280.addr = BMP280_I2C_ADDRESS_0;
    bmp280.i2c = i2c_handle;//&hi2c2;

    for (int attempt = 0; attempt < 5; attempt++)
    {
        if (bmp280_init(&bmp280, &bmp280.params)) {
            initialized = true;
            is_bme280 = (bmp280.id == BME280_CHIP_ID);
            printf("BMP280 identified: %s\r\n", is_bme280 ? "BME280" : "BMP280");
            return true;
        }

        printf("Failed to initialize barometer. Trying again...\r\n");
        osDelay(1000);
    }

    printf("Error: wasn't possible to initialize barometer.\r\n");
    return false;
}

bool Barometer_Read(BarometerData* out)
{
	float pressure, temperature, humidity;

    if (!initialized) {
        printf("Error: barometer is not initialized.\r\n");
        return false;
    }

    if (!bmp280_read_float(&bmp280, &temperature, &pressure, &humidity)) {
        printf("Error trying to read from barometer.\r\n");
        return false;
    }

    out->pressure = pressure;
    out->temperature = temperature;
    out->humidity = humidity;
    out->altitude = 44330.0 * (1.0 - pow((pressure / SEA_LEVEL_PRESSURE), 0.1903));

    return true;
}
