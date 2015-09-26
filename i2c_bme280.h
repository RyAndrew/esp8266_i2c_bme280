/*
The driver for the temperature, pressure, & humidity sensor BME280
Official repository: https://github.com/RyAndrew/esp8266_i2c_bme280
Adapted From: https://github.com/CHERTS/esp8266-i2c_bmp180
This driver depends on the I2C driver https://github.com/zarya/esp8266_i2c_driver/

The MIT License (MIT)

Copyright (C) 2015 Andrew Rymarczyk

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
THE SOFTWARE.
*/

#ifndef __I2C_BME280_H
#define	__I2C_BME280_H

#include "c_types.h"
#include "ets_sys.h"
#include "osapi.h"

#define BME280_W					0xEC
#define BME280_R					0xED
#define BME280_CHIP_ID_REG			0xD0
#define BME280_CHIP_ID				0x60

#define BME280_REG_CTRL_HUM			0xF2
#define BME280_REG_CTRL_MEAS		0xF4
#define BME280_REG_CONFIG			0xF5

#define BME280_MODE_NORMAL			0x03 //reads sensors at set interval
#define BME280_MODE_FORCED			0x01 //reads sensors once when you write this register

//#define BME280_DEBUG 1 //uncomment for debugging messages

bool BME280_Init(uint8_t operationMode);
bool ICACHE_FLASH_ATTR BME280_startI2cWrite(void);
bool ICACHE_FLASH_ATTR BME280_sendI2cWriteData(uint8_t writeReg, uint8_t regData);
bool ICACHE_FLASH_ATTR BME280_sendI2cRead(uint8_t readReg);
bool ICACHE_FLASH_ATTR BME280_sendI2cReadSensorData();
bool BME280_verifyChipId(void);
void BME280_writeConfigRegisters(void);
void BME280_readCalibrationRegisters(void);

signed long int BME280_calibration_T(signed long int adc_T);
unsigned long int BME280_calibration_P(signed long int adc_P);
unsigned long int BME280_calibration_H(signed long int adc_H);

void BME280_readSensorData(void);

unsigned long int BME280_GetTemperatureRaw(void);
unsigned long int BME280_GetPressureRaw(void);
unsigned long int BME280_GetHumidityRaw(void);

signed long int BME280_GetTemperature(void);
unsigned long int BME280_GetPressure(void);
unsigned long int BME280_GetHumidity(void);

signed long int BME280_calibration_Temp(signed long int adc_T);
unsigned long int BME280_calibration_Press(signed long int adc_P);
unsigned long int BME280_calibration_Hum(signed long int adc_H);

#endif
