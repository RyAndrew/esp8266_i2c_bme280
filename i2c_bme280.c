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

#include <math.h>

#include "i2c_bme280.h"
#include "../i2c/i2c.h"

uint16_t calib_dig_T1;
 int16_t calib_dig_T2;
 int16_t calib_dig_T3;
uint16_t calib_dig_P1;
 int16_t calib_dig_P2;
 int16_t calib_dig_P3;
 int16_t calib_dig_P4;
 int16_t calib_dig_P5;
 int16_t calib_dig_P6;
 int16_t calib_dig_P7;
 int16_t calib_dig_P8;
 int16_t calib_dig_P9;
 int8_t  calib_dig_H1;
 int16_t calib_dig_H2;
 int8_t  calib_dig_H3;
 int16_t calib_dig_H4;
 int16_t calib_dig_H5;
 int8_t  calib_dig_H6;

 uint8_t osrs_t = 1;             //Temperature oversampling x 1
 uint8_t osrs_p = 1;             //Pressure oversampling x 1
 uint8_t osrs_h = 1;             //Humidity oversampling x 1

 uint8_t t_sb = 4;               //Tstandby, 5=1000ms, 4=500ms
 uint8_t filter = 0;             //Filter off
 uint8_t spi3w_en = 0;           //3-wire SPI Disable

 uint8_t BME280_OperationMode = BME280_MODE_NORMAL;

 unsigned long int hum_raw, temp_raw, pres_raw;
 signed long int t_fine;
 signed long int temp_act;
 unsigned long int press_act, hum_act;

bool ICACHE_FLASH_ATTR BME280_Init(uint8_t operationMode)
{
	i2c_init();

	if(!BME280_verifyChipId()){
		return 0;
	}

	BME280_OperationMode = operationMode;

	BME280_writeConfigRegisters();

	BME280_readCalibrationRegisters();

	return 1;
}

bool ICACHE_FLASH_ATTR BME280_sendI2cWriteData(uint8_t writeReg, uint8_t regData){
    if(!BME280_startI2cWrite() ){
    	return 0;
    }
	i2c_writeByte(writeReg);
	if(!i2c_check_ack()){
		#ifdef BME280_DEBUG
		ets_uart_printf("BME280_sendI2cWriteData: i2c_writeByte(%X) slave not ack..\r\n", writeReg);
		#endif
		i2c_stop();
		return 0;
	}
	i2c_writeByte(regData);
	if(!i2c_check_ack()){
		#ifdef BME280_DEBUG
		ets_uart_printf("BME280_sendI2cWriteData: i2c_writeByte(%X) slave not ack..\r\n", regData);
		#endif
		i2c_stop();
		return 0;
	}
	i2c_stop();
	return 1;
}

bool ICACHE_FLASH_ATTR BME280_startI2cWrite(){
	i2c_start();
	i2c_writeByte(BME280_W);
	if(!i2c_check_ack()){
		#ifdef BME280_DEBUG
		ets_uart_printf("BME280_startI2cWrite: i2c_writeByte(BME280_W) slave not ack..\r\n");
		#endif
		i2c_stop();
		return 0;
	}
	return 1;
}

bool ICACHE_FLASH_ATTR BME280_sendI2cRead(uint8_t readReg){

	i2c_start();
	i2c_writeByte(BME280_W);
	if(!i2c_check_ack()){
		#ifdef BME280_DEBUG
		ets_uart_printf("BME280_sendI2cRead: i2c_writeByte(BME280_W) slave not ack..\r\n");
		#endif
		i2c_stop();
		return 0;
	}

	i2c_writeByte(readReg);
	if(!i2c_check_ack()){
		#ifdef BME280_DEBUG
		ets_uart_printf("BME280_sendI2cRead: i2c_writeByte(readReg) slave not ack..\r\n");
		#endif
		i2c_stop();
		return 0;
	}
	i2c_start();
	i2c_writeByte(BME280_R);
	if(!i2c_check_ack()){
		#ifdef BME280_DEBUG
		ets_uart_printf("BME280_sendI2cRead: i2c_writeByte(BME280_R) slave not ack..\r\n");
		#endif
		i2c_stop();
		return 1;
	}

}

bool ICACHE_FLASH_ATTR BME280_verifyChipId(void){

	BME280_sendI2cRead(BME280_CHIP_ID_REG);

	uint8_t version = i2c_readByte();
	i2c_send_ack(0);
	i2c_stop();

	if (version != BME280_CHIP_ID ) {
		#ifdef BME280_DEBUG
		ets_uart_printf("BME280: expected chip id 0x%X, found chip id 0x%X\r\n", BME280_CHIP_ID, version);
		#endif
	    return 0;
	}

	return 1;
}

void ICACHE_FLASH_ATTR BME280_writeConfigRegisters(void){

    uint8_t ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | BME280_OperationMode;
    uint8_t ctrl_hum_reg  = osrs_h;

    uint8_t config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en;

    BME280_sendI2cWriteData(BME280_REG_CTRL_HUM, ctrl_hum_reg);
    BME280_sendI2cWriteData(BME280_REG_CTRL_MEAS, ctrl_meas_reg);
    BME280_sendI2cWriteData(BME280_REG_CONFIG, config_reg);

}

void ICACHE_FLASH_ATTR BME280_readCalibrationRegisters(void){

	uint8_t msb, lsb;

	//////////////
	// Read section 0x88
	BME280_sendI2cRead(0x88);

	lsb = i2c_readByte(); i2c_send_ack(1);
	msb = i2c_readByte(); i2c_send_ack(1);
	calib_dig_T1 = (msb << 8) | lsb;
	//#ifdef BME280_DEBUG
	//ets_uart_printf("lsb 0x%X, msb: 0x%X = calib_dig_T1 = %u\r\n", lsb, msb, calib_dig_T1);
	//#endif

	lsb = i2c_readByte(); i2c_send_ack(1);
	msb = i2c_readByte(); i2c_send_ack(1);
	calib_dig_T2 = (msb << 8) | lsb;
	//#ifdef BME280_DEBUG
	//ets_uart_printf("lsb 0x%X, msb: 0x%X = calib_dig_T2 = %d\r\n", lsb, msb, calib_dig_T2);
	//#endif

	lsb = i2c_readByte(); i2c_send_ack(1);
	msb = i2c_readByte(); i2c_send_ack(1);
	calib_dig_T3 = (msb << 8) | lsb;
	//#ifdef BME280_DEBUG
	//ets_uart_printf("lsb 0x%X, msb: 0x%X = calib_dig_T3 = %d\r\n", lsb, msb, calib_dig_T3);
	//#endif

	lsb = i2c_readByte(); i2c_send_ack(1);
	msb = i2c_readByte(); i2c_send_ack(1);
	calib_dig_P1 = (msb << 8) | lsb;
	//#ifdef BME280_DEBUG
	//ets_uart_printf("lsb 0x%X, msb: 0x%X = calib_dig_P1 = %u\r\n", lsb, msb, calib_dig_P1);
	//#endif

	lsb = i2c_readByte(); i2c_send_ack(1);
	msb = i2c_readByte(); i2c_send_ack(1);
	calib_dig_P2 = (msb << 8) | lsb;
	//#ifdef BME280_DEBUG
	//ets_uart_printf("lsb 0x%X, msb: 0x%X = calib_dig_P2 = %d\r\n", lsb, msb, calib_dig_P2);
	//#endif

	lsb = i2c_readByte(); i2c_send_ack(1);
	msb = i2c_readByte(); i2c_send_ack(1);
	calib_dig_P3 = (msb << 8) | lsb;
	//#ifdef BME280_DEBUG
	//ets_uart_printf("lsb 0x%X, msb: 0x%X = calib_dig_P3 = %d\r\n", lsb, msb, calib_dig_P3);
	//#endif

	lsb = i2c_readByte(); i2c_send_ack(1);
	msb = i2c_readByte(); i2c_send_ack(1);
	calib_dig_P4 = (msb << 8) | lsb;
	//#ifdef BME280_DEBUG
	//ets_uart_printf("lsb 0x%X, msb: 0x%X = calib_dig_P4 = %d\r\n", lsb, msb, calib_dig_P4);
	//#endif

	lsb = i2c_readByte(); i2c_send_ack(1);
	msb = i2c_readByte(); i2c_send_ack(1);
	calib_dig_P5 = (msb << 8) | lsb;
	//#ifdef BME280_DEBUG
	//ets_uart_printf("lsb 0x%X, msb: 0x%X = calib_dig_P5 = %d\r\n", lsb, msb, calib_dig_P5);
	//#endif

	lsb = i2c_readByte(); i2c_send_ack(1);
	msb = i2c_readByte(); i2c_send_ack(1);
	calib_dig_P6 = (msb << 8) | lsb;
	//#ifdef BME280_DEBUG
	//ets_uart_printf("lsb 0x%X, msb: 0x%X = calib_dig_P6 = %d\r\n", lsb, msb, calib_dig_P6);
	//#endif

	lsb = i2c_readByte(); i2c_send_ack(1);
	msb = i2c_readByte(); i2c_send_ack(1);
	calib_dig_P7 = (msb << 8) | lsb;
	//#ifdef BME280_DEBUG
	//ets_uart_printf("lsb 0x%X, msb: 0x%X = calib_dig_P7 = %d\r\n", lsb, msb, calib_dig_P7);
	//#endif

	lsb = i2c_readByte(); i2c_send_ack(1);
	msb = i2c_readByte(); i2c_send_ack(1);
	calib_dig_P8 = (msb << 8) | lsb;
	//#ifdef BME280_DEBUG
	//ets_uart_printf("lsb 0x%X, msb: 0x%X = calib_dig_P8 = %d\r\n", lsb, msb, calib_dig_P8);
	//#endif

	lsb = i2c_readByte(); i2c_send_ack(1);
	msb = i2c_readByte();
	i2c_send_ack(0);
	i2c_stop();

	calib_dig_P9 = (msb << 8) | lsb;
	//#ifdef BME280_DEBUG
	//ets_uart_printf("lsb 0x%X, msb: 0x%X = calib_dig_P9 = %d\r\n", lsb, msb, calib_dig_P9);
	//#endif

	//////////////
	// Read section 0xA1
	BME280_sendI2cRead(0xA1);

	msb = i2c_readByte(); i2c_send_ack(0); // STOP
	i2c_stop();
	calib_dig_H1 = msb;
	//#ifdef BME280_DEBUG
	//ets_uart_printf("msb: 0x%X = calib_dig_H1 = %d\r\n", msb, calib_dig_H1);
	//#endif

	//////////////
	// Read section 0xE1
	BME280_sendI2cRead(0xE1);

	lsb = i2c_readByte(); i2c_send_ack(1);
	msb = i2c_readByte(); i2c_send_ack(1);
	calib_dig_H2 = (msb << 8) | lsb;
	//#ifdef BME280_DEBUG
	//ets_uart_printf("lsb 0x%X, msb: 0x%X = calib_dig_H2 = %d\r\n", lsb, msb, calib_dig_H2);
	//#endif

	lsb = i2c_readByte(); i2c_send_ack(1);
	calib_dig_H3 = lsb;
	//#ifdef BME280_DEBUG
	//ets_uart_printf("lsb 0x%X = calib_dig_H3 = %d\r\n", lsb, calib_dig_H3);
	//#endif

	lsb = i2c_readByte(); i2c_send_ack(1);
	msb = i2c_readByte(); i2c_send_ack(1);
	calib_dig_H4 = (lsb << 4) | (0x0f & msb);
	//#ifdef BME280_DEBUG
	//ets_uart_printf("lsb 0x%X, msb: 0x%X = calib_dig_H4 = %d\r\n", lsb, msb, calib_dig_H4);
	//#endif

	lsb = i2c_readByte(); i2c_send_ack(1);
	calib_dig_H5 = (lsb << 4) | ((msb >> 4) & 0x0F);
	//#ifdef BME280_DEBUG
	//ets_uart_printf("lsb 0x%X, msb: 0x%X = calib_dig_H5 = %d\r\n", lsb, msb, calib_dig_H5);
	//#endif

	lsb = i2c_readByte();
	i2c_send_ack(0); // STOP
	i2c_stop();

	calib_dig_H6 = lsb;
	//#ifdef BME280_DEBUG
	//ets_uart_printf("lsb 0x%X = calib_dig_H6 = %d\r\n", lsb, calib_dig_H6);
	//#endif

}

bool ICACHE_FLASH_ATTR BME280_sendI2cTriggerForcedRead(){

    uint8_t ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | BME280_OperationMode;

    BME280_sendI2cWriteData(BME280_REG_CTRL_MEAS, ctrl_meas_reg);

    os_delay_us(10000); // wait 10ms for worst case max sensor read time

	return 1;
}
bool ICACHE_FLASH_ATTR BME280_sendI2cReadSensorData(){

    uint8 msb, lsb, xlsb;

	#ifdef BME280_DEBUG
	ets_uart_printf("operation mode = %d\r\n", BME280_OperationMode);
	#endif

    if(BME280_OperationMode == BME280_MODE_FORCED){
    	if(!BME280_sendI2cTriggerForcedRead()){
    		return 0;
    	}
    }


	if(!BME280_sendI2cRead(0xF7)){
		return 0;
	}

	//0xF7 - pressure
	msb = i2c_readByte(); i2c_send_ack(1);
	lsb = i2c_readByte(); i2c_send_ack(1);
	xlsb = i2c_readByte(); i2c_send_ack(1);
	#ifdef BME280_DEBUG
	ets_uart_printf("pres_raw 0: %X, pres_raw 1: %X, pres_raw 2: %X\r\n", msb, lsb, xlsb);
	#endif

    pres_raw = (msb << 12) | (lsb << 4) | (xlsb >> 4);

	//0xFA - temp
	msb = i2c_readByte(); i2c_send_ack(1);
	lsb = i2c_readByte(); i2c_send_ack(1);
	xlsb = i2c_readByte(); i2c_send_ack(1);
	#ifdef BME280_DEBUG
	ets_uart_printf("temp_raw 3: %X, temp_raw 4: %X, temp_raw 5: %X\r\n",msb, lsb, xlsb);
	#endif

    temp_raw = (msb << 12) | (lsb << 4) | (xlsb >> 4);

	//0xFD - humidity
	msb = i2c_readByte(); i2c_send_ack(1);
	lsb = i2c_readByte(); i2c_send_ack(1);
	#ifdef BME280_DEBUG
	ets_uart_printf("hum_raw 6: %X, hum_raw 7: %X\r\n", msb, lsb);
	#endif

    hum_raw  = (msb << 8) | lsb;

	i2c_stop();

    return 1;
}

unsigned long int ICACHE_FLASH_ATTR BME280_GetTemperatureRaw(){
	return temp_raw;
}

unsigned long int ICACHE_FLASH_ATTR BME280_GetPressureRaw(){
	return pres_raw;
}

unsigned long int ICACHE_FLASH_ATTR BME280_GetHumidityRaw(){
	return hum_raw;
}

void ICACHE_FLASH_ATTR BME280_readSensorData(){

	BME280_sendI2cReadSensorData();

	// test data:
	//temp_raw = 529184;
	//pres_raw = 282960;
	//hum_raw = 28012;

	temp_act = BME280_calibration_Temp(temp_raw);
	press_act = BME280_calibration_Press(pres_raw);
	hum_act = BME280_calibration_Hum(hum_raw);
}

signed long int ICACHE_FLASH_ATTR BME280_GetT_Fine(){
	return t_fine;
}

signed long int ICACHE_FLASH_ATTR BME280_GetTemperature(){
	return temp_act;
}

unsigned long int ICACHE_FLASH_ATTR BME280_GetPressure(){
	return press_act;
}

unsigned long int ICACHE_FLASH_ATTR BME280_GetHumidity(){
	return hum_act;
}

signed long int BME280_calibration_Temp(signed long int adc_T)
{

    signed long int var1, var2, T;
    var1 = ((((adc_T >> 3) - ((signed long int)calib_dig_T1<<1))) * ((signed long int)calib_dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((signed long int)calib_dig_T1)) * ((adc_T>>4) - ((signed long int)calib_dig_T1))) >> 12) * ((signed long int)calib_dig_T3)) >> 14;

    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T;
}

unsigned long int BME280_calibration_Press(signed long int adc_P)
{
    signed long int var1, var2;
    unsigned long int P;
    var1 = (((signed long int)t_fine)>>1) - (signed long int)64000;
    var2 = (((var1>>2) * (var1>>2)) >> 11) * ((signed long int)calib_dig_P6);
    var2 = var2 + ((var1*((signed long int)calib_dig_P5))<<1);
    var2 = (var2>>2)+(((signed long int)calib_dig_P4)<<16);
    var1 = (((calib_dig_P3 * (((var1>>2)*(var1>>2)) >> 13)) >>3) + ((((signed long int)calib_dig_P2) * var1)>>1))>>18;
    var1 = ((((32768+var1))*((signed long int)calib_dig_P1))>>15);
    if (var1 == 0){
        return 0;
    }
    P = (((unsigned long int)(((signed long int)1048576)-adc_P)-(var2>>12)))*3125;
    if(P<0x80000000){
       P = (P << 1) / ((unsigned long int) var1);
    }else{
        P = (P / (unsigned long int)var1) * 2;
    }
    var1 = (((signed long int)calib_dig_P9) * ((signed long int)(((P>>3) * (P>>3))>>13)))>>12;
    var2 = (((signed long int)(P>>2)) * ((signed long int)calib_dig_P8))>>13;
    P = (unsigned long int)((signed long int)P + ((var1 + var2 + calib_dig_P7) >> 4));
    return P;
}

unsigned long int BME280_calibration_Hum(signed long int adc_H)
{
    signed long int v_x1;

    v_x1 = (t_fine - ((signed long int)76800));
    v_x1 = (((((adc_H << 14) -(((signed long int)calib_dig_H4) << 20) - (((signed long int)calib_dig_H5) * v_x1)) +
              ((signed long int)16384)) >> 15) * (((((((v_x1 * ((signed long int)calib_dig_H6)) >> 10) *
              (((v_x1 * ((signed long int)calib_dig_H3)) >> 11) + ((signed long int) 32768))) >> 10) + (( signed long int)2097152)) *
              ((signed long int) calib_dig_H2) + 8192) >> 14));
   v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((signed long int)calib_dig_H1)) >> 4));
   v_x1 = (v_x1 < 0 ? 0 : v_x1);
   v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);
   return (unsigned long int)(v_x1 >> 12);
}
