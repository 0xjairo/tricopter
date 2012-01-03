/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
	APM_BMP085.cpp - Arduino Library for BMP085 absolute pressure sensor
	Code by Jordi Mu�oz and Jose Julio. DIYDrones.com

	This library is free software; you can redistribute it and/or
    modify it under the terms of the GNU Lesser General Public
    License as published by the Free Software Foundation; either
    version 2.1 of the License, or (at your option) any later version.

	Sensor is conected to I2C port
	Sensor End of Conversion (EOC) pin is PC7 (30)

	Variables:
		RawTemp : Raw temperature data
		RawPress : Raw pressure data

		Temp : Calculated temperature (in 0.1�C units)
		Press : Calculated pressure   (in Pa units)

	Methods:
		Init() : Initialization of I2C and read sensor calibration data
		Read() : Read sensor data and calculate Temperature and Pressure
		         This function is optimized so the main host don�t need to wait
				 You can call this function in your main loop
				 It returns a 1 if there are new data.

	Internal functions:
		Command_ReadTemp(): Send commando to read temperature
		Command_ReadPress(): Send commando to read Pressure
		ReadTemp() : Read temp register
		ReadPress() : Read press register
		Calculate() : Calculate Temperature and Pressure in real units


*/

extern "C" {
  // AVR LibC Includes
  #include <inttypes.h>
  #include <avr/interrupt.h>
  #include "WConstants.h"
}

#include <AP_Common.h>
#include <AP_Math.h>		// ArduPilot Mega Vector/Matrix math Library
#include <I2C.h>
#include "AP_Baro_BMP085.h"

#define BMP085_ADDRESS 0x77  //(0xEE >> 1)
#define BMP085_EOC 30        // End of conversion pin PC7

// the apm2 hardware needs to check the state of the
// chip using a direct IO port
// On APM2 prerelease hw, the data ready port is hooked up to PE7, which
// is not available to the arduino digitalRead function.
#define BMP_DATA_READY() (_apm2_hardware?(PINE&0x80):digitalRead(BMP085_EOC))


// Public Methods //////////////////////////////////////////////////////////////
bool AP_Baro_BMP085::init( AP_PeriodicProcess * scheduler )
{
	byte buff[22];
	
	pinMode(BMP085_EOC, INPUT);	 // End Of Conversion (PC7) input

	oss = 3;					 // Over Sampling setting 3 = High resolution
	BMP085_State = 0;		 // Initial state

	// We read the calibration data registers
	if (I2c.read(BMP085_ADDRESS, 0xAA, 22, buff) != 0) {
		healthy = false;
		return false;
	}

	ac1 = ((int)buff[0] << 8) | buff[1];
	ac2 = ((int)buff[2] << 8) | buff[3];
	ac3 = ((int)buff[4] << 8) | buff[5];
	ac4 = ((int)buff[6] << 8) | buff[7];
	ac5 = ((int)buff[8] << 8) | buff[9];
	ac6 = ((int)buff[10] << 8) | buff[11];
	b1 = ((int)buff[12] << 8) | buff[13];
	b2 = ((int)buff[14] << 8) | buff[15];
	mb = ((int)buff[16] << 8) | buff[17];
	mc = ((int)buff[18] << 8) | buff[19];
	md = ((int)buff[20] << 8) | buff[21];

	//Send a command to read Temp
	Command_ReadTemp();
	BMP085_State = 1;

	healthy = true;
	return true;
}

// Read the sensor. This is a state machine
// We read one time Temperature (state=1) and then 4 times Pressure (states 2-5)
uint8_t AP_Baro_BMP085::read()
{
	uint8_t result = 0;

	if (BMP085_State == 1){
		if (BMP_DATA_READY()){
			BMP085_State = 2;
			ReadTemp();						 // On state 1 we read temp
			Command_ReadPress();
		}
	}else{
		if (BMP_DATA_READY()){
			BMP085_State = 1;			// Start again from state = 1
			ReadPress();
			Calculate();
			Command_ReadTemp();			// Read Temp
			result = 1;					// New pressure reading
		}
	}
	return(result);
}

int32_t AP_Baro_BMP085::get_pressure() {
    return Press;
}

int16_t AP_Baro_BMP085::get_temperature() {
    return Temp;
}

float AP_Baro_BMP085::get_altitude() {
    return 0.0; // TODO
}

int32_t AP_Baro_BMP085::get_raw_pressure() {
    return RawPress;
}

int32_t AP_Baro_BMP085::get_raw_temp() {
    return RawTemp;
}

// Private functions: /////////////////////////////////////////////////////////

// Send command to Read Pressure
void AP_Baro_BMP085::Command_ReadPress()
{
	if (I2c.write(BMP085_ADDRESS, 0xF4, 0x34+(oss << 6)) != 0) {
		healthy = false;
	}
}

// Read Raw Pressure values
void AP_Baro_BMP085::ReadPress()
{
	uint8_t buf[3];

	if (I2c.read(BMP085_ADDRESS, 0xF6, 3, buf) != 0) {
		healthy = false;
		return;
	}

	RawPress = (((long)buf[0] << 16) | ((long)buf[1] << 8) | ((long)buf[2])) >> (8 - oss);

	if(_offset_press == 0){
		_offset_press = RawPress;
		RawPress = 0;
	} else{
		RawPress -= _offset_press;
	}
	// filter
	_press_filter[_press_index++] = RawPress;

	if(_press_index >= PRESS_FILTER_SIZE)
		_press_index = 0;

	RawPress = 0;

	// sum our filter
	for (uint8_t i = 0; i < PRESS_FILTER_SIZE; i++){
		RawPress += _press_filter[i];
	}

	// grab result
	RawPress /= PRESS_FILTER_SIZE;
	//RawPress >>= 3;
	RawPress += _offset_press;
}

// Send Command to Read Temperature
void AP_Baro_BMP085::Command_ReadTemp()
{
	if (I2c.write(BMP085_ADDRESS, 0xF4, 0x2E) != 0) {
		healthy = false;
	}
}

// Read Raw Temperature values
void AP_Baro_BMP085::ReadTemp()
{
	uint8_t buf[2];

	if (I2c.read(BMP085_ADDRESS, 0xF6, 2, buf) != 0) {
		healthy = false;
		return;
	}
	RawTemp = buf[0];
	RawTemp = (RawTemp << 8) | buf[1];

	if (_offset_temp == 0){
		_offset_temp = RawTemp;
		RawTemp = 0;
	} else {
		RawTemp -= _offset_temp;
	}

	// filter
	_temp_filter[_temp_index++] = RawTemp;

	if(_temp_index >= TEMP_FILTER_SIZE)
		_temp_index = 0;

	RawTemp = 0;
	// sum our filter
	for(uint8_t i = 0; i < TEMP_FILTER_SIZE; i++){
		RawTemp += _temp_filter[i];
	}

	// grab result
	RawTemp /= TEMP_FILTER_SIZE;
	//RawTemp >>= 4;
	RawTemp += _offset_temp;
}

// Calculate Temperature and Pressure in real units.
void AP_Baro_BMP085::Calculate()
{
	long x1, x2, x3, b3, b5, b6, p;
	unsigned long b4, b7;
	int32_t tmp;

	// See Datasheet page 13 for this formulas
	// Based also on Jee Labs BMP085 example code. Thanks for share.
	// Temperature calculations
	x1 = ((long)RawTemp - ac6) * ac5 >> 15;
	x2 = ((long) mc << 11) / (x1 + md);
	b5 = x1 + x2;
	Temp = (b5 + 8) >> 4;

	// Pressure calculations
	b6 = b5 - 4000;
	x1 = (b2 * (b6 * b6 >> 12)) >> 11;
	x2 = ac2 * b6 >> 11;
	x3 = x1 + x2;
	//b3 = (((int32_t) ac1 * 4 + x3)<<oss + 2) >> 2; // BAD
	//b3 = ((int32_t) ac1 * 4 + x3 + 2) >> 2;  //OK for oss=0
	tmp = ac1;
	tmp = (tmp*4 + x3)<<oss;
	b3 = (tmp+2)/4;
	x1 = ac3 * b6 >> 13;
	x2 = (b1 * (b6 * b6 >> 12)) >> 16;
	x3 = ((x1 + x2) + 2) >> 2;
	b4 = (ac4 * (uint32_t) (x3 + 32768)) >> 15;
	b7 = ((uint32_t) RawPress - b3) * (50000 >> oss);
	p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;

	x1 = (p >> 8) * (p >> 8);
	x1 = (x1 * 3038) >> 16;
	x2 = (-7357 * p) >> 16;
	Press = p + ((x1 + x2 + 3791) >> 4);
}
