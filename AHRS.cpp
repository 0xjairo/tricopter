/*
 * AHRS.cpp
 *
 *  Created on: Mar 3, 2012
 *      Author: jairo
 */

#include "AHRS.h"
#include "GPS_IMU.h"

AHRS::AHRS() {
	// TODO Auto-generated constructor stub
	_status = GPS_NONE;
	_roll = 0.0;
	_pitch = 0.0;
	_yaw = 0.0;

}

void AHRS::init() {
	init_gps(); // imu interface init
}

void AHRS::update() {
	float temp_roll, temp_pitch, temp_yaw;

	decode_gps(); // decode IMU data
	get_imu_data(&temp_roll, &temp_pitch, &temp_yaw, &_status);
	if(_status == GPS_IMU)
	{
		_roll = temp_roll;
		_pitch = temp_pitch;
		_yaw = temp_yaw;
	}

}

byte AHRS::get_status() {
	return _status;
}

float AHRS::get_pitch() {
	return _pitch;
}

float AHRS::get_roll() {
	return _roll;
}

float AHRS::get_yaw() {
	return _yaw;
}

