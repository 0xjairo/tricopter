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
	get_imu_data(&temp_roll, &temp_pitch, &temp_yaw, &_status, &_payload_error_count, &_checksum_error_count, &_state_error_count);
	if(_status == GPS_IMU)
	{
		_roll = temp_roll;
		_pitch = -1*temp_pitch;
		_yaw = temp_yaw;
//	} else {
//		SerialUSB.println("ERROR: IMU");
	}

//	if(_payload_error_count != 0 || _checksum_error_count != 0 || _state_error_count != 0) {
//		SerialUSB.print("payload:");
//		SerialUSB.print(_payload_error_count);
//		SerialUSB.print(" checksum:");
//		SerialUSB.print(_checksum_error_count);
//		SerialUSB.print(" state:");
//		SerialUSB.println(_state_error_count);
//	}
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

