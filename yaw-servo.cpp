/*
 * yaw_servo.cpp
 *
 *  Created on: Jan 1, 2012
 *      Author: jairo
 */

#include "main.h"
#include "yaw-servo.h"
#include "wirish.h"

HardwareTimer timer2(2);

YawServo::YawServo() {
	_center = 0;
	_offset = 0;
	_duty = 0;
	_offset_min = 0;
	_offset_max = 0;
}


void YawServo::init(float center, float offset_min, float offset_max) {
	timer2.setMode(TIMER_CH4, TIMER_PWM);
	timer2.setPrescaleFactor(SERVO_PPM_TIMER_PRESCALE_FACTOR);
	_center = center;
	_offset_min = offset_min;
	_offset_max = offset_max;

	set_offset(0.0);
}

void YawServo::set_offset(float offset) {
	int duty;

    // bound angle
    if(offset > _offset_max)
    	offset = _offset_max;

    if(offset < _offset_min)
    	offset = _offset_min;

    _offset = offset;
    _duty = SERVO_MIN + (int)( (_center + _offset) * SERVO_ANGLE_TO_DUTY);
    if(_duty > SERVO_MAX) _duty = SERVO_MAX;
    if(_duty < SERVO_MIN) _duty = SERVO_MIN;
    pwmWrite(YAW_SERVO_PIN, _duty);
}

float YawServo::get_offset() {
	return _offset;
}

void YawServo::manual_control()
{
	SerialUSB.println("Press \'j\' to lower speed.");
	SerialUSB.println("Press \'k\' to increase speed.");
	SerialUSB.println("Press \'z\' to zero command.");
	SerialUSB.println("Any other key zeroes command and exits");
	SerialUSB.println();

	uint8 input;
	while (1) {

		input = SerialUSB.read();
		if (input == 'j')
		{
			_offset -= 1;
		}
		else if(input == 'k')
		{
			_offset += 1;
		}
		else if(input == 'z')
		{
			_offset = 0;

		}else{
			break;
		}

		set_offset(_offset);
		SerialUSB.println(get_offset());
		delay(20);
	}

	set_offset(0.0);


}
