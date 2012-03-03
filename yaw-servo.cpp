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

void yaw_servo_init()
{
	timer2.setMode(TIMER_CH4, TIMER_PWM);
	timer2.setPrescaleFactor(SERVO_PPM_TIMER_PRESCALE_FACTOR);

	set_servo_angle(0.0);

}


float set_servo_angle(float angle)
{
    int duty;

    // bound angle
    if(angle > YAW_SERVO_ANGLE_MAX)
    	angle = YAW_SERVO_ANGLE_MAX;

    if(angle < YAW_SERVO_ANGLE_MIN)
    	angle = YAW_SERVO_ANGLE_MIN;

    duty = SERVO_MIN + (int)(angle * SERVO_ANGLE_TO_DUTY);
    if(duty > SERVO_MAX) duty = SERVO_MAX;
    pwmWrite(YAW_SERVO_PIN, duty);

    return angle;
}

void yaw_manual_control()
{
	SerialUSB.println("Press \'j\' to lower speed.");
	SerialUSB.println("Press \'k\' to increase speed.");
	SerialUSB.println("Press \'z\' to zero command.");
	SerialUSB.println("Any other key zeroes command and exits");
	SerialUSB.println();

	uint8 input;
	float angle = 0;
	while (1) {

		input = SerialUSB.read();
		if (input == 'j')
		{
			angle -= 1;
		}
		else if(input == 'k')
		{
			angle += 1;
		}
		else if(input == 'z')
		{
			angle = 0;

		}else{
			break;
		}


		SerialUSB.println(set_servo_angle(angle));
		delay(20);
	}

	set_servo_angle(0.0);


}
