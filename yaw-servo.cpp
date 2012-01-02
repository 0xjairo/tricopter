/*
 * yaw_servo.cpp
 *
 *  Created on: Jan 1, 2012
 *      Author: jairo
 */

#include "main.h"
#include "yaw-servo.h"
#include "wirish.h"

void yaw_servo_init()
{
	set_servo_angle(0.0);

}


void set_servo_angle(float angle)
{
    int duty;

    duty = SERVO_MIN + (int)(angle * SERVO_ANGLE_TO_DUTY);
    if(duty > SERVO_MAX) duty = SERVO_MAX;
    pwmWrite(YAW_SERVO_PIN, duty);
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


		if (angle > 90)  angle = 90;

		SerialUSB.println(angle);

		set_servo_angle(angle);
		delay(20);
	}

	set_servo_angle(0.0);


}
