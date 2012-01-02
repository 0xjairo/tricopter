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
