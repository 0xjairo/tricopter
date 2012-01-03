/*
 * yaw_servo.h
 *
 *  Created on: Jan 1, 2012
 *      Author: jairo
 */

#ifndef YAW_SERVO_H_
#define YAW_SERVO_H_

void yaw_servo_init();
float set_servo_angle(float angle);
void yaw_manual_control();


// Servo constants
#define SERVO_MIN 3430
#define SERVO_MAX 6855
#define PPM_CNTS_TO_DEG 0.09
#define SERVO_ANGLE_TO_DUTY 37.44444

#define YAW_SERVO_ANGLE_MAX 26
#define YAW_SERVO_ANGLE_MIN 0


#endif /* YAW_SERVO_H_ */
