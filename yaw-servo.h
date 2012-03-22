/*
 * yaw_servo.h
 *
 *  Created on: Jan 1, 2012
 *      Author: jairo
 */

#ifndef YAW_SERVO_H_
#define YAW_SERVO_H_

class YawServo {
public:
	YawServo();
	void init(float center, float offset_min, float offset_max);
	void set_offset(float offset);
	float get_offset();
	void manual_control();
private:
	float _center;
	float _offset;
	float _offset_min;
	float _offset_max;
	int _duty;
};


// Servo constants
#define SERVO_MIN 3430
#define SERVO_MAX 6855
#define PPM_CNTS_TO_DEG 0.09
#define SERVO_ANGLE_TO_DUTY 37.44444

#define YAW_SERVO_ANGLE_MAX 90
#define YAW_SERVO_ANGLE_MIN 0


#endif /* YAW_SERVO_H_ */
