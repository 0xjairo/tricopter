/*
 * AHRS.h
 *
 *  Created on: Mar 3, 2012
 *      Author: jairo
 */

#ifndef AHRS_H_
#define AHRS_H_

#include "wirish.h"

class AHRS {
public:
	AHRS();

	void init();
	void update();
	byte get_status();
	float get_roll();
	float get_pitch();
	float get_yaw();

private:
	byte _status;
	float _roll;
	float _pitch;
	float _yaw;
};

#endif /* AHRS_H_ */
