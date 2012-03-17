/*
 * MyPID.h
 *
 *  Created on: Mar 17, 2012
 *      Author: jairo
 */

#ifndef MYPID_H_
#define MYPID_H_

#include "wirish.h"

class MyPID {

public:
	MyPID();
	MyPID(float kp, float ki, float kd);
	void set_gain(char k, float val);
	void set_gains(float p, float i, float d);
	float get_gain(char k);
	float get_term(char k);
	float get_error();
	float go(float desired, float actual, uint32 dt);

private:
	float _error;
	float _p, _i, _d;
	float _kp, _ki, _kd;

};

#endif /* MYPID_H_ */
