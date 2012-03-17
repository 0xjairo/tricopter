/*
 * MyPID.cpp
 *
 *  Created on: Mar 17, 2012
 *      Author: jairo
 */

#include "wirish.h"
#include "MyPID.h"

MyPID::MyPID() {
	// TODO Auto-generated constructor stub
	 _p = 0; _i = 0;  _d = 0;
	_kp = 0; _ki = 0; _kd = 0;
	 }

MyPID::MyPID(float kp, float ki, float kd) {
	// TODO Auto-generated constructor stub
	_kp = kp; _ki = ki; _kd = kd;

}

void MyPID::set_gain(char k, float val) {

	switch(k) {

	case 'p':
	case 'P':
		_kp = val;
		break;

	case 'i':
	case 'I':
		_ki = val;
		break;

	case 'd':
	case 'D':
		_kd = val;
		break;
	}
}

void MyPID::set_gains(float p, float i, float d) {
	_kp = p; _ki = i; _kd = d;
}

float MyPID::get_gain(char k) {

	switch(k) {

		case 'p':
		case 'P':
			return _kp;
		case 'i':
		case 'I':
			return _ki;
		case 'd':
		case 'D':
			return _kd;
		default:
			return 0;
		}
}

float MyPID::get_term(char k) {

	switch(k) {

		case 'p':
		case 'P':
			return _p;
		case 'i':
		case 'I':
			return _i;
		case 'd':
		case 'D':
			return _d;
		default:
			return 0;
		}
}

float MyPID::get_error() {
	return _error;
}

float MyPID::go(float desired, float actual, uint32 dt) {

	float last_error;
	float last_derivative;

	/// Low pass filter cut frequency for derivative calculation.
	///
	static const float filter = 7.9577e-3; // Set to  "1 / ( 2 * PI * f_cut )";
	// Examples for _filter:
	// f_cut = 10 Hz -> _filter = 15.9155e-3
	// f_cut = 15 Hz -> _filter = 10.6103e-3
	// f_cut = 20 Hz -> _filter =  7.9577e-3
	// f_cut = 25 Hz -> _filter =  6.3662e-3
	// f_cut = 30 Hz -> _filter =  5.3052e-3

	last_error = _error;
	last_derivative = _derivative;
	_error = desired - actual;

	if ((_kd != 0) && (dt != 0)) {
		_derivative = (_error - last_error)*1000000/(float)dt;

		// discrete low pass filter, cuts out the
		// high frequency noise that can drive the controller crazy
		_derivative = last_derivative +
		        (dt / ( filter + dt)) * (_derivative - last_derivative);

		}

	_p  =      _error * _kp;
	_i += (    _error * _ki* dt)/1000000;
	_d  = _derivative * _kd;

	#define MAX_I 0.010
	if(_i >  MAX_I) _i =  MAX_I;
	if(_i < -MAX_I) _i = -MAX_I;

	return _p + _i + _d;


}
