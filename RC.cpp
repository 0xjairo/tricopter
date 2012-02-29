/*
 * RC.cpp
 *
 *  Created on: Feb 25, 2012
 *      Author: jairo
 */

#include "RC.h"

RC::RC() {
	for(int i=0;i<9;i++) _ppm_sum[i]=0;

	_sp = -1;
	_sync_error = ERROR_SYNC;

	// ppm decode setup
	init_ppm_timer_and_dma();
	ppm_decode_go();

}

void RC::store_ppm_sum() {

}

void RC::update() {

	// read ppm sum data
	rx_read(&_sp, &_rcCmd);

	if(_sp == SP_INVALID)
		return;

	_yaw = _rcCmd.yaw;
	_pitch = _rcCmd.pitch;
	_roll = _rcCmd.roll;
	_throttle = _rcCmd.throttle;

}

uint16 RC::status() {

	if(_sp == SP_INVALID)
		return ERROR_SYNC;
	else
		return SUCCESS;

}

float RC::get_channel(int channel) {
	if(_sp == SP_INVALID)
		return CH_INVALID;

	switch ( channel ) {

	case CH_ROLL:
		return _roll;

	case CH_PITCH:
		return _pitch;

	case CH_THROTTLE:
		return _throttle;

	case CH_YAW:
		return _yaw;

	}
}


int RC::get_sync_pulse()
{
	return _sp;
}
