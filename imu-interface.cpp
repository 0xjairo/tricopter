/*
 * imu-interface.cpp
 *
 *  Created on: Dec 30, 2011
 *      Author: jairo
 */
#include "wirish.h"
#include "imu-interface.h"


void imu_print_data()
{
	while(!SerialUSB.available())
	{
		SerialUSB.print(Serial3.read());
		SerialUSB.print(":");
		delay(20);

	}
}
