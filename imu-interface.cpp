/*
 * imu-interface.cpp
 *
 *  Created on: Dec 30, 2011
 *      Author: jairo
 */
#include "wirish.h"
#include "imu-interface.h"
#include "GPS_IMU.h"

void imu_update()
{
	decode_gps(); // decode IMU data
}

void imu_interface_init()
{
	// init imu rx
	init_gps();

}

uint8 wrap(uint8 i, uint8 n)
{
	if(n<i)
		return wrap(i-n, n);
	return i;
}

void imu_print_data()
{
	while(!SerialUSB.available())
	{
		decode_gps();
		print_imu_data();
		delay(100);
	}
//	int i=0,j=0;
//	uint8 imu_data[14];
//	int idx_msg_start;
//	int val;
//	while(!SerialUSB.available())
//	{
//
//		// blocks until there's data
//		imu_data[i] = Serial3.read();
//
//		if(i==13){
//
//			//look for message preamble: DIYd
//		    idx_msg_start=-1;
//			for(j=0; j<14;j++)
//			{
//				if(imu_data[wrap(j,14)] == 'D')
//					if(imu_data[wrap(j+1,14)] == 'I')
//						if(imu_data[wrap(j+2,14)] == 'Y')
//							if(imu_data[wrap(j+3,14)] == 'd')
//							{
//								idx_msg_start=j;
//								break;
//							}
//
//			}
//			if(idx_msg_start>=0)
//			{
//				for(j=0; j<14; j+=2)
//				{
//					if((j>3 && j<6) || j>11){
//						SerialUSB.print(imu_data[wrap(idx_msg_start+j,14)]);
//						SerialUSB.print("\t");
//						SerialUSB.print(imu_data[wrap(idx_msg_start+j+1,14)]);
//						SerialUSB.print("\t");
//					}else if(j>5){
//						val = (int16)(imu_data[ wrap(idx_msg_start+j+1,14)] << 8) | (imu_data[wrap(idx_msg_start+j,14)]);
//						SerialUSB.print("v:");
//						SerialUSB.print(val);
//						SerialUSB.print("\t");
//					}
//				}
//				SerialUSB.println("");
//
//			}
//
//			i=0;
//		}else{
//			i++;
//		}
//
//	}
}
