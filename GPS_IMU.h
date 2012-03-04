/*
 * GPS_IMU.h
 *
 *  Created on: Jan 3, 2012
 *      Author: jairo
 */

#ifndef GPS_IMU_H_
#define GPS_IMU_H_

#include "wirish.h"

void init_gps(void);
void decode_gps(void);
void IMU_join_data();
void checksum(byte data);
void wait_for_data(byte many);
void print_imu_data();
void get_imu_data(float *roll, float *pitch, float *yaw, byte *status);

// From defines.h in ardupilot 2.5
// GPS flags
// ---------
//GPS_update
#define GPS_NONE 0
#define GPS_POSITION 1
#define GPS_HEADING 2
#define GPS_BOTH 3
#define GPS_IMU 4
#define GPS_IMU_ERROR 8

//GPS_fix
#define VALID_GPS 0x00
#define BAD_GPS 0x01
#define FAILED_GPS 0x03


#endif /* GPS_IMU_H_ */
