/*
 * GPS_IMU.h
 *
 *  Created on: Jan 3, 2012
 *      Author: jairo
 */

#ifndef GPS_IMU_H_
#define GPS_IMU_H_

void init_gps(void);
void decode_gps(void);
void IMU_join_data();
void checksum(byte data);
void wait_for_data(byte many);



#endif /* GPS_IMU_H_ */
