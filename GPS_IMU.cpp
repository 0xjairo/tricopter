#include "GPS_IMU.h"
#include "wirish.h"

byte    GPS_fix                         = BAD_GPS;              // This variable store the status of the GPS
byte    GPS_update                      = GPS_NONE;             // do we have GPS data to work with?
boolean invalid_location        = true;         // used to indicate we can't navigate witout good GPS data - the equations will choke

// used to consruct the GPS data from Bytes to ints and longs
// ----------------------------------------------------------
union long_union {
        int32 dword;
        uint8 byte[4];
} longUnion;

union int_union {
        int16 word;
        uint8 byte[2];
} intUnion;

long roll_sensor                        = 0;            // how much we're turning in degrees * 100
long pitch_sensor                       = 0;            // our angle of attack in degrees * 100
long    ground_course           = 0;                    // degrees * 100 dir of plane

// Performance Monitoring variables
// Data collected and reported for ~1 minute intervals
//int IMU_mainLoop_count = 0;				//Main loop cycles since last report
int G_Dt_max = 0.0;						//Max main loop cycle time in milliseconds
byte gyro_sat_count = 0;
byte adc_constraints = 0;
byte renorm_sqrt_count = 0;
byte renorm_blowup_count = 0;
byte gps_payload_error_count = 0;
byte gps_checksum_error_count = 0;
byte gps_pos_fix_count = 0;
byte gps_nav_fix_count = 0;
byte gps_messages_sent = 0;
byte gps_messages_received = 0;
int imu_messages_received = 0;
byte imu_payload_error_count = 0;
byte imu_checksum_error_count = 0;
long perf_mon_timer = 0;

byte IMU_buffer[24];
byte payload_length	= 0;
byte payload_counter	= 0;

//IMU Checksum
byte ck_a = 0;
byte ck_b = 0;
byte IMU_ck_a = 0;
byte IMU_ck_b = 0;

/****************************************************************
 * Here you have all the stuff for data reception from the IMU_GPS
 ****************************************************************/

/*	GPS_update bit flags -
	 - 0x01 bit = gps lat/lon data received
	 - 0x02 bit = gps alt and speed data received
	 - 0x04 bit = IMU data received
	 - 0x08 bit = PROBLEM - No IMU data last second!

#define GPS_NONE 0
#define GPS_POSITION 1
#define GPS_HEADING 2
#define GPS_BOTH 3
#define GPS_IMU 4
#define GPS_IMU_ERROR 8

*/

void init_gps(void)
{
	Serial3.begin(38400); //Universal Sincronus Asyncronus Receiveing Transmiting
	GPS_update 	= GPS_NONE;
	GPS_fix 	= BAD_GPS;
}

/*
IMU Message format
Byte(s)		 Value
0-3		 Header "DIYd"
4						Payload length	= 6
5						Message ID = 2
6,7			roll		Integer (degrees*100)
8,9			pitch		Integer (degrees*100)
10,11		yaw			Integer (degrees*100)
12,13					checksum


GPS Message format
Byte(s)		 Value
0-3		 Header "DIYd"
4								Payload length = 14
5								Message ID = 3
6-9			longitude			Integer (value*10**7)
10-13		latitude			Integer (value*10**7)
14,15		altitude			Integer (meters*10)
16,17		gps speed			Integer (M/S*100)
18,19		gps course			not used
20,21		checksum
*/

void decode_gps(void)
{
	static unsigned long IMU_timer = 0; //used to set PROBLEM flag if no data is received. 
	static unsigned long GPS_timer = 0;
	static byte IMU_step = 0;
	int numc = 0;
	byte data;
	int message_num;

	numc = Serial3.available();
	if (numc > 0)
		for (int i=0;i<numc;i++)	// Process bytes received
		{
			data = Serial3.read();
			switch(IMU_step)		 //Normally we start from zero. This is a state machine
			{
			case 0:	
				if(data == 0x44) 
					IMU_step++; //First byte of data packet header is correct, so jump to the next step
					//else
					//SerialUSB.println("IMU parser Case 0 fail");	 // This line for debugging only
				break; 

			case 1:	
				if(data == 0x49)
					 IMU_step++;	//Second byte of data packet header is correct
				else {	
					// This line for debugging only
					//SerialUSB.println("IMU parser Case 1 fail");
					IMU_step=0;		 //Second byte is not correct so restart to step zero and try again.	
				}	 
				break;

			case 2:	
				if(data == 0x59) 
					 IMU_step++;	//Third byte of data packet header is correct
				else {
					//SerialUSB.println("IMU parser Case 2 fail");	 // This line for debugging only
					IMU_step=0;		 //Third byte is not correct so restart to step zero and try again.
				}		 
				break;

			case 3:	
				if(data == 0x64)	 
					 IMU_step++;	//Fourth byte of data packet header is correct, Header complete
				else {
					//SerialUSB.println("IMU parser Case 3 fail");	 // This line for debugging only
					IMU_step=0;		 //Fourth byte is not correct so restart to step zero and try again.
				}		 
				break;

			case 4:	
				payload_length = data;
				checksum(payload_length);
				IMU_step++;		
				if (payload_length>22)
				{
					IMU_step=0;	 //Bad data, so restart to step zero and try again.		 
					payload_counter=0;
					ck_a=0;
					ck_b=0;
					imu_payload_error_count++;
				} 
				break;

			case 5:	
					message_num = data;
					checksum(data);
					IMU_step++;		 
				break;
		 
			case 6:				 // Payload data read...
				// We stay in this state until we reach the payload_length
				IMU_buffer[payload_counter] = data;
				checksum(data);
				payload_counter++;
				if (payload_counter >= payload_length) { 
					IMU_step++; 
				}
				break;
			case 7:
				IMU_ck_a=data;	 // First checksum byte
				IMU_step++;
				break;
			case 8:
				IMU_ck_b=data;	 // Second checksum byte

				// We end the IMU/GPS read...
				// Verify the received checksum with the generated checksum.. 
				if((ck_a == IMU_ck_a) && (ck_b == IMU_ck_b)) {
					if (message_num == 0x02) {
						IMU_join_data();
//					} else if (message_num == 0x04) {
//						GPS_join_data1();
//						GPS_timer = DIYmillis();
//					} else if (message_num == 0x05) {
//						GPS_join_data2();
//					} else if (message_num == 0x03) {
//						GPS_join_data();
//						GPS_timer = DIYmillis();
//					} else if (message_num == 0x0a) {
//						PERF_join_data();
					} else {
						SerialUSB.print("Invalid message number = ");
						SerialUSB.println(message_num,DEC);
					}
				} else {
					//SerialUSB.println("Checksum error");	//bad checksum
					imu_checksum_error_count++;
				} 						 
				// Variable initialization
				IMU_step = 0;
				payload_counter = 0;
				ck_a = 0;
				ck_b = 0;
				IMU_timer = millis(); //Restarting timer...
				break;
						}
					}// End for...
	
	if(millis() - IMU_timer > 500){
		digitalWrite(12, LOW);	//If we don't receive any byte in a half second turn off gps fix LED...
		GPS_update = GPS_IMU_ERROR;
	}

	if((millis() - GPS_timer) > 2000){
		digitalWrite(12, LOW);	//If we don't receive any byte in two seconds turn off gps fix LED... 
		if(GPS_fix != FAILED_GPS){
			GPS_fix = BAD_GPS;
		}
		GPS_update = GPS_NONE;

		if((millis() - GPS_timer) > 10000){
			invalid_location = true;
			GPS_fix = FAILED_GPS;
			//SerialUSB.println("XXX \t No GPS, last 10s \t ***");
		}
	}
}
	
 /****************************************************************
 * 
 ****************************************************************/
void IMU_join_data()
{
	imu_messages_received++;
	int j=0;

	 //Storing IMU roll
	intUnion.byte[0] = IMU_buffer[j++];
	intUnion.byte[1] = IMU_buffer[j++];
	roll_sensor = intUnion.word;

	 //Storing IMU pitch
	intUnion.byte[0] = IMU_buffer[j++];
	intUnion.byte[1] = IMU_buffer[j++];
	pitch_sensor = intUnion.word;

	 //Storing IMU heading (yaw)
	intUnion.byte[0] = IMU_buffer[j++];
	intUnion.byte[1] = IMU_buffer[j++];
	ground_course = intUnion.word;

	GPS_update |= GPS_IMU;
}


void checksum(byte data)
{
	ck_a+=data;
	ck_b+=ck_a; 
}

void wait_for_data(byte many)
{
	while(Serial3.available() <= many);
}

 // Join 4 bytes into a long
 // -------------------------
int32 join_4_bytes(byte Buffer[])
{
	longUnion.byte[0] = *Buffer;
	longUnion.byte[1] = *(Buffer+1);
	longUnion.byte[2] = *(Buffer+2);
	longUnion.byte[3] = *(Buffer+3);
	return(longUnion.dword);
}




