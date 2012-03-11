// Include files
#include "main.h"
#include "wirish.h"
#include "utils.h"
#include "esc-control.h"
#include "ppm-decode.h"
#include "yaw-servo.h"
#include "GPS_IMU.h"

#include "RC.h"
#include "AHRS.h"

// ASCII escape character
#define ESC       ((uint8)27)

extern float rx_channels[TX_NUM_CHANNELS];
extern uint16 ppm_timeout;
extern int sync_pulse_confidence;

// -- setup() and loop() ------------------------------------------------------


void setup() {

    // Set up the LED to blink
    pinMode(BOARD_LED_PIN, OUTPUT);
	// Set up ESC pins
	pinMode(ROTOR1_PIN, PWM);
	pinMode(ROTOR2_PIN, PWM);
	pinMode(ROTOR3_PIN, PWM);
	// Yaw servo pin (8)
	pinMode(YAW_SERVO_PIN, PWM);
	// Set up PPM pin
	pinMode(PPM_PIN, INPUT_PULLUP);

    // init motor controllers
    esc_init();

    // yaw servo init
    yaw_servo_init();

    // initialize usb
    SerialUSB.begin();
//    while(!isConnected()); //wait till console attaches.
//    SerialUSB.println("Welcome user!");
}


// Force init to be called *first*, i.e. before static object allocation.
// Otherwise, statically allocated objects that need libmaple may fail.
__attribute__((constructor)) void premain() {
    init();
}

int main(void) {
	uint32 t;
	uint32 t_prev, dt;
	uint32 cpu_util, cpu_util2;
	uint32 tick50hz=0;
	float errorRoll, uRoll, last_errorRoll;
	float KpRoll, KiRoll, KdRoll;
	float pRoll, iRoll, dRoll;
	float m1, m2, m3;
	float derivative, last_derivative;

	/// Low pass filter cut frequency for derivative calculation.
	///
	static const float filter = 7.9577e-3; // Set to  "1 / ( 2 * PI * f_cut )";
	// Examples for _filter:
	// f_cut = 10 Hz -> _filter = 15.9155e-3
	// f_cut = 15 Hz -> _filter = 10.6103e-3
	// f_cut = 20 Hz -> _filter =  7.9577e-3
	// f_cut = 25 Hz -> _filter =  6.3662e-3
	// f_cut = 30 Hz -> _filter =  5.3052e-3

	RC rc;
	AHRS ahrs;

    // init
    setup();
    ahrs.init();
    rc.init();

    toggleLED();

//    while(rc.status() != SUCCESS || rc.get_channel(CH_THROTTLE) > 0.2) {
//    	rc.update();
//    	delay_us(1000);
//    	toggleLED();
//    }

    t_prev = 0;

    iRoll=0.0;
    last_errorRoll = 0;
    last_derivative = 0;
    uRoll = 0;
    while (1) {

    	t = micros();
    	dt = t - t_prev;

    	// main loop (50 Hz)
    	if(dt > 20000) {
    		t_prev = t;

    		rc.update();  // update RC commands
    		ahrs.update();

    		KpRoll = 0.0005;//rc.get_channel(CH_AUX1)*0.001;
    		KiRoll = 0;//rc.get_channel(CH_AUX2)*0.001;
    		KdRoll = rc.get_channel(CH_AUX3)*-0.00005;

//    		uRoll = 50*2*(rc.get_channel(CH_ROLL)-0.5);
    		uRoll += 2*(rc.get_channel(CH_ROLL)-0.5);
    		errorRoll = uRoll - ahrs.get_roll();

    		if ((KdRoll != 0) && (dt != 0)) {
    			derivative = (errorRoll - last_errorRoll)*1000000/(float)dt;

    			// discrete low pass filter, cuts out the
    			// high frequency noise that can drive the controller crazy
    			derivative = last_derivative +
    			        (dt / ( filter + dt)) * (derivative - last_derivative);

    			// update state
    			last_errorRoll 	= errorRoll;
    			last_derivative = derivative;

    			// add in derivative component
    		}

    		pRoll  =  errorRoll * KpRoll;
    		iRoll += (errorRoll * KiRoll* dt)/1000000;
    		dRoll  = KdRoll * derivative;

			#define MAX_I_ROLL 0.010
    		if(iRoll >  MAX_I_ROLL) iRoll =  MAX_I_ROLL;
    		if(iRoll < -MAX_I_ROLL) iRoll = -MAX_I_ROLL;

			#define MIN_THROTTLE 0.12
    		if(rc.get_channel(CH_THROTTLE) < MIN_THROTTLE && rc.get_channel(CH_AUX4) > 0.5)
    		{
    			uRoll = 0.0; // reset desired pitch when throttle falls
    			iRoll = 0;
    		}

			m1 = rc.get_channel(CH_THROTTLE) +      (pRoll + iRoll + dRoll);
			m2 = rc.get_channel(CH_THROTTLE) + (-1)*(pRoll + iRoll + dRoll);
			m3 = 0.0;

			if(m1 > 0.7) m1 = 0.7;
			if(m2 > 0.7) m2 = 0.7;

    		if(rc.get_channel(CH_AUX4) > 0.5 && rc.status() == SUCCESS)
			{
				set_rotor_throttle(1, m1);
				set_rotor_throttle(2, m2);
				set_rotor_throttle(3, m3);
			} else {
				set_rotor_throttle(1,0);
				set_rotor_throttle(2,0);
				set_rotor_throttle(3,0);
			}

			// 10Hz loop
			if(tick50hz %  5 == 0)
			{
				toggleLED();
			}

			// 2Hz loop
//			if(tick50hz % 25 == 0)
			if(tick50hz % 5 == 0) // 10 hz
			{
				// cpu utilization based on the 2000 microseconds (50 Hz) loop
				cpu_util = (micros()-t)*100/dt;

				/*
				 * Safe Print
				 * ----------
				 *
				 * <http://leaflabs.com/docs/lang/api/serialusb.html>
			     * If this logic fails to detect if bytes are going to be read
			     * by the USB host, then the println() take a long time,
			     * causing a very slow LED blink.  If the characters are
			     * printed and read, the blink will only slow a small amount
			     * when "really" connected, and will be fast fast when the
			     * virtual port is only configured.
			     *
			     */
				if( isConnected())
				{
					printkv("rc:", ! rc.status());
					printkv("imu:", ahrs.get_status());
//					printkv("aux1:", rc.get_channel(CH_AUX1));
					printkv("p:", (float)(pRoll*100.0));
					printkv("i:", (float)(iRoll*1000.0));
					printkv("m1:", m1);
					printkv("m2:", m2);
					printkv("kp:", KpRoll*1000);
					printkv("ki:", KiRoll*1000);
					printkv("kd:", KdRoll*100000);

					printkv("thr:", rc.get_channel( CH_THROTTLE));
					printkv("u:", uRoll);
					printkv("y:", ahrs.get_roll());
					printkv("e:", errorRoll);
//					printkv("pitch:", ahrs.get_pitch());
//					printkv("yaw:", ahrs.get_yaw());

					// cpu utilization after printing data
//					printkv("  util:", cpu_util);
//					cpu_util2 = (micros()-t)*100/t_delta;
//					printkv("util2:",cpu_util2);

					SerialUSB.println("");
				}

			}

			tick50hz++;
    	}
    }
    return 0;
}
