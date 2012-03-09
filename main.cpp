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
	uint32 t_prev, t_delta;
	uint32 cpu_util, cpu_util2;
	uint32 tick50hz=0;
	float errorRoll, desiredRoll, KpRoll;
	float m1, m2, m3;

	RC rc;
	AHRS ahrs;

    // init
    setup();
    ahrs.init();
    rc.init();

    toggleLED();

    t_prev = 0;
    while (1) {

    	t = micros();
    	t_delta = t - t_prev;

    	// main loop (50 Hz)
    	if(t_delta > 20000) {
    		t_prev = t;

    		rc.update();  // update RC commands
    		ahrs.update();

    		desiredRoll = 50*2*(rc.get_channel( CH_ROLL )-0.5);
    		errorRoll = desiredRoll - ahrs.get_roll();
    		KpRoll = rc.get_channel( CH_AUX1 )*-0.1;

    		if(rc.get_channel(CH_AUX4) < 0.5 )
    		{
				m1 = rc.get_channel( CH_THROTTLE ) + (-1)*KpRoll*errorRoll;
				m2 = rc.get_channel( CH_THROTTLE ) + (+1)*KpRoll*errorRoll;
				m3 = 0.0;
    		} else {
    			m1 = 0.0;
    			m2 = 0.0;
    			m3 = 0.0;
    		}

    		set_rotor_throttle( 1, m1 );
			set_rotor_throttle( 2, m2 );
			set_rotor_throttle( 3, m2 );

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
				cpu_util = (micros()-t)*100/t_delta;

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
					printkv("aux1:", rc.get_channel(CH_AUX1));
					printkv("kp:", KpRoll);
					printkv("dRoll:", desiredRoll);
					printkv("m1:", m1);
					printkv("m2:", m2);

					printkv("thr:", rc.get_channel( CH_THROTTLE));
					printkv("roll:", ahrs.get_roll());
					printkv("pitch:", ahrs.get_pitch());
					printkv("yaw:", ahrs.get_yaw());

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
