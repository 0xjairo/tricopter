
// Include files
#include "main.h"
#include "wirish.h"
#include "utils.h"
#include "esc-control.h"
#include "ppm-decode.h"
#include "imu-interface.h"
#include "yaw-servo.h"

#include "RC.h"


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

	// imu interface init
	imu_interface_init();

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
	int fault=0;
	uint32 t, t_prev, t_delta;
	uint32 tick50hz=0;
	RC rc;

    // init
    setup();
    rc.init();

    toggleLED();

    t_prev = 0;
    while (1) {

    	t = micros();
    	t_delta = t - t_prev;

    	// main loop (50 Hz)
    	if(t_delta > 20000) {
    		t_prev = t;

    		rc.update();

    		set_rotor_throttle(1, rc.get_channel( CH_THROTTLE ));
			set_rotor_throttle(2, rc.get_channel( CH_THROTTLE ));
			set_rotor_throttle(3, rc.get_channel( CH_THROTTLE ));

			// 10Hz loop
			if(tick50hz %  10 == 0)
			{
				toggleLED();
			}

			// 1Hz loop
			if(tick50hz % 50 == 0)
			{

				SerialUSB.print("throttle:");
				SerialUSB.println(rc.get_channel( CH_THROTTLE ));

				SerialUSB.println("");

			}

			tick50hz++;
    	}
    }
    return 0;
}
