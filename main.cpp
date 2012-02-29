
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
	uint32 t_1Hz;
	RC rc;

    // init
    setup();

    toggleLED();
    delay(2000);
    SerialUSB.println("Starting....");

    t_prev = 0;
    t_1Hz = 0;
    while (1) {

    	t = micros();
    	t_delta = t - t_prev;

    	// main loop (50 Hz)
    	if(t_delta > 20000) {
    		t_prev = t;


	//    	rx_fault = rx_read_commands();
	//    	fault = rx_fault | ppm_timeout |(sync_pulse_confidence>3);

			set_rotor_throttle(1, rc.get_channel( CH_THROTTLE ));
			set_rotor_throttle(2, rc.get_channel( CH_THROTTLE ));
			set_rotor_throttle(3, rc.get_channel( CH_THROTTLE ));

			// 1Hz loop
			if(t_1Hz == 50)
			{
				rc.update();

				toggleLED();
				printData();

				t_1Hz = 0;

			}
			t_1Hz++;
    	}

//        while (SerialUSB.available()) {
//            uint8 input = SerialUSB.read();
//            SerialUSB.println((char)input);
//
//            switch(input) {
//            case '\r':
//                break;
//
//            case ' ':
//                SerialUSB.println("spacebar, nice!");
//                break;
//
//            case '?':
//            	SerialUSB.println("no help available");
//            	break;
//
//            case 'd':
//            	print_ppm_data();
//            	break;
//
//            case 'i':
//            	/// imu data
//            	imu_print_data();
//            	break;
//
//            case 'h':
//                cmd_print_help();
//                break;
//
//            case 't':
//                ppm_decode();
//                break;
//
//            case 's':
//                esc_manual_control();
//                break;
//
//            case 'y':
//            	yaw_manual_control();
//            	break;
//
//            case 'b':
//                cmd_board_info();
//                break;
//
//            default: // -------------------------------
//                SerialUSB.print("Unexpected byte: 0x");
//                SerialUSB.print((int)input, HEX);
//                SerialUSB.println(", press h for help.");
//                break;
//            }
//
//            SerialUSB.print("> ");
//        }
    }
    return 0;
}
