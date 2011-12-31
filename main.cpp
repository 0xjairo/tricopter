

// Include files
#include "main.h"
#include "wirish.h"
#include "utils.h"
#include "esc-control.h"
#include "ppm-decode.h"
#include "imu-interface.h"


// ASCII escape character
#define ESC       ((uint8)27)

// -- setup() and loop() ------------------------------------------------------

void setup() {
	disable_usarts();

    // Set up the LED to blink
    pinMode(BOARD_LED_PIN, OUTPUT);
	// Set up ESC pins
	pinMode(ROTOR1_PIN, PWM);
	pinMode(ROTOR2_PIN, PWM);
	pinMode(ROTOR3_PIN, PWM);
	// Set up PPM pin
	pinMode(PPM_PIN, INPUT_PULLUP);

    // init motor controllers
    esc_init();

	// ppm decode setup
	init_ppm_timer_and_dma();
	ppm_decode_go();

	// init imu rx
	Serial3.begin(28800);

    // initialize usb
    SerialUSB.begin();
    while(!isConnected()); //wait till console attaches.
    SerialUSB.println("Welcome user!");
}


// Force init to be called *first*, i.e. before static object allocation.
// Otherwise, statically allocated objects that need libmaple may fail.
__attribute__((constructor)) void premain() {
    init();
}

int main(void) {
    // init
    setup();

    while (1) {
        toggleLED();
        delay(20);

        while (SerialUSB.available()) {
            uint8 input = SerialUSB.read();
            SerialUSB.println((char)input);

            switch(input) {
            case '\r':
                break;

            case ' ':
                SerialUSB.println("spacebar, nice!");
                break;

            case '?':
            	SerialUSB.println("no help available");
            	break;

            case 'd':
            	print_ppm_data();
            	break;

            case 'i':
            	/// imu data
            	imu_print_data();
            	break;

            case 'h':
                cmd_print_help();
                break;

            case 't':     
                ppm_decode();
                break;

            case 's':
                cmd_servo_sweep();
                break;

            case 'b':
                cmd_board_info();
                break;

            default: // -------------------------------
                SerialUSB.print("Unexpected byte: 0x");
                SerialUSB.print((int)input, HEX);
                SerialUSB.println(", press h for help.");
                break;
            }

            SerialUSB.print("> ");
        }
    }
    return 0;
}
