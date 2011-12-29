

// Include files
#include "main.h"
#include "wirish.h"
#include "utils.h"
#include "esc-control.h"
#include "ppm-decode.h"


// ASCII escape character
#define ESC       ((uint8)27)

// Globals
HardwareTimer timer1(1);

// -- setup() and loop() ------------------------------------------------------

void setup() {
    // Set up the LED to blink
    pinMode(BOARD_LED_PIN, OUTPUT);

    // Setup timer1 for PWM
    timer1.setMode(TIMER_CH1, TIMER_PWM);
    timer1.setPrescaleFactor(21);
    pinMode(ROTOR1_PIN, PWM);
    pinMode(ROTOR2_PIN, PWM);
    pinMode(ROTOR3_PIN, PWM);
    //using timer4, channel1, maps to pin d16 (maple mini) according to maple master pin map.
	pinMode(PPM_PIN, INPUT_PULLUP);



    // ppm decode setup
    // init timer1 and dma
    init_timer_input_capture_dma();

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
        delay(250);

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
            	ppm_decode_interrupt_dma();
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
