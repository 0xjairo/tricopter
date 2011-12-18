#include "main.h"
#include "wirish.h"
#include "utils.h"
#include "esc-control.h"


// Servo constants
#define SERVO_MIN 3430
#define SERVO_MAX 6800
#define PPM_CNTS_TO_DEG 0.09
#define SERVO_ANGLE_TO_DUTY 37.44444 

uint8 gpio_state[BOARD_NR_GPIO_PINS];

const char* dummy_data = ("qwertyuiopasdfghjklzxcvbnmmmmmm,./1234567890-="
                          "qwertyuiopasdfghjklzxcvbnm,./1234567890");



void ppm_decode(void){
    SerialUSB.println("Decoding DIY Drones PPM encoder on pin D27");
    
    int pin = 27;
    uint8 prev_state;
    int time_elapsed = 0;
    int time_start = 0;
    int i = 0;
    int channels[8];
    float angle;

    pinMode(pin, INPUT);
    prev_state = (uint8)digitalRead(pin);

    while(!SerialUSB.available()){

        uint8 current_state = (uint8)digitalRead(pin);
        if (current_state != prev_state) {
            if (current_state) {

                time_elapsed = (micros() - time_start);
                time_start = micros();

            }else{
#ifdef USB_VERBOSE
                SerialUSB.print(i);
                SerialUSB.print(":");
                SerialUSB.print(time_elapsed);
#endif
                if(time_elapsed < 2500 && i < 8){
                    if(i==2){
                        if(time_elapsed < 1000){
                            angle = 0.0;
                        }else if(time_elapsed > 2000){
                            angle = 90.0;
                        }else{
                            angle = (float)((time_elapsed-1000)*PPM_CNTS_TO_DEG);
                        }
                        SerialUSB.print(":");
                        SerialUSB.print(angle);
                        set_servo_angle(angle);
                    }
                    channels[i++] = time_elapsed;
                }else{
#ifdef USB_VERBOSE
                    SerialUSB.println("");
#endif
                    i=0;
                }
#ifdef USB_VERBOSE
                SerialUSB.print("\t");
#endif

                time_elapsed = 0;
            }
            prev_state = current_state;
        }

    }
    SerialUSB.println("Done!");
    
}

void set_servo_angle(float angle)
{
    //SERVO_MIN = 3430
    //SERVO_MAX = 6800
    //SERVO_CNTS_TO_DEG=0.0267
    int duty;
    
    disable_usarts();

    duty = SERVO_MIN + (int)(angle * SERVO_ANGLE_TO_DUTY);
    if(duty > SERVO_MAX) duty = SERVO_MAX;
    pwmWrite(ROTOR1_PIN, duty);
}

void cmd_servo_sweep(void) {
    SerialUSB.println("Testing D16 PWM header with a servo sweep. "
                      "Press any key to stop.");
    SerialUSB.println();

    disable_usarts();
    init_all_timers(21);

    for (uint32 i = 0; i < BOARD_NR_PWM_PINS; i++) {
        if (boardUsesPin(i))
            continue;
        pinMode(boardPWMPins[i], PWM);
        pwmWrite(boardPWMPins[i], 4000);
    }

    // 1.25ms = 4096counts = 0deg
    // 1.50ms = 4915counts = 90deg
    // 1.75ms = 5734counts = 180deg
    //int rate = 4096;
    int rate = 3430; // 1 ms
    while (!SerialUSB.available()) {
        rate += 20;
        if (rate > 6800)//5734) 6800 = 2ms
            rate = 3430;//4096;
        //for (uint32 i = 0; i < BOARD_NR_PWM_PINS; i++) {
        //    if (boardUsesPin(i))
        //        continue;
        //    pwmWrite(boardPWMPins[i], rate);
        //}
        pwmWrite(16, rate);
        delay(20);
    }

    for (uint32 i = 0; i < BOARD_NR_PWM_PINS; i++) {
        if (boardUsesPin(i))
            continue;
        pinMode(boardPWMPins[i], OUTPUT);
    }
    init_all_timers(1);
    enable_usarts();
}

