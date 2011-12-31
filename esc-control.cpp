#include "main.h"
#include "wirish.h"
#include "utils.h"
#include "esc-control.h"


// Servo constants
#define SERVO_MIN 3430
#define SERVO_MAX 6855
#define PPM_CNTS_TO_DEG 0.09
#define SERVO_ANGLE_TO_DUTY 37.44444 

uint8 gpio_state[BOARD_NR_GPIO_PINS];

const char* dummy_data = ("qwertyuiopasdfghjklzxcvbnmmmmmm,./1234567890-="
                          "qwertyuiopasdfghjklzxcvbnm,./1234567890");


HardwareTimer timer1(1);

void esc_init()
{
	// Setup timer1 for PWM
	timer1.setMode(TIMER_CH1, TIMER_PWM);
	timer1.setPrescaleFactor(21);

	// Set throttle to minimum
	for(int i=0;i<3;i++)
	{
		set_rotor_throttle(i+1, 0);
	}

}

void ppm_decode(void){
    SerialUSB.println("Decoding DIY Drones PPM encoder on pin D27");
    
    int pin = 16;
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
    int duty;
    
    disable_usarts();

    duty = SERVO_MIN + (int)(angle * SERVO_ANGLE_TO_DUTY);
    if(duty > SERVO_MAX) duty = SERVO_MAX;
    pwmWrite(ROTOR1_PIN, duty);
}

// rotor = 1,2,3
// rate = 0.0 to 1.00 %
void set_rotor_throttle(int rotor, float rate)
{
	int pin;

	switch(rotor){
	case 1:
		pin = ROTOR1_PIN;
		break;
	case 2:
		pin = ROTOR2_PIN;
		break;
	case 3:
		pin = ROTOR3_PIN;
		break;
	default:
		return;
	}

	if(rate < 0 || rate > 1) return;

	// 1.00ms = 3430counts = 0% (0deg)
	// 1.25ms = 4096counts =
	// 1.50ms = 4915counts = 50% (90deg)
	// 1.75ms = 5734counts =
	// 2.00ms = 6800counts = 100% (180deg)
	int duty = SERVO_MIN + (float)(SERVO_MAX-SERVO_MIN)*rate;

#ifdef COPTER_DEBUG
	SerialUSB.print("Rotor:D");
	SerialUSB.print(pin);
	SerialUSB.print("\tRate:");
	SerialUSB.print(rate);
	SerialUSB.print("\tDuty:");
	SerialUSB.println(duty);
#endif

	pwmWrite(pin, duty);

}

void cmd_servo_sweep(void) {
    SerialUSB.println("Testing rotors.  Sweeping from 0 to 50%"
                      "Press any key to stop.");
    SerialUSB.println();

    disable_usarts();

    uint8 input;
    float rate1 = 0;
    float rate2 = 0;
    while (1) {

    	input = SerialUSB.read();
    	if (input == 'j')
    	{
			rate1 -= 0.01;
		}
    	else if(input == 'k')
    	{
    		rate1 += 0.01;
    	}
    	else if(input == 'z')
    	{
    		rate1 = 0;

    	}else{
    		break;
    	}


        if (rate1 > 1.0)  rate1 = 1.0;

        SerialUSB.println(rate1);

        set_rotor_throttle(1, rate1);
        set_rotor_throttle(2, rate1);
        set_rotor_throttle(3, rate1);
        delay(20);
    }

    set_rotor_throttle(1, 0.0);
    set_rotor_throttle(2, 0.0);
    set_rotor_throttle(3, 0.0);



//    for (uint32 i = 0; i < BOARD_NR_PWM_PINS; i++) {
//        if (boardUsesPin(i))
//            continue;
//        pinMode(boardPWMPins[i], OUTPUT);
//    }
//    init_all_timers(1);
    enable_usarts();
}

