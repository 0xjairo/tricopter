#include "main.h"
#include "wirish.h"
#include "utils.h"
#include "esc-control.h"


uint8 gpio_state[BOARD_NR_GPIO_PINS];

const char* dummy_data = ("qwertyuiopasdfghjklzxcvbnmmmmmm,./1234567890-="
                          "qwertyuiopasdfghjklzxcvbnm,./1234567890");


HardwareTimer timer1(1);

void esc_init()
{
	// Setup timer1 for PWM
	timer1.setMode(TIMER_CH1, TIMER_PWM);
	timer1.setMode(TIMER_CH2, TIMER_PWM);
	timer1.setMode(TIMER_CH3, TIMER_PWM);
	timer1.setPrescaleFactor(ESC_PPM_TIMER_PRESCALE_FACTOR);

	// Set throttle to minimum
	for(int i=0;i<3;i++)
	{
		set_rotor_throttle(i+1, 0);
	}

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

	if(rate < 0)
		rate = 0.0;

	if(rate > 1)
		rate = 1.0;

	// 1.00ms = 3430counts = 0% (0deg)
	// 1.25ms = 4096counts =
	// 1.50ms = 4915counts = 50% (90deg)
	// 1.75ms = 5734counts =
	// 2.00ms = 6800counts = 100% (180deg)
	int duty = ESC_PPM_MIN + (float)(ESC_PPM_MAX-ESC_PPM_MIN)*rate;

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

#ifdef CLI_UTILS
void esc_manual_control(void) {
    SerialUSB.println("Press \'j\' to lower speed.");
    SerialUSB.println("Press \'k\' to increase speed.");
    SerialUSB.println("Press \'z\' to zero command.");
    SerialUSB.println("Any other key zeroes command and exits");
    SerialUSB.println();

    uint8 input;
    float rate1 = 0;
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
    		rate1 = 0.00;

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

}
#endif
