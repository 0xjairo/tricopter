#include "wirish.h"
#include "utils.h"


static uint16 init_all_timers_prescale;

void set_prescale(timer_dev *dev) {
    timer_set_prescaler(dev, init_all_timers_prescale);
}

void init_all_timers(uint16 prescale) {
    init_all_timers_prescale = prescale;
    timer_foreach(set_prescale);
}

void enable_usarts(void) {
    Serial1.begin(BAUD);
    Serial2.begin(BAUD);
    Serial3.begin(BAUD);
#if defined(STM32_HIGH_DENSITY) && !defined(BOARD_maple_RET6)
    Serial4.begin(BAUD);
    Serial5.begin(BAUD);
#endif
}

void disable_usarts(void) {
    Serial1.end();
    Serial2.end();
    Serial3.end();
#if defined(STM32_HIGH_DENSITY) && !defined(BOARD_maple_RET6)
    Serial4.end();
    Serial5.end();
#endif
}

void print_board_array(const char* msg, const uint8 arr[], int len) {
    SerialUSB.print("\t");
    SerialUSB.print(msg);
    SerialUSB.print(" (");
    SerialUSB.print(len);
    SerialUSB.print("): ");
    for (int i = 0; i < len; i++) {
        SerialUSB.print(arr[i], DEC);
        if (i < len - 1) SerialUSB.print(", ");
    }
    SerialUSB.println();
}

//quick method to return if there's anyone on the other end of the serialusb link yet.
boolean isConnected(){
  return (SerialUSB.isConnected() && (SerialUSB.getDTR() || SerialUSB.getRTS()));
}

void cmd_board_info(void) {     // TODO print more information
    SerialUSB.println("Board information");
    SerialUSB.println("=================");

    SerialUSB.print("* Clock speed (MHz): ");
    SerialUSB.println(CYCLES_PER_MICROSECOND);

    SerialUSB.print("* BOARD_LED_PIN: ");
    SerialUSB.println(BOARD_LED_PIN);

    SerialUSB.print("* BOARD_BUTTON_PIN: ");
    SerialUSB.println(BOARD_BUTTON_PIN);

    SerialUSB.print("* GPIO information (BOARD_NR_GPIO_PINS = ");
    SerialUSB.print(BOARD_NR_GPIO_PINS);
    SerialUSB.println("):");
    print_board_array("ADC pins", boardADCPins, BOARD_NR_ADC_PINS);
    print_board_array("PWM pins", boardPWMPins, BOARD_NR_PWM_PINS);
    print_board_array("Used pins", boardUsedPins, BOARD_NR_USED_PINS);
}


void delimiter() {
	SerialUSB.print(" ");
}

void printkv(const char *k, float v) {
	SerialUSB.print(k);
	SerialUSB.print(v,6);
	delimiter();
}

void printkv(const char *k, int v){
	SerialUSB.print(k);
	SerialUSB.print(v);
	delimiter();
}

void printkv(const char *k, unsigned int v) {
	SerialUSB.print(k);
	SerialUSB.print(v);
	delimiter();
}

void printkv(const char *k, char v) {
    SerialUSB.print(k);
    SerialUSB.print(v);
    delimiter();
}
