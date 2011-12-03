#include "utils.h"


static uint16 init_all_timers_prescale;


void cmd_print_help(void) {
    SerialUSB.println("");
    SerialUSB.println("Command Listing");
    SerialUSB.println("\t?: print this menu");
    SerialUSB.println("\tb: print information about the board.");
    SerialUSB.println("\ts: servo sweep on pin D16.");
    SerialUSB.println("\tt: ppm decode on pin D15.");
}



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

