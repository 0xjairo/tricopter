#include "wirish.h"


// Default USART baud rate
#define BAUD     9600


void cmd_print_help(void);
void set_prescale(timer_dev *dev);
void init_all_timers(uint16 prescale);
void enable_usarts(void);
void disable_usarts(void);
void print_board_array(const char* msg, const uint8 arr[], int len);
boolean isConnected();
