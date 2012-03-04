#include "wirish.h"

// Comment out to disable USB printouts
#define USB_VERBOSE

// Default USART baud rate
#define BAUD     9600


void cmd_print_help(void);
void cmd_board_info(void);
void set_prescale(timer_dev *dev);
void init_all_timers(uint16 prescale);
void enable_usarts(void);
void disable_usarts(void);
void print_board_array(const char* msg, const uint8 arr[], int len);
boolean isConnected();

void delimiter();
void printkv(const char*k, float v);
void printkv(const char*k, int v);
void printkv(const char*k, unsigned int v);
