#include "wirish.h"

// Commands
void cmd_print_help(void);
void cmd_servo_sweep(void);
void cmd_board_info(void);
void ppm_decode(void);
void set_servo_angle(float angle);

// Helper functions
void init_all_timers(uint16 prescale);
void enable_usarts(void);
void disable_usarts(void);
void print_board_array(const char* msg, const uint8 arr[], int len);

