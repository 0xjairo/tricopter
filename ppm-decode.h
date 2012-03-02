#ifndef PPM_DECODE_H_
#define PPM_DECODE_H_

#include "wirish.h"

#define TX_NUM_CHANNELS 8

//number of captures to do by dma
#define NUM_TIMERS 9

// timer prescale
#define TIMER_PRESCALE 30

// TIMER_PRESCALE*(1/72 MHz) =
#define TICK_PERIOD_MS ( TIMER_PRESCALE*0.0000138888889f )

// 0.006 = 6 ms is the smallest sync pulse
// 22.5 ms - (8 channels* 2ms) = 6.6 ms
#define SYNC_PULSE_MIN_TICKS (6.0f/TICK_PERIOD_MS)

// 0.0025 = 2.0 ms is the max for any channel
//			+0.5 ms margin
#define CHANNEL_MAX_TICKS (2.5f/TICK_PERIOD_MS)

// sync pulse confidence minimum
#define SP_CONFIDENCE_MINIMUM 3

// sync pulse invalid flag
#define SP_INVALID (-1)

// the order of the elements of rcCmd_t
// determine the channel number of the command
//typedef struct {
//	float roll;
//	float pitch;
//	float throttle;
//	float yaw;
//	float aux1;
//	float aux2;
//	float aux3;
//	float aux4;
//} rcCmd_t;

int 	printData();

int 	rx_read(int *sync_pulse, float *rc);
int 	rx_read_commands();
void 	ppm_sync();

void 	dma_isr();
void 	ppm_timeout_isr();
void 	ppm_decode_go();
void 	init_ppm_timer_and_dma();
void 	init_ppm_timer();
void 	init_ppm_dma_transfer();
void 	print_ppm_data();
void 	ppm_decode(void);

#endif
