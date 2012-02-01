#define TX_NUM_CHANNELS 8
#define SYNC_PULSE_MINIMUM 10000

//number of captures to do by dma
#define NUM_TIMERS 9

// timer prescale
#define TIMER_PRESCALE 26

// TIMER_PRESCALE*(1/72 MHz) =
#define TICK_PERIOD ( TIMER_PRESCALE*0.0000138888889f )

void printData();
void dma_isr();
void ppm_timeout_isr();
void ppm_decode_go();
void init_ppm_timer_and_dma();
void init_ppm_timer();
void init_ppm_dma_transfer();
void print_ppm_data();
void ppm_decode(void);



