#define TX_NUM_CHANNELS 8

//number of captures to do by dma
#define NUM_TIMERS 9

// timer prescale
#define TIMER_PRESCALE 30

// TIMER_PRESCALE*(1/72 MHz) =
#define TICK_PERIOD_MS ( TIMER_PRESCALE*0.0000138888889f )

// 0.006 = 6 ms mis the smallest sync pulse
// 22.5 ms - (8 channels* 2ms) = 6.6 ms
#define SYNC_PULSE_MINIMUM (6.0f/TICK_PERIOD_MS)

int printData();

int rx_read_commands();
void ppm_sync();

void dma_isr();
void ppm_timeout_isr();
void ppm_decode_go();
void init_ppm_timer_and_dma();
void init_ppm_timer();
void init_ppm_dma_transfer();
void print_ppm_data();
void ppm_decode(void);



