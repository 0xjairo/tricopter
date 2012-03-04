#define DEBUG_LEVEL DEBUG_NONE
/*
 * Rotor definitions on tricopter
 * Note that motor 3 is the tail rotor
 *
 *    ( 1 )         ( 2 )   Front
 *        \        /
 *         +------+
 *            |
 *            |
 *          ( 3 )           Tail
 *
 */

//#define COPTER_DEBUG

/* Timer 1, Ch 1 */
#define ROTOR1_PIN 27
/* Timer 1, Ch 2 */
#define ROTOR2_PIN 26
/* Timer 1, Ch 3 */
#define ROTOR3_PIN 25
/* Timer 2, Channel 4 pin */
#define YAW_SERVO_PIN 8

/* Timer 4, Ch 1 */
#define PPM_PIN 16

/* 3_RX  (pin 0 on Maple Mini) */
#define IMU_RX_PIN BOARD_USART3_RX_PIN

/*
 * The PPM signal frequency is 50 Hz.
 * The max count in the timer's register
 * is 65536 (16-bit register).
 * The MCU's clock is 72 MHz.
 * Therefore the prescale factor is:
 * 1/(50 Hz) * (72 MHz) / (65536) = 21.97
 */
#define SERVO_PPM_TIMER_PRESCALE_FACTOR 22

/* min: 1ms
 *   (1 ms * 72 Mhz)/ (PRESCALE_FACTOR)
 * = (1 ms * 72 Mhz)/ 22
 * =  3272.72 ticks
 */
#define PPM_MIN 3273

/* max: 2ms =  6545.45 */
#define PPM_MAX 6545

