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


/* min: 1ms in tick counts */
#define PPM_MIN 3430
/* max: 2ms in tick counts */
#define PPM_MAX 6855

