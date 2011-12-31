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

#define COPTER_DEBUG

/* Timer 1, Ch 1 */
#define ROTOR1_PIN 27
/* Timer 1, Ch 2 */
#define ROTOR2_PIN 26
/* Timer 1, Ch 3 */
#define ROTOR3_PIN 25
/* Mounted on rotor 3 */
#define YAW_SERVO_PIN 24

/* Timer 4, Ch 1 */
#define PPM_PIN 16

/* 3_RX */
#define IMU_RX_PIN 0
