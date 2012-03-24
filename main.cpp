// Include files
#include "main.h"
#include "wirish.h"
#include "utils.h"
#include "esc-control.h"
#include "ppm-decode.h"
#include "yaw-servo.h"
#include "GPS_IMU.h"

#include "RC.h"
#include "AHRS.h"
#include "MyPID.h"

// ASCII escape character
#define ESC       ((uint8)27)

extern float rx_channels[TX_NUM_CHANNELS];
extern uint16 ppm_timeout;
extern int sync_pulse_confidence;

// -- setup() and loop() ------------------------------------------------------

void setup() {

    // Set up the LED to blink
    pinMode(BOARD_LED_PIN, OUTPUT);
    // Set up ESC pins
    pinMode(ROTOR1_PIN, PWM);
    pinMode(ROTOR2_PIN, PWM);
    pinMode(ROTOR3_PIN, PWM);
    // Yaw servo pin (8)
    pinMode(YAW_SERVO_PIN, PWM);
    // Set up PPM pin
    pinMode(PPM_PIN, INPUT_PULLUP);

    // init motor controllers
    esc_init();

    // initialize usb
    SerialUSB.begin();
    //    while(!isConnected()); //wait till console attaches.
    //    SerialUSB.println("Welcome user!");
}

// Force init to be called *first*, i.e. before static object allocation.
// Otherwise, statically allocated objects that need libmaple may fail.
__attribute__((constructor)) void premain() {
    init();
}

int main(void) {
    uint32 t;
    uint32 t_prev, dt;
    uint32 cpu_util, cpu_util2;
    uint32 tick50hz = 0;
    float uRoll, uPitch, uYaw, uThrottle;
    float KpRoll, KiRoll, KdRoll;
    float m1, m2, m3;
    float pidRoll, pidPitch, pidYaw;
    float servoAngle;

    RC rc;
    AHRS ahrs;
    MyPID rollCtrl, pitchCtrl, yawCtrl;
    YawServo yawServo;

    // init
    setup();
    ahrs.init();
    rc.init();
    yawServo.init(YAW_CENTER, YAW_OFFSET_MIN, YAW_OFFSET_MAX);

    toggleLED();

    //    while(rc.status() != SUCCESS || rc.get_channel(CH_THROTTLE) > 0.2) {
    //    	rc.update();
    //    	delay_us(1000);
    //    	toggleLED();
    //    }

    t_prev = 0;
    uRoll = 0;


    KpRoll = 0.00005; //rc.get_channel(CH_AUX1)*0.001; //
    KiRoll = 0; //rc.get_channel(CH_AUX2)*0.001;
    KdRoll = -0.000004; //rc.get_channel(CH_AUX3)*-0.00005;

    rollCtrl.set_gains(KpRoll, KiRoll, KdRoll);
    pitchCtrl.set_gains(KpRoll, KiRoll, KdRoll);
    yawCtrl.set_gains(1.0, 0, 0);

    while (1) {

        t = micros();
        dt = t - t_prev;

        // main loop (50 Hz)
        if (dt > 20000) {
            t_prev = t;

            rc.update(); // update RC commands
            ahrs.update();

            if (rc.status() == SUCCESS) {
                uThrottle = rc.get_channel(CH_THROTTLE);
                uRoll = 100 * (rc.get_channel(CH_ROLL) - 0.5);
                uPitch = 100 * (rc.get_channel(CH_ROLL) - 0.5);
                uYaw = 20.0 * (rc.get_channel(CH_YAW) - 0.5);
            } else {
                uRoll = 0.0;
                uPitch = 0.0;
                //uYaw = 0;
            }

            uThrottle = constrain(uThrottle, 0, MAX_THROTTLE);
            uRoll     = constrain(uRoll, -MAX_ROLL, MAX_ROLL);
            uPitch    = constrain(uPitch, -MAX_PITCH, MAX_PITCH);
            //uYaw     = constrain...

            if (uThrottle < MIN_THROTTLE  && rc.get_channel(CH_AUX4) > 0.5) {
                uRoll = 0.0; // reset desired pitch when throttle falls
            }

            pidRoll    = rollCtrl.go(uRoll, ahrs.get_roll(), dt);
            pidPitch   = pitchCtrl.go(uPitch, ahrs.get_pitch(), dt);
            //servoAngle = yawCtrl.go(uYaw, ahrs.get_yaw(), dt);
            servoAngle = uYaw;


            m1 = uThrottle + (+1) * (pidRoll) + (-2.0/3.0) * pidPitch;
            m2 = uThrottle + (-1) * (pidRoll) + (-2.0/3.0) * pidPitch;
            m3 = uThrottle + ( 0) * (pidRoll) + (+4.0/3.0) * pidPitch;

            yawServo.set_offset(servoAngle);
            if (rc.get_channel(CH_AUX4) > 0.5 && rc.status() == SUCCESS) {
                set_rotor_throttle(1, m1);
                set_rotor_throttle(2, m2);
                set_rotor_throttle(3, m3);

            } else {
                set_rotor_throttle(1, 0);
                set_rotor_throttle(2, 0);
                set_rotor_throttle(3, 0);
            }

            // 10Hz loop
            if (tick50hz % 5 == 0) {
                toggleLED();
            }

            // 2Hz loop
            //			if(tick50hz % 25 == 0)
            if (tick50hz % 5 == 0) // 10 hz
            {
                // cpu utilization based on the 2000 microseconds (50 Hz) loop
                cpu_util = (micros() - t) * 100 / dt;

                /*
                 * Safe Print
                 * ----------
                 *
                 * <http://leaflabs.com/docs/lang/api/serialusb.html>
                 * If this logic fails to detect if bytes are going to be read
                 * by the USB host, then the println() take a long time,
                 * causing a very slow LED blink.  If the characters are
                 * printed and read, the blink will only slow a small amount
                 * when "really" connected, and will be fast fast when the
                 * virtual port is only configured.
                 *
                 */
                if (isConnected()) {
                    printkv("rc:", !rc.status());
                    printkv("imu:", ahrs.get_status());
                    //					printkv("aux1:", 	rc.get_channel(CH_AUX1));
                    printkv("p:", (float) (rollCtrl.get_term('p') * 100.0));
                    printkv("i:", (float) (rollCtrl.get_term('i') * 1000.0));
                    printkv("m1:", m1);
                    printkv("m2:", m2);
                    printkv("kp:", rollCtrl.get_gain('p') * 1000);
                    printkv("ki:", rollCtrl.get_gain('i') * 1000);
                    printkv("kd:", rollCtrl.get_gain('d') * 100000);
                    printkv("servo:", servoAngle);

                    printkv("thr:", rc.get_channel(CH_THROTTLE));
                    printkv("uRoll:", uRoll);
                    printkv("uYaw:", uYaw);
                    printkv("e:", rollCtrl.get_error());
                    printkv("yRoll:", ahrs.get_roll());
                    printkv("yPitch:", ahrs.get_pitch());
                    printkv("yYaw:", ahrs.get_yaw());

                    // cpu utilization after printing data
                    //					printkv("  util:", cpu_util);
                    //					cpu_util2 = (micros()-t)*100/t_delta;
                    //					printkv("util2:",cpu_util2);

                    SerialUSB.println("");
                }

            }

            tick50hz++;
        }
    }
    return 0;
}
