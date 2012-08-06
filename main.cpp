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

#include "parameters.h"
#include "telemetry.h"

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
    uint32 tickMainLoop = 0;

    bool fly_ENA;

    int int_tune_status = PARAM_INT_TUNE_FLAG_IDLE;

    float uRoll, uPitch, uYaw, uThrottle;
    float m1, m2, m3;
    float pidRoll, pidPitch, pidYaw;
    float servoAngle;

    paramTable p;
    telemetryTable tt;

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

    p.roll.Kp = 0.00040;
    p.roll.Ki = 0.00000;
    p.roll.Kd = 0.00000;

    p.pitch.Kp = 0.00050;
    p.pitch.Ki = 0.00000;
    p.pitch.Kd = 0.00000;

    p.yaw.Kp = 0.01000;
    p.yaw.Ki = 0.00000;
    p.yaw.Kd = 0.00000;

    update_gains(&p, &rollCtrl, &pitchCtrl, &yawCtrl);

//    rollCtrl.set_gains(  0.00040, 0.00, 0.00000);
//    pitchCtrl.set_gains( 0.00050, 0.00, 0.00000);
//    yawCtrl.set_gains(1.0, 0, 0);

    fly_ENA = 0;

    while (1) {

        t = micros();
        dt = t - t_prev;

        // main loop (400 Hz)
        if (dt > 2500) {
            t_prev = t;


            // 50Hz loop
            if (tickMainLoop % (MAIN_LOOP_F_HZ/50) == 0) {

                rc.update(); // update RC commands
                ahrs.update();

                fly_ENA = (rc.get_channel(CH_AUX4) > 0.5 && rc.status() == SUCCESS);

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
            }



            pidRoll    = rollCtrl.go(uRoll, ahrs.get_roll(), dt);
            pidPitch   = pitchCtrl.go(uPitch, ahrs.get_pitch(), dt);
            //servoAngle = yawCtrl.go(uYaw, ahrs.get_yaw(), dt);
            servoAngle = uYaw;


            m1 = uThrottle + (+1) * (pidRoll) + (-2.0/3.0) * pidPitch;
            m2 = uThrottle + (-1) * (pidRoll) + (-2.0/3.0) * pidPitch;
            m3 = uThrottle + ( 0) * (pidRoll) + (+4.0/3.0) * pidPitch;

            yawServo.set_offset(servoAngle);
            if (fly_ENA) {
                set_rotor_throttle(1, m1);
                set_rotor_throttle(2, m2);
                set_rotor_throttle(3, m3);

            } else {
                set_rotor_throttle(1, 0);
                set_rotor_throttle(2, 0);
                set_rotor_throttle(3, 0);
            }




            // 10Hz loop
            if (tickMainLoop % (MAIN_LOOP_F_HZ/10) == 0) {
                if(fly_ENA)
                    toggleLED();
                else
                    digitalWrite(BOARD_LED_PIN, 0);
            }

            // 2Hz loop
            // if(tickMainLoop % (MAIN_LOOP_F_HZ/2) == 0)
            if (tickMainLoop % (MAIN_LOOP_F_HZ/10) == 0) // 10 hz
            {
                // cpu utilization based on the 2000 microseconds (50 Hz) loop
                cpu_util = (micros() - t) * 100 / dt;

                int_tune_status = interactive_config(&p);

                switch(int_tune_status) {
                case PARAM_INT_TUNE_FLAG_START:
                    print_gains(&rollCtrl, &pitchCtrl, &yawCtrl);
                    break;

                case PARAM_INT_TUNE_FLAG_ACTIVE:
                    break;

                case PARAM_INT_TUNE_FLAG_DONE:
                    update_gains(&p, &rollCtrl, &pitchCtrl, &yawCtrl);
                    break;

                case PARAM_INT_TUNE_FLAG_IDLE:
                    tt.rc_status = rc.status();
                    tt.ahrs_status = ahrs.get_status();
                    tt.motor1 = m1;
                    tt.motor2 = m2;
                    tt.motor3 = m3;
                    tt.uthrottle = rc.get_channel(CH_THROTTLE);
                    tt.servo  = servoAngle;
                    tt.uroll = uRoll;
                    tt.upitch = uPitch;
                    tt.uyaw = uYaw;
                    tt.yroll = ahrs.get_roll();
                    tt.ypitch = ahrs.get_pitch();
                    tt.yyaw = ahrs.get_yaw();
                    tt.cpu_util = cpu_util;

                    print_telemetry(&tt);
                    break;
                }
            } // 10 Hz // 2Hz

            // update tick
            tickMainLoop++;

        } // main loop (50 hz)
    }  // while()
} // main()

