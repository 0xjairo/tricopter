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

    bool fly_ENA;

    float uRoll, uPitch, uYaw, uThrottle;
    float m1, m2, m3;
    float pidRoll, pidPitch, pidYaw;
    float servoAngle;

    paramTable p;

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

        // main loop (50 Hz)
        if (dt > 20000) {
            t_prev = t;

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
            if (tick50hz % 5 == 0) {
                if(fly_ENA)
                    toggleLED();
                else
                    digitalWrite(BOARD_LED_PIN, 0);
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
//                    //					printkv("aux1:", 	rc.get_channel(CH_AUX1));
//                    printkv("p:", (float) (rollCtrl.get_term('p') * 100.0));
//                    printkv("i:", (float) (rollCtrl.get_term('i') * 1000.0));
                    printkv("m1:", m1);
                    printkv("m2:", m2);
                    printkv("servo:", servoAngle);
//
//                    printkv("thr:", rc.get_channel(CH_THROTTLE));
                    printkv("uRoll:", uRoll);
                    printkv("uYaw:", uYaw);
//                    printkv("e:", rollCtrl.get_error());
                    printkv("yRoll:", ahrs.get_roll());
                    printkv("yPitch:", ahrs.get_pitch());
                    printkv("yYaw:", ahrs.get_yaw());
//
//                    // cpu utilization after printing data
//                    //					printkv("  util:", cpu_util);
//                    //					cpu_util2 = (micros()-t)*100/t_delta;
//                    //					printkv("util2:",cpu_util2);
//

                    SerialUSB.println("");

                    if(interactive_config(&p)){
                        update_gains(&p, &rollCtrl, &pitchCtrl, &yawCtrl);

                    }


                }

            }

            tick50hz++;
        }
    }
    return 0;
}



int interactive_config(paramTable *p) {
    static int inputVal_1e5; // input value 1e5 times larger (need to divide by 1e5)
    float inputVal;
    static int state = 0;
    static pidParams *p_p; // pointer, temp pidParams
    static float *p_v; // pointer, temp param value

    while(SerialUSB.available())
    {
        uint8 input = SerialUSB.read();
        switch(state) {
        case 0:
            if(input == 't') { // tune PID
                state++;
            }
            break;
        case 1: // select axis
            inputVal_1e5 = 0;
            switch(input) {
            case 'r':  // roll
                p_p = &(p->roll);

                state++;
                break;
            case 'p': // pitch
                p_p = &(p->pitch);
                state++;
                break;
            case 'y': // yaw
                p_p = &(p->yaw);
                state++;
                break;
            default: // reset state
                state=0;
                break;
            }
            break;
        case 2:
            switch(input) {
            case 'p': // Kp
                p_v = &(p_p->Kp);
                state++;
                break;
            case 'i': // Ki
                p_v = &(p_p->Ki);
                state++;
                break;
            case 'd': // Kd
                p_v = &(p_p->Kd);
                state++;
                break;
            default:
                state=0;
                break;
            }
            break;
        case 3:
            // check that the next input is a number
            if(input > 0x2F && input < 0x3A)
            {
                inputVal_1e5 += (input-0x30);
                inputVal_1e5 *= 10;
                printkv("inputVal_1e5:", inputVal_1e5);
            }else if(input == '\r') {
                inputVal_1e5 /= 10;
                inputVal = ((float)inputVal_1e5)/100000.0;
//                SerialUSB.print("inputVal:");
//                SerialUSB.print(inputVal, 8);
//
//                SerialUSB.print(" p->roll.Kp=");
//                SerialUSB.print(p->roll.Kp, 8);
//                SerialUSB.print(" *p_v=");
//                SerialUSB.print(*(float *)p_v, 8);

                *(float *)(p_v) =inputVal;

                // done
                state=0;
                printkv("ist:", state);

                return true;


            }else{
                printkv("ERROR, input:", input);
                state=0;
            }
            break;

        default:
            state=0;
            break;
        }
        // interactive session state
        printkv("ist:", state);

    }

    return false;

}

void update_gains(paramTable *p, class MyPID *roll, class MyPID *pitch, class MyPID *yaw)
{
    pidParams *p_axis;
    p_axis = &(p->roll);
    roll->set_gains(p_axis->Kp, p_axis->Ki, p_axis->Kd);

    printkv("rkp:", roll->get_gain('p') );
    printkv("rki:", roll->get_gain('i') );
    printkv("rkd:", roll->get_gain('d') );


    p_axis = &(p->pitch);
    pitch->set_gains(p_axis->Kp, p_axis->Ki, p_axis->Kd);

    printkv("pkp:", pitch->get_gain('p') );
    printkv("pki:", pitch->get_gain('i') );
    printkv("pkd:", pitch->get_gain('d') );


    p_axis = &(p->yaw);
    yaw->set_gains(p_axis->Kp, p_axis->Ki, p_axis->Kd);

    printkv("ykp:", yaw->get_gain('p') );
    printkv("yki:", yaw->get_gain('i') );
    printkv("ykd:", yaw->get_gain('d') );

    SerialUSB.println("");


}
