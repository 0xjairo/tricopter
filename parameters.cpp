/*
 * parameters.cpp
 *
 *  Created on: Apr 21, 2012
 *      Author: jairo
 */

#include "wirish.h"
#include "parameters.h"
#include "utils.h"
#include "MyPID.h"

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
                SerialUSB.println("\r\ninteractive tuning. select axis [r,p,y].  Current gains:");
                state++;

                // signal that interactive tuning started
                return PARAM_INT_TUNE_FLAG_START;
            }
            break;
        case 1: // select axis
            inputVal_1e5 = 0;
            switch(input) {
            case 'r':  // roll
                p_p = &(p->roll);
                SerialUSB.print("\r\nroll selected.");
                state++;
                break;
            case 'p': // pitch
                p_p = &(p->pitch);
                SerialUSB.print("\r\npitch selected.");
                state++;
                break;
            case 'y': // yaw
                p_p = &(p->yaw);
                SerialUSB.print("\r\nyaw selected.");
                state++;
                break;
            default: // reset state
                state=0;
                break;
            }

            if(state != 0)
                SerialUSB.print(" select gain [p,i,d]:");

            break;
        case 2:
            switch(input) {
            case 'p': // Kp
                p_v = &(p_p->Kp);
                SerialUSB.print("\r\nKp selected.");
                state++;
                break;
            case 'i': // Ki
                p_v = &(p_p->Ki);
                SerialUSB.print("\r\nKi selected.");
                state++;
                break;
            case 'd': // Kd
                p_v = &(p_p->Kd);
                SerialUSB.print("\r\nKd selected.");
                state++;
                break;
            default:
                state=0;
                break;
            }

            if(state != 0) {
                SerialUSB.print(" Curr:");
                SerialUSB.print(*p_v,6);
                SerialUSB.println(" input value [0-9]:");
            }
            break;
        case 3:
            // check that the next input is a number
            if(input > 0x2F && input < 0x3A)
            {
                inputVal_1e5 += (input-0x30);
                inputVal_1e5 *= 10;
                SerialUSB.print("\r");
                SerialUSB.print((float)((float)inputVal_1e5/1000000.0),6);
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

                SerialUSB.println("Done.");
                // done
                state=0;

                return PARAM_INT_TUNE_FLAG_DONE;


            }else{
                printkv("ERROR, input:", input);
                state=0;
            }
            break;

        default:
            state=0;
            break;
        }

    }

    if(state == 0)
        return PARAM_INT_TUNE_FLAG_IDLE;
    else
        return PARAM_INT_TUNE_FLAG_ACTIVE;

}



void update_gains(paramTable *p, class MyPID *roll, class MyPID *pitch, class MyPID *yaw)
{
    pidParams *p_axis;

    p_axis = &(p->roll);
    roll->set_gains(p_axis->Kp, p_axis->Ki, p_axis->Kd);


    p_axis = &(p->pitch);
    pitch->set_gains(p_axis->Kp, p_axis->Ki, p_axis->Kd);


    p_axis = &(p->yaw);
    yaw->set_gains(p_axis->Kp, p_axis->Ki, p_axis->Kd);

    print_gains(roll, pitch, yaw);

}

void print_gains(class MyPID *roll, class MyPID *pitch, class MyPID *yaw)
{
    printkv("rkp:", roll->get_gain('p') );
    printkv("rki:", roll->get_gain('i') );
    printkv("rkd:", roll->get_gain('d') );

    printkv("pkp:", pitch->get_gain('p') );
    printkv("pki:", pitch->get_gain('i') );
    printkv("pkd:", pitch->get_gain('d') );

    printkv("ykp:", yaw->get_gain('p') );
    printkv("yki:", yaw->get_gain('i') );
    printkv("ykd:", yaw->get_gain('d') );

    SerialUSB.println("");

}
