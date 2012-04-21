/*
 * Telemetry.cpp
 *
 *  Created on: Apr 21, 2012
 *      Author: jairo
 */

#include "telemetry.h"
#include "utils.h"


void print_telemetry(telemetryTable *t) {
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
       printkv("rc:", !t->rc_status);
       printkv("imu:", t->ahrs_status);
//                    //                    printkv("aux1:",    rc.get_channel(CH_AUX1));
//     printkv("p:", (float) (rollCtrl.get_term('p') * 100.0));
//     printkv("i:", (float) (rollCtrl.get_term('i') * 1000.0));
       printkv("m1:", t->motor1);
       printkv("m2:", t->motor2);
       printkv("servo:", t->servo);
       printkv("thr:",   t->uthrottle);
       printkv("uRoll:", t->uroll);
       printkv("uYaw:",  t->uyaw);
//     printkv("e:", rollCtrl.get_error());
       printkv("yRoll:", t->yroll);
       printkv("yPitch:",t->ypitch);
       printkv("yYaw:",  t->yyaw);

       // cpu utilization after printing data
       printkv("util:", t->cpu_util);
//     // cpu_util2 = (micros()-t)*100/t_delta;
//     // printkv("util2:",cpu_util2);
//
       SerialUSB.println("");

   }
}


