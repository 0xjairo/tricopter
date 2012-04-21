/*
 * Telemetry.h
 *
 *  Created on: Apr 21, 2012
 *      Author: jairo
 */

#ifndef TELEMETRY_H_
#define TELEMETRY_H_

#include "wirish.h"

typedef struct telemetryTable {
    uint16  rc_status;
    byte    ahrs_status;
    float   motor1;
    float   motor2;
    float   motor3;
    float   servo;
    float   uthrottle;
    float   uroll;
    float   upitch;
    float   uyaw;
    float   yroll;
    float   ypitch;
    float   yyaw;
    uint32  cpu_util;
} telemetryTable;

void print_telemetry(telemetryTable *t);

#endif /* TELEMETRY_H_ */
