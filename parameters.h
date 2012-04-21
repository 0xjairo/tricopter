/*
 * parameters.h
 *
 *  Created on: Apr 21, 2012
 *      Author: jairo
 */

#ifndef PARAMETERS_H_
#define PARAMETERS_H_


typedef struct pidParams {
    float Kp;
    float Ki;
    float Kd;
} pidParams;

typedef struct paramTable {
    pidParams roll;
    pidParams pitch;
    pidParams yaw;
} paramTable;

enum {
    // interactive tuning flags
    PARAM_INT_TUNE_FLAG_IDLE,
    PARAM_INT_TUNE_FLAG_START,
    PARAM_INT_TUNE_FLAG_ACTIVE,
    PARAM_INT_TUNE_FLAG_DONE,
};

int interactive_config(paramTable *p);
void update_gains(paramTable *p, class MyPID *roll, class MyPID *pitch, class MyPID *yaw);
void print_gains(class MyPID *roll, class MyPID *pitch, class MyPID *yaw);



#endif /* PARAMETERS_H_ */
