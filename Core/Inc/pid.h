#pragma once

#include <stdint.h>
#include <stdbool.h>
#include "math.h"
#include "string.h"

typedef struct {
    float setpoint; /* the setpoint the PID controller should attempt to reach */
    volatile float processIn; /* the current value of the plant (position, current, etc) */
    float error; /* the signed error between the current value and the setpoint */
    float ierror; /* the accumulated (integral) error */
    float derror; /* the change in error */
    float kp; /* proportional tuning constant */
    float ki; /* integral tuning constant */
    float kd; /* derivative tuning constant */
    float processMin; /* the minimum value the PID controller should clip its output to */
    float processMax; /* the maximum value the PID controller should clip its output to */
    float deadband; /* the zone around the setpoint where the controller will not operate */
    float minValue; /* the zone around the setpoint where the controller will not operate */
    bool useSqrtErrorP;
    float lastError;
} pidController_t;

void PID_init(pidController_t *p);
float PID_task(pidController_t* tuning, float processIn);
