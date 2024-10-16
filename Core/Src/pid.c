#include "pid.h"

void PID_init(pidController_t *p)
{
    p->lastError = 0;
}

float PID_task(pidController_t* p, float processIn)
{
    p->processIn = processIn;
    float error = p->setpoint - processIn;

    // process control deadband
    if(error > p->deadband)
        error -= p->deadband;
    else if(error < -p->deadband)
        error += p->deadband;
    else
        error = 0;

    if(p->useSqrtErrorP)
    {
        if(error < 0)
        {
            error = sqrtf(fabsf(error));
            error *= -1.0f;
        }
        else
        {
        	error = sqrtf(fabsf(error));
        }
    }

    p->error = error;

    if( p->ki != 0.0f)
    {
        p->ierror += p->error;
        p->ierror = clip(p->ierror, p->processMin / p->ki, p->processMax / p->ki);
    }
    else
    {
        p->ierror = 0;
    }

    p->derror = p->error - p->lastError;

    float output = (p->kp * p->error)
                  + (p->ki * p->ierror)
                  + (p->kd * p->derror);
    if(output != 0)
        output = output > 0 ? output + p->minValue : output - p->minValue;
    output = clip(output, p->processMin, p->processMax);

    p->lastError = p->error;

    return output;
}

float clip(float val, float min, float max)
{
    return fminf(fmaxf(val, min), max);
}