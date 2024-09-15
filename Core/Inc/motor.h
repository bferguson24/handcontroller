#pragma once

#include "stm32f7xx_hal.h"

typedef struct {
    // pwm
    
    // dir
    GPIO_TypeDef* dir_port;
    uint16_t dir_pin;
    // fault
    
    // current
    
    // position
    float position;
    // temp

} motor_t;

//Extern = Declare variable
// extern int a ;
void MOTOR_init();
void Timer1_Init();
float MOTOR_readAngle();