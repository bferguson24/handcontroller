#pragma once

#include "stm32f723xx.h"
#include "stm32f7xx_hal.h"
#include "pid.h"
#include <stdint.h>
#include "filter.h"




typedef struct {

//PIN/TIM/ADC Declarations:

    // PWM 
    TIM_HandleTypeDef *htim;
    uint32_t pwm_channel;

    // Dir
    GPIO_TypeDef *dir_port;
    uint16_t dir_pin;
    
    // Fault
    GPIO_TypeDef *fault_port;
    uint16_t fault_pin;

    // Current
    int16_t *currentCounts;


    // Position
    int16_t *positionCounts;
    float theta_filtered;


    float angle_distance;
    float angle_1;
    float angle_2;

    float thetaCalc;
    

    // Temp
    ADC_HandleTypeDef *temp_ADC;


    // Local Variable Definitions:
    uint32_t position; 

    float torque_set;
    int direction;
    float pwmCommand;

    uint32_t current; 
    uint32_t temp;

//DMA buffer containing Peripheral Data
    uint16_t *dma_buffer;

// PID Struct Declaration
    pidController_t pid;

} motor_t;


typedef struct{
    motor_t motors[6];
    int motorCount;
}MotorSet_t
;



// Function Declarations

void Motor_init(motor_t *Motor, uint16_t *external_buffer, size_t buffer_size);
float read_current(motor_t *Motor);
uint16_t read_position(motor_t *Motor);
void torque_set(motor_t *Motor);
void motor_command(motor_t *Motor, float pwm_command,int Direction);
float angleCalc(motor_t *Motor);
void torqueControl(MotorSet_t* motorSet, float W1,float W2);
