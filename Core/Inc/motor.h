#pragma once

#include "stm32f723xx.h"
#include "stm32f7xx_hal.h"
#include "pid.h"
#include <stdint.h>
#include "filter.h"


//Motor Directions
#define CCW 0 
#define CW 1


#define MOTOR_OFF 0
#define MOTOR_ON 1 


typedef struct{

float frictionOffset;
float frictionGain;
float Rsense;

}pcb_t;

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

    int16_t Kcurrent;
    int16_t Kpwm; 


  //Angle Calibration:
    int16_t *positionCounts;

    float theta;


    float angle_range;
    float angle_start;
    float angle_end;
    
    float Rsense;
    float frictionGain;
    float frictionOffset;


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


// //Specific PCB Being used by motor:
//     pcb_t board;

} motor_t;


typedef struct{
    motor_t motors[6];
    int motorCount;
}MotorSet_t
;


//PCB Declarations: 

// extern pcb_t board1;




// Function Declarations

    /**
    * @brief Motor Torque Control
    *
    * Motor torque control loop
    * 
    * @param motor Pointer to specific motor object. 
    * @param torque Torque setpoint. Units expected in [Nm]
    *               
    * @return Error code in future? 
    */
    void torque_control(motor_t *Motor, float torque_in_NM);


    /**
    * @brief PWM motor set
    *
    * Set the corresponsing pwm and dir for the given motor object
    * 
    * @param motor Pointer to specific motor object. 
    * @param pwm pwm duty cycle command, can be + or - 

    *                
    * @return Error code in future? 
    */
    void pwm_set(motor_t *Motor, float pwm_command, int direction);


    /**
    * @brief Motor angle conversion & calculation
    *
    * Set the corresponsing pwm and dir for the given motor object
    * 
    * @param motor Pointer to specific motor object. 
    *               
    * @return Error code in future? 
    */
    float angle_calc(motor_t *Motor);




    void current_control(motor_t *Motor, float current);
