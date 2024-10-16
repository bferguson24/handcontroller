#include "motor.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>  // Include stdlib.h for memory allocation
#include "arm_math.h"



float angleCalc(motor_t *Motor){
    float theta_out = (Motor->angle_1 - *Motor->positionCounts)*(Motor->angle_distance/(Motor->angle_2 - Motor->angle_1));
    return theta_out;
}


float theta1_f;
float theta2_f;
extern MotorSet_t motorSet;


void torqueControl(MotorSet_t motorSet, float W1,float W2){

//Joint Angles
float theta1raw = angleCalc(&motorSet.motors[0]);
float theta2raw = angleCalc(&motorSet.motors[1]);
    
    //Apply Filter
    theta1_f = LowPassFilter(theta1raw,theta1_f, 0.1);
    theta2_f = LowPassFilter(theta2raw,theta2_f, 0.1);

//Torque Set Points // FUTURE REV will calculate this externally

    motorSet.motors[0].pid.setpoint = W1 * cos(theta1_f * PI/180.0f) + W2 * cos((theta1_f + theta2_f) * PI/180.0f);
    motorSet.motors[1].pid.setpoint = -W1 * cos((theta1_f + theta2_f) * PI/180.0f);

//Torque Process In Points

    //T1
    motorSet.motors[0].pid.processIn = *motorSet.motors[0].currentCounts / 4095.0f;

    //T2
    motorSet.motors[1].pid.processIn = *motorSet.motors[1].currentCounts / 4095.0f;

//Execute

    for (int i = 0; i < motorSet.motorCount; i++){
    //Store Direction Value

        float direction = 1;
        if (motorSet.motors[i].pid.setpoint < 0){
            motorSet.motors[i].pid.setpoint = -motorSet.motors[i].pid.setpoint;
            direction = -1;
        }

    //PID Task
        float pwm = PID_task(&motorSet.motors[i].pid,motorSet.motors[i].pid.processIn);
        motorSet.motors[i].pwmCommand = pwm;
        motor_command(&motorSet.motors[i],pwm,direction);
        }   

}




void motor_command(motor_t *Motor, float pwm_command,int Direction){
  //Duty Cycle Input 0 - 1

    //Clipping
    if (pwm_command > 1){
        pwm_command = 1;
    }

    if (pwm_command < -1){
        pwm_command = -1;
    }
    
    float gain = 1; 
    float DutyCycle = pwm_command * gain;

    //Positive Motor Direction
    if (Direction > 0){
        HAL_GPIO_WritePin(Motor->dir_port, Motor->dir_pin,GPIO_PIN_RESET);
    }

    //Negative Direction
    if (Direction <0){
        HAL_GPIO_WritePin(Motor->dir_port, Motor->dir_pin,GPIO_PIN_SET);
    }
    uint32_t pulse = (Motor ->htim -> Init.Period) * DutyCycle;
    __HAL_TIM_SET_COMPARE(Motor->htim, Motor->pwm_channel, pulse);

  }







