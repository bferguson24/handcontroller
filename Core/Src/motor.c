#include "motor.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>  // Include stdlib.h for memory allocation
#include "arm_math.h"







float angleCalc(motor_t *Motor){
    float theta_raw = (Motor->angle_1 - *Motor->positionCounts)*(Motor->angle_distance/(Motor->angle_2 - Motor->angle_1));
    float theta_filtered = LowPassFilter(theta_raw, Motor->theta_filtered, 0.1);
    Motor->theta_filtered = theta_filtered;
    return theta_filtered;
}


extern MotorSet_t motorSet;


void torqueControl(MotorSet_t motorSet, float W1,float W2){

//Joint Angles
// float theta1raw = angleCalc(&motorSet.motors[0]);
// float theta2raw = angleCalc(&motorSet.motors[1]);
    
    //Apply Filter
    // theta1_f = LowPassFilter(theta1raw,theta1_f, 0.1);
    // theta2_f = LowPassFilter(theta2raw,theta2_f, 0.1);

    //Theta1
    // motorSet.motors[0].theta_filtered = angleCalc(&motorSet.motors[0]);
    // float theta1_f = motorSet.motors[0].theta_filtered; 

    //Theta2 
    // motorSet.motors[1].theta_filtered = angleCalc(&motorSet.motors[1]);
    // float theta2_f = motorSet.motors[1].theta_filtered; 

    for (int i = 0 ; i < motorSet.motorCount; i++){
        angleCalc(&motorSet.motors[i]);
    }


    float theta2_f = motorSet.motors[1].theta_filtered;

    float theta3_f = motorSet.motors[2].theta_filtered;

 
    // theta2_f = angleCalc(&motorSet.motors[1]);
    


//Torque Set Points // FUTURE REV will calculate this externally

    // motorSet.motors[0].pid.setpoint = W1 * cos(theta2_f * PI/180.0f) + W2 * cos((theta2_f + theta3_f) * PI/180.0f);
    // motorSet.motors[1].pid.setpoint = -W1 * cos((theta2_f + theta3_f) * PI/180.0f);

//Torque Process In Points

    //T2
    motorSet.motors[1].pid.processIn = *motorSet.motors[1].currentCounts / 4095.0f;

    //T3
    motorSet.motors[2].pid.processIn = *motorSet.motors[2].currentCounts / 4095.0f;

//Execute

//i = 1 Start at J2/J3 Since 
    for (int i = 1; i < motorSet.motorCount; i++){
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







