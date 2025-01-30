#include "motor.h"
#include <stdint.h>
#include <stdbool.h>
#include <stdlib.h>  // Include stdlib.h for memory allocation
#include "arm_math.h"
#include <stdio.h>








float angle_calc(motor_t *Motor){
    float theta_raw = (Motor->angle_start - *Motor->positionCounts)*(Motor->angle_range/(Motor->angle_end - Motor->angle_start));
    float theta_filtered = LowPassFilter(theta_raw, Motor->theta, 0.9);
    // Motor->theta = theta_filtered;
    return theta_raw;
}


extern MotorSet_t motorSet;


void torque_control(motor_t *Motor, float torque){

int status;
int dir; 

//Check Direction and store dir variable        //      if Torque == 0, turn motor OFF
int motorState = (torque == 0) ? (status = MOTOR_OFF) : (status = MOTOR_ON);

torque = (torque < 0) ? (dir = CW, -torque) : (dir = CCW, torque);

if (motorState == MOTOR_ON){
    //Current -> Torque:
    //Kt in Nm / A
    float kt = 0.0965;
    float N = 10;

    //Corrected Friction Torque
    volatile float T_corr = torque + (Motor->frictionGain * torque + Motor->frictionOffset);
    volatile float currentSet = T_corr / (kt * N);

    //Make these parameters motor specific later

    Motor->pid.setpoint = currentSet;
    volatile float currentMeasured = *(Motor->currentCounts) * (3.3/4095.0f) * (1/10.0) * (1/Motor->Rsense);

    Motor->pid.processIn = currentMeasured;
    volatile float pwm_command = PID_task(&Motor->pid, Motor->pid.processIn);

    Motor->pwmCommand = pwm_command;

    //Added Flip Flop motorDir Parameter to account for motor direction w different cables

    //Sign Change if needed
    dir = dir * Motor->motorDir; 

    //Flip dir to 0 or 1;
    dir = (dir < 0) ? (dir == 0) : (dir == 1);

    pwm_set(Motor, pwm_command, dir);
}

else{
    pwm_set(Motor, 0, Motor->motorDir * dir);
    printf("MOTOR OFF");
    
}

};





//Different PCB Definitions:

// pcb_t board1 = {
//     .frictionGain = 0.8127,
//     .frictionOffset = 1.6,
//     .Rsense = 0.0101827 
// };


// void current_control(motor_t *Motor, float current){

// Motor->pid.setpoint = current;
// volatile float currentMeasured = *(Motor->currentCounts) * (3.3/4095.0f) * (1/10.0) * (1/0.0108811);

// Motor->pid.processIn = currentMeasured;
// volatile float pwm_command = PID_task(&Motor->pid, Motor->pid.processIn);

// Motor->pwmCommand = pwm_command;
// pwm_set(Motor, pwm_command, direction);

// };









// void torqueControl(MotorSet_t* motorSet, float W1,float W2){

// //Joint Angles
// // float theta1raw = angleCalc(&motorSet.motors[0]);
// // float theta2raw = angleCalc(&motorSet.motors[1]);
    
//     //Apply Filter
//     // theta1_f = LowPassFilter(theta1raw,theta1_f, 0.1);
//     // theta2_f = LowPassFilter(theta2raw,theta2_f, 0.1);

//     //Theta1
//     // motorSet.motors[0].theta_filtered = angleCalc(&motorSet.motors[0]);
//     // float theta1_f = motorSet.motors[0].theta_filtered; 

//     //Theta2 
//     // motorSet.motors[1].theta_filtered = angleCalc(&motorSet.motors[1]);
//     // float theta2_f = motorSet.motors[1].theta_filtered; 

//     for (int i = 0 ; i < motorSet->motorCount; i++){
//         angleCalc(&motorSet->motors[i]);
//     }


//     float theta2_f = motorSet->motors[1].theta_filtered;

//     float theta3_f = motorSet->motors[2].theta_filtered;

 
//     // theta2_f = angleCalc(&motorSet.motors[1]);
    


// //Torque Set Points // FUTURE REV will calculate this externally

//     motorSet->motors[1].pid.setpoint = W1 * cos(theta2_f * PI/180.0f) + W2 * cos((theta2_f + theta3_f) * PI/180.0f);
//     motorSet->motors[2].pid.setpoint = -W2 * cos((theta2_f + theta3_f) * PI/180.0f);
    
// //Torque Process In Points
 
//     //T2
//     motorSet->motors[1].pid.processIn = *motorSet->motors[1].currentCounts / 4095.0f;

//     //T3
//     motorSet->motors[2].pid.processIn = *motorSet->motors[2].currentCounts / 4095.0f;

// //Execute

// //i = 1 Start at J2/J3 Since 
//     for (int i = 1; i < 2; i++){
//     //Store Direction Value

//         float direction = 1;
//         if (motorSet->motors[i].pid.setpoint < 0){
//             motorSet->motors[i].pid.setpoint = -motorSet->motors[i].pid.setpoint;
//             direction = -1;
//         }

//     //PID Task
//         float pwm = PID_task(&motorSet->motors[i].pid,motorSet->motors[i].pid.processIn);
//         motorSet->motors[i].pwmCommand = pwm;
//         motor_command(&motorSet->motors[i],pwm,direction);
//         }   

//     // motorSet->motors[1].pwmCommand = W1;

//     // motorSet->motors[2].pwmCommand = W2;
    
//     // motor_command(&motorSet->motors[1],motorSet->motors[1].pwmCommand,1);

//     // motor_command(&motorSet->motors[2],motorSet->motors[2].pwmCommand,-1);

// }



void pwm_set(motor_t *Motor, float pwm_command, int direction){

// //Determine Direction of pwm command
// int dir;

// //Check Direction of pwm_command and store dir variable
// pwm_command = (pwm_command < 0) ? (dir = CW, -pwm_command) : (dir = CCW, pwm_command);

// //Clip pwm_command between 0.01 and 1.0
// pwm_command = (pwm_command > 1.0f) ? 1.0f : (pwm_command < 0.01) ? 0.01f : pwm_command;


//Set Motor PWM & Direction Pins
HAL_GPIO_WritePin(Motor->dir_port, Motor->dir_pin,direction);

uint32_t pulse = (Motor ->htim -> Init.Period) * pwm_command;
__HAL_TIM_SET_COMPARE(Motor->htim, Motor->pwm_channel, pulse);
}





//     // //Clipping
//     // if (pwm_command > 1){
//     //     pwm_command = 1;
//     // }

//     // if (pwm_command < -1){
//     //     pwm_command = -1;
//     // }
    
//     // float gain = 1; 
//     float DutyCycle = pwm_command * gain;

//     //Positive Motor Direction
//     if (Direction > 0){
//         HAL_GPIO_WritePin(Motor->dir_port, Motor->dir_pin,GPIO_PIN_RESET);
//     }

//     //Negative Direction
//     if (Direction <0){
//         HAL_GPIO_WritePin(Motor->dir_port, Motor->dir_pin,GPIO_PIN_SET);
//     }
//     uint32_t pulse = (Motor ->htim -> Init.Period) * DutyCycle;
//    __HAL_TIM_SET_COMPARE(Motor->htim, Motor->pwm_channel, pulse);

//   }







