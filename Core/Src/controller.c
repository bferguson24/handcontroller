#include "controller.h"
#include "fast_math_functions.h"
#include "stdio.h" 
#include "utility.h"



// [mm]
const float L1 = 253;
const float L2 = 300; 
const float L1C = 116.13;
const float L2C = 79.67;

// [g]
const float ML1 = 241.21;
const float ML2 = 144.25; 
const float MGIMB = 398.01;

// [m/s^2]
const float gravity = 9.80665;

void grav_comp(controller_t *controller, commands_t status){

float T2_scale = controller->gc_scale_T2; 
float T3_scale = controller->gc_scale_T3; 



    if (status == gc_ON){

        //Calculate Weights
        float WL1 = ML1 * gravity; 
        float WL2 = ML2 * gravity; 
        float Wgimb = MGIMB * gravity; 

        //Get Angles from Controller Object
        float theta1 = controller->J1.theta * PI / 180;
        float theta2 = controller->J2.theta * PI / 180;
        float theta3 = controller->J3.theta * PI / 180;
        float theta4 = controller->J4.theta * PI / 180;

        float T1 = 0; 
        float T2 = T2_scale * ((WL1 * (L1 * cos(theta2))   + WL2 * (L1C * cos(theta2) + L2C * cos(theta2 + theta3)) + Wgimb * (L1 * cos(theta2) + L2 * cos(theta2 + theta3)))  / (1000*1000));
        float T3 = T3_scale * ((WL2 * ((L1C * cos(theta2 + theta3)))   + Wgimb * (L2 * cos(theta2 + theta3)))   / (1000*1000));

        controller->T2 = T2; 
        controller->T3 = T3;


        
        // torque_control(&controller->J1, T1);
        torque_control(&controller->J2, T2);
        torque_control(&controller->J3, T3);
    }   

    else if (status == gc_OFF){
        printf("GRAV COMP OFF");
    }

} 



void position_calc(controller_t *controller){
    //Get Angles from Controller Object

//Button Status: 



// float triggerDepth = ((*controller->raw_trigger_reading) - (controller->trigger_zero))/(controller->trigger_on - controller->trigger_zero);

float triggerDepth = clip(map(*controller ->raw_trigger_reading, controller->trigger_zero, controller->trigger_on, 0, 1), 0,1);


controller->trigger_normal = triggerDepth;


int clutch = HAL_GPIO_ReadPin(controller->clutch_port, controller->clutch_pin);

controller1.clutch_status = clutch; 





///FIX THIS SHiiiiiiit

//Calculate Raw Angles
float theta1_0 = angle_calc(&controller->J1);
float theta2_0 = angle_calc(&controller->J2);
float theta3_0 = angle_calc(&controller->J3);
float theta4_0 = angle_calc(&controller->J4);

// Offset/Correct Relative Rotations

float theta1 = theta1_0;
float theta2 = theta2_0; 
float theta3 = theta3_0 - theta2_0;
float theta4 = theta4_0 + theta2 + theta3;

//Update Angles
controller->J1.theta = theta1;
controller->J2.theta = theta2;
controller->J3.theta = theta3; 
controller->J4.theta = theta4;

//Calculate Position/Orientation
theta1 = theta1 * PI/180;
theta2 = theta2 * PI/180;
theta3 = theta3 * PI/180;
theta4 = theta4 * PI/180;

//Convert to Mira Frame:

float x = L1 * sin(theta2) + L2 * sin(theta2 + theta3)        - controller->homeX;
float y = - (L1 * cos(theta2) + L1 * cos(theta2 + theta3)) * sin(theta1) - controller->homeY;
float z = (L1 * cos(theta2) + L2 * cos(theta2 + theta3)) * cos(theta1) - controller->homeZ;


controller->x = controller->frameScale * (controller->miraXoff + x);
controller->y = controller->frameScale * (controller->miraYoff + y);
controller->z = controller->frameScale * (controller->miraZoff + z);

controller->pitch = theta4 * (180/PI) - controller->pitchOffset;



}

void force_input(controller_t *controller, float Fx, float Fy, float Fz){

    //Add additional motor torque from force inputs to grav comp values:

}