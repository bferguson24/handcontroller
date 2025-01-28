#include "controller.h"
#include "stdio.h" 



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

void gravComp(controller_t *controller, commands_t status){

    if (status == gc_ON){

        //Calculate Weights
        float WL1 = ML1 * gravity; 
        float WL2 = ML2 * gravity; 
        float Wgimb = MGIMB * gravity; 

        //Get Angles from Controller Object
        float theta1 = controller->J1.theta; 
        float theta2 = controller->J2.theta; 
        float theta3 = controller->J3.theta;
        float theta4 = controller->J4.theta;

        float T1 = 0; 
        float T2 = WL1 * (L1 * cos(theta2))   + WL2 * (L1C * cos(theta2) + L2C * cos(theta2 + theta3)) + Wgimb * (L1 * cos(theta2) + L2 * cos(theta2 + theta3));
        float T3 = WL2 * (L1C * cos(theta2 + theta3))   + Wgimb * (L2 * cos(theta2 + theta3));

        
        torque_control(&controller->J1, T1);
        torque_control(&controller->J2, T2);
        torque_control(&controller->J3, T3);
    }   

    else if (status == gc_OFF){
        printf("GRAV COMP OFF");
    }

}


void positionCalc(controller_t *controller){
    //Get Angles from Controller Object
    float theta1 = controller->J1.theta; 
    float theta2 = controller->J2.theta; 
    float theta3 = controller->J3.theta;
    float theta4 = controller->J4.theta;


//Calculate Position/Orientation

    controller->x = 

    controller->y = 

    controller->z = 

    controller->pitch = theta4;




}

void force_input(controller_t *controller, float Fx, float Fy, float Fz){

    //Add additional motor torque from force inputs to grav comp values:

}