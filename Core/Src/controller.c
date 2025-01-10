#include "controller.h"

void gravComp(controller_t *controller, float stiffness, float damping){
    float T1;
    float T2;
    float T3;

    torque_control(&controller->J1, T1);
    torque_control(&controller->J2, T2);
    torque_control(&controller->J3, T3);

}

void force_input(controller_t *controller, float Fx, float Fy, float Fz){

    //Add additional motor torque from force inputs to grav comp values:

}