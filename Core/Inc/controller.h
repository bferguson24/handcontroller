#include "motor.h"

typedef struct {

//x/y/z Motors
motor_t J1;
motor_t J2;
motor_t J3;

//Gimbal Motors
motor_t J4; 
motor_t J5;
motor_t J6;

}controller_t;

extern controller_t controller1;

//Function Declarations:


/**
* @brief Hand Controller Gravity Comp
*
* Gravity compenstation for hand controller. With given inputs
* 
* @param controller Pointer to specific controller object
* @param stiffness stiffness parameter N/m 
* @param damping Damping parameter N/m^2

*               
* @return Error code in future? 
*/
void grav_comp(controller_t *controller, float stiffness, float damping);




/**
* @brief Hand Controller Force Command
*
* Apply additional Force in x/y/z space to hand controller 
* 
* @param controller Pointer to specific controller object
* @param Fx Force in x
* @param Fy Force in y
* @param Fz Force in z
*          
* @return Error code in future? 
*/
void force_input(controller_t *controller, float Fx, float Fy, float Fz);
