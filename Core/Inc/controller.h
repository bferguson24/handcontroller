#pragma once
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

float x; 
float y; 
float z;
float pitch;

}controller_t;

extern controller_t controller1;


typedef enum {
    gc_ON = 1,
    gc_OFF = 0 

} commands_t;


//Constants:


extern const float L1;
extern const float L2;
extern const float LC1;
extern const float LC2;
extern const float ML1;
extern const float ML2;
extern const float MGIMB;
extern const float gravity;


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


void positionCalc(controller_t *controller);
