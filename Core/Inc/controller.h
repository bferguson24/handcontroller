#pragma once
#include "motor.h"


typedef enum {
    gc_ON = 1,
    gc_OFF = 0 

} commands_t;

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


commands_t gc_status;
float gc_scale_T2; 
float gc_scale_T3; 

float trigger_zero; 
float trigger_on; 
float trigger_normal;

float T1;
float T2; 
float T3; 


//Kinematics:
float homeX;
float homeY;
float homeZ;

float miraXoff; 
float miraYoff;
float miraZoff;


float frameScale;
float pitchOffset;

int clutch_status;
GPIO_TypeDef *clutch_port;
uint16_t clutch_pin;



int16_t *raw_trigger_reading; 



}controller_t;




extern controller_t controller1;


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
void grav_comp(controller_t *controller, commands_t status);




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


void position_calc(controller_t *controller);
