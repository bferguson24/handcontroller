#include "motor.h"
#include "stm32f7xx_hal_gpio.h"
// #include "stm32f7"
#include <stdbool.h>





static int a;

//Static Functions/Variables Only accessible inside compilation unit (motor.c)

static void _write_pwm(motor_t* m, float val)
{
    a = 5;
    // HAL_TIM_
}

static void _write_dir(motor_t* m, bool dir)
{
    HAL_GPIO_WritePin(m->dir_port, m->dir_pin, dir);
}

void MOTOR_init()
{

}

int motor_geta(){
        return a;
}
