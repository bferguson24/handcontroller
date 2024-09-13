#include "motor.h"
#include "stm32f7xx_hal_gpio.h"
// #include "stm32f7"
#include <stdbool.h>

void _write_pwm(motor_t* m, float val)
{
    // HAL_TIM_
}

void _write_dir(motor_t* m, bool dir)
{
    HAL_GPIO_WritePin(m->dir_port, m->dir_pin, dir);
}

void MOTOR_init()
{

}
