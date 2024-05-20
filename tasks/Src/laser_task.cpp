#include "laser_task.hpp"
#include "gpio.h"

//打开红点激光器
void LaserTurnOn(void){
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13,GPIO_PIN_SET);
}

//关掉红点激光器
void LaserTurnOff(void){
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_13,GPIO_PIN_RESET);
}