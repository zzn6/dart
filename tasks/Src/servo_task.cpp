/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-02-28 17:31:47
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-04-17 19:37:07
 * @FilePath: \dartrack1\tasks\Src\servo_task.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "servo_task.hpp"
#include "tim.h"

#define ARR 2000//定时器里设定的ARR的值

//设定舵机角度，范围-90度到90度
void ServoSetAngle(float angle)
{
  uint16_t num_CCR = ARR * 0.075 + angle * 200 / 180; // CCR=150->0°,CCR=250->90°,CCR=50->-90°
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, num_CCR);//PA2
}