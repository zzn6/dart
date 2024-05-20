/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-02-28 17:57:23
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-03-16 16:18:18
 * @FilePath: \dartrack1\tasks\Inc\control_rubberband_task.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __CONTROL_RUBBERBAND_TASK_HPP__
#define __CONTROL_RUBBERBAND_TASK_HPP__



/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */


/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

extern float m2006_initial_position;
extern float m2006_second_position;
extern float m2006_third_position;
extern float m2006_fourth_position;
extern float m2006_position_init_flag;
extern float m2006_predicted_position;
extern int8_t m2006_predicted_flag;
extern int16_t m2006_current;

/* USER CODE BEGIN Prototypes */

void Control2006(void);
void Control2006Speed(void);
void Control2006Position(void);
void Change2006PositionT02(void);
void Change2006PositionT03(void);
void Change2006PositionT04(void);
void ChangePositionToRandomTarget(void);
void M2006SpeedPID(float target_speed);
void M2006PositionPID(float target_position);

//void M2006PID(void);

/* USER CODE END Prototypes */



#endif
