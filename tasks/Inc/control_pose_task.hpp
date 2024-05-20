/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-02-28 17:20:55
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-05-06 22:39:19
 * @FilePath: \dartrack1\tasks\Inc\control_pose_task.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __CONTROL_POSE_TASK_HPP__
#define __CONTROL_POSE_TASK_HPP__



/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */


/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

extern uint8_t control_pose_flag;
extern uint8_t change_target_flag;

/* USER CODE BEGIN Prototypes */

void MotorYAWInit(void);
void MotorPITCHInit(void);
void ControlYAW(void);
void ControlPITCH(void);
void ControlPose(void);
void ControlPoseTest(void);
void ChangeHitTarget(void);

/* USER CODE END Prototypes */



#endif
