/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-04-26 22:56:47
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-05-16 23:13:48
 * @FilePath: \dartrack1\tasks\Inc\light_receive_task.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __LIGHT_RECEIVE_TASK_HPP__
#define __LIGHT_RECEIVE_TASK_HPP__

// #ifdef __cplusplus
// extern "C" {
// #endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */


/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

extern float predicted_yaw;
extern float predicted_pitch;
// extern int16_t shoot_flag;
// extern uint8_t target_num_list;
extern uint8_t greenlight_detected_flag;

/* USER CODE BEGIN Prototypes */

void LightInfoReceiveCheck(void);
void LightDataProcess(void);

/* USER CODE END Prototypes */

// #ifdef __cplusplus
// }
// #endif

#endif
