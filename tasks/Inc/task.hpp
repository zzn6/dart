/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-01-20 20:05:49
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-01-20 20:21:03
 * @FilePath: \dartrack1\launch\Inc\task.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __TASK_HPP_
#define __TASK_HPP_

#ifdef __cplusplus
extern "C" {
#endif
/* Includes ------------------------------------------------------------------*/
#include "main.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/

extern uint8_t launch_flag;
extern uint32_t launch_count;
extern uint32_t system_click;
extern uint8_t shoot_flag;
extern uint32_t shoot_wait_count;

/* Exported function prototypes ----------------------------------------------*/

void TaskInit(void);
// void GameTest(void);
// void GameTest2(void);
void GameTest2_Manual(void);
// void GameTest4(void);
void GameTest4_Manual(void);
void GameTest4_Auto(void);

#ifdef __cplusplus
}
#endif

#endif /* CPP_DEMO_TASKS_TASK_HPP_ */