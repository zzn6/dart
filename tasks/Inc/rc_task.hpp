/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-02-28 17:51:35
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-03-24 01:32:50
 * @FilePath: \dartrack1\tasks\Inc\rc_task.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef RC_TASK_HPP_
#define RC_TASK_HPP_

/* Includes ------------------------------------------------------------------*/
#include "DT7.hpp"

namespace remote_control = hello_world::remote_control;
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
/* Exported variables --------------------------------------------------------*/

extern remote_control::DT7* rc_ptr;
/* Exported function prototypes ----------------------------------------------*/

void RcInit(void);
void RcTask(void);

#endif /* CPP_DEMO_TASKS_RC_TASK_HPP_ */
