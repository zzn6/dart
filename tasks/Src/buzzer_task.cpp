/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-05-15 23:51:26
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-05-16 00:25:22
 * @FilePath: \dartrack1\tasks\Src\buzzer_task.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "buzzer_task.hpp"
#include "tim.h"

buzzer::Buzzer* buzzer_ptr = nullptr;

void BuzzerInit(void){
    buzzer_ptr = new buzzer::Buzzer(
      &htim12, TIM_CHANNEL_1, buzzer::kPlayConfigSinglePlayback,
      &buzzer::kWarningList1B);
}

void BuzzerTask(void){
    if(buzzer_ptr->is_playing()) {
        buzzer_ptr->play();
    }
}
