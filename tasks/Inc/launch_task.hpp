/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-01-20 16:39:58
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-05-12 19:00:02
 * @FilePath: \dartrack1\launch\Inc\launch.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __LAUNCH_TASK_HPP__
#define __LAUNCH_TASK_HPP__

// #ifdef __cplusplus
// extern "C" {
// #endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */


/* USER CODE BEGIN Private defines */

#define FREQUENCE 500.0f//程序调用PID的频率

typedef struct {
	 float kp1;//下拉参数
	 float ki1;//下拉参数
	 float kd1;//下拉参数

	 float kp2;//上拉参数
	 float ki2;//上拉参数
	 float kd2;//上拉参数
} PID_Params;

//发射状态enum
enum Stage_t{
	PULLDOWM = 0,//两边同步带下拉
	PULLUP=1,//两边同步带上拉（包括缓慢发射的情况）

	LAUNCH=2,//快速发射

	END=3
};

extern enum Stage_t launch_stage;
extern uint16_t launch_dart_num;
//extern uint8_t launch_init_flag;
extern uint8_t first_round_launch_flag;
extern int8_t second_round_launch_flag;
extern const float ruler_to_2006position;
/* USER CODE END Private defines */

/* USER CODE BEGIN Prototypes */

// void Launch(void);
void Test(void);
// void LaunchTest(void);
// void LaunchTest2(void);
// void LaunchTest3 (void);
void LaunchTest4 (void);
void LaunchTest5(void);
void LaunchTest6(void);
void LaunchFirstDart(void);
//void LaunchFirstDart_2(void);
void LaunchSecondDart(void);
void LaunchThirdDart(void);
void LaunchThirdDart_2(void);
void LaunchFourthDart(void);
//void LaunchInit(void);
/* USER CODE END Prototypes */

// #ifdef __cplusplus
// }
// #endif

#endif
