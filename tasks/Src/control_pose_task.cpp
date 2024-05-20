/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-02-28 17:20:27
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-05-06 17:21:15
 * @FilePath: \dartrack1\tasks\Src\control_pose_task.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "control_pose_task.hpp"
#include "stm32f4xx_hal.h"
#include "gpio.h"
#include "main.h"
#include "tim.h"
#include "math.h"
#include "rc_task.hpp"
#include "light_receive_task.hpp"
#include "stm32f4xx_hal_tim.h"
#include "stm32f4xx_ll_tim.h"
#include "launch_task.hpp"

#define MOTOR_YAW_CCW  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4,GPIO_PIN_SET )//正转
#define MOTOR_YAW_CW  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4,GPIO_PIN_RESET )//反转
#define MOTOR_PITCH_CCW  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12,GPIO_PIN_SET )
#define MOTOR_PITCH_CW  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_12,GPIO_PIN_RESET )
#define TARGET_YAW -4

uint32_t yaw_output_frequency;//期望输出给步进电机的频率
uint32_t auto_reload_value;//tim4的arr值，用于更改输出给步进电机的频率
uint8_t control_pose_flag=0;//等于1表示已经达到预定yaw
uint8_t change_target_flag;//等于1表示yaw已经从前哨战切到基地附近
uint32_t change_target_wait_count;

//目前转速0.5r/s
void MotorYAWInit(void){
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 500);//PD12
	 //MOTOR_YAW_ENABLE;
}

void MotorPITCHInit(void){
	__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_2, 500);//PD13
	 //MOTOR_YAW_ENABLE;
}

//遥控器控制架子yaw(开环)
void ControlYAW(void){
	if(rc_ptr->rc_lh()>0.05){
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
		MOTOR_YAW_CW;
	}
	else if(rc_ptr->rc_lh()<-0.05){
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
		MOTOR_YAW_CCW;
	}
	else if(fabs(rc_ptr->rc_lh())<0.05){
		HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
	}
}

//遥控器控制架子pitch(开环)
void ControlPITCH(void){
	if(rc_ptr->rc_rv()>0.05){
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
		MOTOR_PITCH_CW;
	}
	else if(rc_ptr->rc_rv()<-0.05){
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
		MOTOR_PITCH_CCW;
	}
	else if(fabs(rc_ptr->rc_rv())<0.05){
		HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_2);
	}
}

// void Control(void){
// 	if((imu.yaw>TARGET_YAW)&&(flag==0)){
// 		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
// 		MOTOR_YAW_CCW;
// 		flag=1;
// 	}else if((imu.yaw<=TARGET_YAW)&&(flag==0)){
// 		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
// 		MOTOR_YAW_CW;
// 		flag=1;
// 	}

// 	if((imu.yaw>YAW_MAX)||(imu.yaw<YAW_MIN)||(fabs(imu.yaw-TARGET_YAW)<YAW_ERROR)){
// 		HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
// 	}
// }

//自瞄控制yaw
void ControlPose(void){
	//实现自瞄时距离越远速度越快
	yaw_output_frequency=(2000.0f-500.0f)/(0.01-0.002)*(fabs(predicted_yaw)-0.002)+500;//线性关系，0.002rad->500hz,0.01rad->1800hz,ps:后续发现2000hz超过步进电机最大频率了，此处为了加块速度。就不改了
	if(yaw_output_frequency>1800){
		yaw_output_frequency=1800;
	}
	else if(yaw_output_frequency<500){
		yaw_output_frequency=500;
	}//将输出频率限制在500-1800hz
	auto_reload_value=__LL_TIM_CALC_ARR(84000000,83,yaw_output_frequency);//根据希望的输出频率计算对应的arr值
	__HAL_TIM_SET_AUTORELOAD(&htim4,auto_reload_value);//更改arr值

	if(predicted_yaw>0.0002){
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
		MOTOR_YAW_CW;
		control_pose_flag=0;
	}else if(predicted_yaw<-0.0002){
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
		MOTOR_YAW_CCW;
		control_pose_flag=0;
	}
	else if(fabs(predicted_yaw)<=0.0002){
		HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
		control_pose_flag=1;
	}
}

//自瞄控制yaw,用于测试自瞄
void ControlPoseTest(void){
	yaw_output_frequency=(2000.0f-500.0f)/(0.01-0.002)*(fabs(predicted_yaw)-0.002)+500;
	if(yaw_output_frequency>1800){
		yaw_output_frequency=1800;
	}
	else if(yaw_output_frequency<500){
		yaw_output_frequency=500;
	}
	auto_reload_value=__LL_TIM_CALC_ARR(84000000,83,yaw_output_frequency);
	__HAL_TIM_SET_AUTORELOAD(&htim4,auto_reload_value);

	if(predicted_yaw>0.0002){
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
		MOTOR_YAW_CW;
	}else if(predicted_yaw<-0.0002){
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
		MOTOR_YAW_CCW;
	}
	else if(fabs(predicted_yaw)<=0.0002){
		HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
	}
}

//用于前哨战模式下，打完前两发后将yaw转到基地附近
void ChangeHitTarget(void){
	//if(change_target_flag==0){
		//__HAL_TIM_SET_AUTORELOAD(&htim4,auto_reload_value);
		change_target_wait_count++;
		HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
		MOTOR_YAW_CW;
	// }
	if(change_target_wait_count>15*FREQUENCE){
		change_target_flag=1;
		HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
	}
}