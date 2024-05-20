/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-01-20 20:05:39
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-05-17 23:25:12
 * @FilePath: \dartrack1\launch\Src\task.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/**
 *******************************************************************************
 * @file      : task.cpp
 * @brief     :
 * @history   :
 *  Version     Date            Author          Note
 *  V0.9.0      yyyy-mm-dd      <author>        1. <note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 *  Copyright (c) 2023 Hello World Team, Zhejiang University.
 *  All Rights Reserved.
 *******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
 #include "task.hpp"

// #include <cstring>
#include "control_pose_task.hpp"
#include "rc_task.hpp"
#include "motor_task.hpp"
#include "can.h"
#include "tim.h"
#include "servo_task.hpp"
#include "control_rubberband_task.hpp"
#include "launch_task.hpp"
#include "usart.h"
#include "stm32f4xx_it.h"
#include "reload_dart_task.hpp"
#include "laser_task.hpp"
#include "referee_task.hpp"
#include "light_receive_task.hpp"
#include "buzzer_task.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

uint8_t launch_flag=0;//指示是否需要发射,0表示不需要
uint32_t launch_count=0;//计时，满1秒启动发射，用于指示是否需要发射
uint32_t launch_interval_time=0;
uint8_t send_msg_aerial_flag=0;
uint32_t system_click=0;
uint8_t launch_second_round_flag;
uint8_t shoot_flag=0;
uint32_t shoot_wait_count=0;
/* External variables --------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/

void TaskInit(void)
{
  CAN_FilterInit(&hcan1);
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);
  MotorYAWInit();
  MotorPITCHInit();
  MotorInit();
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);//舵机PWM
  HAL_TIM_PWM_Start(&htim12, TIM_CHANNEL_1);
  LaserTurnOn();
  HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14,GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11,GPIO_PIN_SET);
  //GrabInit();
  RfrDecoderInit();
  RcInit();
  BuzzerInit();
  HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rfr_rx_buffer, 255);
  HAL_UARTEx_ReceiveToIdle_DMA(&huart7, camera_rx_buffer,10);
  //HAL_UART_Receive_IT(&huart7,&camera_rx_buffer,1);
  HAL_TIM_Base_Start_IT(&htim5);
  //HAL_TIM_Base_Start_IT(&htim3);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim==&htim5){
		system_click++;
		if(system_click>15000){
			HAL_GPIO_WritePin(GPIOF, GPIO_PIN_14,GPIO_PIN_RESET);
 			HAL_GPIO_WritePin(GPIOE, GPIO_PIN_11,GPIO_PIN_RESET);
		}
		//BuzzerTask();
		GameTest4_Auto();
		if(shoot_flag==0){
			if(rc_ptr->rc_l_switch()==3){
				if(rc_ptr->rc_r_switch()==1){
					ControlYAW();
					ControlPITCH();
					//send_msg_aerial_flag=1;
					//ServoSetAngle(-90);
				}
				else if(rc_ptr->rc_r_switch()==3){
					HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
					Control2006();
					//send_msg_aerial_flag=0;
					//Control2006Speed();
					//Control2006Position();
					//ServoSetAngle(-10);
				}
				else if(rc_ptr->rc_r_switch()==2&&m3508_set_zeropoint_flag==1){
					HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
					Test();
				}
			}
			else if(rc_ptr->rc_l_switch()==1){
				HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_1);
				if(rc_ptr->rc_r_switch()==1){
					GrabFirstDartTest();
					//ServoSetAngle(-90);
				}
				else if(rc_ptr->rc_r_switch()==3){
					GrabSecondDartTest();
					//ServoSetAngle(-10);
				}
				else if(rc_ptr->rc_r_switch()==2){
					GrabThirdDartTest();
					//ServoSetAngle(-10);
				}
			}
		}
		
		if(rc_ptr->rc_l_switch()==2&&m3508_set_zeropoint_flag==1){
			if((rc_ptr->rc_r_switch()==3)||(rc_ptr->rc_r_switch()==2)){
				//拨轮持续往下拨一秒，启动发射程序
				// if(rc_ptr->rc_wheel()>0.95){
				if((rc_ptr->rc_lh()>0.8)&&(rc_ptr->rc_lv()<-0.8)&&(rc_ptr->rc_rh()<-0.8)&&(rc_ptr->rc_rv()<-0.8)){
					launch_count++;
					if(launch_count>=1*FREQUENCE){
						//launch_flag=1;
						shoot_flag=1;
					}
				}
				else{
					launch_count=0;//未满一秒松手就清零，重新计时
				}
			}
			
			if(rc_ptr->rc_r_switch()==1){
				ControlPoseTest();
				//ServoSetAngle(-90);
			}
			else if(rc_ptr->rc_r_switch()==3){
				//ServoSetAngle(60);
				//GameTest4_Manual();
				// GameTest4_Auto();
			}
			//LaunchTest4();
			else if(rc_ptr->rc_r_switch()==2){
				//GameTest4_Auto();
				//GameTest2_Manual();
				//GameTest4_Manual();
			}
		}
		// GameTest4_Auto();
		RcTask();
	}
}

// //比赛时用的发射流程（能收到裁判系统的数据），还在完善，区域赛不用。
// void GameTest(void){
// 	if(first_round_launch_flag==0){
// 		if(dart_station_info.dart_launch_opening_status==rfr::DartStationStatus::kDartStationStatusMoving){
// 			if(launch_dart_num==0){
// 				//LaunchFirstDart_2();
// 			}
// 		}
// 		if(dart_station_info.dart_launch_opening_status==rfr::DartStationStatus::kDartStationStatusOpened){
// 			// if(control_pose_flag==0){
// 				ControlPose();
// 			//}
// 			if(control_pose_flag==1){
// 				if(launch_dart_num==0){
// 					//LaunchFirstDart_2();
// 				}
// 				else if(launch_dart_num==1){
// 					LaunchSecondDart();
// 				}
// 			}
// 		}
// 	}
// 	else if(first_round_launch_flag==1){
// 		if(dart_station_info.dart_launch_opening_status==rfr::DartStationStatus::kDartStationStatusClosed){
// 			second_round_launch_flag=0;
// 		}
// 		if(dart_station_info.dart_launch_opening_status==rfr::DartStationStatus::kDartStationStatusMoving&&second_round_launch_flag==0){
// 			if(launch_dart_num==2){
// 				//LaunchThirdDart_2();
// 			}
// 		}
// 		if(dart_station_info.dart_launch_opening_status==rfr::DartStationStatus::kDartStationStatusOpened&&second_round_launch_flag==0){
// 			//if(control_pose_flag==0){
// 				ControlPose();
// 			//}
// 			if(control_pose_flag==1){
// 				if(launch_dart_num==2){
// 					//LaunchThirdDart_2();
// 				}
// 				else if(launch_dart_num==3){
// 					LaunchFourthDart();
// 				}
// 			}
// 		}
// 	}
// }

// //手动发射流程,用的时候把上面那段一样的注释掉
// void GameTest2(void){
// 	if(launch_flag==1){
// 		if(control_pose_flag==0)
// 			ControlPose();
// 		else if(control_pose_flag==1){
// 			if(launch_dart_num==0){
// 				LaunchFirstDart();
// 			}
// 			else if(launch_dart_num==1){
// 				LaunchSecondDart();
// 			}
// 			else if(launch_dart_num==2){
// 				LaunchThirdDart();
// 			}
// 			else if(launch_dart_num==3){
// 				LaunchFourthDart();
// 			}
// 		}
// 	}
// }

// //比赛模式,手拨和收到裁判系统都可以发射，4发基地模式
// void GameTest2(void){
// 	if((launch_flag==1)||((dart_station_info.dart_launch_opening_status==rfr::DartStationStatus::kDartStationStatusOpened)&&(com_status_info.game_progress==rfr::CompStage::kCompStageOngoing)&&(com_status_info.stage_remain_time<414))){
// 		if(control_pose_flag==0){
// 			ControlPose();
// 			send_msg_aerial_flag=1;
// 		}
// 		else if(control_pose_flag==1){
// 			if(launch_dart_num==0){
// 				LaunchFirstDart();
// 			}
// 			else if(launch_dart_num==1){
// 				LaunchSecondDart();
// 			}
// 			// else if(launch_dart_num==2&&team_dart_status_info.dart_remaining_time>6){//需要预估时间，之后想办法换个更好的方式
// 			// 	LaunchThirdDart();
// 			// }
// 			else if(launch_dart_num==2){
// 				if(team_dart_status_info.dart_remaining_time>6){
// 					LaunchThirdDart();
// 				}
// 				else if(team_dart_status_info.dart_remaining_time==0){
// 					launch_interval_time++;
// 					if(launch_interval_time>0.38*FREQUENCE){
// 						LaunchThirdDart();
// 					}
// 				}
// 			}
// 			else if(launch_dart_num==3){
// 				LaunchFourthDart();
// 			}
// 		}
// 	}
// 	else if(launch_flag==0&&launch_dart_num==2){
// 		control_pose_flag=0;//在发射完两发后的间隔时间内，将该变量重新置0，以便发射第三发前可以继续自瞄
// 	}
// }

//比赛模式,手拨才可以发射，4发基地模式
void GameTest2_Manual(void){
	if(launch_flag==1){
		if(control_pose_flag==0){
			ControlPose();
		}
		else if(control_pose_flag==1){
			if(launch_dart_num==0){
				LaunchFirstDart();
			}
			else if(launch_dart_num==1){
				LaunchSecondDart();
			}
			else if(launch_dart_num==2){
				LaunchThirdDart();
			}
			else if(launch_dart_num==3){
				LaunchFourthDart();
			}
		}
	}
	else if(launch_flag==0&&launch_dart_num==2){
		control_pose_flag=0;//在发射完两发后的间隔时间内，将该变量重新置0，以便发射第三发前可以继续自瞄
	}
}

// //比赛模式,手拨和收到裁判系统都可以发射,2前哨战+2基地模式。区域赛不用
// void GameTest3(void){
// 	if((launch_flag==1)||((dart_station_info.dart_launch_opening_status==rfr::DartStationStatus::kDartStationStatusOpened)&&(com_status_info.game_progress==rfr::CompStage::kCompStageOngoing)&&(com_status_info.stage_remain_time<414))){
// 		if(control_pose_flag==0){
// 			ControlPose();
// 		}
// 		else if(control_pose_flag==1){
// 			if(launch_dart_num==0){
// 				LaunchFirstDart();
// 			}
// 			else if(launch_dart_num==1){
// 				LaunchSecondDart();
// 			}
// 			// else if(launch_dart_num==2&&team_dart_status_info.dart_remaining_time>6){//需要预估时间，之后想办法换个更好的方式
// 			// 	LaunchThirdDart();
// 			// }
// 			else if(launch_dart_num==2){
// 				if(team_dart_status_info.dart_remaining_time>6){
// 					LaunchThirdDart();
// 				}
// 				else if(team_dart_status_info.dart_remaining_time==0){
// 					launch_interval_time++;
// 					if(launch_interval_time>0.38*FREQUENCE){
// 						LaunchThirdDart();
// 					}
// 				}
				
// 			}
// 			else if(launch_dart_num==3){
// 				LaunchFourthDart();
// 			}
// 		}
// 	}
// 	else if(launch_flag==0&&launch_dart_num==2){
// 		control_pose_flag=0;
// 	}

// 	if(first_round_launch_flag==1){
// 		if(change_target_flag==0){
// 			ChangeHitTarget();
// 		}
		
// 	}
// }

// //比赛模式,手拨和收到裁判系统都可以发射，2固定+2随机
// void GameTest4(void){
// 	if((launch_flag==1)||((dart_station_info.dart_launch_opening_status==rfr::DartStationStatus::kDartStationStatusOpened)&&(com_status_info.game_progress==rfr::CompStage::kCompStageOngoing)&&(com_status_info.stage_remain_time<414))){
// 		if(control_pose_flag==0){
// 			//在发射第三四发前，通过自瞄得到的与目标的偏差角，进行线性预测得到2006应该转到的角度
// 			if(launch_dart_num==2&&m2006_predicted_flag==0){
// 				m2006_predicted_position=m2006_initial_position+predicted_yaw/0.01147f*2.5f*ruler_to_2006position/10.0f;//-280mm->-0.01183rad,280mm->0.01111rad取平均值
// 				m2006_predicted_flag=1;
// 			}
// 			send_msg_aerial_flag=1;
// 			ControlPose();
// 		}
// 		else if(control_pose_flag==1){
// 			if(launch_dart_num==0){
// 				LaunchFirstDart();
// 			}
// 			else if(launch_dart_num==1){
// 				LaunchSecondDart();
// 			}
// 			// else if(launch_dart_num==2&&team_dart_status_info.dart_remaining_time>6){//需要预估时间，之后想办法换个更好的方式
// 			// 	LaunchThirdDart();
// 			// }
// 			else if(launch_dart_num==2){
// 				if(team_dart_status_info.dart_remaining_time>6){
// 					LaunchThirdDart_2();
// 				}
// 				else if(team_dart_status_info.dart_remaining_time==0){
// 					launch_interval_time++;
// 					if(launch_interval_time>0.38*FREQUENCE){
// 						LaunchThirdDart_2();
// 					}
// 				}
// 			}
// 			else if(launch_dart_num==3){
// 				LaunchFourthDart();
// 			}
// 		}
// 	}
// 	// else if((dart_station_info.dart_launch_opening_status==rfr::DartStationStatus::kDartStationStatusClosed)&&(launch_dart_num==2)){
// 	// 	control_pose_flag=0;
// 	// 	m2006_predicted_flag=0;
// 	// }
// 	else if(launch_flag==0&&launch_dart_num==2){
// 		m2006_predicted_flag=0;
// 		control_pose_flag=0;//在发射完两发后的间隔时间内，将两个变量重新置0，以便发射第三发前可以继续自瞄，同时可以预测2006角度
// 	}
// }

//比赛模式,手拨才可以发射，2固定+2随机
void GameTest4_Manual(void){
	if(launch_flag==1){
		if(control_pose_flag==0){
			//在发射第三四发前，通过自瞄得到的与目标的偏差角，进行线性预测得到2006应该转到的角度
			if(launch_dart_num==2&&m2006_predicted_flag==0){
				m2006_predicted_position=m2006_initial_position+predicted_yaw/0.01147f*2.5f*ruler_to_2006position/10.0f;//-280mm->-0.01183rad,280mm->0.01111rad取平均值
				m2006_predicted_flag=1;
			}
			ControlPose();
		}
		else if(control_pose_flag==1){
			if(launch_dart_num==0){
				LaunchFirstDart();
			}
			else if(launch_dart_num==1){
				LaunchSecondDart();
			}
			else if(launch_dart_num==2){
				LaunchThirdDart_2();
			}
			else if(launch_dart_num==3){
				LaunchFourthDart();
			}
		}
	}
	else if(launch_flag==0&&launch_dart_num==2){
		m2006_predicted_flag=0;
		control_pose_flag=0;//在发射完两发后的间隔时间内，将两个变量重新置0，以便发射第三发前可以继续自瞄，同时可以预测2006角度
	}
}

//比赛模式,自动发射，2固定+2随机
void GameTest4_Auto(void){
	if(shoot_flag==1){
		if(control_pose_flag==0){
			//在发射第三四发前，通过自瞄得到的与目标的偏差角，进行线性预测得到2006应该转到的角度
			if(launch_dart_num==2&&m2006_predicted_flag==0){
				m2006_predicted_position=m2006_initial_position+predicted_yaw/0.01147f*2.5f*ruler_to_2006position/10.0f;//-280mm->-0.01183rad,280mm->0.01111rad取平均值
				m2006_fourth_position=m2006_predicted_position+2.5f*ruler_to_2006position/10.0f;
				m2006_predicted_flag=1;
			}
			ControlPose();
		}
		else if(control_pose_flag==1){
			if(launch_dart_num==0){
				LaunchFirstDart();
			}
			else if(launch_dart_num==1){
				LaunchSecondDart();
			}
			else if(launch_dart_num==2){
				LaunchThirdDart_2();
			}
			else if(launch_dart_num==3){
				LaunchFourthDart();
			}
		}
	}
	else if(launch_dart_num==2){
		m2006_predicted_flag=0;
		control_pose_flag=0;//在发射完两发后的间隔时间内，将两个变量重新置0，以便发射第三发前可以继续自瞄，同时可以预测2006角度
		launch_interval_time++;
		if(launch_interval_time>25*FREQUENCE){
			launch_second_round_flag=1;
		}
		if((launch_second_round_flag==1)&&(greenlight_detected_flag==1)&&(com_status_info.game_progress==rfr::CompStage::kCompStageOngoing)){
			shoot_wait_count++;
			if(shoot_wait_count>0.1*FREQUENCE){
				shoot_flag=1;
			}
		}
	}

	if((com_status_info.game_progress==rfr::CompStage::kCompStageOngoing)&&(greenlight_detected_flag==1)&&(launch_dart_num==0)){
		shoot_wait_count++;
		if(shoot_wait_count>0.1*FREQUENCE){
			shoot_flag=1;
		}
	}
}