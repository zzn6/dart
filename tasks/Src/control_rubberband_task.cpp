/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-01-20 16:39:58
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-05-11 15:27:50
 * @FilePath: \dartrack1\Motor\Src\M_Motor.c
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "control_rubberband_task.hpp"
#include "rc_task.hpp"
#include "can.h"
#include "motor_task.hpp"
#include "motor.hpp"
#include "pid.hpp"

namespace pid = hello_world::pid;

// const pid::BasicPidParams params_2006={
//   .auto_reset = false,
// //   .across0 = pid::Arocss0Type::kArocss0None,
//   .kp = 3500,
//   .ki = 0,
//   .kd = 0,
//   .max_interval_ms = 5,
//   .inte_anti_windup = pid::InteAntiWindup(true,-10,10),
//   .out_limit = pid::OutLimit(true,-2500,2500),//限制输出电流
//  };

//M2006位控外环参数(得出期望速度)
const pid::BasicPidParams params_2006={
  .auto_reset = false,
//   .across0 = pid::Arocss0Type::kArocss0None,
  .kp = 35,
  .ki = 0,
  .kd = 0,
  .max_interval_ms = 10,
  .inte_anti_windup = pid::InteAntiWindup(false,-10,10),
  .out_limit = pid::OutLimit(true,-48,48),
 };

//M2006速度控制参数(位控内环参数)
const pid::BasicPidParams params_2006_2={
  .auto_reset = false,
//   .across0 = pid::Arocss0Type::kArocss0None,
  .kp = 1000,
  .ki = 0,
  .kd = 0,
  .max_interval_ms = 10,
  .inte_anti_windup = pid::InteAntiWindup(false,0,0),
  .out_limit = pid::OutLimit(true,-9999,9999),
 };


pid::BasicPid pid_2006(params_2006);//位控外环pid
pid::BasicPid pid_2006_2(params_2006_2);//位控内环pid（速度pid）
pid::BasicPid::Datas debug_datas_2006;//位控外环data
pid::BasicPid::Datas debug_datas_2006_2;//位控内环data
int16_t m2006_current;//can发给m2006的电流
float m2006_input_speed;//输入给2006的期望角速度
float m2006_input_position;//输入给2006的期望角度
float m2006_initial_position;//在launchfirstdart()里调用，得出发射第一发时2006的初始位置
float m2006_second_position;//发射第二发时2006的位置，用于梯度预紧力测试
float m2006_third_position;//发射第三发时2006的位置，用于梯度预紧力测试
float m2006_fourth_position;//发射第四发时2006的位置，用于梯度预紧力测试
float m2006_position_init_flag=0;//等于1表示四个梯度位置已经计算完
float m2006_predicted_position;//预测的2006应该转到的角度，用于随机目标击打
int8_t m2006_predicted_flag=-1;//初始设为-1，因为、、、

//遥控器控制M2006，即调节皮筋预紧力(开环)
void Control2006(void){
	uint8_t tx_data[8]={0};
	m2006_current=(rc_ptr->rc_lh())*9999;

	tx_data[4]=(uint8_t)(m2006_current>>8);
	tx_data[5]=(uint8_t)m2006_current;

	CAN_SendMsg(&hcan1,kM3508TxId,tx_data,8);
}

//遥控器控制2006速度，用于测试速度pid参数（不包含can发送）
void Control2006Speed(void){
	m2006_input_speed=(rc_ptr->rc_lh())*(-48.0f);
	M2006SpeedPID(m2006_input_speed);
}

//遥控器控制2006位置，用于测试位置外环pid参数（不包含can发送）
void Control2006Position(void){
	m2006_input_position+=(rc_ptr->rc_lh())*(-0.15f);
	M2006PositionPID(m2006_input_position);
	M2006SpeedPID(m2006_input_speed);
}

////因为丝杆不自锁，需要m2006抵抗外力。现在换成自锁丝杆了，用不到这个了
// void M2006PID(void){
// 	float ref =0; // 参考值
// 	float fdb= m2006_angle; // 反馈值
// 	//float ffd = ...; // 前馈值
// 	float out ; // 存储输出值
// 	// 不使用前馈值进行计算
// 	pid_2006.calc(&ref, &fdb, nullptr, &out);//，error≈ref-fdb
// 	debug_datas_2006=pid_2006.datas();
// 	m2006_current=-(debug_datas_2006.out);//正电流往上

// 	uint8_t tx_data[8]={0};
// 	tx_data[4]=(uint8_t)(m2006_current>>8);
// 	tx_data[5]=(uint8_t)m2006_current;
// 	CAN_SendMsg(&hcan1,kM3508TxId,tx_data,8);
// }

//2006速度pid计算
void M2006SpeedPID(float target_speed){
	float ref =target_speed; // 参考值
	float fdb= m2006_velocity; // 反馈值
	//float ffd = ...; // 前馈值
	float out ; // 存储输出值
	// 不使用前馈值进行计算
	pid_2006_2.calc(&ref, &fdb, nullptr, &out);//，error≈ref-fdb
	debug_datas_2006_2=pid_2006_2.datas();
	m2006_current=-(debug_datas_2006_2.out);//正电流往上
	// uint8_t tx_data[8]={0};
	// tx_data[4]=(uint8_t)(m2006_current>>8);
	// tx_data[5]=(uint8_t)m2006_current;
	// CAN_SendMsg(&hcan1,kM3508TxId,tx_data,8);
}

//2006位置外环pid计算
void M2006PositionPID(float target_position){
	float ref =target_position; // 参考值
	float fdb= m2006_angle; // 反馈值
	//float ffd = ...; // 前馈值
	float out ; // 存储输出值
	// 不使用前馈值进行计算
	pid_2006.calc(&ref, &fdb, nullptr, &out);//，error≈ref-fdb
	debug_datas_2006=pid_2006.datas();
	m2006_input_speed=debug_datas_2006.out;
}

//发射完第1发后将2006移到第二个位置，用于梯度预紧力测试
void Change2006PositionT02(void){
	M2006PositionPID(m2006_second_position);
	M2006SpeedPID(m2006_input_speed);
}

//发射完第2发后将2006移到第三个位置，用于梯度预紧力测试
void Change2006PositionT03(void){
	M2006PositionPID(m2006_third_position);
	M2006SpeedPID(m2006_input_speed);
}

//发射完第3发后将2006移到第四个位置，用于梯度预紧力测试
void Change2006PositionT04(void){
	M2006PositionPID(m2006_fourth_position);
	M2006SpeedPID(m2006_input_speed);
}

//将2006的位置移到击打随机目标的位置,用于2固定+2随机模式
void ChangePositionToRandomTarget(void){
	M2006PositionPID(m2006_predicted_position);
	M2006SpeedPID(m2006_input_speed);
}