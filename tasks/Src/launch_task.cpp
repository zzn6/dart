#include "launch_task.hpp"
#include "motor_task.hpp"
#include "can.h"
#include "math.h"
#include "servo_task.hpp"
#include "rc_task.hpp"
#include "control_rubberband_task.hpp"
#include "task.hpp"
#include "reload_dart_task.hpp"
#include "referee_task.hpp"
#include "control_pose_task.hpp"

#define TOTAL_RAD 49.20//42.80//从最上面拉到最下面电机输出端转过的角度
#define TOTAL_PULLDOWN_TIME 2.2f//期望下拉花费的时间，单位：s
#define TOTAL_PULLUP_TIME 1.7f//期望上拉花费的时间，单位：s
// #define FREQUENCE 500.0f//程序调用PID的频率

//static const float target_speed= 4.76;//下拉速度rad/s,对应10cm/s,r=2.1cm
static const float kDeltaDownRad=TOTAL_RAD/(TOTAL_PULLDOWN_TIME*FREQUENCE);//程序每次进tim中断后增加的角度（用于下拉时的位置控制）
static const float kDeltaUpRad=TOTAL_RAD/(TOTAL_PULLUP_TIME*FREQUENCE);//程序每次进tim中断后减小的角度（用于上拉时的位置控制）
static const float kReductionRatio= 19.0;//减速比
static const float kMotorRadius=1.7;//2.1;//同步带轮半径,单位cm

int32_t launch_wait_count=0;//用于计时的参数.用于发射流程
float target_rad=0;//同步带位置控制时pid追踪的目标位置（随kDeltaRad的累加或者累减不断改变）
int16_t m3508_current[2];//pid得出的期望输入电流
float m3508_angle_speed[2];//位控外环pid得出的期望同步带速度
enum Stage_t launch_stage=PULLDOWM;//指示发射进度,初始化为下拉

uint8_t m3508_1_disable_flag=0;//指示3508是否需要停止，用于id1，等于1表示需要停止
uint8_t m3508_2_disable_flag=0;//指示3508是否需要停止，用于id2，等于1表示需要停止
uint8_t servo_reset_flag=0;//指示舵机是否需要回到零位，等于0表示需要回到零位
uint8_t current_k=0;//两个电机输出电流差值乘上的系数,现在用不到这个了，设为0，懒得删了

//uint8_t launch_init_flag=0;//蓄力是否完成，1表示蓄力完成,该参数用于比赛模式
uint8_t first_round_launch_flag=0;//第一轮发射是否完毕,该参数用于比赛模式
int8_t second_round_launch_flag=-1;//第二轮发射是否完毕,该参数用于比赛模式

uint16_t launch_dart_num=0;//已经发射的飞镖数量

const float ruler_to_2006position=31.4;//标尺上10mm对应的2006角度的增量

//速度控制pid参数
const PID_Params PID ={
	 .kp1=75,
	 .ki1=0.5,
	 .kd1=0,

//	 .kp2=50,
//	 .ki2=0,
//	 .kd2=0,
	 .kp2=75,
	 .ki2=0.5,
	 .kd2=0,
} ;

//位控外环参数
const PID_Params PID_2 ={
	 .kp1=0.06,
	 .ki1=0.002,//<0.02
	 .kd1=0,

	 .kp2=0.05,
	 .ki2=0,
	 .kd2=0,
} ;

//位控内环参数
const PID_Params PID_3 ={
	 .kp1=75,
	 .ki1=0.5,
	 .kd1=0,

	 .kp2=75,
	 .ki2=0.5,
	 .kd2=0,
} ;

float previous_error_3[2]={0,0};
float integral_3[2]={0,0};
float previous_error_2[2]={0,0};
float integral_2[2]={0,0};

static void PullPID(float pulldown_speed,uint8_t pid_params_state);
static void PullPID2(uint8_t pid_params_state);
static void PullPID3(uint8_t pid_params_state);
static void PullDown(void);
static void PullUp(void);
//static void PullUpFirstDart(void);
static void Launch(void);

//上拉时如果两侧没有到位，可以通过遥控器手动复位，为开环控制
void Test(void){
	m3508_current[0]=(int16_t)((rc_ptr->rc_lh())*2500);
	m3508_current[1]=(int16_t)((rc_ptr->rc_rh())*2500);
	uint8_t tx_data[8]={0};
	tx_data[0]=(uint8_t)(m3508_current[0]>>8);
	tx_data[1]=(uint8_t)m3508_current[0];
	tx_data[2]=(uint8_t)(m3508_current[1]>>8);
	tx_data[3]=(uint8_t)m3508_current[1];
	CAN_SendMsg(&hcan1,0x200,tx_data,8);
}

// //按步骤发射
// void LaunchTest(void){
// 	if(launch_flag==0&&rc_ptr->rc_r_switch()==1){
// 		ServoSetAngle(-85);//扳机复位
// 		PullDown(10);
// 		launch_wait_count++;
// 		if(launch_wait_count>1000)//计时一秒，防止刚开始速度过小直接进入下一阶段
// 		{
// 		if((fabs(M3508_state_1.angular_velocity)<0.8)&&(fabs(M3508_state_2.angular_velocity)<0.8)){
// 			//launch_wait_count_2++;
// 			uint8_t tx_data[8]={0};
// 			CAN_SendMsg(&hcan1,0x200,tx_data,8);
// 			//if(launch_wait_count_2>1500){
// 				launch_flag=1;
// 				launch_wait_count=0;
// 				//launch_wait_count_2=0;
// 			//}
// 		}
// 		}
// 	}
// 	else if(launch_flag==1&&rc_ptr->rc_r_switch()==3){
// //		ServoSetAngle(-40);//扳机下拉，发射
// 		PullUp(-10);
// 		launch_wait_count++;
// 		if(launch_wait_count>1000)
// 		{
// 			//ServoSetAngle(-85);//扳机复位
// 		if((fabs(M3508_state_1.angular_velocity)<0.5)&&(fabs(M3508_state_2.angular_velocity)<0.5)){
// 			launch_flag=2;
// 			launch_wait_count=0;
// 			uint8_t tx_data[8]={0};
// 			CAN_SendMsg(&hcan1,0x200,tx_data,8);
// 		}
// 		}
// 	}
// 	else if(launch_flag==2&&rc_ptr->rc_r_switch()==2){
// 		ServoSetAngle(-40);//扳机下拉，发射
// 		//PullUp(-10);
// 		launch_wait_count++;
// 		if(launch_wait_count>1000)
// 		{
// 			ServoSetAngle(-85);//扳机复位
// 			launch_flag=0;
// 			launch_wait_count=0;;
// //		if((fabs(M3508_state_1.angular_velocity)<0.5)&&(fabs(M3508_state_2.angular_velocity)<0.5)){
// //			launch_flag=2;
// //			launch_wait_count=0;
// //			uint8_t tx_data[8]={0};
// //			CAN_SendMsg(&hcan1,0x200,tx_data,8);
// //		}
// 		}
// 	}
// }

// //带负载发射
// void LaunchTest2(void){
// 	if(launch_flag==0&&rc_ptr->rc_r_switch()==1){
// 		ServoSetAngle(-85);//扳机复位
// 		PullDown(10);
// 		launch_wait_count++;
// 		if(launch_wait_count>1000)//计时一秒，防止刚开始速度过小直接进入下一阶段
// 		{
// 		if((fabs(M3508_state_1.angular_velocity)<0.8)&&(fabs(M3508_state_2.angular_velocity)<0.8)){
// 			//launch_wait_count_2++;
// 			uint8_t tx_data[8]={0};
// 			CAN_SendMsg(&hcan1,0x200,tx_data,8);
// 			//if(launch_wait_count_2>1500){
// 				launch_flag=1;
// 				launch_wait_count=0;
// 				//launch_wait_count_2=0;
// 			//}
// 		}
// 		}
// 	}
// 	else if(launch_flag==1&&rc_ptr->rc_r_switch()==3){
// 		ServoSetAngle(-40);//扳机下拉，发射
// 		PullUp(-10);
// 		launch_wait_count++;
// 		if(launch_wait_count>1000)
// 		{
// 			ServoSetAngle(-85);//扳机复位
// 		if((fabs(M3508_state_1.angular_velocity)<0.5)&&(fabs(M3508_state_2.angular_velocity)<0.5)){
// 			launch_flag=0;
// 			launch_wait_count=0;
// 			uint8_t tx_data[8]={0};
// 			CAN_SendMsg(&hcan1,0x200,tx_data,8);
// 		}
// 		}
// 	}
// }
 
// //位控下拉,缓慢发射
// void LaunchTest3 (void){
// 	if(rc_ptr->rc_r_switch()==1&&launch_stage==PULLDOWM){
// 		ServoSetAngle(-85);//扳机复位
// 		target_rad+=kDeltaDownRad;
// 		launch_wait_count++;
// 		//留1.5秒，防止一开始就因为速度太小被错误判定；同时采用位置和速度双判定，哪个电机先到位哪个就先停止运动
// 		if(launch_wait_count>750)
// 		{
// 			if((m3508_angle[0]>TOTAL_RAD)||(fabs(M3508_state_1.angular_velocity)<0.5)){
// 				m3508_1_disable_flag=1;
// 			}
// 			if((m3508_angle[1]>TOTAL_RAD)||(fabs(M3508_state_2.angular_velocity)<0.5)){
// 				m3508_2_disable_flag=1;
// 			}
// 			if(m3508_1_disable_flag==1&&m3508_2_disable_flag==1){
// 				launch_stage=PULLUP;
// 				launch_wait_count=0;
// 				target_rad=TOTAL_RAD;
// 			}
// 		}
// 		PullPID2(1);
// 		PullPID3(1);
// 		uint8_t tx_data[8]={0};
// 		tx_data[0]=(uint8_t)(m3508_current[0]>>8);
// 		tx_data[1]=(uint8_t)m3508_current[0];
// 		tx_data[2]=(uint8_t)(m3508_current[1]>>8);
// 		tx_data[3]=(uint8_t)m3508_current[1];
// 		CAN_SendMsg(&hcan1,0x200,tx_data,8);
// 		if(launch_stage==PULLUP){
// 			m3508_1_disable_flag=0;
// 			m3508_2_disable_flag=0;
// 		}
// 	}
// 	else if(rc_ptr->rc_r_switch()==3&&launch_stage==PULLUP){
// 		target_rad-=kDeltaUpRad;
// 		launch_wait_count++;

// 		if(launch_wait_count>750)
// 		{
// 			ServoSetAngle(-85);//扳机复位
// 			servo_reset_flag=1;
// 			if((m3508_angle[0]<0)||(fabs(M3508_state_1.angular_velocity)<0.5)){
// 				m3508_1_disable_flag=1;
// 			}
// 			if((m3508_angle[1]<0)||(fabs(M3508_state_2.angular_velocity)<0.5)){
// 				m3508_2_disable_flag=1;
// 			}
// 			if(m3508_1_disable_flag==1&&m3508_2_disable_flag==1){
// 				launch_stage=PULLDOWM;
// 				launch_wait_count=0;
// 				target_rad=0;
				
// 			}
// 		}

// 		PullPID2(2);
// 		PullPID3(2);
// 		uint8_t tx_data[8]={0};
// 		tx_data[0]=(uint8_t)(m3508_current[0]>>8);
// 		tx_data[1]=(uint8_t)m3508_current[0];
// 		tx_data[2]=(uint8_t)(m3508_current[1]>>8);
// 		tx_data[3]=(uint8_t)m3508_current[1];
// 		CAN_SendMsg(&hcan1,0x200,tx_data,8);
// 		if(servo_reset_flag==0){
// 			ServoSetAngle(-40);//扳机下拉，发射
// 		}
// 		if(launch_stage==PULLDOWM){
// 			m3508_1_disable_flag=0;
// 			m3508_2_disable_flag=0;
// 			servo_reset_flag=0;
// 		}
// 	}
// }

//位控下拉,直接发射,用于平时发射单发飞镖的测试，不包含机械臂的运动
void LaunchTest4 (void){
	// M2006PID();
	if(launch_stage==PULLDOWM){
		ServoSetAngle(-90);//扳机复位
		target_rad+=kDeltaDownRad;
		if(target_rad>TOTAL_RAD){
			target_rad=TOTAL_RAD;
		}
		//检测运动到指定位置后过0.2s后进入下一发射阶段（留0.2s是因为存在惯性，需要时间来收敛到指定位置）
		if((fabs(m3508_angle[0]-TOTAL_RAD)<0.02)&&(fabs(m3508_angle[1]-TOTAL_RAD)<0.02)){
			launch_wait_count++;
			if(launch_wait_count>0.2*FREQUENCE){
				launch_stage=PULLUP;
				launch_wait_count=0;
				target_rad=TOTAL_RAD;
			    m3508_2_disable_flag=1;
		    	m3508_1_disable_flag=1;
			}
		}
		PullPID2(1);
		PullPID3(1);
		if(launch_stage==PULLUP){
			m3508_1_disable_flag=0;
			m3508_2_disable_flag=0;
     		previous_error_2[0]=previous_error_2[1]=0;
			previous_error_3[0]=previous_error_3[1]=0;
			integral_2[0]=integral_2[1]=0;
			integral_3[0]=integral_3[1]=0;
		}
	}
	else if(launch_stage==PULLUP){
	    target_rad-=kDeltaUpRad;
		if(target_rad<0){
			target_rad=0;
		}
		if((m3508_angle[0]<0.02)&&(m3508_angle[1]<0.02)){
			launch_wait_count++;
			if(launch_wait_count>0.2*FREQUENCE){
				launch_stage=LAUNCH;
		 		launch_wait_count=0;
				target_rad=0;
				m3508_2_disable_flag=1;
		    	m3508_1_disable_flag=1;
			}
		}
		PullPID2(2);
		PullPID3(2);
		if(launch_stage==LAUNCH){
			m3508_1_disable_flag=0;
			m3508_2_disable_flag=0;
			previous_error_2[0]=previous_error_2[1]=0;
			previous_error_3[0]=previous_error_3[1]=0;
			integral_2[0]=integral_2[1]=0;
			integral_3[0]=integral_3[1]=0;
		}
	}
	else if(launch_stage==LAUNCH){
		ServoSetAngle(60);//扳机下拉，发射
		launch_wait_count++;
		if(launch_wait_count>0.8*FREQUENCE)
		{
			ServoSetAngle(-90);//扳机复位
			launch_stage=PULLDOWM;
			launch_wait_count=0;
			launch_count=0;
			launch_flag=0;
			launch_dart_num++;
		}
	}
}

//先快后慢测试。测试效果不好，暂时不用
void LaunchTest5 (void){
	if(rc_ptr->rc_r_switch()==1&&launch_stage==PULLDOWM){
		ServoSetAngle(-85);//扳机复位
		target_rad+=kDeltaDownRad;
		//launch_wait_count++;
		if(target_rad>TOTAL_RAD){
			target_rad=TOTAL_RAD;
		}
		if((fabs(m3508_angle[0]-TOTAL_RAD)<0.02)&&(fabs(m3508_angle[1]-TOTAL_RAD)<0.02)){
			launch_wait_count++;
			if(launch_wait_count>0.8*FREQUENCE){
				launch_stage=PULLUP;
				launch_wait_count=0;
				target_rad=TOTAL_RAD;
			}
		}
		PullPID2(1);
		PullPID3(1);
		if(launch_stage==PULLUP){
			// m3508_1_disable_flag=0;
			// m3508_2_disable_flag=0;
     		previous_error_2[0]=previous_error_2[1]=0;
			previous_error_3[0]=previous_error_3[1]=0;
			integral_2[0]=integral_2[1]=0;
			integral_3[0]=integral_3[1]=0;
		}
	}
	else if(rc_ptr->rc_r_switch()==3&&launch_stage==PULLUP){
		launch_wait_count++;
		if(launch_wait_count>=(FREQUENCE*TOTAL_PULLUP_TIME)){
			launch_wait_count=FREQUENCE*TOTAL_PULLUP_TIME;
		}
		target_rad=(TOTAL_PULLUP_TIME-launch_wait_count/FREQUENCE)*(TOTAL_PULLUP_TIME-launch_wait_count/FREQUENCE)*TOTAL_RAD/((TOTAL_PULLUP_TIME)*(TOTAL_PULLUP_TIME));
	    //target_rad-=kDeltaUpRad;

		// if(target_rad<0){
		// 	target_rad=0;
		// }
		// if(launch_wait_count>750)
		// {
		// 	// if((m3508_angle[0]<0)||(fabs(M3508_state_1.angular_velocity)<0.5)){
		// 	// 	m3508_1_disable_flag=1;
		// 	// }
		// 	// if((m3508_angle[1]<0)||(fabs(M3508_state_2.angular_velocity)<0.5)){
		// 	// 	m3508_2_disable_flag=1;
		// 	// }
		// 	// if(m3508_1_disable_flag==1&&m3508_2_disable_flag==1){
		// 	// 	launch_stage=LAUNCH;
		// 	// 	launch_wait_count=0;
		// 	// 	target_rad=0;
		// 	// }

		// 	// if((fabs()<0.05)){
		// 	// 	m3508_1_disable_flag=1;
		// 	// }
		// 	if((fabs(M3508_state_2.angular_velocity)<0.2)&&(fabs(M3508_state_1.angular_velocity)<0.2)){
		// 		m3508_2_disable_flag=1;
		// 		m3508_1_disable_flag=1;
		// 	}
		// 	if(m3508_1_disable_flag==1&&m3508_2_disable_flag==1){
		// 		launch_stage=LAUNCH;
		// 		launch_wait_count=0;
		// 		target_rad=0;
		// 	}
		// }

		// if((m3508_angle[0]<0.01)&&(m3508_angle[1]<0.01)){
		// 	//launch_wait_count++;
		// 	if(launch_wait_count>(100+FREQUENCE*TOTAL_PULLUP_TIME)){
		// 		launch_stage=LAUNCH;
		//  		launch_wait_count=0;
		// 		target_rad=0;
		// 	}
		// }
		if(launch_wait_count==(FREQUENCE*TOTAL_PULLUP_TIME)){
			//launch_wait_count++;
			//if(launch_wait_count>(100+FREQUENCE*TOTAL_PULLUP_TIME)){
				launch_stage=LAUNCH;
		 		launch_wait_count=0;
				target_rad=0;
			//}
		}
		PullPID2(2);
		PullPID3(2);
		if(launch_stage==LAUNCH){
			// m3508_1_disable_flag=0;
			// m3508_2_disable_flag=0;
			previous_error_2[0]=previous_error_2[1]=0;
			previous_error_3[0]=previous_error_3[1]=0;
			integral_2[0]=integral_2[1]=0;
			integral_3[0]=integral_3[1]=0;
		}
	}
	else if(rc_ptr->rc_r_switch()==2&&launch_stage==LAUNCH){
		ServoSetAngle(-40);//扳机下拉，发射
		launch_wait_count++;
		if(launch_wait_count>FREQUENCE)
		{
			ServoSetAngle(-85);//扳机复位
			launch_stage=PULLDOWM;
			launch_wait_count=0;
		}
	}
}

//位控下拉,缓慢发射(还存在问题)
void LaunchTest6 (void){
	// M2006PID();
	if(launch_stage==PULLDOWM){
		ServoSetAngle(-90);//扳机复位
		target_rad+=kDeltaDownRad;
		//launch_wait_count++;
		if(target_rad>TOTAL_RAD){
			target_rad=TOTAL_RAD;
		}
		if((fabs(m3508_angle[0]-TOTAL_RAD)<0.02)&&(fabs(m3508_angle[1]-TOTAL_RAD)<0.02)){
			launch_wait_count++;
			if(launch_wait_count>0.2*FREQUENCE){
				launch_stage=PULLUP;
				launch_wait_count=0;
				target_rad=TOTAL_RAD;
			    m3508_2_disable_flag=1;
		    	m3508_1_disable_flag=1;
			}
		}
		PullPID2(1);
		PullPID3(1);
		if(launch_stage==PULLUP){
			m3508_1_disable_flag=0;
			m3508_2_disable_flag=0;
     		previous_error_2[0]=previous_error_2[1]=0;
			previous_error_3[0]=previous_error_3[1]=0;
			integral_2[0]=integral_2[1]=0;
			integral_3[0]=integral_3[1]=0;
		}
	}
	else if(launch_stage==PULLUP){
		ServoSetAngle(60);//扳机下拉，发射
		// slow_launch_wait_count++;
		// if(slow_launch_wait_count<300){
		// 	PullPID2(1);
		// 	PullPID3(1);
		// }
		// else{
	    target_rad-=kDeltaUpRad;
		//launch_wait_count++;
		if(target_rad<0){
			target_rad=0;
		}
		if((m3508_angle[0]<0.02)&&(m3508_angle[1]<0.02)){
			launch_wait_count++;
			if(launch_wait_count>0.2*FREQUENCE){
				launch_stage=LAUNCH;
		 		launch_wait_count=0;
				target_rad=0;
				m3508_2_disable_flag=1;
		    	m3508_1_disable_flag=1;
			}
		}
		PullPID2(1);
		PullPID3(1);
		if(launch_stage==LAUNCH){
			m3508_1_disable_flag=0;
			m3508_2_disable_flag=0;
			previous_error_2[0]=previous_error_2[1]=0;
			previous_error_3[0]=previous_error_3[1]=0;
			integral_2[0]=integral_2[1]=0;
			integral_3[0]=integral_3[1]=0;

			ServoSetAngle(-90);//扳机复位
			launch_stage=PULLDOWM;
			// launch_wait_count=0;
			launch_count=0;
			//slow_launch_wait_count=0;
			launch_flag=0;
			launch_dart_num++;
		}
		
	}
	// else if(launch_stage==LAUNCH){
	// 	ServoSetAngle(60);//扳机下拉，发射
	// 	launch_wait_count++;
	// 	if(launch_wait_count>400)
	// 	{
	// 		ServoSetAngle(-90);//扳机复位
	// 		launch_stage=PULLDOWM;
	// 		launch_wait_count=0;
	// 		launch_count=0;
	// 		launch_flag=0;
	// 		launch_dart_num++;
	// 	}
	// }
}

//发射第一个飞镖
void LaunchFirstDart(void){
	if(m2006_position_init_flag==0){
		m2006_initial_position=m2006_angle;
		m2006_second_position=m2006_initial_position+2.5f*ruler_to_2006position/10.0f;
		m2006_third_position=m2006_second_position+2.5f*ruler_to_2006position/10.0f;
		//m2006_fourth_position=m2006_third_position+0.0f*ruler_to_2006position/10.0f;
		// m2006_fourth_position=m2006_predicted_position+2.5f*ruler_to_2006position/10.0f;
		
		m2006_position_init_flag=1;

		m2006_predicted_position=m2006_initial_position;//给个保底值，防止打随机目标的时候if判断没进去导致predicted_position为默认值0
	}//螺距2mm，标尺10mm->2006:31.4rad
	if(launch_stage==PULLDOWM){
		PullDown();
	}
	else if(launch_stage==PULLUP){
		m2006_current=0;
		PullUp();
	}
	else if(launch_stage==LAUNCH){
		//Launch();
		launch_wait_count++;
		if(launch_wait_count>0.65*FREQUENCE&&launch_wait_count<1.05*FREQUENCE){
			ServoSetAngle(60);//扳机下拉，发射
		}
		if(launch_wait_count>1.05*FREQUENCE)
		{
			ServoSetAngle(-90);//扳机复位
			launch_stage=PULLDOWM;
			launch_wait_count=0;
			//launch_init_flag=1;
			launch_dart_num++;
			launch_count=0;
			//launch_flag=0;
		}
	}
}

// //发射第一个飞镖，用于能接到裁判系统数据自动发射
// void LaunchFirstDart_2(void){
// 	if(launch_stage==PULLDOWM){
// 		PullDown();
// 	}
// 	else if(launch_stage==PULLUP){
// 		PullUp();
// 	}
// 	else if(launch_stage==LAUNCH&&dart_station_info.dart_launch_opening_status==rfr::DartStationStatus::kDartStationStatusOpened){
// 		ServoSetAngle(60);//扳机下拉，发射
// 		launch_wait_count++;
// 		//M2006PID();
// 		if(launch_wait_count>0.6*FREQUENCE)
// 		{
// 			ServoSetAngle(-90);//扳机复位
// 			launch_stage=PULLDOWM;
// 			launch_wait_count=0;
// 			//launch_init_flag=1;
// 			launch_dart_num++;
// 			//比赛时删掉
// 			launch_count=0;
// 			//launch_flag=0;
// 		}
// 	}
// }

// //之后考虑将234枚飞镖的上拉发射单独写个函数
// void LaunchSecondDart(void){
// 	//M2006PID();
// 	if(launch_stage==PULLDOWM){
// 		PullDown();
// 	}
// 	else if(launch_stage==PULLUP){
// 		GrabFirstDart();
// 	    target_rad-=kDeltaUpRad;
// 		if(target_rad<0){
// 			target_rad=0;
// 		}
// 		if((m3508_angle[0]<0.02)&&(m3508_angle[1]<0.02)){
// 			launch_wait_count++;
// 			if(launch_wait_count>100){
// 				//launch_stage=LAUNCH;
// 		 		launch_wait_count=0;
// 				target_rad=0;
// 				m3508_2_disable_flag=1;
// 		    	m3508_1_disable_flag=1;
// 			}
// 		}
// 		PullPID2(2);
// 		PullPID3(2);
// 		uint8_t tx_data[8]={0};
// 		tx_data[0]=(uint8_t)(m3508_current[0]>>8);
// 		tx_data[1]=(uint8_t)m3508_current[0];
// 		tx_data[2]=(uint8_t)(m3508_current[1]>>8);
// 		tx_data[3]=(uint8_t)m3508_current[1];
// 		CAN_SendMsg(&hcan1,0x200,tx_data,8);
// 		if(launch_stage==LAUNCH){
// 			m3508_1_disable_flag=0;
// 			m3508_2_disable_flag=0;
// 			previous_error_2[0]=previous_error_2[1]=0;
// 			previous_error_3[0]=previous_error_3[1]=0;
// 			integral_2[0]=integral_2[1]=0;
// 			integral_3[0]=integral_3[1]=0;
// 		}
// 	}
// 	else if(launch_stage==LAUNCH){
// 		GrabFirstDart();
// 		ServoSetAngle(60);//扳机下拉，发射
// 		if(servo_reset_flag==0)
// 		{	
// 			launch_wait_count++;
// 			if(launch_wait_count>400){
// 				launch_wait_count=0;
// 				ServoSetAngle(-90);//扳机复位
// 			launch_stage=PULLDOWM;
// 			launch_wait_count=0;
// 			launch_count=0;
// 			launch_flag=0;
// 			launch_dart_num++;
// 			}
// 		}
// 	}
// }

//发射第二个飞镖
void LaunchSecondDart(void){
	if(launch_stage==PULLDOWM){
		Change2006PositionT02();
		PullDown();
		GrabFirstDart();
	}
	else if(launch_stage==PULLUP){
		m2006_current=0;
		PullUp();
		GrabFirstDart();
	}
	else if(launch_stage==LAUNCH){
		// if(servo_reset_flag==0)
		// {	
		// 	ServoSetAngle(60);//扳机下拉，发射
		// 	launch_wait_count++;
		// 	if(launch_wait_count>300){
		// 		launch_wait_count=0;
		// 		ServoSetAngle(-90);//扳机复位
		// 		servo_reset_flag=1;
		// 	//launch_stage=PULLDOWM;
		// 	//launch_wait_count=0;
		// 	//launch_count=0;
		// 	//launch_flag=0;
				
		// 	}
		// }
		// if(launch_stage==PULLDOWM){
		// 	launch_dart_num++;
		// }
		if(grab_firstdart_flag==1){
			ServoSetAngle(60);//扳机下拉，发射
			launch_wait_count++;
			//M2006PID();
			if(launch_wait_count>0.4*FREQUENCE)
			{
				ServoSetAngle(-90);//扳机复位
				launch_stage=PULLDOWM;
				launch_wait_count=0;
				//launch_init_flag=0;
				launch_dart_num++;
				first_round_launch_flag=1;
				//control_pose_flag=0;
				launch_count=0;
				launch_flag=0;
				shoot_flag=0;
				shoot_wait_count=0;
			}
		}
		GrabFirstDart();
	}
}

//发射第三个飞镖
void LaunchThirdDart(void){
	if(launch_stage==PULLDOWM){
		//Change2006PositionT03();
		PullDown();
		GrabSecondDart();
	}
	else if(launch_stage==PULLUP){
		m2006_current=0;
		PullUp();
		GrabSecondDart();
	}
	else if(launch_stage==LAUNCH){
		if(grab_seconddart_flag==1){
			ServoSetAngle(60);//扳机下拉，发射
			launch_wait_count++;
			//M2006PID();
			if(launch_wait_count>0.4*FREQUENCE)
			{
				ServoSetAngle(-90);//扳机复位
				launch_stage=PULLDOWM;
				launch_wait_count=0;
				//launch_init_flag=1;
				launch_dart_num++;
				launch_count=0;
				//launch_flag=0;
			}
		}
		GrabSecondDart();
	}
}

//发射第三个飞镖,用于能接到裁判系统数据自动发射(该发射用于击打后两个随机目标)
void LaunchThirdDart_2(void){
	if(launch_stage==PULLDOWM){
		ChangePositionToRandomTarget();
		PullDown();
		GrabSecondDart();
	}
	else if(launch_stage==PULLUP){
		m2006_current=0;
		PullUp();
		GrabSecondDart();
	}
	else if(launch_stage==LAUNCH){
		if(grab_seconddart_flag==1){
			ServoSetAngle(60);//扳机下拉，发射
			launch_wait_count++;
			//M2006PID();
			if(launch_wait_count>0.4*FREQUENCE)
			{
				ServoSetAngle(-90);//扳机复位
				launch_stage=PULLDOWM;
				launch_wait_count=0;
				//launch_init_flag=1;
				launch_dart_num++;
				launch_count=0;
				//launch_flag=0;
			}
		}
		GrabSecondDart();
	}
}


//发射第四个飞镖
void LaunchFourthDart(void){
	if(launch_stage==PULLDOWM){
		Change2006PositionT04();
		PullDown();
		GrabThirdDart();
	}
	else if(launch_stage==PULLUP){
		m2006_current=0;
		PullUp();
		GrabThirdDart();
	}
	else if(launch_stage==LAUNCH){
		if(grab_thirddart_flag==1){
			ServoSetAngle(60);//扳机下拉，发射
			launch_wait_count++;
			//M2006PID();
			if(launch_wait_count>0.4*FREQUENCE)
			{
				ServoSetAngle(-90);//扳机复位
				launch_stage=PULLDOWM;
				launch_wait_count=0;
				//launch_init_flag=1;
				launch_dart_num++;
				second_round_launch_flag=1;
				//control_pose_flag=0;
				launch_count=0;
				launch_flag=0;
				shoot_flag=0;
				shoot_wait_count=0;
			}
		}
		GrabThirdDart();
	}
}

// void LaunchInit(void){
// 	if(launch_stage==PULLDOWM){
// 		PullDown();
// 	}
// 	else if(launch_stage==PULLUP){
// 		PullUp();
// 	}
// }

// static void PullUpFirstDart(void){
// 	target_rad-=kDeltaUpRad;
// 	//launch_wait_count++;
// 	if(target_rad<0){
// 		target_rad=0;
// 	}
// 	if((m3508_angle[0]<0.02)&&(m3508_angle[1]<0.02)){
// 		launch_wait_count++;
// 		if(launch_wait_count>100){
// 			launch_stage=LAUNCH;
// 			launch_wait_count=0;
// 			target_rad=0;
// 			m3508_2_disable_flag=1;
// 			m3508_1_disable_flag=1;
// 		}
// 	}
// 	PullPID2(2);
// 	PullPID3(2);
// 	//M2006PID();
// 	if(launch_stage==LAUNCH){
// 		//launch_init_flag=1;
// 		m3508_1_disable_flag=0;
// 		m3508_2_disable_flag=0;
// 		previous_error_2[0]=previous_error_2[1]=0;
// 		previous_error_3[0]=previous_error_3[1]=0;
// 		integral_2[0]=integral_2[1]=0;
// 		integral_3[0]=integral_3[1]=0;
// 	}
// }




//同步带上拉
static void PullUp(void){
	target_rad-=kDeltaUpRad;
	//launch_wait_count++;
	if(target_rad<0){
		target_rad=0;
	}
	if((m3508_angle[0]<0.02)&&(m3508_angle[1]<0.02)){
		launch_wait_count++;
		if(launch_wait_count>0.2*FREQUENCE){
			launch_stage=LAUNCH;
			launch_wait_count=0;
			target_rad=0;
			m3508_2_disable_flag=1;
			m3508_1_disable_flag=1;
		}
	}
	PullPID2(2);
	PullPID3(2);
	//M2006PID();
	if(launch_stage==LAUNCH){
		m3508_1_disable_flag=0;
		m3508_2_disable_flag=0;
		previous_error_2[0]=previous_error_2[1]=0;
		previous_error_3[0]=previous_error_3[1]=0;
		integral_2[0]=integral_2[1]=0;
		integral_3[0]=integral_3[1]=0;
	}
}

//同步带下拉
static void PullDown(void){
	ServoSetAngle(-90);//扳机复位
	target_rad+=kDeltaDownRad;
	//launch_wait_count++;
	if(target_rad>TOTAL_RAD){
		target_rad=TOTAL_RAD;
	}
	if((fabs(m3508_angle[0]-TOTAL_RAD)<0.02)&&(fabs(m3508_angle[1]-TOTAL_RAD)<0.02)){
		launch_wait_count++;
		if(launch_wait_count>0.2*FREQUENCE){
			launch_stage=PULLUP;
			launch_wait_count=0;
			target_rad=TOTAL_RAD;
			m3508_2_disable_flag=1;
			m3508_1_disable_flag=1;
		}
	}
	PullPID2(1);
	PullPID3(1);
	//M2006PID();
	if(launch_stage==PULLUP){
		m3508_1_disable_flag=0;
		m3508_2_disable_flag=0;
		previous_error_2[0]=previous_error_2[1]=0;
		previous_error_3[0]=previous_error_3[1]=0;
		integral_2[0]=integral_2[1]=0;
		integral_3[0]=integral_3[1]=0;
	}
}

//舵机扣动发射
static void Launch(void){
	ServoSetAngle(60);//扳机下拉，发射
	launch_wait_count++;
	//M2006PID();
	if(launch_wait_count>0.6*FREQUENCE)
	{
		ServoSetAngle(-90);//扳机复位
		launch_stage=PULLDOWM;
		launch_wait_count=0;
		//launch_init_flag=1;
		launch_dart_num++;
		//比赛时删掉
		launch_count=0;
		launch_flag=0;
	}
}

static void PullPID(float pulldown_speed,uint8_t pid_params_state)
{
	float angle_velocity[2]={M3508_state_1.angular_velocity,M3508_state_2.angular_velocity};
    static float previous_error[2];
    static float integral[2];
    float derivative[2];
    float error[2];
    float result[2];
    //static int16_t feedback_current_error;

    //由于两个电机镜像放置,error的处理会不同
    for(int i=0;i<2;i++)
    {
    if(i==0){
    	error[i]=-pulldown_speed*kReductionRatio/kMotorRadius-angle_velocity[i];
    }
    else{
    	error[i]=pulldown_speed*kReductionRatio/kMotorRadius-angle_velocity[i];
    }

    integral[i] +=error[i];
    derivative[i]=(error[i]-previous_error[i]);
    previous_error[i]=error[i];

    if(pid_params_state==1){
    	result[i]=PID.kp1*error[i]+PID.ki1*integral[i]+PID.kd1*derivative[i];
    }
    else if(pid_params_state==2){
    	result[i]=PID.kp2*error[i]+PID.ki2*integral[i]+PID.kd2*derivative[i];
    }

    if(result[i]>=12000.0)
    {
        result[i]=12000.0;
    }
    else if(result[i]<=-12000.0)
    {
        result[i]=-12000.0;
    }//限流

    m3508_current[i]=(int16_t)(result[i]);


    }
}

// //输入参数为期望下拉速度，正号表示下拉,cm/s
// static void PullDown(float speed){
// 	uint8_t tx_data[8]={0};
// 	PullPID(speed,1);
// 	tx_data[0]=(uint8_t)(m3508_current[0]>>8);
// 	tx_data[1]=(uint8_t)m3508_current[0];
// 	tx_data[2]=(uint8_t)(m3508_current[1]>>8);
// 	tx_data[3]=(uint8_t)m3508_current[1];
// 	CAN_SendMsg(&hcan1,0x200,tx_data,8);
// }

// //输入参数为期望上拉速度，负号表示上拉,cm/s
// static void PullUp(float speed){
// 	uint8_t tx_data[8]={0};
// 	PullPID(speed,2);
// 	tx_data[0]=(uint8_t)(m3508_current[0]>>8);
// 	tx_data[1]=(uint8_t)m3508_current[0];
// 	tx_data[2]=(uint8_t)(m3508_current[1]>>8);
// 	tx_data[3]=(uint8_t)m3508_current[1];
// 	CAN_SendMsg(&hcan1,0x200,tx_data,8);
// }

//位控外环PID,得到期望角速度,输入参数为pid状态，1表示采用下拉pid参数，2表示采用上拉pid参数
static void PullPID2(uint8_t pid_params_state)
{
	//target_rad+=kDeltaRad;
	//float angle_velocity[2]={M3508_state_1.angular_velocity,M3508_state_2.angular_velocity};


    float derivative_2[2];
    float error_2[2];
    float result_2[2];
    //static int16_t feedback_current_error;

    //由于两个电机镜像放置,error的处理会不同
    for(int i=0;i<2;i++)
    {
		if(i==0){
			error_2[i]=-1500*(target_rad-m3508_angle[i]);
		}
		else{
			error_2[i]=1500*(target_rad-m3508_angle[i]);
		}

		integral_2[i] +=error_2[i];
		derivative_2[i]=(error_2[i]-previous_error_2[i]);
		previous_error_2[i]=error_2[i];

		if(pid_params_state==1){
			result_2[i]=PID_2.kp1*error_2[i]+PID_2.ki1*integral_2[i]+PID_2.kd1*derivative_2[i];
		}
		else if(pid_params_state==2){
			result_2[i]=PID_2.kp2*error_2[i]+PID_2.ki2*integral_2[i]+PID_2.kd2*derivative_2[i];
		}

		m3508_angle_speed[i]=result_2[i];
    }
}

//位控内环PID,得到期望输入电流,输入参数为pid状态，1表示采用下拉pid参数，2表示采用上拉pid参数,函数最后包含了can发送电流
static void PullPID3(uint8_t pid_params_state)
{
    float angle_velocity_3[2]={M3508_state_1.angular_velocity,M3508_state_2.angular_velocity};

    float derivative_3[2];
    float error_3[2];
    float result_3[2];
    static int16_t feedback_current_error;
	
    //由于之前外环已经做过取负的处理，因此此处两边error的处理相同
    for(int i=0;i<2;i++)
    {

    error_3[i]=m3508_angle_speed[i]*kReductionRatio/kMotorRadius-angle_velocity_3[i];
    
    integral_3[i] +=error_3[i];
    derivative_3[i]=(error_3[i]-previous_error_3[i]);
    previous_error_3[i]=error_3[i];

 	if(pid_params_state==1){
		if(i==0){
			if(m3508_1_disable_flag==0){
				result_3[i]=PID_3.kp1*error_3[i]+PID_3.ki1*integral_3[i]+PID_3.kd1*derivative_3[i];
			}
			else {
				result_3[i]=0;
			}
		}
		if(i==1){
			if(m3508_2_disable_flag==0){
				result_3[i]=PID_3.kp1*error_3[i]+PID_3.ki1*integral_3[i]+PID_3.kd1*derivative_3[i];
			}
			else {
				result_3[i]=0;
			}
		}
    }
    else if(pid_params_state==2){
    	if(i==0){
			if(m3508_1_disable_flag==0){
				result_3[i]=PID_3.kp2*error_3[i]+PID_3.ki2*integral_3[i]+PID_3.kd2*derivative_3[i];
			}
			else {
				result_3[i]=0;
			}
		}
		if(i==1){
			if(m3508_2_disable_flag==0){
				result_3[i]=PID_3.kp2*error_3[i]+PID_3.ki2*integral_3[i]+PID_3.kd2*derivative_3[i];
			}
			else {
				result_3[i]=0;
			}
		}
	}

	// if(pid_params_state==1){
	// 	feedback_current_error=fabs(M3508_state_1.feedback_current+M3508_state_2.feedback_current);
	// 	if(fabs(M3508_state_1.feedback_current)>fabs(M3508_state_2.feedback_current)){
	// 		if(i==1){
	// 			result_3[i]+=current_k*feedback_current_error;
	// 		}
	// 	}
	// 	else {
	// 		if(i==0){
	// 		result_3[i]-=current_k*feedback_current_error;
	// 		}
	// 	}
	// }

    if(result_3[i]>=15000.0)
    {
        result_3[i]=15000.0;
    }
    else if(result_3[i]<=-15000.0)
    {
        result_3[i]=-15000.0;
    }//限流

    m3508_current[i]=(int16_t)(result_3[i]);
	}
	uint8_t tx_data[8]={0};
	tx_data[0]=(uint8_t)(m3508_current[0]>>8);
	tx_data[1]=(uint8_t)m3508_current[0];
	tx_data[2]=(uint8_t)(m3508_current[1]>>8);
	tx_data[3]=(uint8_t)m3508_current[1];
	tx_data[4]=(uint8_t)(m2006_current>>8);
	tx_data[5]=(uint8_t)m2006_current;
	CAN_SendMsg(&hcan1,0x200,tx_data,8);
}