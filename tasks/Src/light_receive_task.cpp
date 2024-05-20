/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-04-26 22:56:25
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-05-16 23:54:58
 * @FilePath: \dartrack1\tasks\Src\light_receive_task.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "light_receive_task.hpp"
#include "usart.h"

float predicted_yaw;//自瞄预测的yaw的偏差角(rad)，灯在右边角度是正的
float predicted_pitch;//自瞄预测的pitch的偏差角
// int16_t shoot_flag;
// uint8_t target_num_list;

uint8_t light_receive_data[12];//存储符合帧头的正确自瞄数据
uint8_t light_receive_flag;//等于1表示一帧数据接收完毕，可以去进一步处理数据
int16_t yaw_angle;
int16_t pitch_angle;
uint8_t greenlight_detected_flag=0;//等于1表示舱门已开，相机已经检测到绿灯，准备发射

// void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
// { 
//     if(huart==&huart7)
//     {
//         LightInfoReceiveCheck();
//         LightDataProcess();
//         HAL_UART_Receive_IT(&huart7,&camera_rx_buffer,1);
//     }
// }

//检查接收到的数据，利用状态机判断是否接收
void LightInfoReceiveCheck(void){
    static uint8_t light_receive_state=0;
	static uint8_t light_receive_i=3;

    if(camera_rx_buffer[0] == 0xaa)
    {
        if(camera_rx_buffer[1]==0xbb)
        {
            if(camera_rx_buffer[2]==0xcc)
            {
                if(camera_rx_buffer[9]==0xff)
                {
                    light_receive_flag = 1;
                }
            }
        }
    }

    // if(light_receive_state ==0)
    // {
    //     if(camera_rx_buffer==0xaa)
    //     {	
    //         light_receive_state =1;
    //     }
    // }
    // else if(light_receive_state==1)
    // {	
    //     if(camera_rx_buffer==0xbb)
    //     {
    //         light_receive_state =2;
    //     }
    //     else
    //         light_receive_state =0;
    // }
    // else if(light_receive_state==2)
    // {	
    //     if(camera_rx_buffer==0xcc)
    //     {
    //         light_receive_state =3;
    //         light_receive_data[0]=0xaa;
    //         light_receive_data[1]=0xbb;
    //         light_receive_data[2]=0xcc;//前三个帧头
    //     }
    //     else
    //         light_receive_state =0;
    // }
    // else if(light_receive_state==3)
    // {	
    //     light_receive_data[light_receive_i]=camera_rx_buffer;
    //     light_receive_i++;        
    //     if(light_receive_i>=10)
    //     {
    //         light_receive_state=0;
    //         light_receive_flag=1;//接收完毕将Flag置1
    //         light_receive_i=3;
    //     } 
    // }
}

//一帧数据接收完毕时对其进行处理
void LightDataProcess(void){
    if(light_receive_flag==1){
        yaw_angle=(camera_rx_buffer[3] << 8) | camera_rx_buffer[4];
        pitch_angle=(camera_rx_buffer[5] << 8) | (camera_rx_buffer[6]);
        predicted_pitch=pitch_angle/12500.0f;
        predicted_yaw=yaw_angle/12500.0f;
        greenlight_detected_flag=camera_rx_buffer[7];
        //target_num_list=camera_rx_buffer[7];
        //shoot_flag=camera_rx_buffer[8];
        //greenlight_detected_flag=camera_rx_buffer[9];

        light_receive_flag=0;//处理完毕重新将flag置零
    }
}