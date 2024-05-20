/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-03-30 16:57:16
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-05-14 16:49:52
 * @FilePath: \dartrack1\tasks\Src\reload_dart_task.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "reload_dart_task.hpp"
#include "usart.h"
#include "stdio.h"
#include "launch_task.hpp"

uint8_t grab_firstdart_stage=0;//指示抓取第一个飞镖的进度
uint8_t grab_seconddart_stage=0;//指示抓取第2个飞镖的进度
uint8_t grab_thirddart_stage=0;//指示抓取第3个飞镖的进度

uint8_t grab_firstdart_test_stage=0;//指示抓取第一个飞镖的进度,仅用于调试机械臂动作
uint8_t grab_seconddart_test_stage=0;//指示抓取第2个飞镖的进度,仅用于调试机械臂动作
uint8_t grab_thirddart_test_stage=0;//指示抓取第3个飞镖的进度,仅用于调试机械臂动作

uint8_t grab_firstdart_flag=0;//指示抓取第一个飞镖流程是否完成，1表示完成
uint8_t grab_seconddart_flag=0;//指示抓取第2个飞镖流程是否完成，1表示完成
uint8_t grab_thirddart_flag=0;//指示抓取第3个飞镖流程是否完成，1表示完成

uint32_t grab_firstdart_wait_count=0;//计时
uint32_t grab_seconddart_wait_count=0;//计时
uint32_t grab_thirddart_wait_count=0;//计时
uint32_t grab_firstdart_test_waitcount=0;//计时,仅用于调试机械臂动作
uint32_t grab_seconddart_test_waitcount=0;//计时,仅用于调试机械臂动作
uint32_t grab_thirddart_test_waitcount=0;//计时,仅用于调试机械臂动作

uint8_t grab_dart_num=0;//已经抓取的飞镖数量

uint8_t grab_firstdart_flag_2=0;//为了发某个指令时只需要发一次而随便设的变量
uint8_t grab_seconddart_flag_2=0;//为了发某个指令时只需要发一次而随便设的变量
uint8_t grab_thirddart_flag_2=0;//为了发某个指令时只需要发一次而随便设的变量
uint8_t grab_firstdart_flag_2_test=0;//为了发某个指令时只需要发一次而随便设的变量,仅用于调试机械臂动作
uint8_t grab_seconddart_flag_2_test=0;//为了发某个指令时只需要发一次而随便设的变量,仅用于调试机械臂动作
uint8_t grab_thirddart_flag_2_test=0;//为了发某个指令时只需要发一次而随便设的变量,仅用于调试机械臂动作

const char move_buff_1[]="{\"T\":1,\"P1\":199,\"P2\":51.328,\"P3\":66.79,\"P4\":49.57,\"P5\":250,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":400,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":125}";
const char move_buff_2[]="{\"T\":1,\"P1\":199,\"P2\":36,\"P3\":85,\"P4\":49,\"P5\":250,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
const char move_buff_3[]="{\"T\":1,\"P1\":182,\"P2\":36,\"P3\":85,\"P4\":49,\"P5\":250,\"S1\":400,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":80,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
const char move_buff_4[]="{\"T\":1,\"P1\":182,\"P2\":48.66,\"P3\":77.87,\"P4\":43,\"P5\":250,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
const char move_buff_5[]="{\"T\":1,\"P1\":182,\"P2\":48.66,\"P3\":77.87,\"P4\":43,\"P5\":210,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
const char move_buff_6[]="{\"T\":1,\"P1\":182,\"P2\":0,\"P3\":90,\"P4\":90,\"P5\":180,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";

const char move_buff_7[]="{\"T\":1,\"P1\":164.5,\"P2\":51.328,\"P3\":66.79,\"P4\":49.57,\"P5\":250,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":400,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":125}";
const char move_buff_8[]="{\"T\":1,\"P1\":164.5,\"P2\":36,\"P3\":85,\"P4\":49,\"P5\":250,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
const char move_buff_9[]="{\"T\":1,\"P1\":182.5,\"P2\":36,\"P3\":85,\"P4\":49,\"P5\":250,\"S1\":400,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":80,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
const char move_buff_10[]="{\"T\":1,\"P1\":182.5,\"P2\":48.66,\"P3\":77.87,\"P4\":43,\"P5\":250,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
const char move_buff_11[]="{\"T\":1,\"P1\":182.5,\"P2\":48.66,\"P3\":77.87,\"P4\":43,\"P5\":210,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
const char move_buff_12[]="{\"T\":1,\"P1\":182.5,\"P2\":0,\"P3\":90,\"P4\":90,\"P5\":180,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";

const char move_buff_13[]="{\"T\":1,\"P1\":148,\"P2\":51.328,\"P3\":66.79,\"P4\":49.57,\"P5\":250,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":400,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":125}";
const char move_buff_14[]="{\"T\":1,\"P1\":148,\"P2\":36,\"P3\":85,\"P4\":49,\"P5\":250,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
const char move_buff_15[]="{\"T\":1,\"P1\":182.5,\"P2\":36,\"P3\":85,\"P4\":49,\"P5\":250,\"S1\":400,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":80,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
const char move_buff_16[]="{\"T\":1,\"P1\":182.5,\"P2\":48.66,\"P3\":77.87,\"P4\":43,\"P5\":250,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
const char move_buff_17[]="{\"T\":1,\"P1\":182.5,\"P2\":48.66,\"P3\":77.87,\"P4\":43,\"P5\":210,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
const char move_buff_18[]="{\"T\":1,\"P1\":182.5,\"P2\":0,\"P3\":90,\"P4\":90,\"P5\":180,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";

//抓取每个飞镖都分六个动作，每个动作进行计时估计（因为动作完成后舵机不会给反馈，因此只能靠人为计时估计每个动作的时间）
//抓取第一个飞镖
void GrabFirstDart(void){
    //动作一
    if(grab_firstdart_stage==0){
        if(grab_firstdart_flag_2==0){
            // char tx_buff[]="{\"T\":1,\"P1\":199,\"P2\":51.328,\"P3\":66.79,\"P4\":49.57,\"P5\":250,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":400,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":125}";
            // HAL_UART_Transmit(&huart8,(uint8_t *)tx_buff,sizeof(tx_buff),1600);
            HAL_UART_Transmit(&huart8,(uint8_t *)move_buff_1,sizeof(move_buff_1),1600);
            grab_firstdart_flag_2=1;
        }
        grab_firstdart_wait_count++;
        if(grab_firstdart_wait_count>1.6*FREQUENCE){
            grab_firstdart_stage=1;
            grab_firstdart_wait_count=0;
            grab_firstdart_flag_2=0;
        }
    }
    //动作二
    if(grab_firstdart_stage==1){
        if(grab_firstdart_flag_2==0){
            // char tx_buff[]="{\"T\":1,\"P1\":199,\"P2\":36,\"P3\":85,\"P4\":49,\"P5\":250,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
            // HAL_UART_Transmit(&huart8,(uint8_t *)tx_buff,sizeof(tx_buff),700);
            HAL_UART_Transmit(&huart8,(uint8_t *)move_buff_2,sizeof(move_buff_2),700);
            grab_firstdart_flag_2=1;
        }
        grab_firstdart_wait_count++;
        if(grab_firstdart_wait_count>0.7*FREQUENCE){
            grab_firstdart_stage=2;
            grab_firstdart_wait_count=0;
            grab_firstdart_flag_2=0;
        }
    }
    //动作三
    if(grab_firstdart_stage==2){
        if(grab_firstdart_flag_2==0){
            // char tx_buff[]="{\"T\":1,\"P1\":182,\"P2\":36,\"P3\":85,\"P4\":49,\"P5\":250,\"S1\":400,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":80,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
            // HAL_UART_Transmit(&huart8,(uint8_t *)tx_buff,sizeof(tx_buff),900);
            HAL_UART_Transmit(&huart8,(uint8_t *)move_buff_3,sizeof(move_buff_3),900);
            grab_firstdart_flag_2=1;
        }
        grab_firstdart_wait_count++;
        if(grab_firstdart_wait_count>0.9*FREQUENCE){
            grab_firstdart_stage=3;
            grab_firstdart_wait_count=0;
            grab_firstdart_flag_2=0;
        }
    }
    //动作四
    if(grab_firstdart_stage==3){
        if(grab_firstdart_flag_2==0){
            // char tx_buff[]="{\"T\":1,\"P1\":182,\"P2\":48.66,\"P3\":77.87,\"P4\":43,\"P5\":250,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
            // HAL_UART_Transmit(&huart8,(uint8_t *)tx_buff,sizeof(tx_buff),600);
            HAL_UART_Transmit(&huart8,(uint8_t *)move_buff_4,sizeof(move_buff_4),600);
            grab_firstdart_flag_2=1;
        }
        grab_firstdart_wait_count++;
        if(grab_firstdart_wait_count>0.6*FREQUENCE){
            grab_firstdart_stage=4;
            grab_firstdart_wait_count=0;
            grab_firstdart_flag_2=0;
        }
    }
    //动作5
    if(grab_firstdart_stage==4){
        if(grab_firstdart_flag_2==0){
            // char tx_buff[]="{\"T\":1,\"P1\":182,\"P2\":48.66,\"P3\":77.87,\"P4\":43,\"P5\":210,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
            // HAL_UART_Transmit(&huart8,(uint8_t *)tx_buff,sizeof(tx_buff),600);
            HAL_UART_Transmit(&huart8,(uint8_t *)move_buff_5,sizeof(move_buff_5),600);
            grab_firstdart_flag_2=1;
        }
        grab_firstdart_wait_count++;
        if(grab_firstdart_wait_count>0.6*FREQUENCE){
            grab_firstdart_stage=5;
            grab_firstdart_wait_count=0;
            grab_firstdart_flag_2=0;
        }
    }
    //动作6
    if(grab_firstdart_stage==5){
        if(grab_firstdart_flag_2==0){
            // char tx_buff[]="{\"T\":1,\"P1\":182,\"P2\":0,\"P3\":90,\"P4\":90,\"P5\":180,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
            // HAL_UART_Transmit(&huart8,(uint8_t *)tx_buff,sizeof(tx_buff),700);
            HAL_UART_Transmit(&huart8,(uint8_t *)move_buff_6,sizeof(move_buff_6),700);
            grab_firstdart_flag_2=1;
        }
        grab_firstdart_wait_count++;
        if(grab_firstdart_wait_count>0.7*FREQUENCE){
            grab_firstdart_stage=6;
            grab_firstdart_wait_count=0;
            grab_firstdart_flag_2=0;
            grab_firstdart_flag=1;
            grab_dart_num++;
        }
    }
    // //动作7
    // if(grab_firstdart_stage==6){
    //     if(grab_flag==0){
    //         char tx_buff[]="{\"T\":1,\"P1\":183,\"P2\":0,\"P3\":90,\"P4\":90,\"P5\":180,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
    //         HAL_UART_Transmit(&huart8,(uint8_t *)tx_buff,sizeof(tx_buff),1000);   
    //         grab_flag=1;         
    //     }
    //     grab_wait_count++;
    //     if(grab_wait_count>500){
    //         grab_firstdart_stage=7;
    //         grab_wait_count=0;
    //         grab_firstdart_flag=1;
    //         grab_flag=0;
    //         grab_dart_num++;
    //         //launch_stage==LAUNCH;
    //     }
    // }
}

//抓取第一个飞镖,仅用于调试机械臂动作
void GrabFirstDartTest(void){
    //动作一
    if(grab_firstdart_test_stage==0){
        if(grab_firstdart_flag_2_test==0){
            // char tx_buff[]="{\"T\":1,\"P1\":199,\"P2\":51.328,\"P3\":66.79,\"P4\":49.57,\"P5\":250,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":400,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":125}";
            // HAL_UART_Transmit(&huart8,(uint8_t *)tx_buff,sizeof(tx_buff),1600);
            HAL_UART_Transmit(&huart8,(uint8_t *)move_buff_1,sizeof(move_buff_1),1600);
            grab_firstdart_flag_2_test=1;
        }
        grab_firstdart_test_waitcount++;
        if(grab_firstdart_test_waitcount>1.6*FREQUENCE){
            grab_firstdart_test_stage=1;
            grab_firstdart_test_waitcount=0;
            grab_firstdart_flag_2_test=0;
        }
    }
    //动作二
    if(grab_firstdart_test_stage==1){
        if(grab_firstdart_flag_2_test==0){
            // char tx_buff[]="{\"T\":1,\"P1\":199,\"P2\":36,\"P3\":85,\"P4\":49,\"P5\":250,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
            // HAL_UART_Transmit(&huart8,(uint8_t *)tx_buff,sizeof(tx_buff),700);
            HAL_UART_Transmit(&huart8,(uint8_t *)move_buff_2,sizeof(move_buff_2),700);
            grab_firstdart_flag_2_test=1;
        }
        grab_firstdart_test_waitcount++;
        if(grab_firstdart_test_waitcount>0.7*FREQUENCE){
            grab_firstdart_test_stage=2;
            grab_firstdart_test_waitcount=0;
            grab_firstdart_flag_2_test=0;
        }
    }
    //动作三
    if(grab_firstdart_test_stage==2){
        if(grab_firstdart_flag_2_test==0){
            // char tx_buff[]="{\"T\":1,\"P1\":182,\"P2\":36,\"P3\":85,\"P4\":49,\"P5\":250,\"S1\":400,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":80,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
            // HAL_UART_Transmit(&huart8,(uint8_t *)tx_buff,sizeof(tx_buff),900);
            HAL_UART_Transmit(&huart8,(uint8_t *)move_buff_3,sizeof(move_buff_3),900);
            grab_firstdart_flag_2_test=1;
        }
        grab_firstdart_test_waitcount++;
        if(grab_firstdart_test_waitcount>0.9*FREQUENCE){
            grab_firstdart_test_stage=3;
            grab_firstdart_test_waitcount=0;
            grab_firstdart_flag_2_test=0;
        }
    }
    //动作四
    if(grab_firstdart_test_stage==3){
        if(grab_firstdart_flag_2_test==0){
            // char tx_buff[]="{\"T\":1,\"P1\":182,\"P2\":48.66,\"P3\":77.87,\"P4\":43,\"P5\":250,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
            // HAL_UART_Transmit(&huart8,(uint8_t *)tx_buff,sizeof(tx_buff),600);
            HAL_UART_Transmit(&huart8,(uint8_t *)move_buff_4,sizeof(move_buff_4),600);
            grab_firstdart_flag_2_test=1;
        }
        grab_firstdart_test_waitcount++;
        if(grab_firstdart_test_waitcount>0.6*FREQUENCE){
            grab_firstdart_test_stage=4;
            grab_firstdart_test_waitcount=0;
            grab_firstdart_flag_2_test=0;
        }
    }
    //动作5
    if(grab_firstdart_test_stage==4){
        if(grab_firstdart_flag_2_test==0){
            // char tx_buff[]="{\"T\":1,\"P1\":182,\"P2\":48.66,\"P3\":77.87,\"P4\":43,\"P5\":210,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
            // HAL_UART_Transmit(&huart8,(uint8_t *)tx_buff,sizeof(tx_buff),600);
            HAL_UART_Transmit(&huart8,(uint8_t *)move_buff_5,sizeof(move_buff_5),600);
            grab_firstdart_flag_2_test=1;
        }
        grab_firstdart_test_waitcount++;
        if(grab_firstdart_test_waitcount>0.6*FREQUENCE){
            grab_firstdart_test_stage=5;
            grab_firstdart_test_waitcount=0;
            grab_firstdart_flag_2_test=0;
        }
    }
    //动作6
    if(grab_firstdart_test_stage==5){
        if(grab_firstdart_flag_2_test==0){
            // char tx_buff[]="{\"T\":1,\"P1\":182,\"P2\":0,\"P3\":90,\"P4\":90,\"P5\":180,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
            // HAL_UART_Transmit(&huart8,(uint8_t *)tx_buff,sizeof(tx_buff),2000);
            HAL_UART_Transmit(&huart8,(uint8_t *)move_buff_6,sizeof(move_buff_6),2000);
            grab_firstdart_flag_2_test=1;
        }
        grab_firstdart_test_waitcount++;
        if(grab_firstdart_test_waitcount>2*FREQUENCE){
            grab_firstdart_test_stage=0;
            grab_firstdart_test_waitcount=0;
            grab_firstdart_flag_2_test=0;
        }
    }
}

//抓第二发飞镖
void GrabSecondDart(void){
     //动作一
    if(grab_seconddart_stage==0){
        if(grab_seconddart_flag_2==0){
            // char tx_buff[]="{\"T\":1,\"P1\":164.5,\"P2\":51.328,\"P3\":66.79,\"P4\":49.57,\"P5\":250,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":400,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":125}";
            // HAL_UART_Transmit(&huart8,(uint8_t *)tx_buff,sizeof(tx_buff),1600);
            HAL_UART_Transmit(&huart8,(uint8_t *)move_buff_7,sizeof(move_buff_7),1600);
            grab_seconddart_flag_2=1;
        }
        grab_seconddart_wait_count++;
        if(grab_seconddart_wait_count>1.6*FREQUENCE){
            grab_seconddart_stage=1;
            grab_seconddart_wait_count=0;
            grab_seconddart_flag_2=0;
        }
    }
    //动作二
    if(grab_seconddart_stage==1){
        if(grab_seconddart_flag_2==0){
            // char tx_buff[]="{\"T\":1,\"P1\":164.5,\"P2\":36,\"P3\":85,\"P4\":49,\"P5\":250,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
            // HAL_UART_Transmit(&huart8,(uint8_t *)tx_buff,sizeof(tx_buff),700);
            HAL_UART_Transmit(&huart8,(uint8_t *)move_buff_8,sizeof(move_buff_8),700);
            grab_seconddart_flag_2=1;
        }
        grab_seconddart_wait_count++;
        if(grab_seconddart_wait_count>0.7*FREQUENCE){
            grab_seconddart_stage=2;
            grab_seconddart_wait_count=0;
            grab_seconddart_flag_2=0;
        }
    }
    //动作三
    if(grab_seconddart_stage==2){
        if(grab_seconddart_flag_2==0){
            // char tx_buff[]="{\"T\":1,\"P1\":182.5,\"P2\":36,\"P3\":85,\"P4\":49,\"P5\":250,\"S1\":400,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":80,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
            // HAL_UART_Transmit(&huart8,(uint8_t *)tx_buff,sizeof(tx_buff),900);
            HAL_UART_Transmit(&huart8,(uint8_t *)move_buff_9,sizeof(move_buff_9),900);
            grab_seconddart_flag_2=1;
        }
        grab_seconddart_wait_count++;
        if(grab_seconddart_wait_count>0.9*FREQUENCE){
            grab_seconddart_stage=3;
            grab_seconddart_wait_count=0;
            grab_seconddart_flag_2=0;
        }
    }
    //动作四
    if(grab_seconddart_stage==3){
        if(grab_seconddart_flag_2==0){
            // char tx_buff[]="{\"T\":1,\"P1\":182.5,\"P2\":48.66,\"P3\":77.87,\"P4\":43,\"P5\":250,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
            // HAL_UART_Transmit(&huart8,(uint8_t *)tx_buff,sizeof(tx_buff),600);
            HAL_UART_Transmit(&huart8,(uint8_t *)move_buff_10,sizeof(move_buff_10),600);
            grab_seconddart_flag_2=1;
        }
        grab_seconddart_wait_count++;
        if(grab_seconddart_wait_count>0.6*FREQUENCE){
            grab_seconddart_stage=4;
            grab_seconddart_wait_count=0;
            grab_seconddart_flag_2=0;
        }
    }
    //动作5
    if(grab_seconddart_stage==4){
        if(grab_seconddart_flag_2==0){
            // char tx_buff[]="{\"T\":1,\"P1\":182.5,\"P2\":48.66,\"P3\":77.87,\"P4\":43,\"P5\":210,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
            // HAL_UART_Transmit(&huart8,(uint8_t *)tx_buff,sizeof(tx_buff),600);
            HAL_UART_Transmit(&huart8,(uint8_t *)move_buff_11,sizeof(move_buff_11),600);
            grab_seconddart_flag_2=1;
        }
        grab_seconddart_wait_count++;
        if(grab_seconddart_wait_count>0.6*FREQUENCE){
            grab_seconddart_stage=5;
            grab_seconddart_wait_count=0;
            grab_seconddart_flag_2=0;
        }
    }
    //动作6
    if(grab_seconddart_stage==5){
        if(grab_seconddart_flag_2==0){
            // char tx_buff[]="{\"T\":1,\"P1\":182.5,\"P2\":0,\"P3\":90,\"P4\":90,\"P5\":180,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
            // HAL_UART_Transmit(&huart8,(uint8_t *)tx_buff,sizeof(tx_buff),700);
            HAL_UART_Transmit(&huart8,(uint8_t *)move_buff_12,sizeof(move_buff_12),700);
            grab_seconddart_flag_2=1;
        }
        grab_seconddart_wait_count++;
        if(grab_seconddart_wait_count>0.7*FREQUENCE){
            grab_seconddart_stage=6;
            grab_seconddart_wait_count=0;
            grab_seconddart_flag_2=0;
            grab_seconddart_flag=1;
            grab_dart_num++;
        }
    }
}

//抓第二发飞镖，用于机械臂调试
void GrabSecondDartTest(void){
     //动作一
    if(grab_seconddart_test_stage==0){
        if(grab_seconddart_flag_2_test==0){
            // char tx_buff[]="{\"T\":1,\"P1\":164.5,\"P2\":51.328,\"P3\":66.79,\"P4\":49.57,\"P5\":250,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":400,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":125}";
            // HAL_UART_Transmit(&huart8,(uint8_t *)tx_buff,sizeof(tx_buff),1600);
            HAL_UART_Transmit(&huart8,(uint8_t *)move_buff_7,sizeof(move_buff_7),1600);
            grab_seconddart_flag_2_test=1;
        }
        grab_seconddart_test_waitcount++;
        if(grab_seconddart_test_waitcount>1.6*FREQUENCE){
            grab_seconddart_test_stage=1;
            grab_seconddart_test_waitcount=0;
            grab_seconddart_flag_2_test=0;
        }
    }
    //动作二
    if(grab_seconddart_test_stage==1){
        if(grab_seconddart_flag_2_test==0){
            // char tx_buff[]="{\"T\":1,\"P1\":164.5,\"P2\":36,\"P3\":85,\"P4\":49,\"P5\":250,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
            // HAL_UART_Transmit(&huart8,(uint8_t *)tx_buff,sizeof(tx_buff),700);
            HAL_UART_Transmit(&huart8,(uint8_t *)move_buff_8,sizeof(move_buff_8),700);
            grab_seconddart_flag_2_test=1;
        }
        grab_seconddart_test_waitcount++;
        if(grab_seconddart_test_waitcount>0.7*FREQUENCE){
            grab_seconddart_test_stage=2;
            grab_seconddart_test_waitcount=0;
            grab_seconddart_flag_2_test=0;
        }
    }
    //动作三
    if(grab_seconddart_test_stage==2){
        if(grab_seconddart_flag_2_test==0){
            // char tx_buff[]="{\"T\":1,\"P1\":182.5,\"P2\":36,\"P3\":85,\"P4\":49,\"P5\":250,\"S1\":400,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":80,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
            // HAL_UART_Transmit(&huart8,(uint8_t *)tx_buff,sizeof(tx_buff),900);
            HAL_UART_Transmit(&huart8,(uint8_t *)move_buff_9,sizeof(move_buff_9),900);
            grab_seconddart_flag_2_test=1;
        }
        grab_seconddart_test_waitcount++;
        if(grab_seconddart_test_waitcount>0.9*FREQUENCE){
            grab_seconddart_test_stage=3;
            grab_seconddart_test_waitcount=0;
            grab_seconddart_flag_2_test=0;
        }
    }
    //动作四
    if(grab_seconddart_test_stage==3){
        if(grab_seconddart_flag_2_test==0){
            // char tx_buff[]="{\"T\":1,\"P1\":182.5,\"P2\":48.66,\"P3\":77.87,\"P4\":43,\"P5\":250,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
            // HAL_UART_Transmit(&huart8,(uint8_t *)tx_buff,sizeof(tx_buff),600);
            HAL_UART_Transmit(&huart8,(uint8_t *)move_buff_10,sizeof(move_buff_10),600);
            grab_seconddart_flag_2_test=1;
        }
        grab_seconddart_test_waitcount++;
        if(grab_seconddart_test_waitcount>0.6*FREQUENCE){
            grab_seconddart_test_stage=4;
            grab_seconddart_test_waitcount=0;
            grab_seconddart_flag_2_test=0;
        }
    }
    //动作5
    if(grab_seconddart_test_stage==4){
        if(grab_seconddart_flag_2_test==0){
            // char tx_buff[]="{\"T\":1,\"P1\":182.5,\"P2\":48.66,\"P3\":77.87,\"P4\":43,\"P5\":210,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
            // HAL_UART_Transmit(&huart8,(uint8_t *)tx_buff,sizeof(tx_buff),600);
            HAL_UART_Transmit(&huart8,(uint8_t *)move_buff_11,sizeof(move_buff_11),600);
            grab_seconddart_flag_2_test=1;
        }
        grab_seconddart_test_waitcount++;
        if(grab_seconddart_test_waitcount>0.6*FREQUENCE){
            grab_seconddart_test_stage=5;
            grab_seconddart_test_waitcount=0;
            grab_seconddart_flag_2_test=0;
        }
    }
    //动作6
    if(grab_seconddart_test_stage==5){
        if(grab_seconddart_flag_2_test==0){
            // char tx_buff[]="{\"T\":1,\"P1\":182.5,\"P2\":0,\"P3\":90,\"P4\":90,\"P5\":180,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
            // HAL_UART_Transmit(&huart8,(uint8_t *)tx_buff,sizeof(tx_buff),2000);
            HAL_UART_Transmit(&huart8,(uint8_t *)move_buff_12,sizeof(move_buff_12),2000);
            grab_seconddart_flag_2_test=1;
        }
        grab_seconddart_test_waitcount++;
        if(grab_seconddart_test_waitcount>2*FREQUENCE){
            grab_seconddart_test_stage=0;
            grab_seconddart_test_waitcount=0;
            grab_seconddart_flag_2_test=0;
        }
    }
}

//抓取第三个飞镖
void GrabThirdDart(void){
     //动作一
    if(grab_thirddart_stage==0){
        if(grab_thirddart_flag_2==0){
            // char tx_buff[]="{\"T\":1,\"P1\":148,\"P2\":51.328,\"P3\":66.79,\"P4\":49.57,\"P5\":250,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":400,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":125}";
            // HAL_UART_Transmit(&huart8,(uint8_t *)tx_buff,sizeof(tx_buff),1800);
            HAL_UART_Transmit(&huart8,(uint8_t *)move_buff_13,sizeof(move_buff_13),1800);
            grab_thirddart_flag_2=1;
        }
        grab_thirddart_wait_count++;
        if(grab_thirddart_wait_count>1.8*FREQUENCE){
            grab_thirddart_stage=1;
            grab_thirddart_wait_count=0;
            grab_thirddart_flag_2=0;
        }
    }
    //动作二
    if(grab_thirddart_stage==1){
        if(grab_thirddart_flag_2==0){
            // char tx_buff[]="{\"T\":1,\"P1\":148,\"P2\":36,\"P3\":85,\"P4\":49,\"P5\":250,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
            // HAL_UART_Transmit(&huart8,(uint8_t *)tx_buff,sizeof(tx_buff),700);
            HAL_UART_Transmit(&huart8,(uint8_t *)move_buff_14,sizeof(move_buff_14),700);
            grab_thirddart_flag_2=1;
        }
        grab_thirddart_wait_count++;
        if(grab_thirddart_wait_count>0.7*FREQUENCE){
            grab_thirddart_stage=2;
            grab_thirddart_wait_count=0;
            grab_thirddart_flag_2=0;
        }
    }
    //动作三
    if(grab_thirddart_stage==2){
        if(grab_thirddart_flag_2==0){
            // char tx_buff[]="{\"T\":1,\"P1\":182.5,\"P2\":36,\"P3\":85,\"P4\":49,\"P5\":250,\"S1\":400,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":80,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
            // HAL_UART_Transmit(&huart8,(uint8_t *)tx_buff,sizeof(tx_buff),900);
            HAL_UART_Transmit(&huart8,(uint8_t *)move_buff_15,sizeof(move_buff_15),900);
            grab_thirddart_flag_2=1;
        }
        grab_thirddart_wait_count++;
        if(grab_thirddart_wait_count>0.9*FREQUENCE){
            grab_thirddart_stage=3;
            grab_thirddart_wait_count=0;
            grab_thirddart_flag_2=0;
        }
    }
    //动作四
    if(grab_thirddart_stage==3){
        if(grab_thirddart_flag_2==0){
            // char tx_buff[]="{\"T\":1,\"P1\":182.5,\"P2\":48.66,\"P3\":77.87,\"P4\":43,\"P5\":250,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
            // HAL_UART_Transmit(&huart8,(uint8_t *)tx_buff,sizeof(tx_buff),600);
            HAL_UART_Transmit(&huart8,(uint8_t *)move_buff_16,sizeof(move_buff_16),600);
            grab_thirddart_flag_2=1;
        }
        grab_thirddart_wait_count++;
        if(grab_thirddart_wait_count>0.6*FREQUENCE){
            grab_thirddart_stage=4;
            grab_thirddart_wait_count=0;
            grab_thirddart_flag_2=0;
        }
    }
    //动作5
    if(grab_thirddart_stage==4){
        if(grab_thirddart_flag_2==0){
            // char tx_buff[]="{\"T\":1,\"P1\":182.5,\"P2\":48.66,\"P3\":77.87,\"P4\":43,\"P5\":210,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
            // HAL_UART_Transmit(&huart8,(uint8_t *)tx_buff,sizeof(tx_buff),600);
            HAL_UART_Transmit(&huart8,(uint8_t *)move_buff_17,sizeof(move_buff_17),600);
            grab_thirddart_flag_2=1;
        }
        grab_thirddart_wait_count++;
        if(grab_thirddart_wait_count>0.6*FREQUENCE){
            grab_thirddart_stage=5;
            grab_thirddart_wait_count=0;
            grab_thirddart_flag_2=0;
        }
    }
    //动作6
    if(grab_thirddart_stage==5){
        if(grab_thirddart_flag_2==0){
            // char tx_buff[]="{\"T\":1,\"P1\":182.5,\"P2\":0,\"P3\":90,\"P4\":90,\"P5\":180,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
            // HAL_UART_Transmit(&huart8,(uint8_t *)tx_buff,sizeof(tx_buff),500);
            HAL_UART_Transmit(&huart8,(uint8_t *)move_buff_18,sizeof(move_buff_18),500);
            grab_thirddart_flag_2=1;
        }
        grab_thirddart_test_waitcount++;
        if(grab_thirddart_test_waitcount>0.5*FREQUENCE){
            grab_thirddart_stage=6;
            grab_thirddart_test_waitcount=0;
            grab_thirddart_flag_2=0;
            grab_thirddart_flag=1;
            grab_dart_num++;
        }
    }
}

//抓取第三个飞镖,用于机械臂调试
void GrabThirdDartTest(void){
     //动作一
    if(grab_thirddart_test_stage==0){
        if(grab_thirddart_flag_2_test==0){
            // char tx_buff[]="{\"T\":1,\"P1\":148,\"P2\":51.328,\"P3\":66.79,\"P4\":49.57,\"P5\":250,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":400,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":125}";
            // HAL_UART_Transmit(&huart8,(uint8_t *)tx_buff,sizeof(tx_buff),1800);
            HAL_UART_Transmit(&huart8,(uint8_t *)move_buff_13,sizeof(move_buff_13),1800);
            grab_thirddart_flag_2_test=1;
        }
        grab_thirddart_test_waitcount++;
        if(grab_thirddart_test_waitcount>1.8*FREQUENCE){
            grab_thirddart_test_stage=1;
            grab_thirddart_test_waitcount=0;
            grab_thirddart_flag_2_test=0;
        }
    }
    //动作二
    if(grab_thirddart_test_stage==1){
        if(grab_thirddart_flag_2_test==0){
            // char tx_buff[]="{\"T\":1,\"P1\":148,\"P2\":36,\"P3\":85,\"P4\":49,\"P5\":250,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
            // HAL_UART_Transmit(&huart8,(uint8_t *)tx_buff,sizeof(tx_buff),700);
            HAL_UART_Transmit(&huart8,(uint8_t *)move_buff_14,sizeof(move_buff_14),700);
            grab_thirddart_flag_2_test=1;
        }
        grab_thirddart_test_waitcount++;
        if(grab_thirddart_test_waitcount>0.7*FREQUENCE){
            grab_thirddart_test_stage=2;
            grab_thirddart_test_waitcount=0;
            grab_thirddart_flag_2_test=0;
        }
    }
    //动作三
    if(grab_thirddart_test_stage==2){
        if(grab_thirddart_flag_2_test==0){
            // char tx_buff[]="{\"T\":1,\"P1\":182.5,\"P2\":36,\"P3\":85,\"P4\":49,\"P5\":250,\"S1\":400,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":80,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
            // HAL_UART_Transmit(&huart8,(uint8_t *)tx_buff,sizeof(tx_buff),1000);
            HAL_UART_Transmit(&huart8,(uint8_t *)move_buff_15,sizeof(move_buff_15),900);
            grab_thirddart_flag_2_test=1;
        }
        grab_thirddart_test_waitcount++;
        if(grab_thirddart_test_waitcount>0.9*FREQUENCE){
            grab_thirddart_test_stage=3;
            grab_thirddart_test_waitcount=0;
            grab_thirddart_flag_2_test=0;
        }
    }
    //动作四
    if(grab_thirddart_test_stage==3){
        if(grab_thirddart_flag_2_test==0){
            // char tx_buff[]="{\"T\":1,\"P1\":182.5,\"P2\":48.66,\"P3\":77.87,\"P4\":43,\"P5\":250,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
            // HAL_UART_Transmit(&huart8,(uint8_t *)tx_buff,sizeof(tx_buff),600);
            HAL_UART_Transmit(&huart8,(uint8_t *)move_buff_16,sizeof(move_buff_16),600);
            grab_thirddart_flag_2_test=1;
        }
        grab_thirddart_test_waitcount++;
        if(grab_thirddart_test_waitcount>0.6*FREQUENCE){
            grab_thirddart_test_stage=4;
            grab_thirddart_test_waitcount=0;
            grab_thirddart_flag_2_test=0;
        }
    }
    //动作5
    if(grab_thirddart_test_stage==4){
        if(grab_thirddart_flag_2_test==0){
            // char tx_buff[]="{\"T\":1,\"P1\":182.5,\"P2\":48.66,\"P3\":77.87,\"P4\":43,\"P5\":210,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
            // HAL_UART_Transmit(&huart8,(uint8_t *)tx_buff,sizeof(tx_buff),600);
            HAL_UART_Transmit(&huart8,(uint8_t *)move_buff_17,sizeof(move_buff_17),600);
            grab_thirddart_flag_2_test=1;
        }
        grab_thirddart_test_waitcount++;
        if(grab_thirddart_test_waitcount>0.6*FREQUENCE){
            grab_thirddart_test_stage=5;
            grab_thirddart_test_waitcount=0;
            grab_thirddart_flag_2_test=0;
        }
    }
    //动作6
    if(grab_thirddart_test_stage==5){
        if(grab_thirddart_flag_2_test==0){
            // char tx_buff[]="{\"T\":1,\"P1\":182.5,\"P2\":0,\"P3\":90,\"P4\":90,\"P5\":180,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
            // HAL_UART_Transmit(&huart8,(uint8_t *)tx_buff,sizeof(tx_buff),2000);
            HAL_UART_Transmit(&huart8,(uint8_t *)move_buff_18,sizeof(move_buff_18),2000);
            grab_thirddart_flag_2_test=1;
        }
        grab_thirddart_test_waitcount++;
        if(grab_thirddart_test_waitcount>2*FREQUENCE){
            grab_thirddart_test_stage=0;
            grab_thirddart_test_waitcount=0;
            grab_thirddart_flag_2_test=0;
        }
    }
}

//机械臂初始化位置为第一个镖体旁边,暂时先不用这个了
void GrabInit(void){
    char tx_buff[]="{\"T\":1,\"P1\":181.5,\"P2\":0,\"P3\":90,\"P4\":90,\"P5\":180,\"S1\":0,\"S2\":0,\"S3\":0,\"S4\":0,\"S5\":0,\"A1\":0,\"A2\":0,\"A3\":0,\"A4\":0,\"A5\":0}";
    HAL_UART_Transmit(&huart8,(uint8_t *)tx_buff,sizeof(tx_buff),1000);
}