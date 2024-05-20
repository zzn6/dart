/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-01-20 16:39:58
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-03-12 14:38:34
 * @FilePath: \dartrack1\Motor\Inc\HW_can.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MOTOR_TASK_HPP_
#define __MOTOR_TASK_HPP_

// #ifdef __cplusplus
// extern "C" {
// #endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
/* Exported macro ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/

extern const uint32_t kM3508RxId1 ;  // 电机反馈报文 ID
extern const uint32_t kM3508RxId2 ;
extern const uint32_t kM2006RxId;

extern const uint32_t kM3508TxId;  // 电机控制报文 ID

extern uint8_t m3508_set_zeropoint_flag;

/* Exported types ------------------------------------------------------------*/

//电机当前状态
typedef struct _motor_state_t {
  float angular_velocity;  // 电机角速度，单位：rad/s
  float angle;             // 电机角度，单位：rad
  int16_t feedback_current;//返回的实际转矩电流
} MotorState_t;

extern MotorState_t M3508_state_1;
extern MotorState_t M3508_state_2;
extern MotorState_t M2006_state;
extern float m3508_angle [2];
extern float m2006_angle;
extern float m2006_velocity;

/* Exported variables --------------------------------------------------------*/
/* Exported function prototypes ----------------------------------------------*/

void CAN_FilterInit(CAN_HandleTypeDef* hcan);

void CAN_SendMsg(CAN_HandleTypeDef* hcan, uint32_t id, uint8_t* msg, uint8_t len);

void MotorInit();

//void M3508SetZero(void);

// #ifdef __cplusplus
// }
// #endif

#endif /* __HW_CAN_H_ */
