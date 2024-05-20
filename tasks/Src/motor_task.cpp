/* Includes ------------------------------------------------------------------*/
#include "motor_task.hpp"
#include "can.h"
#include "motor_base.hpp"
#include "motor.hpp"
#include "servo_task.hpp"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/

static const float pi = 3.1415926f;

static const float kRaw2RadCoff = 2 * pi / 8191;  // 电机原始角度范围到弧度的转换系数
static const float kRpm2RadpsCoff = 2 * pi / 60;  // rpm 到 rad/s 的转换系数
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

MotorState_t M3508_state_1;
MotorState_t M3508_state_2;
MotorState_t M2006_state;//M2006

/* External variables --------------------------------------------------------*/

const uint32_t kM3508RxId1 = 0x201;  // 电机反馈报文 ID,左边2，右边1
const uint32_t kM3508RxId2 = 0x202;
const uint32_t kM2006RxId = 0x203;
const uint32_t kM3508TxId = 0x200;  // 电机控制报文 ID

float m3508_angle [2];//3508当前角度
float m2006_angle;//M2006输出端角度，rad
float m2006_velocity;//M2006输出端角速度，rad/s
// float m2006_feedback_current;
uint8_t m3508_set_zeropoint_flag=0;//指示两个3508是否已经全部设为零点，0表示还未设为零点
uint8_t m3508_set_zeropoint_flag_1=0;//指示3508是否需要设为零点，0表示需要,用于id1
uint8_t m3508_set_zeropoint_flag_2=0;//指示3508是否需要设为零点，0表示需要，用于id2
uint8_t m2006_set_zeropoint_flag=0;//指示2006是否需要设为零点，0表示需要，用于id3
//uint32_t m3508_set_zeropoint_waitcount=0;

namespace motor = hello_world::motor;
motor::Motor* motor_ptr = nullptr;
motor::Motor* motor_ptr_2 = nullptr;
motor::Motor* motor_ptr_3 = nullptr;//M2006

motor::OptionalParams optional_param = {
    .input_type = motor::kInputTypeRaw,                     // 报文原始输入
    .angle_range = motor::kAngleRangeNegInfToPosInf,           // 角度范围 [-无穷, +无穷)
    .dir = motor::kDirRev,                                   // 电机正方向与规定正方向相反
    .remove_build_in_reducer = false,                         // bu移除自带减速器
    .angle_offset = 0,//0.16,                                      // 电机输出端实际角度与规定角度的差为 0
    .ex_redu_rat = 1,                                        // 电机额外加的减速器减速比为 1
    .max_raw_input_lim = std::numeric_limits<float>::max(),  // 不对报文进行限制
    .max_torq_input_lim = 1000.0f,                              // 限制输出端最大输出力矩为 3N·m
    .max_curr_input_lim = 1000.0f,                             // 限制电机最大电流为 10A
};

motor::OptionalParams optional_param_2 = {
    .input_type = motor::kInputTypeRaw,                     // 报文原始输入
    .angle_range = motor::kAngleRangeNegInfToPosInf,           // 角度范围 [-无穷, +无穷)
    .dir = motor::kDirFwd,                                   // 电机正方向与规定正方向相反
    .remove_build_in_reducer = false,                         // bu移除自带减速器
    .angle_offset = 0,//-0.16,                                      // 电机输出端实际角度与规定角度的差为 0
    .ex_redu_rat = 1,                                        // 电机额外加的减速器减速比为 1
    .max_raw_input_lim = std::numeric_limits<float>::max(),  // 不对报文进行限制
    .max_torq_input_lim = 1000.0f,                              // 限制输出端最大输出力矩为 3N·m
    .max_curr_input_lim = 1000.0f,                             // 限制电机最大电流为 10A
};

motor::OptionalParams optional_param_3 = {
    .input_type = motor::kInputTypeRaw,                     // 报文原始输入
    .angle_range = motor::kAngleRangeNegInfToPosInf,           // 角度范围 [-无穷, +无穷)
    .dir = motor::kDirRev,                                   // 电机正方向与规定正方向相反
    .remove_build_in_reducer = false,                         // bu移除自带减速器
    .angle_offset = 0,//0.16,                                      // 电机输出端实际角度与规定角度的差为 0
    .ex_redu_rat = 1,                                        // 电机额外加的减速器减速比为 1
    .max_raw_input_lim = std::numeric_limits<float>::max(),  // 不对报文进行限制
    .max_torq_input_lim = 1000.0f,                              // 限制输出端最大输出力矩为 3N·m
    .max_curr_input_lim = 1000.0f,                             // 限制电机最大电流为 10A
};
/* Private function prototypes -----------------------------------------------*/

/**
 * @brief       进行 CAN 过滤器的初始化
 * @param        hcan: CAN的句柄
 * @retval       None
 * @note        使用 CAN 接收时必须调用该函数对过滤器进行配置，否则将无法接收到数据，目前只使用 CAN1
 */
void CAN_FilterInit(CAN_HandleTypeDef* hcan)
{
  CAN_FilterTypeDef can_filter;

  /* 将过滤器配置为任何 ID 都接收 */
  can_filter.FilterMode = CAN_FILTERMODE_IDMASK;
  can_filter.FilterScale = CAN_FILTERSCALE_32BIT;
  can_filter.FilterIdHigh = 0x0000;
  can_filter.FilterIdLow = 0x0000;
  can_filter.FilterMaskIdHigh = 0x0000;
  can_filter.FilterMaskIdLow = 0x0000;

  can_filter.FilterActivation = ENABLE;               // 开启过滤器
  can_filter.SlaveStartFilterBank = 14;
  can_filter.FilterBank = 0;

  if(hcan==&hcan1)
  {
    can_filter.FilterBank = 0;
    can_filter.FilterFIFOAssignment = CAN_FilterFIFO0;  // 对FIFO0进行配置
  }
//  else if(hcan==&hcan2)
//  {
//    can_filter.FilterBank = 14;
//    can_filter.FilterFIFOAssignment = CAN_FILTER_FIFO1;  // 对FIFO1进行配置
//  }

  /* 进行 CAN 过滤器配置 */
  if (HAL_CAN_ConfigFilter(hcan, &can_filter) != HAL_OK) {
    Error_Handler();
  }
}

/**
 * @brief       利用 CAN 发送信息
 * @param        id: CAN 的 ID
 * @param        hcan: CAN的句柄
 * @param        msg: 待发送的数据
 * @param        len: 待发送的数据长度
 * @retval       None
 * @note        None
 */
void CAN_SendMsg(CAN_HandleTypeDef* hcan, uint32_t id, uint8_t* msg, uint8_t len)
{
  CAN_TxHeaderTypeDef tx_header = {0}; 
      tx_header.DLC = len;
      tx_header.IDE = CAN_ID_STD;    // 标准帧
      tx_header.RTR = CAN_RTR_DATA;  // 数据帧
      tx_header.StdId = id;

  uint32_t tx_mailbox=0;
  if (HAL_CAN_AddTxMessage(hcan, &tx_header, msg, &tx_mailbox) != HAL_OK) {
    // TODO(Hello World): 可添加发送失败处理
    //ServoSetAngle(-10);
  }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef* hcan)
{
  if (hcan == &hcan1) {
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)  // 获得接收到的数据头和数据
    {
        //判断两个3508是否都设置了零点
        if(m3508_set_zeropoint_flag_1==1&&m3508_set_zeropoint_flag_2==1){
          m3508_set_zeropoint_flag=1;
        }

        //电机有收到数据后置零点
      	if(((motor_ptr->decode(rx_data, rx_header.StdId))==hello_world::motor::kStateOk)&&m3508_set_zeropoint_flag_1==0)
        {
          //M3508SetZero();
          motor_ptr->setAngleValue(0);
          m3508_set_zeropoint_flag_1=1;
        }

        //电机有收到数据后置零点
        if(((motor_ptr_2->decode(rx_data, rx_header.StdId))==hello_world::motor::kStateOk)&&m3508_set_zeropoint_flag_2==0)
        {
          //M3508SetZero();
          motor_ptr_2->setAngleValue(0);
          m3508_set_zeropoint_flag_2=1;
        }

        //电机有收到数据后置零点
        if(((motor_ptr_3->decode(rx_data, rx_header.StdId))==hello_world::motor::kStateOk)&&m2006_set_zeropoint_flag==0)
        {
          //M3508SetZero();
          motor_ptr_3->setAngleValue(0);
          m2006_set_zeropoint_flag=1;
        }

      //motor_ptr_2->decode(rx_data, rx_header.StdId);
      m3508_angle[0]=motor_ptr->angle();
      m3508_angle[1]=motor_ptr_2->angle();
      m2006_angle=motor_ptr_3->angle();
      m2006_velocity=motor_ptr_3->vel();
      //m2006_feedback_current=motor_ptr_3->curr();
      /* 帧头校验 */
      if (rx_header.StdId == kM3508RxId1) {
        int16_t raw_angle = (rx_data[0] << 8) | rx_data[1];
        M3508_state_1.angle = raw_angle * kRaw2RadCoff;

        int16_t angular_velocity_rpm = (rx_data[2] << 8) | rx_data[3];
        float angular_velocity = angular_velocity_rpm * kRpm2RadpsCoff;

        /* 角度归一化到[π, -π) */
        if (M3508_state_1.angle > M_PI) {
        	M3508_state_1.angle-= 2 * M_PI;
        }
        M3508_state_1.angular_velocity = angular_velocity;  // 避免读出未处理完成的中间变量

        int16_t feedbk_current=(rx_data[4] << 8) | rx_data[5];
        M3508_state_1.feedback_current=feedbk_current;
      }
        /* 帧头校验 */
      else if (rx_header.StdId == kM3508RxId2) {
        int16_t raw_angle = (rx_data[0] << 8) | rx_data[1];
       M3508_state_2.angle = raw_angle * kRaw2RadCoff;

        int16_t angular_velocity_rpm = (rx_data[2] << 8) | rx_data[3];
        float angular_velocity = angular_velocity_rpm * kRpm2RadpsCoff;

        /* 角度归一化到[π, -π) */
        if (M3508_state_2.angle > M_PI) {
        	M3508_state_2.angle-= 2 * M_PI;
        }
        M3508_state_2.angular_velocity = angular_velocity;  // 避免读出未处理完成的中间变量

        int16_t feedbk_current=(rx_data[4] << 8) | rx_data[5];
        M3508_state_2.feedback_current=feedbk_current;
      }
        /* 帧头校验 */
      else if (rx_header.StdId == kM2006RxId) {
        int16_t raw_angle = (rx_data[0] << 8) | rx_data[1];
        M2006_state.angle = raw_angle * kRaw2RadCoff;

        int16_t angular_velocity_rpm = (rx_data[2] << 8) | rx_data[3];
        float angular_velocity = angular_velocity_rpm * kRpm2RadpsCoff;

        /* 角度归一化到[π, -π) */
        if (M2006_state.angle > M_PI) {
        	M2006_state.angle-= 2 * M_PI;
        }
        M2006_state.angular_velocity = angular_velocity;  // 避免读出未处理完成的中间变量
      }
    }
  }

  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING);  // 再次使能FIFO0接收中断
}


//void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef* hcan)
//{
//  if (hcan == &hcan2) {
//    CAN_RxHeaderTypeDef rx_header;
//    uint8_t rx_data[8];
//    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO1, &rx_header, rx_data) == HAL_OK)  // 获得接收到的数据头和数据
//    {
//        /* 帧头校验 */
//       if (rx_header.StdId == kPITCHRxId) {
//          motor_pitch.decode(&motor_pitch, rx_data, kMotorPitchParams.kMasterID);
//      }
//    }
//
//  HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO1_MSG_PENDING);  // 再次使能FIFO1接收中断
//}
//}

void MotorInit(){
   motor_ptr = new motor::M3508(1, optional_param);
   motor_ptr_2 = new motor::M3508(2, optional_param_2);
   motor_ptr_3 = new motor::M2006(3,optional_param_3);
}

// //把3508上电时的位置设定为零位
// void M3508SetZero(void){
// 		motor_ptr->setAngleValue(0);
// 		motor_ptr_2->setAngleValue(0);
// }






