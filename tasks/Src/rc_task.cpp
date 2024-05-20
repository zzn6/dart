/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-02-28 17:51:21
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-05-16 22:59:13
 * @FilePath: \dartrack1\tasks\Src\rc_task.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "rc_task.hpp"
#include "stm32f4xx_hal.h"
#include "usart.h"
#include "referee_task.hpp"
#include "light_receive_task.hpp"
#include "task.hpp"
//#include "iwdg.h"
/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

static const uint8_t kRxBufLen = remote_control::kRcRxDataLen;
static uint8_t rx_buf[kRxBufLen];
/* External variables --------------------------------------------------------*/

remote_control::DT7* rc_ptr;
/* Private function prototypes -----------------------------------------------*/

void RcInit(void)
{
  rc_ptr = new remote_control::DT7;

  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buf, kRxBufLen);
}

void RcTask(void)
{
  HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buf, kRxBufLen);
}

//包含遥控器和裁判系统串口的回调
uint16_t wheel_raw = 0;
float wheel_num;
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t Size)
{
  if (huart == &huart1) {
    if (Size == remote_control::kRcRxDataLen) {
        //HAL_IWDG_Refresh(&hiwdg);
        system_click=0;
        rc_ptr->decode(rx_buf);
        wheel_raw = (uint16_t)(rx_buf[16] | (rx_buf[17] << 8));
        wheel_num=rc_ptr->rc_wheel();
    }

    HAL_UARTEx_ReceiveToIdle_DMA(&huart1, rx_buf, kRxBufLen);
  }
  else if (huart == &huart6)
	{
    for (uint16_t i = 0; i < Size; i++) {
      rfr_decoder.processByte(rfr_rx_buffer[i]);
    } 
    for(uint16_t i=0;i<Size;i++){
      rfr_rx_buffer[i]=0;
    }
    dart_station_info=robot_dart_client_cmd_pkg.getData();
    team_dart_status_info=team_dart_status_pkg.getData();
    com_status_info=com_status_pkg.getData();
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, rfr_rx_buffer, 255);
  }
  else if (huart == &huart7)
	{
    if(Size == 10)
    {
      LightInfoReceiveCheck();
      LightDataProcess();
      for(uint16_t i=0;i<Size;i++)
			{
				camera_rx_buffer[i]=0;
	    }
    }
    HAL_UARTEx_ReceiveToIdle_DMA(&huart7, camera_rx_buffer, 10);
  }
}
