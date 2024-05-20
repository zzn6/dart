/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-04-06 16:53:31
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-05-13 15:07:52
 * @FilePath: \dartrack1\tasks\Inc\referee_task.hpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#ifndef __REFEREE_TASK_HPP__
#define __REFEREE_TASK_HPP__

// #ifdef __cplusplus
// extern "C" 
// #endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "referee.hpp"
#include "rfr_official_pkgs.hpp"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */
namespace rfr = hello_world::referee;

extern rfr::RfrDecoder rfr_decoder;
extern rfr::TeamDartStatusPackage team_dart_status_pkg;
extern rfr::RobotDartClientCmdPackage robot_dart_client_cmd_pkg;
extern rfr::CompStatusPackage com_status_pkg;
// extern rfr::RobotDartClientCmdPackage::Data dart_station_info;
extern rfr::RobotDartClientCmdData dart_station_info;
extern rfr::TeamDartStatusData team_dart_status_info;
extern rfr::CompStatusData com_status_info;
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

void RfrDecoderInit(void);
//void SendCmdToAerial(void) ;

/* USER CODE BEGIN Prototypes */



/* USER CODE END Prototypes */

// #ifdef __cplusplus
// 
// #endif

#endif
