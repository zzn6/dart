/*
 * @Author: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @Date: 2024-04-06 16:53:15
 * @LastEditors: error: error: git config user.name & please set dead value or install git && error: git config user.email & please set dead value or install git & please set dead value or install git
 * @LastEditTime: 2024-05-13 09:28:54
 * @FilePath: \dartrack1\tasks\Src\referee_task.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include "referee_task.hpp"
#include "rfr_pkg_dart2aerial.hpp"
#include "usart.h"

rfr::RfrDecoder rfr_decoder;
rfr::RfrEncoder rfr_encoder;

uint8_t tx_data[hello_world::referee::kRefereeMaxFrameLength]={0};
size_t tx_len = 0;

// pkgs
// rfr::RobotPerformancePackage robot_perf_pkg;
// rfr::RobotPowerHeatPackage robot_power_heat_pkg;
// rfr::RobotShooterPackage robot_shooter_pkg;
rfr::TeamDartStatusPackage team_dart_status_pkg;//飞镖机器人状态数据包
rfr::RobotDartClientCmdPackage robot_dart_client_cmd_pkg;//飞镖客户端命令数据包
rfr::CompStatusPackage com_status_pkg;//比赛状态数据包
rfr::Dart2AerialCmdPackage dart_to_aerial_pkg;
// rfr::InterGraphic7Package inter_graphic7_pkg;
// more pkgs...
rfr::RobotDartClientCmdData dart_station_info;//飞镖客户端命令数据
rfr::TeamDartStatusData team_dart_status_info;//飞镖机器人状态数据
rfr::CompStatusData com_status_info;//比赛状态数据
rfr::DartToAerialCmdData dart_to_aerial_cmd_info;

void RfrDecoderInit(void) {
    // add pkgs
    // rfr_decoder.appendRxPackage(&robot_perf_pkg);
    // rfr_decoder.appendRxPackage(&robot_power_heat_pkg);
    // rfr_decoder.appendRxPackage(&robot_shooter_pkg);
    rfr_decoder.appendRxPackage(&team_dart_status_pkg);
    rfr_decoder.appendRxPackage(&robot_dart_client_cmd_pkg);
    rfr_decoder.appendRxPackage(&com_status_pkg);
    //dart_station_info.dart_launch_opening_status=10;
    //team_dart_status_info.dart_remaining_time=20;

    //rfr::TeamDartStatusData::dart_remaining_time
    // more pkgs...
    
}

// // not valid callback function, just for example
// void UartCallbackWithDataArray(uint8_t* data_ptr, uint16_t len) {
//     // method 1: use pointer to data array
//     // ! 请注意，当多个包在短时间内同时达到串口，但包与包之间的间隔太短，并不能触发 DMA 的空闲中断
//     // ！而当前版本中的 decodeFrame 函数在成功解包一次之后就会跳过之后的字节流，哪怕后续字节流中还有有效数据
//     // ! 因此，哪怕是在 DMA 空闲中断中解包，也建议使用方法 2 解包，即逐字节解包
//     // rfr_decoder.decodeFrame(data_ptr, len);
//     // method 2: input byte one by one [recommended]
//     // rfr_decoder.processByte()
//     for (uint16_t i = 0; i < len; i++) {
//         rfr_decoder.processByte(data_ptr[i]);
//     } 
// }

// void SendCmdToAerial(void) {
//     // encode pkgs
//     // if(rfr_encoder.encodeFrame(&dart_to_aerial_pkg, tx_data, &tx_len)==true){
//     //     HAL_UART_Transmit(&huart6,tx_data,tx_len,100);
//     // }
//     //rfr_encoder.encodeFrame(&dart_to_aerial_pkg, tx_data, &tx_len);
//     //HAL_UART_Transmit(&huart6,tx_data,135,100);
//     HAL_UART_Transmit_IT(&huart6,tx_data,135);
//     // send data(not valid function, just for example)
    
// };
