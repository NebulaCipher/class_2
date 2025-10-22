/* Includes ------------------------------------------------------------------*/
#include "HW_fdcan.hpp"
#include "stdint.h"
#include "gm6020.hpp" // 引入GM6020类

/* Private macro -------------------------------------------------------------*/
/* Private constants ---------------------------------------------------------*/
/* Private types -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/

/* External variables --------------------------------------------------------*/
// 声明CubeMX生成的FDCAN句柄
extern FDCAN_HandleTypeDef hfdcan1;
extern FDCAN_HandleTypeDef hfdcan2;
extern FDCAN_HandleTypeDef hfdcan3;
// 声明外部电机实例
extern GM6020 motor_m1;

/* Private function prototypes -----------------------------------------------*/

/**
 * @brief GM6020 构造函数
 * @param id 电机反馈的 CAN ID
 */
GM6020::GM6020(uint32_t id)
    : id_(id), input_(0), angle_(0), vel_(0), current_(0), temp_(0)
{
}

/**
 * @brief 解析 GM6020 电机反馈报文 (8字节)
 * GM6020反馈报文格式:
 * 字节0-1: 编码器位置 (0-8191)
 * 字节2-3: 速度 (RPM)
 * 字节4-5: 扭矩/电流
 * 字节6: 温度 (摄氏度)
 * 字节7: 保留
 * @param data 接收到的 8 字节 CAN 数据
 * @return 成功返回 true
 */
bool GM6020::decode(uint8_t* data)
{
    if (!data) return false;
    
    // 大端模式解析（高位在前）
    int16_t angle_raw = (data[0] << 8) | data[1];
    int16_t speed_raw = (data[2] << 8) | data[3];
    int16_t current_raw = (data[4] << 8) | data[5];
    uint8_t temp_raw = data[6];

    // 原始编码器值 (0-8191) 转换为角度 (度)
    // 假设 8192 对应一圈 360 度

    angle_ = (4096 - angle_raw) * (2 * M_PI) / 8192.0f;      
    
    // GM6020 速度单位通常是 RPM (转/分钟)
    vel_ = speed_raw; 
    
    // 电流单位通常是原始值 (取决于具体型号，可能与编码值有关)
    current_ = current_raw; 
    
    temp_ = temp_raw;
    
    return true;
}

/**
 * @brief 封装 GM6020 电机控制报文 (8字节)
 * 报文格式（ID 0x1FF 或 0x200）：
 * 字节0-1: 电机1的电流指令 (Input_)
 * 字节2-3: 电机2的电流指令 (Motor 2)
 * 字节4-7: 其他电机（置0）
 *
 * @param data 发送的 8 字节 CAN 数据
 * @return 成功返回 true
 */
bool GM6020::encode(uint8_t* data)
{
    if (!data) return false;
    
    // 1. 将存储的期望电流 (input_) 转换为 GM6020 需要的 16位整数格式
    // input_ 的值来自 PID 输出，假设已经在 motor_control.cpp 中限幅在 ±20000 内
    int16_t send_current = static_cast<int16_t>(input_); 
    
    // 2. 将 16位整数拆分成高低字节，写入 CAN 报文的前两个字节
    //    由于 PackMotorCurrents 负责提供正确的偏移，ID为3,这里只处理 data[4] 和 data[5]
    data[4] = (send_current >> 8) & 0xFF;  // 高字节
    data[5] = send_current & 0xFF;         // 低字节
    
    return true;
}

