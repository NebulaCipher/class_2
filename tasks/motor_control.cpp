#include "motor_control.hpp"
#include "pid.hpp"
#include "gm6020.hpp"
#include "HW_fdcan.hpp"
#include <cmath>
#include <cstring>

// 宏定义：GM6020的CAN发送ID (控制 ID 1-4)
#define GM6020_TX_ID_1_4 0x1FE 

extern FDCAN_HandleTypeDef hfdcan1;
extern TIM_HandleTypeDef htim6;

// 实例化电机：假设我们只控制电机1 (ID: 0X1FE)
GM6020 motor_m1(0x1FE); 

// PID 参数定义
// 1. 速度环 PID (输出电流)
PidParams speed_pid_params = {
    .kp = 13.0f, 
    .ki = 0.01f, 
    .kd = 0.0f,
    .maxOut = 16384.0f, // GM6020最大输出电流值（原始值）
    .minOut = -16384.0f 
};
// 2. 位置环 PID (输出速度)
PidParams pos_pid_params = {
    .kp = 140.0f, 
    .ki = 0.01f, 
    .kd = 0.0f,
    .maxOut = 500.0f, // 最大速度 (RPM)
    .minOut = -500.0f
};

// 实例化PID控制器
Pid speed_pid(speed_pid_params);
Pid pos_pid(pos_pid_params);


/**
 * @brief  将电机的电流指令封装到CAN报文中
 * @param  m1: 电机1实例
 * @param  data: 输出的8字节CAN报文
 */
static void PackMotorCurrents(GM6020& m1, uint8_t *data) {
    std::memset(data, 0, 8);
    // GM6020的CAN报文格式：
    
    // 调用GM6020::encode，处理报文并设置电流
    m1.encode(data); 
}

// 控制模式切换，1为速度闭环，2为位置闭环
static uint8_t control_mode = 2; 

/**
 * @brief  速度闭环控制
 */
static void SpeedControl_Process(void) {
    // 定义一个正弦速度曲线作为参考值
    static uint32_t ms_count = 0;
    ms_count++; 
    // 幅值为 100 RPM，周期为 2000ms
    static float ref_vel;
    ref_vel = 100.0f * sinf(2.0f * M_PI * (float)(ms_count % 2000) / 2000.0f); 
    // 1. 获取电机当前速度
    static float fdb_vel;
    fdb_vel = motor_m1.vel();
    
    // 2. 计算速度 PID 输出 (电流指令)
    float current_input = speed_pid.pidCalc(ref_vel, fdb_vel, 0);
    
    // 3. 设置电机期望电流
    motor_m1.setInput(current_input);

}

/**
 * @brief  位置闭环控制
 */
static void PositionControl_Process(void) {
    // 角度目标值 (电机1)
    // static float target_angle_rad = M_PI / 6.0f; // 初始目标 pi/6 弧度
    static float target_angle_rad = M_PI / 3.0f;
    // static float target_angle_rad = M_PI / 4.0f;

    // 实现第一个要求: $-\5pi/6$ 到 $\5pi/6$ 切换运动
    // 将弧度转换为角度 (GM6020::angle 返回的是角度值)
    // static float target_angle_deg;
    // target_angle_deg = target_angle_rad * 180.0f / M_PI;
    
    // 1. 位置环 (外环)
    static float fdb_angle2;
    fdb_angle2 = motor_m1.angle();
    // 位置环输出速度指令 (RPM)
    static float ref_vel2;
    ref_vel2 = pos_pid.pidCalc(target_angle_rad, fdb_angle2, 1); 

    // 2. 速度环 (内环)
    static float fdb_vel2;
    fdb_vel2 = motor_m1.vel();
    // 速度环输出电流指令 (原始值)
    float current_input = speed_pid.pidCalc(ref_vel2, fdb_vel2, 0);

    // 3. 设置电机期望电流
    motor_m1.setInput(current_input);

    // 切换目标：每 2000ms 切换一次 (200次 MotorControl_Task)
    static uint16_t switch_count = 0;
    switch_count++;
    if (switch_count >= 200) {
        switch_count = 0;
        // target_angle_rad = (target_angle_rad > 0) ? -5*M_PI / 6.0f : 5*M_PI / 6.0f;
        target_angle_rad = (target_angle_rad > M_PI / 2.0f) ? M_PI / 3.0f : 2*M_PI / 3.0f;
        // target_angle_rad = (target_angle_rad > 0) ? -M_PI : M_PI / 4.0f;
    }
}


/**
 * @brief  控制任务初始化
 */
void MotorControl_Init(void) {

    HAL_FDCAN_Start(&hfdcan1);

    // 1. 配置 FDCAN1 滤波器以接收电机反馈消息
    FdcanFilterInit(&hfdcan1, FDCAN_FILTER_TO_RXFIFO0); 

    // 2. 启动 CAN 接收中断
    HAL_FDCAN_ActivateNotification(&hfdcan1, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0);

    // 3. 启动定时器 6，用于周期性调用 MotorControl_Task
    HAL_TIM_Base_Start_IT(&htim6);

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim->Instance == TIM6) {
        MotorControl_Task();
    }
}
/**
 * @brief  主控制任务，需要周期性调用
 */
void MotorControl_Task(void) {
    // 1. 选择控制模式
    if (control_mode == 1) {
        SpeedControl_Process(); // 速度闭环
    } else {
        PositionControl_Process(); // 位置闭环
    }

    //数据封装
    uint8_t tx_data[8];
    PackMotorCurrents(motor_m1, tx_data); 

    //数据发送，FdcanSendMsg函数启动发送过程，假设所有电机都连接在 hfdcan1 上，发送 ID 为 0x1FE
    FdcanSendMsg(&hfdcan1, tx_data, GM6020_TX_ID_1_4, 8);  


}