/**
 *******************************************************************************
 * @file      :HW_fdcan.hpp
 * @brief     :
 * @history   :
 * Version     Date            Author          Note
 * V1.0.0      yyyy-mm-dd      <author>        1。<note>
 *******************************************************************************
 * @attention :
 *******************************************************************************
 * Copyright (c) 2025 Hello World Team,Zhejiang University.
 * All Rights Reserved.
 *******************************************************************************
 */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef _HW_FDCAN_H_
#define _HW_FDCAN_H_
/* Includes ------------------------------------------------------------------*/
#include "fdcan.h"
#ifdef __cplusplus
#include "gm6020.hpp" // 引入GM6020类定义
// 将 C/C++ 共享的函数声明放入 extern "C" 块，防止 C++ 编译器名称修饰
extern "C" { 
#endif

/* Exported macro ------------------------------------------------------------*/
/* Exported types ------------------------------------------------------------*/
void FdcanFilterInit(FDCAN_HandleTypeDef *hfdcan, uint32_t fifo);

void FdcanSendMsg(FDCAN_HandleTypeDef *hfdcan, uint8_t *msg, uint32_t id,
                  uint8_t len);

#ifdef __cplusplus
} // 结束 extern "C" 块

/* Exported variables --------------------------------------------------------*/
// 仅在 C++ 环境下声明 C++ 对象
// 声明将在 motor_control.cpp 中定义的电机实例
extern GM6020 motor_m1;


#endif /* __cplusplus */

#endif /* _HW_FDCAN_H_ */