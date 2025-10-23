#pragma once

// 声明控制任务函数，供 main.c调用
#ifdef __cplusplus
extern "C" {
#endif

void MotorControl_Init(void);
void MotorControl_Task(void); // 周期性任务，在其中切换/执行速度或位置闭环

#ifdef __cplusplus
}
#endif