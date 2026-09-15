#ifndef APP_TASK_H
#define APP_TASK_H

#include <stdint.h>

// FreeRTOS任务空入口，队友可在同名非weak函数中重写。
uint32_t ManagerTaskInit(void);
void ManagerTaskLoop(void);
uint32_t GimbalTaskInit(void);
void GimbalTaskLoop(void);
uint32_t ChassisTaskInit(void);
void ChassisTaskLoop(void);
uint32_t ShootTaskInit(void);
void ShootTaskLoop(void);

#endif  // APP_TASK_H
