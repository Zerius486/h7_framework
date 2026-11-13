/**
 * @file app_robot.c
 * @brief 机器人具体业务层实现
 * @attention
 * 机器人业务逻辑、控制策略和任务接口在本文件中实现。
 * app_config.h/.c、app_protocol.h/.c 与本文件平级，配置参数和通信协议。
 * 框架暂不支持的业务功能在本文件中实现。
 * 如果后续需要复用，可以在提出Issue后，进行代码的重构和优化。
 */

#include "app_task.h"

#include "app_config.h"
#include "bsp_iwdg.h"
#include "dev_peripheral.h"
#include "stm32h7xx_hal.h"

/**
 * @brief 初始化管理任务
 * @return 任务周期（ms）
 */
uint32_t ManagerTaskInit(void)
{
  PeripheralManagerInit();
  (void)BspIwdgInit(500U);
  return g_app_task_config.manager_period_ms;
}

/**
 * @brief 执行管理任务
 */
void ManagerTaskLoop(void)
{
  uint32_t now_tick = HAL_GetTick();
  // 统一执行所有已登记外设的超时检测、重连探测与状态机流转
  PeripheralManagerUpdateAll(now_tick);

  // 关键外设安全联锁保护：若关键外设发生严重故障或离线超时，执行全局紧急停机归零
  if (PeripheralManagerHasCriticalFault() ||
      !PeripheralManagerIsAllCriticalOnline())
  {
    PeripheralManagerEmergencyStopAll();
  }

  BspIwdgFeed();
}

/**
 * @brief 初始化云台任务
 * @return 任务周期（ms）
 */
uint32_t GimbalTaskInit(void)
{
  return g_app_task_config.gimbal_period_ms;
}

/**
 * @brief 执行云台任务
 */
void GimbalTaskLoop(void)
{
}

/**
 * @brief 初始化底盘任务
 * @return 任务周期（ms）
 */
uint32_t ChassisTaskInit(void)
{
  return g_app_task_config.chassis_period_ms;
}

/**
 * @brief 执行底盘任务
 */
void ChassisTaskLoop(void)
{
}

/**
 * @brief 初始化发射任务
 * @return 任务周期（ms）
 */
uint32_t ShootTaskInit(void)
{
  return g_app_task_config.shoot_period_ms;
}

/**
 * @brief 执行发射任务
 */
void ShootTaskLoop(void)
{
}
