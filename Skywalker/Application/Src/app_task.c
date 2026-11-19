#include "app_task.h"

#include "cmsis_os2.h"
#include "main.h"

static uint32_t AppTaskNormalizePeriod(uint32_t period_ms)
{
  return period_ms == 0U ? 1U : period_ms;
}

static void AppTaskDelayUntil(uint32_t *next_wake_time, uint32_t period_ms)
{
  *next_wake_time += period_ms;
  if (osDelayUntil(*next_wake_time) != osOK)
  {
    (void)osDelay(1U);
    *next_wake_time = osKernelGetTickCount();
  }
}

// 默认为空实现，Application层可覆盖。
__weak uint32_t ManagerTaskInit(void)
{
  return 1U;
}
__weak void ManagerTaskLoop(void)
{
}
__weak uint32_t GimbalTaskInit(void)
{
  return 1U;
}
__weak void GimbalTaskLoop(void)
{
}
__weak uint32_t ChassisTaskInit(void)
{
  return 1U;
}
__weak void ChassisTaskLoop(void)
{
}
__weak uint32_t ShootTaskInit(void)
{
  return 1U;
}
__weak void ShootTaskLoop(void)
{
}

/**
 * @brief Manager线程入口，覆盖CubeMX生成的weak函数
 * @param argument 线程参数
 */
void StartManger(void *argument)
{
  (void)argument;
  uint32_t period_ms = AppTaskNormalizePeriod(ManagerTaskInit());
  uint32_t next_wake_time = osKernelGetTickCount();
  for (;;)
  {
    ManagerTaskLoop();
    AppTaskDelayUntil(&next_wake_time, period_ms);
  }
}

/**
 * @brief Gimbal线程入口，覆盖CubeMX生成的weak函数
 * @param argument 线程参数
 */
void StartGimbal(void *argument)
{
  (void)argument;
  uint32_t period_ms = AppTaskNormalizePeriod(GimbalTaskInit());
  uint32_t next_wake_time = osKernelGetTickCount();
  for (;;)
  {
    GimbalTaskLoop();
    AppTaskDelayUntil(&next_wake_time, period_ms);
  }
}

/**
 * @brief Chassis线程入口，覆盖CubeMX生成的weak函数
 * @param argument 线程参数
 */
void StartChassis(void *argument)
{
  (void)argument;
  uint32_t period_ms = AppTaskNormalizePeriod(ChassisTaskInit());
  uint32_t next_wake_time = osKernelGetTickCount();
  for (;;)
  {
    ChassisTaskLoop();
    AppTaskDelayUntil(&next_wake_time, period_ms);
  }
}

/**
 * @brief Shoot线程入口，覆盖CubeMX生成的weak函数
 * @param argument 线程参数
 */
void StartShoot(void *argument)
{
  (void)argument;
  uint32_t period_ms = AppTaskNormalizePeriod(ShootTaskInit());
  // 须在Init完成后再采样起始时间，避免初始化耗时导致启动时osDelayUntil发生追赶
  uint32_t next_wake_time = osKernelGetTickCount();
  for (;;)
  {
    ShootTaskLoop();
    AppTaskDelayUntil(&next_wake_time, period_ms);
  }
}
