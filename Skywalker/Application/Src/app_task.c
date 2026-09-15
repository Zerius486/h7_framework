#include "app_task.h"

#include "cmsis_os2.h"
#include "main.h"

static uint32_t AppTaskNormalizePeriod(uint32_t period_ms) {
  return period_ms == 0U ? 1U : period_ms;
}

// 任务钩子默认为空实现，业务层提供同名强符号即可覆盖。
__weak uint32_t ManagerTaskInit(void) { return 1U; }
__weak void ManagerTaskLoop(void) {}
__weak uint32_t GimbalTaskInit(void) { return 1U; }
__weak void GimbalTaskLoop(void) {}
__weak uint32_t ChassisTaskInit(void) { return 1U; }
__weak void ChassisTaskLoop(void) {}
__weak uint32_t ShootTaskInit(void) { return 1U; }
__weak void ShootTaskLoop(void) {}

/**
 * @brief Manager线程入口，覆盖CubeMX生成的weak函数
 * @param argument 线程参数
 */
void StartManger(void *argument) {
  (void)argument;
  uint32_t period_ms = AppTaskNormalizePeriod(ManagerTaskInit());
  uint32_t next_wake_time = osKernelGetTickCount();
  for (;;) {
    ManagerTaskLoop();
    next_wake_time += period_ms;
    osDelayUntil(next_wake_time);
  }
}

/**
 * @brief Gimbal线程入口，覆盖CubeMX生成的weak函数
 * @param argument 线程参数
 */
void StartGimbal(void *argument) {
  (void)argument;
  uint32_t period_ms = AppTaskNormalizePeriod(GimbalTaskInit());
  uint32_t next_wake_time = osKernelGetTickCount();
  for (;;) {
    GimbalTaskLoop();
    next_wake_time += period_ms;
    osDelayUntil(next_wake_time);
  }
}

/**
 * @brief Chassis线程入口，覆盖CubeMX生成的weak函数
 * @param argument 线程参数
 */
void StartChassis(void *argument) {
  (void)argument;
  uint32_t period_ms = AppTaskNormalizePeriod(ChassisTaskInit());
  uint32_t next_wake_time = osKernelGetTickCount();
  for (;;) {
    ChassisTaskLoop();
    next_wake_time += period_ms;
    osDelayUntil(next_wake_time);
  }
}

/**
 * @brief Shoot线程入口，覆盖CubeMX生成的weak函数
 * @param argument 线程参数
 */
void StartShoot(void *argument) {
  (void)argument;
  uint32_t next_wake_time = osKernelGetTickCount();
  uint32_t period_ms = AppTaskNormalizePeriod(ShootTaskInit());
  for (;;) {
    ShootTaskLoop();
    next_wake_time += period_ms;
    osDelayUntil(next_wake_time);
  }
}
