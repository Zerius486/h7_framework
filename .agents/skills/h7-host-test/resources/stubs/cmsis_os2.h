#ifndef CMSIS_OS2_H
#define CMSIS_OS2_H

#include <stdint.h>

#include "stm32h7xx_hal.h"

typedef enum
{
  osKernelInactive = 0,
  osKernelReady = 1,
  osKernelRunning = 2,
} osKernelState_t;

static inline osKernelState_t osKernelGetState(void)
{
  return osKernelRunning;
}

static inline uint32_t osKernelGetTickCount(void)
{
  return HAL_GetTick();
}

static inline int32_t osDelay(uint32_t ticks)
{
  HAL_Delay(ticks);
  return 0;
}

#endif // CMSIS_OS2_H
