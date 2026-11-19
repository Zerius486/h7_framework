#include "bsp_iwdg.h"

#include "stm32h7xx_hal.h"

static IWDG_HandleTypeDef g_hiwdg1 = {0};

bool BspIwdgInit(uint32_t timeout_ms)
{
  if (timeout_ms == 0U)
  {
    return false;
  }

  // LSI_VALUE = 32000Hz，32 分频后计数器时钟约 1kHz（1ms/tick），最大重载值 4095
  uint32_t reload = timeout_ms - 1U;
  if (reload > 0x0FFFU)
  {
    reload = 0x0FFFU;
  }

#ifdef DEBUG
  // 调试暂停内核时冻结 IWDG1，避免单步断点触发误复位
  __HAL_DBGMCU_FREEZE_IWDG1();
#endif

  g_hiwdg1.Instance = IWDG1;
  g_hiwdg1.Init.Prescaler = IWDG_PRESCALER_32;
  g_hiwdg1.Init.Window = IWDG_WINDOW_DISABLE;
  g_hiwdg1.Init.Reload = reload;

  if (HAL_IWDG_Init(&g_hiwdg1) != HAL_OK)
  {
    g_hiwdg1.Instance = NULL;
    return false;
  }
  return true;
}

void BspIwdgFeed(void)
{
  if (g_hiwdg1.Instance == IWDG1)
  {
    (void)HAL_IWDG_Refresh(&g_hiwdg1);
  }
}
