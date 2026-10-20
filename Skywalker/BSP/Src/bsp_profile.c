#include "bsp_profile.h"

#include "stm32h7xx_hal.h"
#include <string.h>

/**
 * @brief 启用Cortex-M7周期计数器，不重置已有计数
 */
void ProfileInit(void)
{
  CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

/**
 * @brief 清除单个测量对象的统计
 * @param profile 测量对象指针
 */
void ProfileReset(ProfileObject *profile)
{
  memset(profile, 0, sizeof(*profile));
}

/**
 * @brief 记录测量开始周期
 * @param profile 测量对象指针
 */
void ProfileBegin(ProfileObject *profile)
{
  profile->begin = DWT->CYCCNT;
}

/**
 * @brief 更新测量统计，单次测量须短于计数器回绕周期
 * @param profile 测量对象指针
 */
void ProfileEnd(ProfileObject *profile)
{
  uint32_t elapsed = DWT->CYCCNT - profile->begin;
  profile->last = elapsed;
  if (profile->count == 0U || elapsed < profile->minimum)
  {
    profile->minimum = elapsed;
  }
  if (elapsed > profile->maximum)
  {
    profile->maximum = elapsed;
  }
  profile->count++;
  profile->total += elapsed;
}
