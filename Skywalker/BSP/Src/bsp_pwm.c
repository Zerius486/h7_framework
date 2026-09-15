#include "bsp_pwm.h"

#include "stm32h7xx_hal.h"
#include "stm32h7xx_hal_tim.h"
#include "tim.h"

// PWM对象实例
PwmObject g_pwm_object[kPwmChannelNbr] = {0};

/**
 * @brief 获取PWM对象实例索引
 * @param htim TIM句柄
 * @param channel 通道
 * @return PWM对象实例索引，0xFF表示无效索引
 */
uint8_t PwmIndex(TIM_HandleTypeDef *htim, uint32_t channel) {
  if (htim == &htim3 && channel == TIM_CHANNEL_4) {
    return 0;
  }
  return 0xFF;
}

/**
 * @brief PWM通道初始化
 * @param htim TIM句柄
 * @param channel 通道
 */
void PwmInit(TIM_HandleTypeDef *htim, uint32_t channel) {
  uint32_t index = PwmIndex(htim, channel);
  if (index == 0xFFU) {
    return;
  }

  g_pwm_object[index].htim = htim;
  g_pwm_object[index].channel = channel;
  g_pwm_object[index].arr = __HAL_TIM_GET_AUTORELOAD(htim);
  if (g_pwm_object[index].arr == 0U) {
    g_pwm_object[index].arr = 20000U;
  }
  g_pwm_object[index].ccr = 0U;
  g_pwm_object[index].duty_ratio = 0.0f;
  (void)HAL_TIM_PWM_Start(htim, channel);
}

/**
 * @brief 设置PWM占空比
 * @param htim TIM句柄
 * @param channel 通道
 * @param duty_ratio 占空比
 */
void PwmSetDutyRatio(TIM_HandleTypeDef *htim, uint32_t channel,
                     float duty_ratio) {
  uint32_t index = PwmIndex(htim, channel);
  if (index == 0xFFU) {
    return;
  }

  if (duty_ratio < 0.0f) {
    duty_ratio = 0.0f;
  } else if (duty_ratio > 1.0f) {
    duty_ratio = 1.0f;
  }

  g_pwm_object[index].duty_ratio = duty_ratio;
  g_pwm_object[index].ccr =
      (uint32_t)((float)g_pwm_object[index].arr * duty_ratio);
  __HAL_TIM_SET_COMPARE(htim, channel, g_pwm_object[index].ccr);
}
