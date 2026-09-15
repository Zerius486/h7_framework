#ifndef BSP_PWM_H
#define BSP_PWM_H

#include <stm32h7xx_hal.h>

// PWM通道数量
enum { kPwmChannelNbr = 1 };

// PWM对象结构体
typedef struct {
  TIM_HandleTypeDef *htim;  // 定时器句柄
  uint32_t channel;         // PWM通道
  uint32_t arr;             // 自动重装载寄存器值
  uint32_t ccr;             // 捕获比较寄存器值
  float duty_ratio;         // 占空比
} PwmObject;

extern PwmObject g_pwm_object[kPwmChannelNbr];

void PwmInit(TIM_HandleTypeDef *htim, uint32_t channel);
void PwmSetDutyRatio(TIM_HandleTypeDef *htim, uint32_t channel,
                     float duty_ratio);

#endif  // BSP_PWM_H
