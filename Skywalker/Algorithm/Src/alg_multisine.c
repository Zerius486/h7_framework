#include "alg_multisine.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/**
 * @brief 初始化固定周期的闭环多正弦参考
 * @param multisine 激励对象指针
 * @param config 配置结构体指针
 * @return 是否成功
 */
bool MultisineInit(MultisineObject *multisine, const MultisineConfig *config)
{
  if (multisine == NULL || config == NULL || !isfinite(config->sampling_time) ||
      config->sampling_time <= 0.0f || !isfinite(config->fundamental_hz) ||
      config->fundamental_hz <= 0.0f || !isfinite(config->amplitude) ||
      config->amplitude <= 0.0f || !isfinite(config->offset) ||
      !isfinite(config->lower) || !isfinite(config->upper) ||
      config->lower >= config->upper ||
      config->offset - config->amplitude < config->lower ||
      config->offset + config->amplitude > config->upper ||
      config->tone_count == 0U || config->tone_count > MULTISINE_MAX_TONES ||
      config->settling_periods == 0U || config->measurement_periods < 2U)
  {
    return false;
  }
  double ticks =
      round(1.0 / ((double)config->sampling_time * config->fundamental_hz));
  double periods =
      (double)config->settling_periods + config->measurement_periods;
  if (ticks < 4.0 || ticks * periods > UINT32_MAX)
  {
    return false;
  }
  for (uint8_t i = 0U; i < config->tone_count; i++)
  {
    if (config->harmonic[i] == 0U || 2.0 * config->harmonic[i] >= ticks)
    {
      return false;
    }
    for (uint8_t j = 0U; j < i; j++)
    {
      if (config->harmonic[i] == config->harmonic[j])
      {
        return false;
      }
    }
  }
  memset(multisine, 0, sizeof(*multisine));
  multisine->config = *config;
  multisine->period_ticks = (uint32_t)ticks;
  multisine->fundamental_hz =
      1.0f / (multisine->period_ticks * config->sampling_time);
  multisine->reference = config->offset;
  multisine->initialized = true;
  return true;
}

/**
 * @brief 从第一个过渡周期重新开始激励
 * @param multisine 激励对象指针
 * @return 是否成功
 */
bool MultisineStart(MultisineObject *multisine)
{
  if (multisine == NULL || !multisine->initialized || multisine->active)
  {
    return false;
  }
  multisine->tick = 0U;
  multisine->period = 0U;
  multisine->sample = 0U;
  multisine->measuring = false;
  multisine->active = true;
  return true;
}

/**
 * @brief 生成Schroeder相位多正弦，首周期平滑增幅
 * @param multisine 激励对象指针
 * @return 本周期参考信号
 */
float MultisineCalculate(MultisineObject *multisine)
{
  if (multisine == NULL || !multisine->initialized)
  {
    return 0.0f;
  }
  if (!multisine->active)
  {
    return multisine->config.offset;
  }
  const MultisineConfig *c = &multisine->config;
  multisine->period = (uint16_t)(multisine->tick / multisine->period_ticks);
  multisine->sample = multisine->tick % multisine->period_ticks;
  if (multisine->period >=
      (uint16_t)c->settling_periods + c->measurement_periods)
  {
    MultisineCancel(multisine);
    return c->offset;
  }
  multisine->measuring = multisine->period >= c->settling_periods;
  float phase = 6.2831853f * (float)multisine->sample / multisine->period_ticks;
  float value = 0.0f;
  for (uint8_t i = 0U; i < c->tone_count; i++)
  {
    float shift = -3.1415927f * (float)i * (float)(i + 1U) / c->tone_count;
    value += sinf(c->harmonic[i] * phase + shift);
  }
  float envelope =
      multisine->period == 0U ? 0.5f - 0.5f * cosf(0.5f * phase) : 1.0f;
  multisine->reference =
      c->offset + envelope * c->amplitude * value / c->tone_count;
  multisine->tick++;
  return multisine->reference;
}

/**
 * @brief 取消激励并返回工作点
 * @param multisine 激励对象指针
 */
void MultisineCancel(MultisineObject *multisine)
{
  if (multisine != NULL)
  {
    multisine->active = false;
    multisine->measuring = false;
    multisine->reference = multisine->config.offset;
  }
}
