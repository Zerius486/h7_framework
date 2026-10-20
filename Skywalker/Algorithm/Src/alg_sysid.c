#include "alg_sysid.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/**
 * @brief 初始化固定尺寸积分辨识器
 * @param sysid 辨识器对象指针
 * @param config 配置结构体指针
 * @return 是否成功
 */
bool SysidInit(SysidObject *sysid, const SysidConfig *config)
{
  if (sysid == NULL || config == NULL || !isfinite(config->sampling_time) ||
      config->sampling_time <= 0.0f || !isfinite(config->window_time) ||
      config->window_time < config->sampling_time ||
      !isfinite(config->current_scale) || config->current_scale <= 0.0f ||
      !isfinite(config->omega_scale) || config->omega_scale <= 0.0f ||
      !isfinite(config->memory_time) || config->memory_time < 0.0f)
  {
    return false;
  }
  memset(sysid, 0, sizeof(*sysid));
  sysid->config = *config;
  for (int i = 0; i < 4; i++)
  {
    sysid->covariance[i][i] = 10000.0f;
  }
  return true;
}

/**
 * @brief 计算角速度方向，零速处使用连续过渡
 * @param omega 角速度（rad/s）
 * @param scale 角速度尺度（rad/s）
 * @return 摩擦方向
 */
static float SysidDirection(float omega, float scale)
{
  return fminf(fmaxf(omega / (0.01f * scale), -1.0f), 1.0f);
}

/**
 * @brief 用非重叠积分窗口更新电流坐标机械模型
 * @param sysid 辨识器对象指针
 * @param current 实际反馈电流（A）
 * @param omega 与控制器一致的角速度（rad/s）
 * @return 是否完成一个辨识窗口
 */
bool SysidUpdate(SysidObject *sysid, float current, float omega)
{
  if (sysid == NULL || !isfinite(current) || !isfinite(omega))
  {
    return false;
  }
  if (!sysid->has_sample)
  {
    sysid->has_sample = true;
    sysid->start_omega = omega;
    sysid->previous_omega = omega;
    sysid->previous_current = current;
    return false;
  }
  const SysidConfig *config = &sysid->config;
  float dt = config->sampling_time;
  sysid->integral[0] += 0.5f * (current + sysid->previous_current) * dt;
  sysid->integral[1] += 0.5f * (omega + sysid->previous_omega) * dt;
  sysid->integral[2] +=
      0.5f *
      (SysidDirection(omega, config->omega_scale) +
       SysidDirection(sysid->previous_omega, config->omega_scale)) *
      dt;
  sysid->previous_omega = omega;
  sysid->previous_current = current;
  sysid->elapsed += dt;
  if (sysid->elapsed + 0.25f * dt < config->window_time)
  {
    return false;
  }

  float x[4] = {(omega - sysid->start_omega) / config->omega_scale,
                sysid->integral[1] /
                    (config->omega_scale * config->window_time),
                sysid->integral[2] / config->window_time,
                sysid->elapsed / config->window_time};
  float y = sysid->integral[0] / (config->current_scale * config->window_time);
  float prediction = 0.0f;
  float px[4] = {0};
  for (int i = 0; i < 4; i++)
  {
    prediction += x[i] * sysid->coefficient[i];
    for (int j = 0; j < 4; j++)
    {
      px[i] += sysid->covariance[i][j] * x[j];
    }
  }
  float decay = config->memory_time > 0.0f
                    ? expf(-sysid->elapsed / config->memory_time)
                    : 1.0f;
  decay = fmaxf(decay, 0.98f);
  float denominator = decay;
  for (int i = 0; i < 4; i++)
  {
    denominator += x[i] * px[i];
  }
  float error = y - prediction;
  if (fabsf(x[0]) > 0.0002f && isfinite(denominator) && denominator > 1.0e-9f)
  {
    for (int i = 0; i < 4; i++)
    {
      sysid->coefficient[i] += px[i] * error / denominator;
      for (int j = i; j < 4; j++)
      {
        float value =
            (sysid->covariance[i][j] - px[i] * px[j] / denominator) / decay;
        if (i == j)
        {
          value = fmaxf(value, 1.0e-9f);
        }
        sysid->covariance[i][j] = value;
        sysid->covariance[j][i] = value;
      }
    }
    // 单向运动时摩擦与偏置可能共线，限制未观测方向的协方差增长。
    float scale[4];
    for (int i = 0; i < 4; i++)
    {
      scale[i] = sqrtf(fminf(10000.0f / sysid->covariance[i][i], 1.0f));
    }
    for (int i = 0; i < 4; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        sysid->covariance[i][j] *= scale[i] * scale[j];
      }
    }
    // 初始收敛窗口不参与一步预测质量统计。
    if (sysid->count >= 12U)
    {
      sysid->energy = decay * sysid->energy + y * y;
      sysid->residual = decay * sysid->residual + error * error;
    }
    sysid->excitation = decay * sysid->excitation + x[0] * x[0];
    sysid->count++;
  }
  memset(sysid->integral, 0, sizeof(sysid->integral));
  sysid->elapsed = 0.0f;
  sysid->start_omega = omega;
  return true;
}

/**
 * @brief 读取具有足够动态激励的机械模型
 * @param sysid 辨识器对象指针
 * @param model 模型输出指针
 * @return 模型是否可用于整定
 */
bool SysidGetModel(const SysidObject *sysid, SysidModel *model)
{
  if (sysid == NULL || model == NULL)
  {
    return false;
  }
  model->inertia = sysid->coefficient[0] * sysid->config.current_scale *
                   sysid->config.window_time / sysid->config.omega_scale;
  model->viscous = sysid->coefficient[1] * sysid->config.current_scale /
                   sysid->config.omega_scale;
  model->friction = sysid->coefficient[2] * sysid->config.current_scale;
  model->bias = sysid->coefficient[3] * sysid->config.current_scale;
  model->fitness = 1.0f - sysid->residual / fmaxf(sysid->energy, 1.0e-12f);
  model->count = sysid->count;
  return model->count >= 40U && sysid->excitation > 1.0e-4f &&
         sysid->covariance[0][0] < 100.0f && isfinite(model->inertia) &&
         model->inertia > 0.0f && isfinite(model->viscous) &&
         model->viscous >= 0.0f && isfinite(model->friction) &&
         isfinite(model->bias) && isfinite(model->fitness) &&
         model->fitness >= 0.75f;
}
