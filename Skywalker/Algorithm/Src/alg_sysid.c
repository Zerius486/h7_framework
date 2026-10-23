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
      !isfinite(config->memory_time) || config->memory_time < 0.0f ||
      (config->method != kSysidExponential && config->method != kSysidSubspace))
  {
    return false;
  }
  memset(sysid, 0, sizeof(*sysid));
  sysid->config = *config;
  for (int i = 0; i < 4; i++)
  {
    sysid->covariance[i][i] = 10000.0f;
    sysid->information[i][i] = 0.0001;
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
 * @brief 用Cholesky分解计算四维信息矩阵的逆
 * @param information 信息矩阵
 * @param covariance 协方差矩阵
 * @return 是否为有限正定矩阵
 */
static bool SysidInvertInformation(const double information[4][4],
                                   double covariance[4][4])
{
  double lower[4][4] = {0};
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j <= i; j++)
    {
      double value = information[i][j];
      for (int k = 0; k < j; k++)
      {
        value -= lower[i][k] * lower[j][k];
      }
      if (!isfinite(value) || (i == j && value <= 0.0))
      {
        return false;
      }
      lower[i][j] = i == j ? sqrt(value) : value / lower[j][j];
    }
  }
  for (int column = 0; column < 4; column++)
  {
    double value[4] = {0};
    for (int i = 0; i < 4; i++)
    {
      double sum = i == column ? 1.0 : 0.0;
      for (int j = 0; j < i; j++)
      {
        sum -= lower[i][j] * value[j];
      }
      value[i] = sum / lower[i][i];
    }
    for (int i = 3; i >= 0; i--)
    {
      double sum = value[i];
      for (int j = i + 1; j < 4; j++)
      {
        sum -= lower[j][i] * value[j];
      }
      value[i] = sum / lower[i][i];
      covariance[i][column] = value[i];
      if (!isfinite(covariance[i][column]))
      {
        return false;
      }
    }
  }
  return true;
}

/**
 * @brief 按标量观测执行SIFt-RLS，不使用通用SVD
 * @param sysid 辨识器对象指针
 * @param x 归一化回归向量
 * @param error 一步预测误差
 * @param decay 遗忘系数
 * @return 是否更新成功
 */
static bool SysidSubspaceUpdate(SysidObject *sysid, const float x[4],
                                float error, float decay)
{
  double rx[4] = {0};
  double norm = 0.0;
  double projection = 0.0;
  for (int i = 0; i < 4; i++)
  {
    norm += (double)x[i] * x[i];
    for (int j = 0; j < 4; j++)
    {
      rx[i] += sysid->information[i][j] * x[j];
    }
    projection += x[i] * rx[i];
  }
  if (norm < 1.0e-8 || !isfinite(projection) || projection <= 0.0)
  {
    return false;
  }
  double information[4][4];
  double covariance[4][4];
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      information[i][j] = sysid->information[i][j] -
                          (1.0 - decay) * rx[i] * rx[j] / projection +
                          (double)x[i] * x[j];
    }
  }
  if (!SysidInvertInformation(information, covariance))
  {
    return false;
  }
  for (int i = 0; i < 4; i++)
  {
    double gain = 0.0;
    for (int j = 0; j < 4; j++)
    {
      gain += covariance[i][j] * x[j];
    }
    sysid->coefficient[i] += (float)(gain * error);
    for (int j = 0; j < 4; j++)
    {
      sysid->covariance[i][j] = (float)covariance[i][j];
    }
  }
  memcpy(sysid->information, information, sizeof(information));
  return true;
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
  bool updated = false;
  if (config->method == kSysidSubspace)
  {
    updated = SysidSubspaceUpdate(sysid, x, error, decay);
  }
  else if (fabsf(x[0]) > 0.0002f && isfinite(denominator) &&
           denominator > 1.0e-9f)
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
    updated = true;
  }
  if (updated)
  {
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
