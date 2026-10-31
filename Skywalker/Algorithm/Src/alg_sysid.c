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
      !isfinite(config->minimum_speed) || config->minimum_speed < 0.0f ||
      config->current_delay_samples > SYSID_MAX_CURRENT_DELAY ||
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
 * @brief 累计回归向量与实际电流的充分统计量
 * @param statistics 统计结构体指针
 * @param x 归一化回归向量
 * @param y 归一化电流积分
 * @param decay 遗忘系数
 */
static void SysidAccumulate(SysidStatistics *statistics, const float x[4],
                            float y, float decay)
{
  for (int i = 0; i < 4; i++)
  {
    statistics->output[i] = decay * statistics->output[i] + (double)x[i] * y;
    for (int j = 0; j < 4; j++)
    {
      statistics->information[i][j] =
          decay * statistics->information[i][j] + (double)x[i] * x[j];
    }
  }
  statistics->energy = decay * statistics->energy + (double)y * y;
  statistics->count++;
}

/**
 * @brief 计算指定模型在记录窗口上的残差平方和
 * @param statistics 回归统计结构体指针
 * @param coefficient 模型系数
 * @return 残差平方和
 */
static double SysidResidual(const SysidStatistics *statistics,
                            const double coefficient[4])
{
  double residual = statistics->energy;
  for (int i = 0; i < 4; i++)
  {
    residual -= 2.0 * coefficient[i] * statistics->output[i];
    for (int j = 0; j < 4; j++)
    {
      residual +=
          coefficient[i] * statistics->information[i][j] * coefficient[j];
    }
  }
  return fmax(residual, 0.0);
}

/**
 * @brief 枚举阻尼与摩擦约束并重新拟合自由参数
 * @param statistics 回归统计结构体指针
 * @param coefficient 最优系数输出
 * @param variance 惯量系数方差输出，可传NULL
 * @return 是否得到有限解，惯量正性由调用方检查
 */
static bool SysidFit(const SysidStatistics *statistics, double coefficient[4],
                     double *variance)
{
  double best = INFINITY;
  for (unsigned int mask = 0U; mask < 4U; mask++)
  {
    bool fixed[4] = {false, (mask & 1U) != 0U, (mask & 2U) != 0U, false};
    double information[4][4];
    double covariance[4][4];
    for (int i = 0; i < 4; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        information[i][j] = fixed[i] || fixed[j]
                                ? (i == j ? 1.0 : 0.0)
                                : statistics->information[i][j];
      }
    }
    if (!SysidInvertInformation(information, covariance))
    {
      continue;
    }
    double candidate[4] = {0};
    for (int i = 0; i < 4; i++)
    {
      for (int j = 0; j < 4; j++)
      {
        if (!fixed[i] && !fixed[j])
        {
          candidate[i] += covariance[i][j] * statistics->output[j];
        }
      }
    }
    if (!isfinite(candidate[0]) || !isfinite(candidate[1]) ||
        !isfinite(candidate[2]) || !isfinite(candidate[3]) ||
        candidate[1] < 0.0 || candidate[2] < 0.0)
    {
      continue;
    }
    double residual = SysidResidual(statistics, candidate);
    if (residual < best)
    {
      best = residual;
      memcpy(coefficient, candidate, sizeof(candidate));
      if (variance != NULL)
      {
        unsigned int parameters =
            4U - (fixed[1] ? 1U : 0U) - (fixed[2] ? 1U : 0U);
        double noise = fmax(residual, 1.0e-12 * statistics->energy);
        *variance = noise / fmax((double)statistics->count - parameters, 1.0) *
                    covariance[0][0];
      }
    }
  }
  return isfinite(best);
}

/**
 * @brief 用相邻不重叠的动态数据段检验惯量与交叉预测
 * @param sysid 辨识器对象指针
 */
static void SysidValidate(SysidObject *sysid)
{
  if (sysid->block.count < SYSID_VALIDATION_WINDOWS)
  {
    return;
  }
  sysid->validation_status = kSysidInsufficientSamples;
  sysid->validation_fitness = 0.0f;
  double current[4], previous[4], current_variance, previous_variance;
  if (sysid->previous_block.count >= SYSID_VALIDATION_WINDOWS)
  {
    sysid->validation_status = kSysidInsufficientExcitation;
    if (!SysidFit(&sysid->block, current, &current_variance) ||
        !SysidFit(&sysid->previous_block, previous, &previous_variance))
    {
      sysid->previous_block = sysid->block;
      memset(&sysid->block, 0, sizeof(sysid->block));
      return;
    }
    double a = 1.0 - SysidResidual(&sysid->block, previous) /
                         fmax(sysid->block.energy, 1.0e-12);
    double b = 1.0 - SysidResidual(&sysid->previous_block, current) /
                         fmax(sysid->previous_block.energy, 1.0e-12);
    sysid->validation_fitness = (float)fmin(a, b);
    bool validated = current[0] > 3.0 * sqrt(current_variance) &&
                     previous[0] > 3.0 * sqrt(previous_variance) &&
                     fabs(current[0] - previous[0]) <=
                         3.0 * sqrt(current_variance + previous_variance) &&
                     sysid->validation_fitness >= 0.75f;
    if (current[0] <= 0.0 || previous[0] <= 0.0)
    {
      sysid->validation_status = kSysidInvalidModel;
    }
    else if (current[0] > 3.0 * sqrt(current_variance) &&
             previous[0] > 3.0 * sqrt(previous_variance))
    {
      sysid->validation_status =
          validated ? kSysidModelReady : kSysidInconsistentModel;
    }
  }
  sysid->previous_block = sysid->block;
  memset(&sysid->block, 0, sizeof(sysid->block));
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
  sysid->current_history[sysid->history_index] = current;
  uint8_t delay = sysid->config.current_delay_samples;
  uint8_t index =
      (uint8_t)((sysid->history_index + SYSID_MAX_CURRENT_DELAY + 1U - delay) %
                (SYSID_MAX_CURRENT_DELAY + 1U));
  sysid->history_index =
      (uint8_t)((sysid->history_index + 1U) % (SYSID_MAX_CURRENT_DELAY + 1U));
  if (sysid->history_count < delay)
  {
    sysid->history_count++;
    return false;
  }
  current = sysid->current_history[index];
  float minimum_speed = sysid->config.minimum_speed > 0.0f
                            ? sysid->config.minimum_speed
                            : 0.01f * sysid->config.omega_scale;
  if (!sysid->has_sample)
  {
    sysid->has_sample = true;
    sysid->start_omega = omega;
    sysid->previous_omega = omega;
    sysid->previous_current = current;
    sysid->sliding_window = fabsf(omega) >= minimum_speed;
    return false;
  }
  const SysidConfig *config = &sysid->config;
  float dt = config->sampling_time;
  sysid->sliding_window = sysid->sliding_window &&
                          fabsf(omega) >= minimum_speed &&
                          omega * sysid->start_omega > 0.0f;
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
  if (!sysid->sliding_window)
  {
    sysid->rejected_windows++;
    memset(sysid->integral, 0, sizeof(sysid->integral));
    sysid->elapsed = 0.0f;
    sysid->start_omega = omega;
    sysid->sliding_window = fabsf(omega) >= minimum_speed;
    return true;
  }
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
  else if (isfinite(denominator) && denominator > 1.0e-9f)
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
    sysid->excitation = decay * sysid->excitation + x[0] * x[0];
    sysid->count++;
    SysidAccumulate(&sysid->statistics, x, y, decay);
    // 匀速窗口参与摩擦估计，不消耗惯量验证的数据段。
    if (fabsf(x[0]) > 0.0002f)
    {
      SysidAccumulate(&sysid->block, x, y, 1.0f);
      SysidValidate(sysid);
    }
  }
  memset(sysid->integral, 0, sizeof(sysid->integral));
  sysid->elapsed = 0.0f;
  sysid->start_omega = omega;
  sysid->sliding_window = fabsf(omega) >= minimum_speed;
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
  *model = (SysidModel){.count = sysid->count,
                        .validation_fitness = sysid->validation_fitness,
                        .status = sysid->count < 2U * SYSID_VALIDATION_WINDOWS
                                      ? kSysidInsufficientSamples
                                      : kSysidInvalidModel};
  double coefficient[4];
  // 约束拟合采用递推估计的局部二次目标，两个更新方法共用。
  double covariance[4][4];
  SysidStatistics objective = {0};
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      covariance[i][j] = sysid->covariance[i][j];
    }
  }
  if (!SysidInvertInformation(covariance, objective.information))
  {
    return false;
  }
  for (int i = 0; i < 4; i++)
  {
    for (int j = 0; j < 4; j++)
    {
      objective.output[i] +=
          objective.information[i][j] * sysid->coefficient[j];
    }
    objective.energy += sysid->coefficient[i] * objective.output[i];
  }
  if (!SysidFit(&objective, coefficient, NULL))
  {
    return false;
  }
  model->inertia = (float)coefficient[0] * sysid->config.current_scale *
                   sysid->config.window_time / sysid->config.omega_scale;
  model->viscous = (float)coefficient[1] * sysid->config.current_scale /
                   sysid->config.omega_scale;
  model->friction = (float)coefficient[2] * sysid->config.current_scale;
  model->bias = (float)coefficient[3] * sysid->config.current_scale;
  model->fitness =
      (float)(1.0 - SysidResidual(&sysid->statistics, coefficient) /
                        fmax(sysid->statistics.energy, 1.0e-12));
  model->validation_fitness = sysid->validation_fitness;
  model->count = sysid->count;
  bool physical = isfinite(model->inertia) && model->inertia > 0.0f &&
                  isfinite(model->viscous) && model->viscous >= 0.0f &&
                  isfinite(model->friction) && isfinite(model->bias) &&
                  isfinite(model->fitness) && model->fitness >= 0.75f;
  if (model->count < 2U * SYSID_VALIDATION_WINDOWS)
  {
    model->status = kSysidInsufficientSamples;
  }
  else if (!physical)
  {
    model->status = kSysidInvalidModel;
  }
  else if (sysid->excitation <= 1.0e-4f || sysid->covariance[0][0] >= 100.0f)
  {
    model->status = kSysidInsufficientExcitation;
  }
  else
  {
    model->status = sysid->validation_status;
  }
  return model->status == kSysidModelReady;
}

/**
 * @brief 清除跨阶段的积分窗口，保留已辨识参数与电流历史
 * @param sysid 辨识器对象指针
 */
void SysidResetWindow(SysidObject *sysid)
{
  if (sysid != NULL)
  {
    memset(sysid->integral, 0, sizeof(sysid->integral));
    sysid->elapsed = 0.0f;
    sysid->has_sample = false;
  }
}

/**
 * @brief 初始化独立速度预测，后续样本不再修改模型
 * @param prediction 预测对象指针
 * @param config 采样与配对配置
 * @param model 待验证模型
 * @return 是否成功
 */
bool SysidPredictionInit(SysidPrediction *prediction, const SysidConfig *config,
                         const SysidModel *model)
{
  if (prediction == NULL || config == NULL || model == NULL ||
      !isfinite(config->sampling_time) || config->sampling_time <= 0.0f ||
      !isfinite(config->minimum_speed) || config->minimum_speed < 0.0f ||
      !isfinite(config->omega_scale) || config->omega_scale <= 0.0f ||
      config->current_delay_samples > SYSID_MAX_CURRENT_DELAY ||
      !isfinite(model->inertia) || model->inertia <= 0.0f ||
      !isfinite(model->viscous) || model->viscous < 0.0f ||
      !isfinite(model->friction) || model->friction < 0.0f ||
      !isfinite(model->bias))
  {
    return false;
  }
  memset(prediction, 0, sizeof(*prediction));
  prediction->config = *config;
  prediction->model = *model;
  return true;
}

/**
 * @brief 用配对电流自由预测连续滑动片段，片段内不注入速度反馈
 * @param prediction 预测对象指针
 * @param current 实际反馈电流（A）
 * @param omega 当前角速度（rad/s）
 * @return 本周期是否计入预测误差
 */
bool SysidPredictionUpdate(SysidPrediction *prediction, float current,
                           float omega)
{
  if (prediction == NULL || !isfinite(current) || !isfinite(omega) ||
      prediction->model.inertia <= 0.0f)
  {
    return false;
  }
  uint8_t delay = prediction->config.current_delay_samples;
  prediction->current_history[prediction->history_index] = current;
  uint8_t index = (uint8_t)((prediction->history_index +
                             SYSID_MAX_CURRENT_DELAY + 1U - delay) %
                            (SYSID_MAX_CURRENT_DELAY + 1U));
  prediction->history_index = (uint8_t)((prediction->history_index + 1U) %
                                        (SYSID_MAX_CURRENT_DELAY + 1U));
  if (prediction->history_count < delay)
  {
    prediction->history_count++;
    return false;
  }
  current = prediction->current_history[index];
  float minimum = prediction->config.minimum_speed > 0.0f
                      ? prediction->config.minimum_speed
                      : 0.01f * prediction->config.omega_scale;
  if (fabsf(omega) < minimum)
  {
    prediction->active = false;
    prediction->previous_current = current;
    return false;
  }
  if (!prediction->active || omega * prediction->initial <= 0.0f ||
      prediction->elapsed >= 0.25f)
  {
    prediction->predicted = omega;
    prediction->initial = omega;
    prediction->elapsed = 0.0f;
    prediction->previous_current = current;
    prediction->active = true;
    prediction->segments++;
    return false;
  }
  const SysidModel *model = &prediction->model;
  float dt = prediction->config.sampling_time;
  float input = 0.5f * (current + prediction->previous_current) - model->bias;
  float velocity = prediction->predicted;
  float direction =
      velocity == 0.0f ? copysignf(1.0f, input) : copysignf(1.0f, velocity);
  float force = input - model->friction * direction;
  float rate = model->viscous / model->inertia;
  float next = expf(-rate * dt) * velocity +
               force / model->inertia *
                   (rate > 1.0e-6f ? -expm1f(-rate * dt) / rate : dt);
  if ((velocity == 0.0f || velocity * next < 0.0f) &&
      fabsf(input) <= model->friction)
  {
    next = 0.0f;
  }
  prediction->predicted = next;
  prediction->previous_current = current;
  prediction->elapsed += dt;
  double error = (double)next - omega;
  double change = (double)omega - prediction->initial;
  prediction->error += error * error;
  prediction->energy += change * change;
  prediction->samples++;
  return true;
}

/**
 * @brief 读取独立片段的速度预测质量
 * @param prediction 预测对象指针
 * @param model 模型及预测指标输出
 * @return 是否具有足够动态数据且预测通过
 */
bool SysidPredictionGetResult(const SysidPrediction *prediction,
                              SysidModel *model)
{
  if (prediction == NULL || model == NULL || prediction->samples == 0U)
  {
    return false;
  }
  model->velocity_rmse = (float)sqrt(prediction->error / prediction->samples);
  model->velocity_fitness =
      (float)(1.0 - prediction->error / fmax(prediction->energy, 1.0e-12));
  return prediction->samples * prediction->config.sampling_time >= 0.5f &&
         prediction->segments >= 4U &&
         prediction->energy > 1.0e-4 * prediction->config.omega_scale *
                                  prediction->config.omega_scale &&
         isfinite(model->velocity_fitness) && model->velocity_fitness >= 0.75f;
}
