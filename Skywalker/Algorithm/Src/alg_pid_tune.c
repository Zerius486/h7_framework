#include "alg_pid_tune.h"

#include <float.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

/**
 * @brief 初始化低维PI敏感度优化器
 * @param tune 优化对象指针
 * @param config 配置结构体指针
 * @return 是否成功
 */
bool PidTuneInit(PidTuneObject *tune, const PidTuneConfig *config)
{
  if (tune == NULL || config == NULL || !isfinite(config->model.inertia) ||
      config->model.inertia <= 0.0f || !isfinite(config->model.viscous) ||
      config->model.viscous < 0.0f || !isfinite(config->sampling_time) ||
      config->sampling_time <= 0.0f || !isfinite(config->settling_time) ||
      config->settling_time < 0.0f || !isfinite(config->feedback_scale) ||
      config->feedback_scale <= 0.0f || !isfinite(config->output_weight) ||
      config->output_weight < 0.0f || !isfinite(config->change_weight) ||
      config->change_weight < 0.0f)
  {
    return false;
  }
  memset(tune, 0, sizeof(*tune));
  tune->config = *config;
  float rate = config->model.viscous / config->model.inertia;
  tune->decay = expf(-rate * config->sampling_time);
  tune->input_gain =
      rate > 1.0e-6f
          ? -expm1f(-rate * config->sampling_time) / config->model.viscous
          : config->sampling_time / config->model.inertia;
  return isfinite(tune->input_gain) && tune->input_gain > 0.0f;
}

/**
 * @brief 清除一轮损失与敏感度，不清除已验证结果
 * @param tune 优化对象指针
 */
static void PidTuneResetTrial(PidTuneObject *tune)
{
  memset(tune->velocity_sensitivity, 0, sizeof(tune->velocity_sensitivity));
  memset(tune->integral_sensitivity, 0, sizeof(tune->integral_sensitivity));
  memset(tune->error_sensitivity, 0, sizeof(tune->error_sensitivity));
  memset(tune->output_sensitivity, 0, sizeof(tune->output_sensitivity));
  memset(tune->gradient, 0, sizeof(tune->gradient));
  memset(tune->curvature, 0, sizeof(tune->curvature));
  tune->elapsed = 0.0f;
  tune->loss = 0.0;
  tune->samples = 0U;
  tune->limited = false;
}

/**
 * @brief 从正增益PI开始测试，初始积分电流固定
 * @param tune 优化对象指针
 * @param pid 已初始化的PI控制器
 * @return 是否成功，暂不支持微分、前馈、死区和积分分离
 */
bool PidTuneStart(PidTuneObject *tune, const PidObject *pid)
{
  if (tune == NULL || pid == NULL || !pid->is_enabled || !isfinite(pid->kp) ||
      pid->kp <= 0.0f || !isfinite(pid->ki) || pid->ki <= 0.0f ||
      pid->kd != 0.0f || pid->is_feedforward_enabled ||
      pid->is_deadband_enabled || pid->is_integralseparate_enabled ||
      !pid->is_antiwindup_enabled || !isfinite(pid->output_max) ||
      pid->output_max <= 0.0f || !isfinite(pid->integral_max) ||
      pid->integral_max <= 0.0f ||
      pid->sampling_time != tune->config.sampling_time ||
      !isfinite(tune->input_gain) || tune->input_gain <= 0.0f)
  {
    return false;
  }
  tune->initial_gain[0] = pid->kp;
  tune->initial_gain[1] = pid->ki;
  tune->output_scale = pid->output_max;
  tune->result = (PidTuneResult){.kp = pid->kp, .ki = pid->ki};
  tune->state = kPidTuneRunning;
  PidTuneResetTrial(tune);
  return true;
}

/**
 * @brief 累计一个标量残差的损失、梯度与预测曲率
 * @param tune 优化对象指针
 * @param residual 归一化残差
 * @param sensitivity 残差对两个对数增益的敏感度
 * @param weight 损失权重
 */
static void PidTuneMeasure(PidTuneObject *tune, float residual,
                           const float sensitivity[2], float weight)
{
  tune->loss += (double)weight * residual * residual;
  for (int i = 0; i < 2; i++)
  {
    tune->gradient[i] += (double)weight * residual * sensitivity[i];
    for (int j = 0; j < 2; j++)
    {
      tune->curvature[i][j] += (double)weight * sensitivity[i] * sensitivity[j];
    }
  }
}

/**
 * @brief 计算PI输出并传播积分、限幅和机械响应敏感度
 * @param tune 优化对象指针
 * @param pid 本轮固定增益的PI控制器
 * @param reference 目标反馈量
 * @param feedback 实际反馈量
 * @return PI输出，仍须按应用发送最终执行量
 */
float PidTuneCalculate(PidTuneObject *tune, PidObject *pid, float reference,
                       float feedback)
{
  if (tune == NULL || pid == NULL)
  {
    return 0.0f;
  }
  if (tune->state != kPidTuneRunning)
  {
    return PidCalculate(pid, reference, feedback);
  }
  if (!isfinite(reference) || !isfinite(feedback))
  {
    tune->state = kPidTuneInvalidSample;
    return 0.0f;
  }
  if (tune->elapsed == 0.0f)
  {
    // 初始积分电流固定，误差积分随Ki反比变化。
    tune->integral_sensitivity[1] = -pid->error_sum;
    tune->previous_output = pid->output;
  }
  float dt = tune->config.sampling_time;
  float error = reference - feedback;
  float potential =
      pid->kp * error +
      pid->ki * (pid->error_sum + 0.5f * (error + pid->previous_error) * dt);
  bool integrate = !((potential > pid->output_max && error > 0.0f) ||
                     (potential < -pid->output_max && error < 0.0f));
  bool crossing = error * pid->previous_error < 0.0f;
  float output = PidCalculate(pid, reference, feedback);
  float output_sensitivity[2];
  float error_sensitivity[2];
  float change_sensitivity[2];
  for (int i = 0; i < 2; i++)
  {
    float sensitivity = -tune->velocity_sensitivity[i];
    float integral = tune->integral_sensitivity[i];
    if (crossing)
    {
      integral *= 0.9f;
    }
    if (integrate)
    {
      integral += 0.5f * (sensitivity + tune->error_sensitivity[i]) * dt;
    }
    if (fabsf(pid->error_sum) >= pid->integral_max)
    {
      integral = i == 1 ? -pid->error_sum : 0.0f;
    }
    float request = pid->kp * sensitivity + pid->ki * integral;
    request += i == 0 ? pid->kp * error : pid->ki * pid->error_sum;
    if (fabsf(output) >= pid->output_max)
    {
      request = 0.0f;
    }
    error_sensitivity[i] = sensitivity / tune->config.feedback_scale;
    output_sensitivity[i] = request / tune->output_scale;
    change_sensitivity[i] =
        (request - tune->output_sensitivity[i]) / tune->output_scale;
    tune->integral_sensitivity[i] = integral;
    tune->error_sensitivity[i] = sensitivity;
    tune->output_sensitivity[i] = request;
    tune->velocity_sensitivity[i] =
        tune->decay * tune->velocity_sensitivity[i] +
        tune->input_gain * request;
    if (!isfinite(tune->velocity_sensitivity[i]) || !isfinite(integral))
    {
      tune->state = kPidTuneInvalidSample;
      return output;
    }
  }
  tune->elapsed += dt;
  if (tune->elapsed > tune->config.settling_time)
  {
    PidTuneMeasure(tune, error / tune->config.feedback_scale, error_sensitivity,
                   1.0f);
    PidTuneMeasure(tune, output / tune->output_scale, output_sensitivity,
                   tune->config.output_weight);
    PidTuneMeasure(tune, (output - tune->previous_output) / tune->output_scale,
                   change_sensitivity, tune->config.change_weight);
    tune->samples++;
  }
  tune->previous_output = output;
  tune->requested_output = output;
  return output;
}

/**
 * @brief 标记模型未描述的外部限幅，本轮不生成参数更新
 * @param tune 优化对象指针
 * @param executed_output 最终发送的执行量
 */
void PidTuneTrackOutput(PidTuneObject *tune, float executed_output)
{
  if (tune != NULL && tune->state == kPidTuneRunning &&
      (!isfinite(executed_output) ||
       fabsf(executed_output - tune->requested_output) >
           1.0e-4f * tune->output_scale))
  {
    tune->limited = true;
  }
}

/**
 * @brief 恢复已验证最优增益并结束本次优化
 * @param tune 优化对象指针
 * @param pid PI控制器对象指针
 * @param state 结束状态
 */
static void PidTuneComplete(PidTuneObject *tune, PidObject *pid,
                            PidTuneState state)
{
  (void)PidSetGains(pid, tune->result.kp, tune->result.ki, 0.0f);
  tune->state = state;
}

/**
 * @brief 按预测二次损失选择沿梯度的步长并保留已验证最优结果
 * @param tune 优化对象指针
 * @param pid PI控制器对象指针
 * @return 是否需要以新增益重复同一测试轨迹
 */
bool PidTuneFinish(PidTuneObject *tune, PidObject *pid)
{
  if (tune == NULL || pid == NULL)
  {
    return false;
  }
  if (tune->state != kPidTuneRunning)
  {
    if (tune->state == kPidTuneInvalidSample)
    {
      PidTuneComplete(tune, pid, kPidTuneInvalidSample);
    }
    return false;
  }
  if (tune->limited || tune->samples == 0U || !isfinite(tune->loss))
  {
    if (tune->state != kPidTuneIdle)
    {
      PidTuneComplete(tune, pid,
                      tune->limited ? kPidTuneLimited : kPidTuneInvalidSample);
    }
    return false;
  }
  float loss = (float)(tune->loss / tune->samples);
  bool improved = tune->result.trials == 0U || loss < tune->result.loss;
  if (tune->result.trials == 0U)
  {
    tune->result.initial_loss = loss;
  }
  tune->result.trials++;
  if (improved)
  {
    tune->result.kp = pid->kp;
    tune->result.ki = pid->ki;
    tune->result.loss = loss;
  }
  double norm = 0.0;
  double curvature = 0.0;
  for (int i = 0; i < 2; i++)
  {
    norm += tune->gradient[i] * tune->gradient[i];
    for (int j = 0; j < 2; j++)
    {
      curvature +=
          tune->gradient[i] * tune->curvature[i][j] * tune->gradient[j];
    }
  }
  if (!improved || tune->result.trials >= 6U || !isfinite(norm) ||
      !isfinite(curvature) || norm <= 1.0e-18 || curvature <= 1.0e-18)
  {
    PidTuneComplete(tune, pid, kPidTuneComplete);
    return false;
  }
  float step[2];
  float maximum = 0.0f;
  for (int i = 0; i < 2; i++)
  {
    step[i] = (float)(-norm * tune->gradient[i] / curvature);
    maximum = fmaxf(maximum, fabsf(step[i]));
  }
  float fraction = fminf(1.0f, 0.22314355f / fmaxf(maximum, FLT_MIN));
  float kp = fminf(
      fmaxf(pid->kp * expf(fraction * step[0]), 0.25f * tune->initial_gain[0]),
      4.0f * tune->initial_gain[0]);
  float ki = fminf(
      fmaxf(pid->ki * expf(fraction * step[1]), 0.25f * tune->initial_gain[1]),
      4.0f * tune->initial_gain[1]);
  if ((fabsf(kp / pid->kp - 1.0f) < 0.001f &&
       fabsf(ki / pid->ki - 1.0f) < 0.001f) ||
      !PidSetGains(pid, kp, ki, 0.0f))
  {
    PidTuneComplete(tune, pid, kPidTuneComplete);
    return false;
  }
  PidTuneResetTrial(tune);
  return true;
}
