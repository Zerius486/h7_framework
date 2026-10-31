#include "alg_motor_tune.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

/**
 * @brief 限制浮点数范围
 * @param value 输入值
 * @param lower 下限
 * @param upper 上限
 * @return 限幅后的值
 */
static float MotorTuneClamp(float value, float lower, float upper)
{
  return fminf(fmaxf(value, lower), upper);
}

/**
 * @brief 检查物理参数搜索范围
 * @param range 搜索范围指针
 * @param lower 允许下限
 * @param upper 允许上限
 * @return 是否有效
 */
static bool MotorTuneRangeValid(const MotorTuneRange *range, float lower,
                                float upper)
{
  return isfinite(range->lower) && isfinite(range->upper) &&
         range->lower >= lower && range->upper <= upper &&
         range->upper >= range->lower && range->count >= 1U &&
         range->count <= 3U &&
         ((range->count == 1U) == (range->lower == range->upper));
}

/**
 * @brief 判断整定是否正在接管输出
 * @param tune 整定对象指针
 * @return 是否处于测试阶段
 */
bool MotorTuneIsActive(const MotorTuneObject *tune)
{
  return tune != NULL && tune->state >= kMotorTuneIdentify &&
         tune->state <= kMotorTuneValidate;
}

/**
 * @brief 结束无法继续的测试
 * @param tune 整定对象指针
 * @param error 失败原因
 * @return 零电流
 */
static float MotorTuneFail(MotorTuneObject *tune, MotorTuneError error)
{
  tune->state = kMotorTuneFailed;
  tune->error = error;
  tune->pending = false;
  tune->requested_current = 0.0f;
  AutotuneCancel(&tune->baseline);
  return 0.0f;
}

/**
 * @brief 初始化单电机性能整定对象
 * @param tune 整定对象指针
 * @param config 配置结构体指针
 * @return 是否成功
 */
bool MotorTuneInit(MotorTuneObject *tune, const MotorTuneConfig *config)
{
  if (tune == NULL || config == NULL ||
      !isfinite(config->plant.sampling_time) ||
      config->plant.sampling_time <= 0.0f ||
      !MotorTuneRangeValid(&config->bandwidth, 0.1f,
                           0.08f / config->plant.sampling_time) ||
      !MotorTuneRangeValid(&config->damping, 0.2f, 3.0f) ||
      !MotorTuneRangeValid(&config->feedforward, 0.0f, 1.5f) ||
      !isfinite(config->speed_amplitude) || config->speed_amplitude <= 0.0f ||
      config->speed_amplitude > 0.8f * config->plant.speed_limit ||
      !isfinite(config->position_amplitude) ||
      config->position_amplitude < 0.0f ||
      (config->plant.tune_position && config->position_amplitude <= 0.0f) ||
      !isfinite(config->segment_time) || config->segment_time < 0.1f ||
      config->segment_time > 10.0f ||
      config->segment_time < 20.0f * config->plant.sampling_time ||
      !isfinite(config->settling_band) || config->settling_band <= 0.0f ||
      config->settling_band >= 0.5f || config->plant.speed_offset != 0.0f ||
      config->objectives > 63U ||
      config->search_budget > MOTOR_TUNE_MAX_CANDIDATES ||
      config->validation_repeats > 3U)
  {
    return false;
  }
  const float limits[] = {config->limits.rms_error,
                          config->limits.overshoot,
                          config->limits.settling_time,
                          config->limits.idle_ripple,
                          config->limits.rms_slew,
                          config->limits.saturation,
                          config->comparison_tolerance.rms_error,
                          config->comparison_tolerance.overshoot,
                          config->comparison_tolerance.settling_time,
                          config->comparison_tolerance.idle_ripple,
                          config->comparison_tolerance.rms_slew,
                          config->comparison_tolerance.saturation};
  for (uint8_t i = 0U; i < 12U; i++)
  {
    if (!isfinite(limits[i]) || limits[i] < 0.0f)
    {
      return false;
    }
  }
  if (config->limits.saturation > 1.0f)
  {
    return false;
  }
  AutotuneConfig plant = config->plant;
  plant.adaptive = false;
  plant.optimize = false;
  memset(tune, 0, sizeof(*tune));
  if (!AutotuneInit(&tune->baseline, &plant))
  {
    return false;
  }
  tune->config = *config;
  if (tune->config.objectives == 0U)
  {
    tune->config.objectives =
        MOTOR_TUNE_OBJECTIVE_ERROR | MOTOR_TUNE_OBJECTIVE_SLEW;
  }
  if (tune->config.search_budget == 0U)
  {
    tune->config.search_budget = config->bandwidth.count *
                                 config->damping.count *
                                 config->feedforward.count;
  }
  if (tune->config.validation_repeats == 0U)
  {
    tune->config.validation_repeats = 2U;
  }
  tune->segment_ticks =
      (uint32_t)ceilf(config->segment_time / config->plant.sampling_time);
  tune->initialized = true;
  return true;
}

/**
 * @brief 从当前反馈开始单电机性能整定
 * @param tune 整定对象指针
 * @param position 当前连续角度（rad）
 * @param omega 当前角速度（rad/s）
 * @return 是否成功启动
 */
bool MotorTuneStart(MotorTuneObject *tune, float position, float omega)
{
  if (tune == NULL || !tune->initialized || MotorTuneIsActive(tune))
  {
    return false;
  }
  MotorTuneConfig config = tune->config;
  float room = fminf(position - config.plant.position_min,
                     config.plant.position_max - position);
  if (!isfinite(position) || !isfinite(omega) || room <= 0.0f ||
      (config.plant.tune_position && config.position_amplitude > 0.3f * room))
  {
    return false;
  }
  if (!MotorTuneInit(tune, &config) ||
      !AutotuneStart(&tune->baseline, position, omega, NULL))
  {
    return false;
  }
  tune->center = position;
  // 按整个速度序列的最大累计位移保留行程余量。
  tune->speed_amplitude =
      fminf(config.speed_amplitude, 0.15f * room / config.segment_time);
  tune->result.profile.tested_speed = tune->speed_amplitude;
  tune->state = kMotorTuneIdentify;
  return true;
}

/**
 * @brief 取消测试并停止测试输出
 * @param tune 整定对象指针
 */
void MotorTuneCancel(MotorTuneObject *tune)
{
  if (MotorTuneIsActive(tune))
  {
    AutotuneCancel(&tune->baseline);
    tune->state = kMotorTuneCancelled;
    tune->pending = false;
    tune->requested_current = 0.0f;
  }
}

/**
 * @brief 按物理设计参数生成一个候选
 * @param tune 整定对象指针
 * @param candidate 候选指针
 */
static void MotorTuneDesign(const MotorTuneObject *tune,
                            MotorTuneCandidate *candidate)
{
  const SysidModel *model = &tune->result.model;
  float wn = candidate->bandwidth;
  float kp = 2.0f * candidate->damping * wn * model->inertia - model->viscous;
  float ki = model->inertia * wn * wn;
  float ff = candidate->feedforward * model->inertia;
  float limit = tune->config.plant.current_limit;
  if (!isfinite(kp) || !isfinite(ki) || !isfinite(ff) || kp <= 0.0f ||
      ki <= 0.0f || !isfinite(limit / ki))
  {
    candidate->state = kMotorTuneInvalidDesign;
    return;
  }
  candidate->speed =
      (PidConfig){.kp = kp,
                  .ki = ki,
                  .sampling_time = tune->config.plant.sampling_time,
                  .is_derivativeonfeedback_enabled = true,
                  .is_feedforward_enabled = ff > 0.0f,
                  .kff = ff,
                  .is_antiwindup_enabled = true,
                  .output_max = limit,
                  .integral_max = limit / ki};
  candidate->position =
      (PidConfig){.kp = 0.15f * wn,
                  .sampling_time = tune->config.plant.sampling_time,
                  .is_derivativeonfeedback_enabled = true,
                  .is_antiwindup_enabled = true,
                  .output_max = tune->config.plant.speed_limit};
}

/**
 * @brief 加入范围内尚未测试的候选
 * @param tune 整定对象指针
 * @param value 自然频率、阻尼和前馈比例
 * @return 是否加入
 */
static bool MotorTuneAdd(MotorTuneObject *tune, const float value[3])
{
  if (tune->result.candidate_count >= tune->config.search_budget)
  {
    return false;
  }
  for (uint8_t i = 0U; i < tune->result.candidate_count; i++)
  {
    const MotorTuneCandidate *c = &tune->result.candidate[i];
    if (fabsf(c->bandwidth - value[0]) <= 1.0e-5f * fmaxf(1.0f, value[0]) &&
        fabsf(c->damping - value[1]) <= 1.0e-5f &&
        fabsf(c->feedforward - value[2]) <= 1.0e-5f)
    {
      return false;
    }
  }
  MotorTuneCandidate *c =
      &tune->result.candidate[tune->result.candidate_count++];
  *c = (MotorTuneCandidate){
      .bandwidth = value[0], .damping = value[1], .feedforward = value[2]};
  MotorTuneDesign(tune, c);
  return true;
}

/**
 * @brief 生成中心和单轴边界初始候选
 * @param tune 整定对象指针
 */
static void MotorTuneCandidates(MotorTuneObject *tune)
{
  const MotorTuneRange *range[3] = {&tune->config.bandwidth,
                                    &tune->config.damping,
                                    &tune->config.feedforward};
  float center[3];
  for (uint8_t d = 0U; d < 3U; d++)
  {
    center[d] = 0.5f * (range[d]->lower + range[d]->upper);
  }
  (void)MotorTuneAdd(tune, center);
  for (uint8_t d = 0U; d < 3U; d++)
  {
    if (range[d]->count == 1U)
    {
      continue;
    }
    for (uint8_t side = 0U; side < 2U; side++)
    {
      float value[3] = {center[0], center[1], center[2]};
      value[d] = side == 0U ? range[d]->lower : range[d]->upper;
      (void)MotorTuneAdd(tune, value);
    }
  }
}

/**
 * @brief 以当前反馈和负载初始化控制器状态
 * @param tune 整定对象指针
 * @param position 当前连续角度（rad）
 * @param omega 当前角速度（rad/s）
 * @param current 当前反馈电流（A）
 */
static void MotorTunePrime(MotorTuneObject *tune, float position, float omega,
                           float current)
{
  PidReset(&tune->speed_pid);
  PidReset(&tune->position_pid);
  tune->speed_pid.reference = omega;
  tune->speed_pid.feedback = omega;
  tune->speed_pid.previous_reference = omega;
  tune->speed_pid.previous_feedback = omega;
  tune->speed_pid.error_sum = MotorTuneClamp(current / tune->speed_pid.ki,
                                             -tune->speed_pid.integral_max,
                                             tune->speed_pid.integral_max);
  tune->position_pid.reference = position;
  tune->position_pid.feedback = position;
}

/**
 * @brief 回到统一初始位置再开始下一轮
 * @param tune 整定对象指针
 * @param next 下一测试阶段
 * @param position 当前连续角度（rad）
 * @param omega 当前角速度（rad/s）
 * @param current 当前反馈电流（A）
 */
static void MotorTunePrepare(MotorTuneObject *tune, MotorTuneState next,
                             float position, float omega, float current)
{
  tune->state = kMotorTunePrepare;
  tune->next_state = next;
  tune->elapsed = 0.0f;
  tune->ready_time = 0.0f;
  tune->speed_pid = tune->baseline.speed_pid;
  tune->position_pid = tune->baseline.position_pid;
  MotorTunePrime(tune, position, omega, current);
}

/**
 * @brief 开始一轮具有独立统计的测试
 * @param tune 整定对象指针
 * @param position 当前连续角度（rad）
 * @param omega 当前角速度（rad/s）
 * @param current 当前反馈电流（A）
 */
static void MotorTuneBegin(MotorTuneObject *tune, float position, float omega,
                           float current)
{
  tune->state = tune->next_state;
  if (tune->state != kMotorTuneCharacterize)
  {
    MotorTuneCandidate *candidate =
        &tune->result.candidate[tune->candidate_index];
    PidInit(&tune->speed_pid, &candidate->speed);
    PidInit(&tune->position_pid, &candidate->position);
  }
  MotorTunePrime(tune, position, omega, current);
  tune->metrics = (MotorTuneMetrics){.settled = true};
  tune->tick = 0U;
  tune->segment = 0U;
  tune->samples = 0U;
  tune->limited_samples = 0U;
  tune->sum_error = 0.0f;
  tune->sum_slew = 0.0f;
  tune->previous_current = tune->executed_current;
  tune->previous_target = 0.0f;
  tune->target = 0.0f;
  tune->hold_samples = 0U;
  tune->hold_mean = 0.0f;
  tune->hold_variance = 0.0f;
  tune->last_outside = 0.0f;
}

/**
 * @brief 读取相互独立的训练和验证轨迹目标
 * @param tune 整定对象指针
 * @return 归一化目标
 */
static float MotorTuneTarget(const MotorTuneObject *tune)
{
  static const float speed[16] = {0.0f, 0.12f, 0.12f, 0.0f,  -0.12f, -0.12f,
                                  1.0f, 1.0f,  -1.0f, -1.0f, 0.0f,   0.0f,
                                  0.0f, 0.0f,  0.0f,  0.0f};
  static const float position[16] = {0.0f, 0.35f, 0.35f, 0.0f,  -0.35f, -0.35f,
                                     1.0f, 1.0f,  -1.0f, -1.0f, 0.0f,   0.0f,
                                     0.0f, 0.0f,  0.0f,  0.0f};
  static const float validation[16] = {
      0.0f,  -0.4f,  -0.4f,  0.25f, 0.25f, -0.85f, -0.85f, 0.85f,
      0.85f, -0.25f, -0.25f, 0.4f,  0.4f,  0.0f,   0.0f,   0.0f};
  if (tune->state == kMotorTuneCharacterize)
  {
    if (tune->position_test)
    {
      static const float hold[4] = {-0.8f, 0.0f, 0.8f, 0.0f};
      return hold[tune->segment - 12U];
    }
    return speed[tune->segment];
  }
  if (tune->state == kMotorTuneValidate)
  {
    return validation[tune->segment];
  }
  return tune->position_test ? position[tune->segment] : speed[tune->segment];
}

/**
 * @brief 判断实测性能是否满足期望范围
 * @param tune 整定对象指针
 * @return 是否可行
 */
static bool MotorTuneFeasible(const MotorTuneObject *tune)
{
  const MotorTuneMetrics *m = &tune->metrics;
  const MotorTuneLimits *l = &tune->config.limits;
  return tune->samples > 0U && m->settled && isfinite(m->rms_error) &&
         isfinite(m->rms_slew) &&
         (l->rms_error == 0.0f || m->rms_error <= l->rms_error) &&
         (l->overshoot == 0.0f || m->overshoot <= l->overshoot) &&
         (l->settling_time == 0.0f || m->settling_time <= l->settling_time) &&
         (l->idle_ripple == 0.0f || m->idle_ripple <= l->idle_ripple) &&
         (l->rms_slew == 0.0f || m->rms_slew <= l->rms_slew) &&
         (l->saturation == 0.0f || m->saturation <= l->saturation);
}

/**
 * @brief 比较两组性能的帕累托支配关系
 * @param tune 整定对象指针
 * @param a 第一组性能
 * @param b 第二组性能
 * @return 第一组是否支配第二组
 */
static bool MotorTuneDominates(const MotorTuneObject *tune,
                               const MotorTuneMetrics *a,
                               const MotorTuneMetrics *b)
{
  const MotorTuneLimits *t = &tune->config.comparison_tolerance;
  const float av[6] = {a->rms_error,   a->overshoot, a->settling_time,
                       a->idle_ripple, a->rms_slew,  a->saturation};
  const float bv[6] = {b->rms_error,   b->overshoot, b->settling_time,
                       b->idle_ripple, b->rms_slew,  b->saturation};
  const float tolerance[6] = {t->rms_error,   t->overshoot, t->settling_time,
                              t->idle_ripple, t->rms_slew,  t->saturation};
  bool better = false;
  for (uint8_t i = 0U; i < 6U; i++)
  {
    if ((tune->config.objectives & (1U << i)) == 0U)
    {
      continue;
    }
    if (av[i] > bv[i])
    {
      return false;
    }
    better = better || av[i] + tolerance[i] < bv[i];
  }
  return better;
}

/**
 * @brief 优先细化已验证前沿，无可行点时围绕约束违背最小的候选探索
 * @param tune 整定对象指针
 * @return 是否加入下一批候选
 */
static bool MotorTuneRefine(MotorTuneObject *tune)
{
  if (tune->result.candidate_count >= tune->config.search_budget ||
      tune->search_round >= 4U)
  {
    return false;
  }
  uint8_t count = tune->result.candidate_count;
  uint8_t best = 0U;
  float violation = INFINITY;
  const MotorTuneLimits *l = &tune->config.limits;
  const float limits[6] = {l->rms_error,   l->overshoot, l->settling_time,
                           l->idle_ripple, l->rms_slew,  l->saturation};
  for (uint8_t i = 0U; i < count; i++)
  {
    const MotorTuneCandidate *c = &tune->result.candidate[i];
    if (c->state == kMotorTuneInvalidDesign)
    {
      continue;
    }
    const MotorTuneMetrics *m =
        c->validation_runs > 0U ? &c->validation : &c->training;
    const float values[6] = {m->rms_error,   m->overshoot, m->settling_time,
                             m->idle_ripple, m->rms_slew,  m->saturation};
    float score = m->settled ? 0.0f : 1.0f;
    for (uint8_t j = 0U; j < 6U; j++)
    {
      if (limits[j] > 0.0f)
      {
        score += fmaxf(values[j] / limits[j] - 1.0f, 0.0f);
      }
    }
    if (score < violation)
    {
      violation = score;
      best = i;
    }
  }
  if (!isfinite(violation))
  {
    return false;
  }
  const MotorTuneRange *range[3] = {&tune->config.bandwidth,
                                    &tune->config.damping,
                                    &tune->config.feedforward};
  float fraction = 0.25f / (float)(1U << tune->search_round++);
  bool added = false;
  for (uint8_t i = 0U; i < count; i++)
  {
    const MotorTuneCandidate *anchor = &tune->result.candidate[i];
    bool selected = anchor->state == kMotorTuneVerified;
    for (uint8_t j = 0U; selected && j < count; j++)
    {
      if (tune->result.candidate[j].state == kMotorTuneVerified &&
          MotorTuneDominates(tune, &tune->result.candidate[j].validation,
                             &anchor->validation))
      {
        selected = false;
      }
    }
    if ((!selected && (tune->result.verified_count > 0U || i != best)))
    {
      continue;
    }
    float center[3] = {anchor->bandwidth, anchor->damping, anchor->feedforward};
    for (uint8_t d = 0U; d < 3U; d++)
    {
      if (range[d]->count == 1U)
      {
        continue;
      }
      for (uint8_t side = 0U; side < 2U; side++)
      {
        float value[3] = {center[0], center[1], center[2]};
        value[d] =
            MotorTuneClamp(center[d] + (side == 0U ? -1.0f : 1.0f) * fraction *
                                           (range[d]->upper - range[d]->lower),
                           range[d]->lower, range[d]->upper);
        added = MotorTuneAdd(tune, value) || added;
      }
    }
  }
  return added;
}

/**
 * @brief 从已验证候选中生成完整帕累托集合
 * @param tune 整定对象指针
 */
static void MotorTunePareto(MotorTuneObject *tune)
{
  tune->result.pareto_count = 0U;
  for (uint8_t i = 0U; i < tune->result.candidate_count; i++)
  {
    if (tune->result.candidate[i].state != kMotorTuneVerified)
    {
      continue;
    }
    bool dominated = false;
    for (uint8_t j = 0U; j < tune->result.candidate_count; j++)
    {
      if (tune->result.candidate[j].state == kMotorTuneVerified &&
          MotorTuneDominates(tune, &tune->result.candidate[j].validation,
                             &tune->result.candidate[i].validation))
      {
        dominated = true;
        break;
      }
    }
    if (!dominated)
    {
      tune->result.pareto[tune->result.pareto_count++] = i;
    }
  }
  if (tune->result.pareto_count == 0U)
  {
    (void)MotorTuneFail(tune, kMotorTuneNoFeasibleCandidate);
  }
  else
  {
    tune->state = kMotorTuneComplete;
  }
}

/**
 * @brief 选择下一组候选并回到统一初始状态
 * @param tune 整定对象指针
 * @param position 当前连续角度（rad）
 * @param omega 当前角速度（rad/s）
 * @param current 当前反馈电流（A）
 */
static void MotorTuneNext(MotorTuneObject *tune, float position, float omega,
                          float current)
{
  while (tune->candidate_index < tune->result.candidate_count &&
         tune->result.candidate[tune->candidate_index].state !=
             kMotorTuneUntested)
  {
    tune->candidate_index++;
  }
  if (tune->candidate_index == tune->result.candidate_count)
  {
    if (!MotorTuneRefine(tune))
    {
      MotorTunePareto(tune);
      return;
    }
    MotorTuneNext(tune, position, omega, current);
    return;
  }
  MotorTunePrepare(tune, kMotorTuneSearch, position, omega, current);
}

/**
 * @brief 记录基准闭环下的方向、低速与换向特征
 * @param tune 整定对象指针
 * @param position 当前连续角度（rad）
 * @param omega 当前角速度（rad/s）
 * @param current 当前反馈电流（A）
 * @param progress 当前段进度
 */
static void MotorTuneCharacterize(MotorTuneObject *tune, float position,
                                  float omega, float current, float progress)
{
  MotorTuneProfile *profile = &tune->result.profile;
  uint8_t s = tune->segment;
  float dt = tune->config.plant.sampling_time;
  float time = (float)tune->tick * dt;
  if (tune->position_test)
  {
    if (s < 15U && progress >= 0.75f &&
        fabsf(omega) <= 0.02f * tune->speed_amplitude &&
        fabsf(position - tune->center - tune->target) <=
            tune->config.settling_band * tune->config.position_amplitude)
    {
      MotorTuneHold *hold = &profile->hold[s - 12U];
      hold->count++;
      hold->position += (position - hold->position) / (float)hold->count;
      hold->current += (current - hold->current) / (float)hold->count;
    }
    return;
  }
  uint8_t direction = tune->target > 0.0f ? 1U : 0U;
  if (tune->target != 0.0f && progress >= 0.5f)
  {
    float error = tune->target - omega;
    tune->direction_sum[direction] += error * error;
    profile->direction_count[direction]++;
    if (s == 2U || s == 5U)
    {
      tune->low_speed_sum[direction] += error * error;
      profile->low_speed_count[direction]++;
    }
    if (s == 7U || s == 9U)
    {
      tune->working_sum[direction] += omega / tune->target;
      profile->working_count[direction]++;
    }
  }
  if (s == 1U || s == 4U)
  {
    if (!profile->startup_observed[direction])
    {
      profile->startup_delay[direction] = time;
      profile->startup_current[direction] =
          fmaxf(profile->startup_current[direction], fabsf(current));
      profile->startup_observed[direction] =
          omega * copysignf(1.0f, tune->target) > 0.02f * tune->speed_amplitude;
    }
  }
  if (s == 8U && !profile->reversal_observed)
  {
    profile->reversal_delay = time;
    profile->reversal_travel = fabsf(position - tune->crossing_position);
    profile->reversal_observed = omega < -0.02f * tune->speed_amplitude;
  }
}

/**
 * @brief 更新保持段超调、稳定时间与静止波动
 * @param tune 整定对象指针
 * @param feedback 当前测试坐标反馈
 * @param amplitude 测试幅度
 * @param progress 当前段进度
 */
static void MotorTuneMeasure(MotorTuneObject *tune, float feedback,
                             float amplitude, float progress)
{
  float error = tune->reference - feedback;
  tune->sum_error += (error / amplitude) * (error / amplitude);
  tune->samples++;
  if (progress < 0.5f)
  {
    return;
  }
  float dt = tune->config.plant.sampling_time;
  float hold_time =
      ((float)tune->tick - 0.5f * (float)tune->segment_ticks) * dt;
  float change = tune->target - tune->previous_target;
  if (change != 0.0f)
  {
    float overshoot = (feedback - tune->target) / change;
    tune->metrics.overshoot = fmaxf(tune->metrics.overshoot, overshoot);
  }
  if (fabsf(tune->target - feedback) > tune->config.settling_band * amplitude)
  {
    tune->last_outside = hold_time + dt;
  }
  if (progress >= 0.75f)
  {
    tune->hold_samples++;
    float delta = error - tune->hold_mean;
    tune->hold_mean += delta / (float)tune->hold_samples;
    tune->hold_variance += delta * (error - tune->hold_mean);
  }
}

/**
 * @brief 结束一个轨迹段并汇总保持统计
 * @param tune 整定对象指针
 */
static void MotorTuneSegmentFinish(MotorTuneObject *tune)
{
  float dt = tune->config.plant.sampling_time;
  float duration = 0.5f * (float)tune->segment_ticks * dt;
  tune->metrics.settling_time =
      fmaxf(tune->metrics.settling_time, tune->last_outside);
  if (tune->last_outside > duration - 0.1f * duration)
  {
    tune->metrics.settled = false;
  }
  if (tune->hold_samples > 1U)
  {
    tune->metrics.idle_ripple = fmaxf(
        tune->metrics.idle_ripple,
        sqrtf(fmaxf(0.0f, tune->hold_variance) / (float)tune->hold_samples));
  }
  tune->previous_target = tune->target;
  tune->tick = 0U;
  tune->segment++;
  tune->hold_samples = 0U;
  tune->hold_mean = 0.0f;
  tune->hold_variance = 0.0f;
  tune->last_outside = 0.0f;
}

/**
 * @brief 结束包含最终执行量的完整测试窗口
 * @param tune 整定对象指针
 * @param position 当前连续角度（rad）
 * @param omega 当前角速度（rad/s）
 * @param current 当前反馈电流（A）
 */
static void MotorTuneFinish(MotorTuneObject *tune, float position, float omega,
                            float current)
{
  if (tune->state == kMotorTuneCharacterize)
  {
    MotorTuneProfile *p = &tune->result.profile;
    for (uint8_t i = 0U; i < 2U; i++)
    {
      if (p->direction_count[i] > 0U)
      {
        p->direction_rms[i] =
            sqrtf(tune->direction_sum[i] / (float)p->direction_count[i]);
      }
      if (p->low_speed_count[i] > 0U)
      {
        p->low_speed_rms[i] =
            sqrtf(tune->low_speed_sum[i] / (float)p->low_speed_count[i]);
      }
      if (p->working_count[i] > 0U)
      {
        p->working_gain[i] = tune->working_sum[i] / (float)p->working_count[i];
      }
    }
    MotorTuneNext(tune, position, omega, current);
    return;
  }
  tune->metrics.rms_error = sqrtf(tune->sum_error / (float)tune->samples);
  tune->metrics.rms_slew = sqrtf(tune->sum_slew / (float)tune->samples);
  tune->metrics.saturation =
      (float)tune->limited_samples / (float)tune->samples;
  MotorTuneCandidate *candidate =
      &tune->result.candidate[tune->candidate_index];
  bool feasible = MotorTuneFeasible(tune);
  if (tune->state == kMotorTuneSearch)
  {
    candidate->training = tune->metrics;
    tune->result.evaluated_count++;
    if (feasible)
    {
      MotorTunePrepare(tune, kMotorTuneValidate, position, omega, current);
      return;
    }
    candidate->state = kMotorTuneRejected;
  }
  else
  {
    MotorTuneMetrics *m = &candidate->validation;
    m->rms_error = fmaxf(m->rms_error, tune->metrics.rms_error);
    m->overshoot = fmaxf(m->overshoot, tune->metrics.overshoot);
    m->settling_time = fmaxf(m->settling_time, tune->metrics.settling_time);
    m->idle_ripple = fmaxf(m->idle_ripple, tune->metrics.idle_ripple);
    m->rms_slew = fmaxf(m->rms_slew, tune->metrics.rms_slew);
    m->saturation = fmaxf(m->saturation, tune->metrics.saturation);
    m->settled = feasible;
    candidate->validation_runs++;
    if (feasible &&
        candidate->validation_runs < tune->config.validation_repeats)
    {
      MotorTunePrepare(tune, kMotorTuneValidate, position, omega, current);
      return;
    }
    candidate->state =
        feasible ? kMotorTuneVerified : kMotorTuneValidationRejected;
    if (feasible)
    {
      tune->result.verified_count++;
    }
  }
  tune->candidate_index++;
  MotorTuneNext(tune, position, omega, current);
}

/**
 * @brief 更新测试并生成本周期电流请求
 * @param tune 整定对象指针
 * @param position 当前连续角度（rad）
 * @param omega 控制反馈角速度（rad/s）
 * @param actual_current 实际反馈电流（A）
 * @return 测试电流（A）
 */
float MotorTuneUpdate(MotorTuneObject *tune, float position, float omega,
                      float actual_current)
{
  if (!MotorTuneIsActive(tune))
  {
    return 0.0f;
  }
  if (!isfinite(position) || !isfinite(omega) || !isfinite(actual_current))
  {
    return MotorTuneFail(tune, kMotorTuneInvalidSample);
  }
  if (position <= tune->config.plant.position_min ||
      position >= tune->config.plant.position_max ||
      fabsf(omega) > 1.2f * tune->config.plant.speed_limit)
  {
    return MotorTuneFail(tune, kMotorTuneMotionLimit);
  }
  if (tune->pending && !tune->tracked)
  {
    return MotorTuneFail(tune, kMotorTuneOutputMissing);
  }
  // 当前反馈对应上一周期已执行的参考与电流。
  if (tune->pending && tune->tick > 0U &&
      (tune->state == kMotorTuneCharacterize ||
       tune->state == kMotorTuneSearch || tune->state == kMotorTuneValidate))
  {
    float amplitude = tune->position_test ? tune->config.position_amplitude
                                          : tune->speed_amplitude;
    float feedback = tune->position_test ? position - tune->center : omega;
    float progress = (float)(tune->tick - 1U) / (float)tune->segment_ticks;
    MotorTuneMeasure(tune, feedback, amplitude, progress);
    if (tune->state == kMotorTuneCharacterize)
    {
      MotorTuneCharacterize(tune, position, omega, actual_current, progress);
    }
  }
  tune->pending = false;
  tune->tracked = false;
  if (tune->state == kMotorTuneIdentify)
  {
    tune->requested_current =
        AutotuneUpdate(&tune->baseline, position, omega, actual_current, omega);
    if (tune->baseline.state == kAutotuneFailed)
    {
      return MotorTuneFail(tune, kMotorTuneIdentificationFailed);
    }
    if (tune->baseline.state == kAutotuneComplete)
    {
      tune->result.model = tune->baseline.result.model;
      MotorTuneCandidates(tune);
      MotorTunePrepare(tune, kMotorTuneCharacterize, position, omega,
                       actual_current);
    }
    else
    {
      tune->pending = true;
      return tune->requested_current;
    }
  }
  if (tune->state != kMotorTunePrepare && tune->tick == tune->segment_ticks)
  {
    MotorTuneSegmentFinish(tune);
    if (tune->segment == 16U)
    {
      MotorTuneFinish(tune, position, omega, actual_current);
      if (!MotorTuneIsActive(tune))
      {
        tune->requested_current = 0.0f;
        return 0.0f;
      }
    }
  }
  float dt = tune->config.plant.sampling_time;
  if (tune->state == kMotorTunePrepare)
  {
    tune->elapsed += dt;
    float reference = MotorTuneClamp(
        tune->baseline.result.position_kp * (tune->center - position),
        -0.3f * tune->speed_amplitude, 0.3f * tune->speed_amplitude);
    tune->requested_current = PidCalculate(&tune->speed_pid, reference, omega);
    float room = fminf(tune->center - tune->config.plant.position_min,
                       tune->config.plant.position_max - tune->center);
    float tolerance = tune->config.plant.tune_position
                          ? 0.02f * tune->config.position_amplitude
                          : 0.005f * room;
    if (fabsf(position - tune->center) <= tolerance &&
        fabsf(omega) <= 0.02f * tune->speed_amplitude)
    {
      tune->ready_time += dt;
    }
    else
    {
      tune->ready_time = 0.0f;
    }
    if (tune->ready_time >= 0.25f)
    {
      MotorTuneBegin(tune, position, omega, actual_current);
      // 本周期仍执行准备阶段输出，下一周期才采集候选数据。
      tune->pending = true;
      return tune->requested_current;
    }
    if (tune->elapsed > 30.0f + 20.0f / tune->baseline.result.position_kp)
    {
      return MotorTuneFail(tune, kMotorTunePrepareTimeout);
    }
    tune->pending = true;
    return tune->requested_current;
  }
  tune->position_test =
      tune->config.plant.tune_position &&
      (tune->state != kMotorTuneCharacterize || tune->segment >= 12U);
  float amplitude = tune->position_test ? tune->config.position_amplitude
                                        : tune->speed_amplitude;
  if (tune->tick == 0U)
  {
    if (tune->state == kMotorTuneCharacterize && tune->segment == 12U &&
        tune->position_test)
    {
      tune->previous_target = 0.0f;
    }
    tune->target = MotorTuneTarget(tune) * amplitude;
    if (tune->state == kMotorTuneCharacterize && tune->segment == 8U)
    {
      tune->crossing_position = position;
    }
  }
  float progress = (float)tune->tick / (float)tune->segment_ticks;
  float blend = MotorTuneClamp(2.0f * progress, 0.0f, 1.0f);
  blend = blend * blend * (3.0f - 2.0f * blend);
  tune->reference =
      tune->previous_target + blend * (tune->target - tune->previous_target);
  float reference = tune->position_test
                        ? PidCalculate(&tune->position_pid,
                                       tune->center + tune->reference, position)
                        : tune->reference;
  tune->requested_current = PidCalculate(&tune->speed_pid, reference, omega);
  if (!isfinite(tune->requested_current))
  {
    return MotorTuneFail(tune, kMotorTuneInvalidSample);
  }
  tune->tick++;
  tune->pending = true;
  return tune->requested_current;
}

/**
 * @brief 反馈本周期最终执行指令并累计限幅与输出变化
 * @param tune 整定对象指针
 * @param executed_current 最终发送的电流指令（A）
 * @return 是否接受本次反馈
 */
bool MotorTuneTrackOutput(MotorTuneObject *tune, float executed_current)
{
  if (!MotorTuneIsActive(tune) || !tune->pending || tune->tracked)
  {
    return false;
  }
  if (!isfinite(executed_current))
  {
    (void)MotorTuneFail(tune, kMotorTuneInvalidSample);
    return false;
  }
  tune->tracked = true;
  tune->executed_current = executed_current;
  if (tune->state == kMotorTuneIdentify)
  {
    AutotuneTrackOutput(&tune->baseline, executed_current);
    return true;
  }
  float tracking_rate =
      tune->state == kMotorTuneSearch || tune->state == kMotorTuneValidate
          ? tune->result.candidate[tune->candidate_index].bandwidth
          : tune->baseline.result.bandwidth;
  PidTrackOutput(&tune->speed_pid, executed_current - tune->requested_current,
                 tracking_rate);
  if ((tune->state == kMotorTuneSearch || tune->state == kMotorTuneValidate) &&
      tune->tick > 0U)
  {
    float slew = (executed_current - tune->previous_current) /
                 tune->config.plant.sampling_time;
    tune->sum_slew += slew * slew;
    float limit = tune->config.plant.current_limit;
    if (fabsf(tune->requested_current) >= 0.999f * limit ||
        fabsf(executed_current - tune->requested_current) > 0.001f * limit)
    {
      tune->limited_samples++;
    }
  }
  tune->previous_current = executed_current;
  return true;
}

/**
 * @brief 导出通过轨迹验证的帕累托候选控制器配置
 * @param tune 整定对象指针
 * @param pareto_index 帕累托集合下标
 * @param speed 速度控制器配置输出指针
 * @param position 位置控制器配置输出指针，速度模式可传NULL
 * @return 是否成功
 */
bool MotorTuneExport(const MotorTuneObject *tune, uint8_t pareto_index,
                     PidConfig *speed, PidConfig *position)
{
  if (tune == NULL || speed == NULL || tune->state != kMotorTuneComplete ||
      pareto_index >= tune->result.pareto_count ||
      (tune->config.plant.tune_position && position == NULL))
  {
    return false;
  }
  const MotorTuneCandidate *candidate =
      &tune->result.candidate[tune->result.pareto[pareto_index]];
  *speed = candidate->speed;
  if (position != NULL)
  {
    *position = candidate->position;
  }
  return true;
}
