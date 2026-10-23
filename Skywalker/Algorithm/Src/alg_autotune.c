#include "alg_autotune.h"

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
static float AutotuneClamp(float value, float lower, float upper)
{
  return fminf(fmaxf(value, lower), upper);
}

/**
 * @brief 初始化独立的验证控制器
 * @param pid 控制器对象指针
 * @param kp 比例增益
 * @param ki 积分增益
 * @param limit 输出上限
 * @param dt 采样时间（s）
 */
static void AutotunePidInit(PidObject *pid, float kp, float ki, float limit,
                            float dt)
{
  PidConfig config = {.kp = kp,
                      .ki = ki,
                      .sampling_time = dt,
                      .output_max = limit,
                      .integral_max = ki > 0.0f ? limit / ki : 0.0f,
                      .is_antiwindup_enabled = true,
                      .is_derivativeonfeedback_enabled = true};
  PidInit(pid, &config);
}

/**
 * @brief 切换测试阶段并清除验证统计
 * @param tune 整定对象指针
 * @param state 新状态
 */
static void AutotunePhase(AutotuneObject *tune, AutotuneState state)
{
  tune->state = state;
  tune->elapsed = 0.0f;
  tune->sum_error = 0.0f;
  tune->peak_error = 0.0f;
  tune->samples = 0U;
  tune->velocity_sine = 0.0f;
  tune->velocity_cosine = 0.0f;
}

/**
 * @brief 初始化积分辨识工作区
 * @param tune 整定对象指针
 * @param memory_time 递推记忆时间（s）
 */
static void AutotuneIdentifyInit(AutotuneObject *tune, float memory_time)
{
  SysidConfig config = {.sampling_time = tune->config.sampling_time,
                        .window_time =
                            fmaxf(0.02f, 4.0f * tune->config.sampling_time),
                        .current_scale = tune->config.current_limit,
                        .omega_scale = tune->config.speed_limit,
                        .memory_time = memory_time,
                        .method = tune->config.identification};
  (void)SysidInit(&tune->sysid, &config);
  (void)SysidInit(&tune->rotor_sysid, &config);
}

/**
 * @brief 按模型和期望带宽生成并联PI参数
 * @param tune 整定对象指针
 * @param bandwidth 期望自然频率（rad/s）
 */
static void AutotuneDesign(AutotuneObject *tune, float bandwidth)
{
  float inertia = tune->result.model.inertia;
  float viscous = tune->result.model.viscous;
  float kp =
      fmaxf(2.0f * bandwidth * inertia - viscous, 0.1f * bandwidth * inertia);
  float ki = inertia * bandwidth * bandwidth;
  tune->result.bandwidth = bandwidth;
  tune->result.speed_kp = kp;
  tune->result.speed_ki = ki;
  tune->result.position_kp = 0.15f * bandwidth;
  AutotunePidInit(&tune->speed_pid, kp, ki, tune->config.current_limit,
                  tune->config.sampling_time);
  AutotunePidInit(&tune->position_pid, tune->result.position_kp, 0.0f,
                  tune->config.speed_limit, tune->config.sampling_time);
}

/**
 * @brief 保存性能优化相对模型设计的增益比例
 * @param tune 整定对象指针
 * @param speed 已验证的速度控制器
 */
static void AutotuneGainScale(AutotuneObject *tune, const PidObject *speed)
{
  float bandwidth = tune->result.bandwidth;
  float inertia = tune->result.model.inertia;
  float kp = fmaxf(2.0f * bandwidth * inertia - tune->result.model.viscous,
                   0.1f * bandwidth * inertia);
  tune->gain_scale[0] = speed->kp / kp;
  tune->gain_scale[1] = speed->ki / (inertia * bandwidth * bandwidth);
}

/**
 * @brief 用当前反馈和已辨识负载初始化验证状态
 * @param tune 整定对象指针
 * @param position 当前连续角度（rad）
 * @param omega 当前角速度（rad/s）
 */
static void AutotunePrime(AutotuneObject *tune, float position, float omega)
{
  PidObject *speed = &tune->speed_pid;
  speed->reference = omega;
  speed->previous_reference = omega;
  speed->feedback = omega;
  speed->previous_feedback = omega;
  float direction =
      AutotuneClamp(omega / (0.01f * tune->config.speed_limit), -1.0f, 1.0f);
  float current = tune->result.model.viscous * omega +
                  tune->result.model.friction * direction +
                  tune->result.model.bias;
  speed->error_sum =
      AutotuneClamp(current, -speed->output_max, speed->output_max) / speed->ki;
  tune->position_pid.reference = position;
  tune->position_pid.previous_reference = position;
  tune->position_pid.feedback = position;
  tune->position_pid.previous_feedback = position;
}

/**
 * @brief 初始化自动整定对象
 * @param tune 整定对象指针
 * @param config 配置结构体指针
 * @return 是否成功
 */
bool AutotuneInit(AutotuneObject *tune, const AutotuneConfig *config)
{
  if (tune == NULL || config == NULL || !isfinite(config->sampling_time) ||
      config->sampling_time <= 0.0f || config->sampling_time > 0.05f ||
      !isfinite(config->current_limit) || config->current_limit <= 0.0f ||
      !isfinite(config->speed_limit) || config->speed_limit <= 0.0f ||
      !isfinite(config->position_min) || !isfinite(config->position_max) ||
      config->position_max <= config->position_min ||
      !isfinite(config->speed_offset) ||
      fabsf(config->speed_offset) >= 0.7f * config->speed_limit ||
      !isfinite(config->response_time) || config->response_time < 0.0f ||
      (config->identification != kSysidExponential &&
       config->identification != kSysidSubspace) ||
      (config->tune_position && config->speed_offset != 0.0f))
  {
    return false;
  }
  memset(tune, 0, sizeof(*tune));
  tune->config = *config;
  AutotuneIdentifyInit(tune, 0.0f);
  return true;
}

/**
 * @brief 判断整定是否正在接管输出
 * @param tune 整定对象指针
 * @return 是否处于测试阶段
 */
bool AutotuneIsActive(const AutotuneObject *tune)
{
  return tune != NULL && tune->state >= kAutotuneIdentify &&
         tune->state <= kAutotuneDob;
}

/**
 * @brief 从当前反馈开始一次自动整定
 * @param tune 整定对象指针
 * @param position 当前连续角度（rad）
 * @param omega 当前角速度（rad/s）
 * @param dob 名义扰动观测器，无补偿时传NULL
 * @return 是否成功启动
 */
bool AutotuneStart(AutotuneObject *tune, float position, float omega,
                   const DobObject *dob)
{
  if (tune == NULL || AutotuneIsActive(tune) || !isfinite(position) ||
      !isfinite(omega) || position <= tune->config.position_min ||
      position >= tune->config.position_max ||
      fabsf(omega) > tune->config.speed_limit)
  {
    return false;
  }
  AutotuneConfig config = tune->config;
  (void)AutotuneInit(tune, &config);
  tune->center = position;
  tune->random = 0x713AC529U;
  tune->test_amplitude = 0.2f * config.current_limit;
  tune->has_dob = dob != NULL && dob->is_enabled;
  if (tune->has_dob)
  {
    tune->dob = *dob;
    DobReset(&tune->dob);
  }
  AutotunePhase(tune, kAutotuneIdentify);
  return true;
}

/**
 * @brief 取消测试并停止测试输出
 * @param tune 整定对象指针
 */
void AutotuneCancel(AutotuneObject *tune)
{
  if (AutotuneIsActive(tune))
  {
    tune->state = kAutotuneCancelled;
    tune->requested_current = 0.0f;
  }
}

/**
 * @brief 结束无法通过模型或响应验证的测试
 * @param tune 整定对象指针
 * @param error 失败原因
 * @return 零电流
 */
static float AutotuneFail(AutotuneObject *tune, AutotuneError error)
{
  tune->state = kAutotuneFailed;
  tune->error = error;
  tune->requested_current = 0.0f;
  return 0.0f;
}

/**
 * @brief 生成带工作点和位置约束的辨识电流
 * @param tune 整定对象指针
 * @param position 当前连续角度（rad）
 * @param omega 当前角速度（rad/s）
 * @return 测试电流（A）
 */
static float AutotuneExcite(AutotuneObject *tune, float position, float omega)
{
  tune->excitation_time += tune->config.sampling_time;
  tune->excitation_peak =
      fmaxf(tune->excitation_peak, fabsf(omega - tune->config.speed_offset));
  if (tune->excitation_time >= 0.5f)
  {
    if (tune->excitation_peak < 0.02f * tune->config.speed_limit)
    {
      tune->test_amplitude =
          fminf(1.4f * tune->test_amplitude, 0.7f * tune->config.current_limit);
    }
    else if (tune->excitation_peak > 0.5f * tune->config.speed_limit)
    {
      tune->test_amplitude *= 0.8f;
    }
    tune->excitation_time = 0.0f;
    tune->excitation_peak = 0.0f;
  }
  if (tune->pulse_ticks == 0U)
  {
    tune->random ^= tune->random << 13;
    tune->random ^= tune->random >> 17;
    tune->random ^= tune->random << 5;
    tune->pulse_ticks =
        (uint32_t)((0.025f + 0.015f * (float)(tune->random % 8U)) /
                   tune->config.sampling_time) +
        1U;
  }
  tune->pulse_ticks--;
  float span = tune->config.position_max - tune->config.position_min;
  float request =
      (tune->random & 1U) != 0U ? tune->test_amplitude : -tune->test_amplitude;
  request += 0.4f * tune->config.current_limit *
             (tune->config.speed_offset - omega) / tune->config.speed_limit;
  if (tune->config.tune_position)
  {
    request +=
        1.5f * tune->config.current_limit * (tune->center - position) / span;
  }
  // 仅测试期间按运动余量限制激励。
  float margin = 0.1f * span;
  if (position < tune->config.position_min + margin && omega < 0.0f)
  {
    request = tune->config.current_limit;
  }
  else if (position > tune->config.position_max - margin && omega > 0.0f)
  {
    request = -tune->config.current_limit;
  }
  if (fabsf(omega) > 0.8f * tune->config.speed_limit)
  {
    request = -copysignf(tune->config.current_limit, omega);
  }
  return request;
}

/**
 * @brief 更新验证误差统计
 * @param tune 整定对象指针
 * @param error 归一化误差
 */
static void AutotuneMeasure(AutotuneObject *tune, float error)
{
  tune->sum_error += error * error;
  tune->peak_error = fmaxf(tune->peak_error, fabsf(error));
  tune->samples++;
  tune->result.rms_error = sqrtf(tune->sum_error / (float)tune->samples);
  tune->result.peak_error = tune->peak_error;
}

/**
 * @brief 计算有界且连续更新的加速度前馈
 * @param tune 整定对象指针
 * @param speed 速度控制器对象指针
 * @return 前馈电流（A）
 */
static float AutotuneFeedforward(AutotuneObject *tune, const PidObject *speed)
{
  float acceleration = (speed->reference - speed->previous_reference) /
                       tune->config.sampling_time;
  float requested =
      AutotuneClamp(tune->result.model.inertia * acceleration,
                    -0.3f * speed->output_max, 0.3f * speed->output_max);
  tune->feedforward += 0.1f * (requested - tune->feedforward);
  return tune->feedforward;
}

/**
 * @brief 更新整定阶段并生成电流请求
 * @param tune 整定对象指针
 * @param position 当前连续角度（rad）
 * @param omega 与控制器一致的角速度（rad/s）
 * @param actual_current 实际反馈电流（A）
 * @param rotor_omega 转子角速度（rad/s）
 * @return 测试电流（A）
 */
float AutotuneUpdate(AutotuneObject *tune, float position, float omega,
                     float actual_current, float rotor_omega)
{
  if (!AutotuneIsActive(tune))
  {
    return 0.0f;
  }
  if (!isfinite(position) || !isfinite(omega) || !isfinite(actual_current) ||
      !isfinite(rotor_omega))
  {
    return AutotuneFail(tune, kAutotuneInvalidSample);
  }
  if (position <= tune->config.position_min ||
      position >= tune->config.position_max ||
      fabsf(omega) > 1.2f * tune->config.speed_limit)
  {
    return AutotuneFail(tune, kAutotuneMotionLimit);
  }
  float dt = tune->config.sampling_time;
  if (tune->optimization_pending)
  {
    tune->optimization_pending = false;
    tune->optimization_finished =
        !PidTuneFinish(&tune->optimizer, &tune->speed_pid);
    tune->result.speed_kp = tune->speed_pid.kp;
    tune->result.speed_ki = tune->speed_pid.ki;
    AutotunePrime(tune, position, omega);
    AutotunePhase(tune, kAutotuneSpeed);
  }
  tune->elapsed += dt;
  tune->total_time += dt;
  float request = 0.0f;
  if (tune->state == kAutotuneIdentify)
  {
    (void)SysidUpdate(&tune->sysid, actual_current, omega);
    if (tune->has_dob)
    {
      (void)SysidUpdate(&tune->rotor_sysid, actual_current, rotor_omega);
    }
    request = AutotuneExcite(tune, position, omega);
    if (tune->elapsed >= 4.0f)
    {
      if (!SysidGetModel(&tune->sysid, &tune->result.model))
      {
        if (tune->elapsed < 12.0f)
        {
          tune->requested_current = request;
          return AutotuneClamp(request, -tune->config.current_limit,
                               tune->config.current_limit);
        }
        return AutotuneFail(tune, kAutotuneInsufficientModel);
      }
      if (tune->has_dob)
      {
        SysidModel rotor_model;
        if (!SysidGetModel(&tune->rotor_sysid, &rotor_model))
        {
          if (tune->elapsed < 12.0f)
          {
            tune->requested_current = request;
            return AutotuneClamp(request, -tune->config.current_limit,
                                 tune->config.current_limit);
          }
          return AutotuneFail(tune, kAutotuneInsufficientModel);
        }
        tune->rotor_inertia = rotor_model.inertia;
      }
      float bandwidth = tune->config.response_time > 0.0f
                            ? 4.0f / tune->config.response_time
                            : 30.0f;
      bandwidth = fminf(bandwidth, 0.08f / dt);
      bandwidth = fminf(bandwidth, 0.4f * tune->config.current_limit /
                                       (tune->result.model.inertia * 0.12f *
                                        tune->config.speed_limit));
      AutotuneDesign(tune, fmaxf(bandwidth, 0.5f));
      AutotunePrime(tune, position, omega);
      tune->test_amplitude = 0.12f * tune->config.speed_limit;
      if (tune->config.tune_position)
      {
        tune->center = position;
        float room = fminf(position - tune->config.position_min,
                           tune->config.position_max - position);
        tune->test_amplitude =
            fminf(tune->test_amplitude, 0.05f * room * tune->result.bandwidth);
      }
      AutotunePhase(tune, kAutotuneSpeed);
      request = 0.0f;
    }
  }
  else
  {
    bool position_test =
        tune->state == kAutotunePosition ||
        (tune->state == kAutotuneDob && tune->config.tune_position);
    float frequency = position_test ? 0.35f * tune->result.position_kp
                                    : 0.25f * tune->result.bandwidth;
    float duration = fmaxf(3.0f, 12.5663706f / fmaxf(frequency, 0.05f));
    if (tune->state == kAutotuneSpeed && tune->config.optimize &&
        tune->optimizer.state == kPidTuneIdle)
    {
      PidTuneConfig config = {.model = tune->result.model,
                              .sampling_time = dt,
                              .settling_time = 0.25f * duration,
                              .feedback_scale = tune->test_amplitude,
                              .output_weight = 0.01f,
                              .change_weight = 0.01f};
      if (!PidTuneInit(&tune->optimizer, &config) ||
          !PidTuneStart(&tune->optimizer, &tune->speed_pid))
      {
        return AutotuneFail(tune, kAutotuneInvalidSample);
      }
    }
    float phase = frequency * tune->elapsed;
    float reference =
        tune->config.speed_offset + tune->test_amplitude * sinf(phase);
    float error = reference - omega;
    if (position_test)
    {
      float room = fminf(tune->center - tune->config.position_min,
                         tune->config.position_max - tune->center);
      float amplitude =
          fminf(0.2f * room, tune->config.speed_limit /
                                 fmaxf(tune->result.position_kp, 0.01f));
      reference = tune->center + amplitude * sinf(phase);
      error = reference - position;
      reference = PidCalculate(&tune->position_pid, reference, position);
      if (tune->elapsed > duration * 0.25f)
      {
        AutotuneMeasure(tune, error / fmaxf(amplitude, 1.0e-5f));
      }
    }
    else if (tune->elapsed > duration * 0.25f)
    {
      AutotuneMeasure(tune, error / fmaxf(tune->test_amplitude, 1.0e-5f));
    }
    request = tune->state == kAutotuneSpeed && tune->config.optimize
                  ? PidTuneCalculate(&tune->optimizer, &tune->speed_pid,
                                     reference, omega)
                  : PidCalculate(&tune->speed_pid, reference, omega);
    if (tune->config.adaptive &&
        !(tune->state == kAutotuneSpeed && tune->config.optimize))
    {
      request += AutotuneFeedforward(tune, &tune->speed_pid);
    }
    if (tune->state == kAutotuneSpeed &&
        tune->elapsed >= duration - 6.2831853f / frequency)
    {
      tune->velocity_sine +=
          (omega - tune->config.speed_offset) * sinf(phase) * dt;
      tune->velocity_cosine +=
          (omega - tune->config.speed_offset) * cosf(phase) * dt;
    }
    if (tune->state == kAutotuneDob)
    {
      request += DobUpdate(&tune->dob, actual_current, rotor_omega);
    }
    if (tune->elapsed >= duration)
    {
      if (tune->samples == 0U || tune->result.rms_error > 0.55f ||
          tune->result.peak_error > 1.3f)
      {
        if (++tune->attempt > 3U)
        {
          return AutotuneFail(tune, kAutotuneTrackingError);
        }
        if (tune->state == kAutotuneDob)
        {
          DobConfig config = tune->dob.config;
          config.filter_cutoff_hz *= 0.5f;
          (void)DobInit(&tune->dob, &config);
          tune->result.dob_cutoff_hz = config.filter_cutoff_hz;
          AutotunePhase(tune, kAutotuneDob);
        }
        else if (tune->state == kAutotunePosition)
        {
          tune->result.position_kp *= 0.6f;
          tune->position_pid.kp = tune->result.position_kp;
          AutotunePhase(tune, kAutotunePosition);
        }
        else
        {
          memset(&tune->optimizer, 0, sizeof(tune->optimizer));
          tune->optimization_finished = false;
          AutotuneDesign(tune, 0.6f * tune->result.bandwidth);
          AutotunePrime(tune, position, omega);
          if (tune->config.tune_position)
          {
            tune->test_amplitude *= 0.6f;
          }
          AutotunePhase(tune, kAutotuneSpeed);
        }
      }
      else if (tune->state == kAutotuneSpeed && tune->config.optimize &&
               !tune->optimization_finished &&
               (tune->optimizer.state == kPidTuneRunning ||
                tune->optimizer.state == kPidTuneInvalidSample))
      {
        tune->optimization_pending = true;
      }
      else if (tune->state == kAutotuneSpeed && tune->config.tune_position)
      {
        float lag = atan2f(-tune->velocity_cosine, tune->velocity_sine);
        float time_constant = tanf(AutotuneClamp(lag, 0.0f, 1.2f)) / frequency;
        tune->result.position_kp =
            fminf(tune->result.position_kp, 0.1f / fmaxf(time_constant, dt));
        tune->position_pid.kp = tune->result.position_kp;
        tune->attempt = 0U;
        AutotunePhase(tune, kAutotunePosition);
      }
      else if (tune->state != kAutotuneDob && tune->has_dob)
      {
        DobConfig config = tune->dob.config;
        config.J_motor = tune->rotor_inertia * config.torque_constant;
        config.J_load = 0.0f;
        config.filter_cutoff_hz =
            fminf(tune->result.bandwidth / 12.5663706f, 0.05f / dt);
        config.max_compensation =
            fminf(config.max_compensation, 0.5f * tune->config.current_limit);
        (void)DobInit(&tune->dob, &config);
        tune->result.dob_cutoff_hz = config.filter_cutoff_hz;
        tune->attempt = 0U;
        AutotunePhase(tune, kAutotuneDob);
      }
      else
      {
        AutotunePhase(tune, kAutotuneComplete);
        request = 0.0f;
      }
    }
  }
  tune->requested_current = request;
  return AutotuneClamp(request, -tune->config.current_limit,
                       tune->config.current_limit);
}

/**
 * @brief 将最终限幅及功率分配结果反馈给验证积分状态
 * @param tune 整定对象指针
 * @param executed_current 实际发送的电流指令（A）
 */
void AutotuneTrackOutput(AutotuneObject *tune, float executed_current)
{
  if (AutotuneIsActive(tune) && tune->state != kAutotuneIdentify)
  {
    if (tune->state == kAutotuneSpeed && tune->config.optimize)
    {
      PidTuneTrackOutput(&tune->optimizer, executed_current);
    }
    PidTrackOutput(&tune->speed_pid, executed_current - tune->requested_current,
                   tune->result.bandwidth);
  }
}

/**
 * @brief 将通过验证的参数及控制状态应用到运行对象
 * @param tune 整定对象指针
 * @param speed 速度控制器对象指针
 * @param position 位置控制器对象指针，速度模式传NULL
 * @param dob 扰动观测器对象指针，无补偿时传NULL
 * @return 是否成功应用
 */
bool AutotuneApply(AutotuneObject *tune, PidObject *speed, PidObject *position,
                   DobObject *dob)
{
  if (tune == NULL || speed == NULL || tune->state != kAutotuneComplete ||
      tune->applied || (tune->config.tune_position && position == NULL))
  {
    return false;
  }
  *speed = tune->speed_pid;
  if (tune->config.tune_position)
  {
    *position = tune->position_pid;
  }
  if (tune->has_dob && dob != NULL)
  {
    *dob = tune->dob;
  }
  tune->applied = true;
  tune->validated_model = tune->result.model;
  AutotuneGainScale(tune, speed);
  AutotuneIdentifyInit(tune, 20.0f);
  return true;
}

/**
 * @brief 从已保存配置恢复模型及自适应状态
 * @param tune 整定对象指针
 * @param model 已验证的电流坐标模型
 * @param bandwidth 已验证的设计自然频率（rad/s）
 * @param speed 已初始化的速度控制器
 * @param position 已初始化的位置控制器，速度模式传NULL
 * @param dob 已初始化的扰动观测器，无补偿时传NULL
 * @return 是否成功
 */
bool AutotuneRestore(AutotuneObject *tune, const SysidModel *model,
                     float bandwidth, const PidObject *speed,
                     const PidObject *position, const DobObject *dob)
{
  if (tune == NULL || model == NULL || speed == NULL ||
      !isfinite(model->inertia) || model->inertia <= 0.0f ||
      !isfinite(model->viscous) || model->viscous < 0.0f ||
      !isfinite(model->friction) || !isfinite(model->bias) ||
      !isfinite(model->fitness) || model->fitness < 0.75f ||
      !isfinite(bandwidth) || bandwidth <= 0.0f || speed->kp <= 0.0f ||
      speed->ki <= 0.0f || (tune->config.tune_position && position == NULL))
  {
    return false;
  }
  tune->result.model = *model;
  tune->validated_model = *model;
  tune->result.bandwidth = bandwidth;
  tune->result.speed_kp = speed->kp;
  tune->result.speed_ki = speed->ki;
  tune->speed_pid = *speed;
  if (position != NULL)
  {
    tune->position_pid = *position;
    tune->result.position_kp = position->kp;
  }
  tune->has_dob = dob != NULL && dob->is_enabled;
  if (tune->has_dob)
  {
    tune->dob = *dob;
    tune->result.dob_cutoff_hz = dob->config.filter_cutoff_hz;
  }
  tune->state = kAutotuneComplete;
  tune->applied = true;
  AutotuneGainScale(tune, speed);
  AutotuneIdentifyInit(tune, 20.0f);
  return true;
}

/**
 * @brief 在有效运动数据支持下缓慢更新模型和速度PI
 * @param tune 整定对象指针
 * @param speed 运行速度控制器对象指针
 * @param omega 当前角速度（rad/s）
 * @param actual_current 实际反馈电流（A）
 * @return 增量加速度前馈电流（A）
 */
float AutotuneAdapt(AutotuneObject *tune, PidObject *speed, float omega,
                    float actual_current)
{
  if (tune == NULL || speed == NULL || !tune->applied ||
      !tune->config.adaptive || !isfinite(omega) || !isfinite(actual_current))
  {
    return 0.0f;
  }
  (void)SysidUpdate(&tune->sysid, actual_current, omega);
  tune->adaptive_time += tune->config.sampling_time;
  if (tune->adaptive_time >= 1.0f)
  {
    tune->adaptive_time = 0.0f;
    SysidModel model;
    if (tune->sysid.count - tune->adaptive_count >= 3U &&
        SysidGetModel(&tune->sysid, &model) &&
        model.inertia > 0.5f * tune->validated_model.inertia &&
        model.inertia < 2.0f * tune->validated_model.inertia)
    {
      float kp =
          fmaxf(2.0f * tune->result.bandwidth * model.inertia - model.viscous,
                0.1f * tune->result.bandwidth * model.inertia);
      float ki =
          model.inertia * tune->result.bandwidth * tune->result.bandwidth;
      if (tune->config.optimize)
      {
        kp *= tune->gain_scale[0];
        ki *= tune->gain_scale[1];
      }
      kp = AutotuneClamp(kp, 0.98f * speed->kp, 1.02f * speed->kp);
      ki = AutotuneClamp(ki, 0.98f * speed->ki, 1.02f * speed->ki);
      (void)PidSetGains(speed, kp, ki, 0.0f);
      speed->integral_max = speed->output_max / ki;
      tune->result.speed_kp = kp;
      tune->result.speed_ki = ki;
      tune->result.model = model;
      tune->speed_pid = *speed;
      tune->result.revision++;
    }
    tune->adaptive_count = tune->sysid.count;
  }
  // DOB已覆盖摩擦和恒定负载，前馈仅补偿名义加速需求。
  return AutotuneFeedforward(tune, speed);
}
