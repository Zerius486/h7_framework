#include "alg_power.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "alg_conversion.h"

/**
 * @brief 使用适合初版自研超电的保守默认值初始化预算配置
 * @param config 配置结构体指针
 */
void PowerBudgetConfigInit(PowerBudgetConfig *config)
{
  if (config == NULL)
  {
    return;
  }
  config->mode = kPowerModeRefereeOnly;
  config->referee_buffer_target = 40.0f;
  config->referee_buffer_danger = 15.0f;
  config->referee_buffer_kp = 2.0f;
  config->min_power_ratio = 0.70f;
  config->supercap_energy_target = 50.0f;
  config->supercap_energy_min = 15.0f;
  config->supercap_kp = 2.0f;
  config->boost_margin_watts = 60.0f;
  config->supercap_timeout_ms = 200U;
}

/**
 * @brief 初始化能量预算控制器
 * @param ctrl 控制器指针
 * @param config 配置指针
 * @return true 表示成功
 */
bool PowerBudgetInit(PowerBudgetController *ctrl,
                     const PowerBudgetConfig *config)
{
  if (ctrl == NULL || config == NULL ||
      !isfinite(config->referee_buffer_target) ||
      !isfinite(config->referee_buffer_danger) ||
      !isfinite(config->referee_buffer_kp) ||
      !isfinite(config->min_power_ratio) ||
      !isfinite(config->supercap_energy_target) ||
      !isfinite(config->supercap_energy_min) ||
      !isfinite(config->supercap_kp) ||
      !isfinite(config->boost_margin_watts) ||
      config->referee_buffer_target < config->referee_buffer_danger ||
      config->referee_buffer_kp < 0.0f || config->min_power_ratio <= 0.0f ||
      config->min_power_ratio > 1.0f || config->supercap_kp < 0.0f ||
      config->boost_margin_watts < 0.0f)
  {
    return false;
  }

  memset(ctrl, 0, sizeof(*ctrl));
  ctrl->config = *config;
  ctrl->initialized = true;
  return true;
}

/**
 * @brief 复位能量预算控制器状态
 * @param ctrl 控制器指针
 */
void PowerBudgetReset(PowerBudgetController *ctrl)
{
  if (ctrl == NULL)
  {
    return;
  }
  ctrl->power_budget = 0.0f;
  ctrl->last_cap_tick = 0U;
  ctrl->cap_recovery_tick = 0U;
  ctrl->has_cap_sample = false;
  ctrl->cap_recovering = false;
  ctrl->supercap_active = false;
  ctrl->supercap_fault_latched = false;
}

/**
 * @brief 根据裁判系统上限、缓冲能量与超电状态计算本拍功率预算
 * @param ctrl 控制器指针
 * @param referee_power_limit 裁判系统底盘功率上限（W）
 * @param referee_buffer_energy 裁判系统当前缓冲能量（J）
 * @param cap_online 超电通信是否在线
 * @param cap_energy_pct 超电剩余能量百分比（0~100）
 * @param cap_fault 超电自身是否上报故障/保护
 * @param now_ms 当前系统节拍（ms）
 * @return 本拍允许的底盘总电功率预算（W）
 */
float PowerBudgetUpdate(PowerBudgetController *ctrl, float referee_power_limit,
                        float referee_buffer_energy, bool cap_online,
                        float cap_energy_pct, bool cap_fault, uint32_t now_ms)
{
  if (ctrl == NULL || !ctrl->initialized || !isfinite(referee_power_limit) ||
      referee_power_limit <= 0.0f)
  {
    return 0.0f;
  }

  const PowerBudgetConfig *c = &ctrl->config;
  float safe_buffer = isfinite(referee_buffer_energy)
                          ? MathClamp(referee_buffer_energy, 0.0f, 250.0f)
                          : 0.0f;
  float min_power = referee_power_limit * c->min_power_ratio;

  // 1. 裁判系统缓冲能量闭环预算（无论超电是否启用，始终作为连续单调的硬兜底）
  float buffer_budget =
      referee_power_limit +
      c->referee_buffer_kp * (safe_buffer - c->referee_buffer_target);
  float max_referee_budget =
      referee_power_limit + fmaxf(0.0f, safe_buffer - c->referee_buffer_danger);
  buffer_budget = MathClamp(buffer_budget, min_power, max_referee_budget);
  if (safe_buffer < c->referee_buffer_danger)
  {
    float danger_at_edge = fminf(
        MathClamp(referee_power_limit +
                      c->referee_buffer_kp *
                          (c->referee_buffer_danger - c->referee_buffer_target),
                  min_power, referee_power_limit),
        referee_power_limit * 0.85f);
    float danger_ratio =
        MathClamp(safe_buffer / fmaxf(c->referee_buffer_danger, 1.0f), 0.0f,
                  1.0f);
    float danger_cap = min_power + (danger_at_edge - min_power) * danger_ratio;
    buffer_budget = fminf(buffer_budget, danger_cap);
  }

  // 2. 自研超电健康度检查与故障/超时消抖锁存：异常退回纯裁判模式，连续恢复后自动解除
  bool cap_data_valid = cap_online && !cap_fault && isfinite(cap_energy_pct) &&
                        cap_energy_pct >= 0.0f && cap_energy_pct <= 105.0f;
  if (cap_data_valid)
  {
    ctrl->last_cap_tick = now_ms;
    ctrl->has_cap_sample = true;
    if (ctrl->supercap_fault_latched)
    {
      if (!ctrl->cap_recovering)
      {
        ctrl->cap_recovering = true;
        ctrl->cap_recovery_tick = now_ms;
      }
      else if ((int32_t)(now_ms - ctrl->cap_recovery_tick) >=
               (int32_t)c->supercap_timeout_ms)
      {
        ctrl->supercap_fault_latched = false;
        ctrl->cap_recovering = false;
      }
    }
  }
  else
  {
    ctrl->cap_recovering = false;
    if (cap_fault ||
        (ctrl->has_cap_sample &&
         (int32_t)(now_ms - ctrl->last_cap_tick) >
             (int32_t)c->supercap_timeout_ms))
    {
      ctrl->supercap_fault_latched = true;
    }
  }

  if (c->mode == kPowerModeSupercapAssist && !ctrl->supercap_fault_latched &&
      cap_data_valid && cap_energy_pct > c->supercap_energy_min &&
      safe_buffer > c->referee_buffer_danger)
  {
    // 叠加式连续超发：在 cap_energy_min 与 buffer_danger 边界均连续退回 buffer_budget，避免掉电反跳
    float cap_extra =
        MathClamp(c->supercap_kp * (cap_energy_pct - c->supercap_energy_min),
                  0.0f, c->boost_margin_watts);
    float buf_span = fmaxf(c->referee_buffer_target - c->referee_buffer_danger,
                           1.0f);
    float buf_gate =
        MathClamp((safe_buffer - c->referee_buffer_danger) / buf_span, 0.0f,
                  1.0f);
    float cap_budget =
        MathClamp(buffer_budget + buf_gate * cap_extra, min_power,
                  referee_power_limit + c->boost_margin_watts);
    ctrl->supercap_active = (buf_gate * cap_extra > 1.0e-3f);
    ctrl->power_budget = cap_budget;
  }
  else
  {
    ctrl->supercap_active = false;
    ctrl->power_budget = buffer_budget;
  }

  return ctrl->power_budget;
}

/**
 * @brief 计算单电机消耗电功率（非负截断，不将未验证的再生发电计入预算）
 * @param model 电机功率模型指针
 * @param current 电机电流（A）
 * @param omega 电机角速度（rad/s）
 * @return 消耗电功率（W）
 */
float PowerEstimateMotor(const PowerMotorModel *model, float current,
                         float omega)
{
  if (model == NULL || !isfinite(current) || !isfinite(omega))
  {
    return 0.0f;
  }
  float p = model->copper_loss * current * current +
            model->torque_constant * omega * current +
            model->iron_loss * omega * omega + model->idle_loss;
  return fmaxf(p, 0.0f);
}

/**
 * @brief 估算多电机底盘总消耗电功率
 * @param models 电机模型数组
 * @param currents 电流数组（A）
 * @param omegas 角速度数组（rad/s）
 * @param count 电机数量
 * @return 总消耗电功率（W）
 */
float PowerEstimateTotal(const PowerMotorModel *models, const float *currents,
                         const float *omegas, uint8_t count)
{
  if (models == NULL || currents == NULL || omegas == NULL ||
      count > POWER_MAX_MOTORS)
  {
    return 0.0f;
  }
  float total = 0.0f;
  for (uint8_t i = 0U; i < count; ++i)
  {
    total += PowerEstimateMotor(&models[i], currents[i], omegas[i]);
  }
  return total;
}

static float PowerSolveScaleQuadratic(float a, float b, float c)
{
  if (a > 1.0e-9f)
  {
    float disc = b * b - 4.0f * a * c;
    if (!(disc >= 0.0f))
    {
      return 0.0f;
    }
    float scale = (-b + sqrtf(disc)) / (2.0f * a);
    return MathClamp(scale, 0.0f, 1.0f);
  }
  if (b > 1.0e-9f)
  {
    return MathClamp(-c / b, 0.0f, 1.0f);
  }
  return (c <= 0.0f) ? 1.0f : 0.0f;
}

static float PowerSolveMotorCurrentForBudget(const PowerMotorModel *model,
                                             float desired_current, float omega,
                                             float allocated_power)
{
  if (PowerEstimateMotor(model, desired_current, omega) <= allocated_power)
  {
    return desired_current;
  }
  if (fabsf(desired_current) <= 1.0e-6f)
  {
    return 0.0f;
  }
  float a = model->copper_loss;
  float b = model->torque_constant * omega;
  float p_passive = model->iron_loss * omega * omega + model->idle_loss;
  float c = p_passive - allocated_power;

  if (a > 1.0e-9f)
  {
    float i_opt = -b / (2.0f * a);
    float disc = b * b - 4.0f * a * c;
    if (b * desired_current < 0.0f)
    {
      // 制动/发电象限：仅当 |desired_current| > |i_opt| 时向顶点 i_opt 收缩才能降低功耗
      if (fabsf(desired_current) <= fabsf(i_opt))
      {
        return desired_current;
      }
      if (!(disc >= 0.0f))
      {
        return i_opt;
      }
      float sqrt_disc = sqrtf(disc);
      if (desired_current >= 0.0f)
      {
        float root = (-b + sqrt_disc) / (2.0f * a);
        return MathClamp(root, i_opt, desired_current);
      }
      else
      {
        float root = (-b - sqrt_disc) / (2.0f * a);
        return MathClamp(root, desired_current, i_opt);
      }
    }

    if (allocated_power <= p_passive || !(disc >= 0.0f))
    {
      return 0.0f;
    }
    float sqrt_disc = sqrtf(disc);
    if (desired_current >= 0.0f)
    {
      float root = (-b + sqrt_disc) / (2.0f * a);
      return MathClamp(root, 0.0f, desired_current);
    }
    else
    {
      float root = (-b - sqrt_disc) / (2.0f * a);
      return MathClamp(root, desired_current, 0.0f);
    }
  }

  // 退化为线性情况
  if (b * desired_current < 0.0f)
  {
    return desired_current;
  }
  float rem_p = allocated_power - p_passive;
  if (rem_p <= 0.0f || fabsf(b) <= 1.0e-9f)
  {
    return 0.0f;
  }
  float max_i = rem_p / b;
  if (desired_current >= 0.0f)
  {
    return MathClamp(max_i, 0.0f, desired_current);
  }
  return MathClamp(max_i, desired_current, 0.0f);
}

/**
 * @brief 按功率预算执行解析等比例缩放或 KKT 闭式加权分配
 * @param models 电机功率模型数组
 * @param desired_currents 期望电流数组（A）
 * @param omegas 当前角速度数组（rad/s）
 * @param weights KKT 权重数组（仅 kPowerAllocKktWeighted 使用，传 NULL 默认全 1）
 * @param count 电机数量
 * @param budget 当前允许功率预算（W）
 * @param strategy 分配策略
 * @param allocated_currents 输出分配后电流数组（A）
 * @return 分配后预估总功率（W）
 */
float PowerAllocate(const PowerMotorModel *models, const float *desired_currents,
                    const float *omegas, const float *weights, uint8_t count,
                    float budget, PowerAllocStrategy strategy,
                    float *allocated_currents)
{
  if (models == NULL || desired_currents == NULL || omegas == NULL ||
      allocated_currents == NULL || count == 0U || count > POWER_MAX_MOTORS)
  {
    return 0.0f;
  }
  if (!isfinite(budget) || budget <= 0.0f)
  {
    memset(allocated_currents, 0, count * sizeof(float));
    return 0.0f;
  }

  float motor_p[POWER_MAX_MOTORS];
  float p_passive[POWER_MAX_MOTORS];
  float p_min[POWER_MAX_MOTORS];
  bool is_braking[POWER_MAX_MOTORS];
  float total_p = 0.0f;
  for (uint8_t i = 0U; i < count; ++i)
  {
    if (!isfinite(desired_currents[i]) || !isfinite(omegas[i]))
    {
      memset(allocated_currents, 0, count * sizeof(float));
      return 0.0f;
    }
    motor_p[i] = PowerEstimateMotor(&models[i], desired_currents[i], omegas[i]);
    p_passive[i] = fmaxf(
        models[i].iron_loss * omegas[i] * omegas[i] + models[i].idle_loss, 0.0f);
    float b = models[i].torque_constant * omegas[i];
    is_braking[i] = (b * desired_currents[i] < 0.0f);
    if (is_braking[i])
    {
      if (models[i].copper_loss > 1.0e-9f)
      {
        float i_opt = -b / (2.0f * models[i].copper_loss);
        p_min[i] = (fabsf(desired_currents[i]) <= fabsf(i_opt))
                       ? motor_p[i]
                       : PowerEstimateMotor(&models[i], i_opt, omegas[i]);
      }
      else
      {
        p_min[i] = motor_p[i];
      }
    }
    else
    {
      p_min[i] = fminf(motor_p[i], p_passive[i]);
    }
    total_p += motor_p[i];
    allocated_currents[i] = desired_currents[i];
  }

  if (total_p <= budget)
  {
    return total_p;
  }

  if (strategy == kPowerAllocKktWeighted)
  {
    // Active-Set KKT 水漫求解：保护高权重轴且不跨越电机不可削减功耗下限 p_min
    bool active[POWER_MAX_MOTORS];
    float p_alloc[POWER_MAX_MOTORS];
    for (uint8_t i = 0U; i < count; ++i)
    {
      active[i] = (motor_p[i] > p_min[i] + 1.0e-5f);
      p_alloc[i] = motor_p[i];
    }

    for (uint8_t iter = 0U; iter < count; ++iter)
    {
      float fixed_p = 0.0f;
      float active_p = 0.0f;
      float sum_inv_w = 0.0f;
      for (uint8_t i = 0U; i < count; ++i)
      {
        if (active[i])
        {
          float w =
              (weights != NULL && isfinite(weights[i]) && weights[i] > 1.0e-6f)
                  ? weights[i]
                  : 1.0f;
          active_p += motor_p[i];
          sum_inv_w += 1.0f / w;
        }
        else
        {
          fixed_p += p_alloc[i];
        }
      }

      if (sum_inv_w <= 0.0f)
      {
        break;
      }
      if (fixed_p >= budget)
      {
        for (uint8_t i = 0U; i < count; ++i)
        {
          if (active[i])
          {
            p_alloc[i] = p_min[i];
          }
        }
        break;
      }

      float lambda = (active_p + fixed_p - budget) / sum_inv_w;
      bool clipped = false;
      for (uint8_t i = 0U; i < count; ++i)
      {
        if (!active[i])
        {
          continue;
        }
        float w =
            (weights != NULL && isfinite(weights[i]) && weights[i] > 1.0e-6f)
                ? weights[i]
                : 1.0f;
        float trial = motor_p[i] - lambda / w;
        if (trial <= p_min[i] + 1.0e-6f)
        {
          p_alloc[i] = p_min[i];
          active[i] = false;
          clipped = true;
        }
      }
      if (!clipped)
      {
        for (uint8_t i = 0U; i < count; ++i)
        {
          if (active[i])
          {
            float w = (weights != NULL && isfinite(weights[i]) &&
                       weights[i] > 1.0e-6f)
                          ? weights[i]
                          : 1.0f;
            p_alloc[i] = motor_p[i] - lambda / w;
          }
        }
        break;
      }
    }

    for (uint8_t i = 0U; i < count; ++i)
    {
      allocated_currents[i] = PowerSolveMotorCurrentForBudget(
          &models[i], desired_currents[i], omegas[i], p_alloc[i]);
    }
    return PowerEstimateTotal(models, allocated_currents, omegas, count);
  }

  // 解析等比例缩减：仅缩放正做功驱动轮，防止制动/再生发电项在负功率区反向补贴导致总功率越界
  float fixed_p = 0.0f;
  float a = 0.0f;
  float b = 0.0f;
  float passive_scalable_sum = 0.0f;
  bool scalable[POWER_MAX_MOTORS];
  for (uint8_t i = 0U; i < count; ++i)
  {
    scalable[i] = (!is_braking[i] && motor_p[i] > p_passive[i] + 1.0e-5f);
    if (scalable[i])
    {
      a += models[i].copper_loss * desired_currents[i] * desired_currents[i];
      b += models[i].torque_constant * omegas[i] * desired_currents[i];
      passive_scalable_sum +=
          models[i].iron_loss * omegas[i] * omegas[i] + models[i].idle_loss;
    }
    else
    {
      fixed_p += motor_p[i];
    }
  }
  float scale = PowerSolveScaleQuadratic(
      a, b, fixed_p + passive_scalable_sum - budget);
  for (uint8_t i = 0U; i < count; ++i)
  {
    if (scalable[i])
    {
      allocated_currents[i] = desired_currents[i] * scale;
    }
  }

  float after_p = PowerEstimateTotal(models, allocated_currents, omegas, count);
  if (after_p > budget + 1.0e-3f)
  {
    // 若重度反接制动电机超出最优再生点导致残余超标，再对其按余量削减
    float reducible_brake_sum = 0.0f;
    for (uint8_t i = 0U; i < count; ++i)
    {
      if (is_braking[i] && motor_p[i] > p_min[i] + 1.0e-5f)
      {
        reducible_brake_sum += (motor_p[i] - p_min[i]);
      }
    }
    if (reducible_brake_sum > 1.0e-6f)
    {
      float excess = after_p - budget;
      float cut_ratio = MathClamp(excess / reducible_brake_sum, 0.0f, 1.0f);
      for (uint8_t i = 0U; i < count; ++i)
      {
        if (is_braking[i] && motor_p[i] > p_min[i] + 1.0e-5f)
        {
          float target_p =
              motor_p[i] - cut_ratio * (motor_p[i] - p_min[i]);
          allocated_currents[i] = PowerSolveMotorCurrentForBudget(
              &models[i], desired_currents[i], omegas[i], target_p);
        }
      }
      after_p = PowerEstimateTotal(models, allocated_currents, omegas, count);
    }
  }
  return after_p;
}

static const float kPowerSysidScale[4] = {1.0f, 100.0f, 1000.0f, 10000.0f};

static void PowerSysidBoundCovariance(float cov[4][4])
{
  float scale[4];
  for (int i = 0; i < 4; ++i)
  {
    if (cov[i][i] < 1.0e-6f)
    {
      cov[i][i] = 1.0e-6f;
    }
    for (int j = i + 1; j < 4; ++j)
    {
      float sym = 0.5f * (cov[i][j] + cov[j][i]);
      cov[i][j] = sym;
      cov[j][i] = sym;
    }
    scale[i] = (cov[i][i] > 100.0f) ? sqrtf(100.0f / cov[i][i]) : 1.0f;
  }
  for (int i = 0; i < 4; ++i)
  {
    for (int j = 0; j < 4; ++j)
    {
      cov[i][j] *= scale[i] * scale[j];
    }
  }
}

/**
 * @brief 初始化在线4参数功率模型RLS辨识器
 * @param sysid 辨识器指针
 * @param initial_sum 全底盘求和后的初始模型参数（例如4倍单电机idle_loss）
 * @param forgetting 遗忘因子（建议 0.990~0.999）
 * @return true 表示成功
 */
bool PowerSysidInit(PowerModelSysid *sysid, const PowerMotorModel *initial_sum,
                    float forgetting)
{
  if (sysid == NULL || initial_sum == NULL || !isfinite(forgetting) ||
      forgetting <= 0.5f || forgetting > 1.0f)
  {
    return false;
  }
  memset(sysid, 0, sizeof(*sysid));
  sysid->theta[0] = initial_sum->idle_loss;
  sysid->theta[1] = initial_sum->copper_loss;
  sysid->theta[2] = initial_sum->torque_constant;
  sysid->theta[3] = initial_sum->iron_loss;
  sysid->forgetting = forgetting;

  sysid->min_theta[0] = 0.0f;
  sysid->max_theta[0] = 30.0f;
  sysid->min_theta[1] = 0.0f;
  sysid->max_theta[1] = 10.0f;
  sysid->min_theta[2] = 0.0f;
  sysid->max_theta[2] = 5.0f;
  sysid->min_theta[3] = 0.0f;
  sysid->max_theta[3] = 0.1f;

  // 在尺度归一化坐标下初始化协方差阵，保持 float32 条件数 O(1)
  sysid->covariance[0][0] = 4.0f;
  sysid->covariance[1][1] = 10.0f;
  sysid->covariance[2][2] = 10.0f;
  sysid->covariance[3][3] = 4.0f;
  sysid->initialized = true;
  return true;
}

/**
 * @brief 利用超电板采样得到的底盘总功率在线更新功率模型参数
 * @param sysid 辨识器指针
 * @param currents 各电机实测电流（A）
 * @param omegas 各电机实测角速度（rad/s）
 * @param count 电机数量
 * @param measured_power 实测总电功率（W）
 * @return true 表示本拍完成参数更新
 */
bool PowerSysidUpdate(PowerModelSysid *sysid, const float *currents,
                      const float *omegas, uint8_t count, float measured_power)
{
  if (sysid == NULL || !sysid->initialized || currents == NULL ||
      omegas == NULL || count == 0U || count > POWER_MAX_MOTORS ||
      !isfinite(measured_power) || measured_power < 0.0f)
  {
    return false;
  }

  float x_raw[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  for (uint8_t i = 0U; i < count; ++i)
  {
    if (!isfinite(currents[i]) || !isfinite(omegas[i]))
    {
      return false;
    }
    x_raw[1] += currents[i] * currents[i];
    x_raw[2] += omegas[i] * currents[i];
    x_raw[3] += omegas[i] * omegas[i];
  }

  float xs[4];
  float theta_s[4];
  for (int i = 0; i < 4; ++i)
  {
    xs[i] = x_raw[i] / kPowerSysidScale[i];
    theta_s[i] = sysid->theta[i] * kPowerSysidScale[i];
  }

  // 静止/零激励死区：冻结遗忘更新，防止无激励时协方差指数膨胀
  float dynamic_energy = xs[1] * xs[1] + xs[2] * xs[2] + xs[3] * xs[3];
  if (dynamic_energy < 1.0e-5f)
  {
    return false;
  }

  float pred = 0.0f;
  float px[4] = {0};
  for (int i = 0; i < 4; ++i)
  {
    pred += theta_s[i] * xs[i];
    for (int j = 0; j < 4; ++j)
    {
      px[i] += sysid->covariance[i][j] * xs[j];
    }
  }

  float denom = sysid->forgetting;
  for (int i = 0; i < 4; ++i)
  {
    denom += xs[i] * px[i];
  }
  if (!isfinite(denom) || denom <= 1.0e-6f)
  {
    return false;
  }

  float err = measured_power - pred;
  for (int i = 0; i < 4; ++i)
  {
    float updated_physical = (theta_s[i] + px[i] * err / denom) /
                             kPowerSysidScale[i];
    sysid->theta[i] = MathClamp(updated_physical, sysid->min_theta[i],
                                sysid->max_theta[i]);
    for (int j = i; j < 4; ++j)
    {
      float v = (sysid->covariance[i][j] - px[i] * px[j] / denom) /
                sysid->forgetting;
      sysid->covariance[i][j] = v;
      sysid->covariance[j][i] = v;
    }
  }
  PowerSysidBoundCovariance(sysid->covariance);
  sysid->sample_count++;
  return true;
}
