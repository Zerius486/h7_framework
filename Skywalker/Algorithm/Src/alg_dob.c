#include "alg_dob.h"

#include <math.h>
#include <stddef.h>
#include <string.h>
/**
 * @brief 初始化扰动观测器
 * @param dob 扰动观测器对象指针
 * @param config 扰动观测器配置结构体指针
 * @return true表示成功，false表示参数无效
 */
bool DobInit(DobObject *dob, const DobConfig *config)
{
  if (dob == NULL || config == NULL || !isfinite(config->J_motor) ||
      !isfinite(config->J_load) || !isfinite(config->reduction_ratio) ||
      !isfinite(config->torque_constant) || !isfinite(config->sampling_time) ||
      !isfinite(config->filter_cutoff_hz) ||
      !isfinite(config->max_compensation) || config->J_motor < 0.0f ||
      config->J_load < 0.0f || config->reduction_ratio < 1.0f ||
      config->torque_constant <= 0.0f || config->sampling_time <= 0.0f ||
      config->filter_cutoff_hz <= 0.0f ||
      config->filter_cutoff_hz >= 0.5f / config->sampling_time ||
      config->max_compensation <= 0.0f)
  {
    return false;
  }

  float equivalent_inertia =
      config->J_motor +
      config->J_load / (config->reduction_ratio * config->reduction_ratio);
  if (!isfinite(equivalent_inertia) || equivalent_inertia <= 0.0f)
  {
    return false;
  }

  memset(dob, 0, sizeof(*dob));
  dob->config = *config;
  dob->J_equivalent = equivalent_inertia;
  dob->filter_decay = expf(-2.0f * 3.14159265358979323846f *
                           config->filter_cutoff_hz * config->sampling_time);
  dob->is_enabled = true;
  return true;
}
/**
 * @brief 重置扰动观测器状态
 * @param dob 扰动观测器对象指针
 */
void DobReset(DobObject *dob)
{
  if (dob == NULL)
  {
    return;
  }
  dob->filtered_current = 0.0f;
  dob->filtered_omega = 0.0f;
  dob->estimated_disturbance = 0.0f;
  dob->compensation_current = 0.0f;
  dob->has_sample = false;
}
/**
 * @brief 更新扰动观测器
 * @param dob 扰动观测器对象指针
 * @param actual_torque_current 实际转矩电流（A）
 * @param rotor_omega 电机转子角速度（rad/s）
 * @return 补偿电流（A）
 */
float DobUpdate(DobObject *dob, float actual_torque_current, float rotor_omega)
{
  if (dob == NULL || !dob->is_enabled || !isfinite(actual_torque_current) ||
      !isfinite(rotor_omega))
  {
    return 0.0f;
  }

  if (!dob->has_sample)
  {
    dob->filtered_omega = rotor_omega;
    dob->has_sample = true;
    return 0.0f;
  }

  const float alpha = dob->filter_decay;
  // 对实际电流和转子角速度进行低通滤波。
  dob->filtered_current =
      alpha * dob->filtered_current + (1.0f - alpha) * actual_torque_current;
  dob->filtered_omega =
      alpha * dob->filtered_omega + (1.0f - alpha) * rotor_omega;

  // Q(s)[Kt*i - J*s*omega]，其中Q(s)=wc/(s+wc)。
  const float cutoff_rad_s =
      2.0f * 3.14159265358979323846f * dob->config.filter_cutoff_hz;
  const float filtered_acceleration =
      cutoff_rad_s * (rotor_omega - dob->filtered_omega);
  dob->estimated_disturbance =
      dob->config.torque_constant * dob->filtered_current -
      dob->J_equivalent * filtered_acceleration;

  float compensation = dob->estimated_disturbance / dob->config.torque_constant;
  if (compensation > dob->config.max_compensation)
  {
    compensation = dob->config.max_compensation;
  }
  else if (compensation < -dob->config.max_compensation)
  {
    compensation = -dob->config.max_compensation;
  }
  dob->compensation_current = compensation;
  return compensation;
}
