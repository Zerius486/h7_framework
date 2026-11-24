#include "alg_feedforward.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "alg_conversion.h"

static const float kFeedforwardDefaultCoulombSmoothOmega = 0.1f;
static const float kFeedforwardTwoPi = 6.28318530717958647692f;

static float FeedforwardSanitize(float value)
{
  return isfinite(value) ? value : 0.0f;
}

static float FeedforwardSymmetricClamp(float value, float limit)
{
  if (!isfinite(value))
  {
    return 0.0f;
  }
  if (isfinite(limit) && limit > 0.0f)
  {
    return MathClamp(value, -limit, limit);
  }
  return value;
}

/**
 * @brief 初始化云台双级前馈控制器
 * @param ff 前馈对象指针
 * @param config 前馈配置指针，传 NULL 时保持全零关闭态
 */
void GimbalFeedforwardInit(GimbalFeedforward *ff,
                           const GimbalFeedforwardConfig *config)
{
  if (ff == NULL)
  {
    return;
  }

  memset(ff, 0, sizeof(*ff));
  if (config != NULL)
  {
    ff->config = *config;
    ff->config.kv_vel = FeedforwardSanitize(ff->config.kv_vel);
    ff->config.ka_acc = FeedforwardSanitize(ff->config.ka_acc);
    ff->config.kv_viscous = FeedforwardSanitize(ff->config.kv_viscous);
    ff->config.kc_coulomb = FeedforwardSanitize(ff->config.kc_coulomb);
    ff->config.coulomb_smooth_omega =
        (isfinite(ff->config.coulomb_smooth_omega) &&
         ff->config.coulomb_smooth_omega > 0.0f)
            ? ff->config.coulomb_smooth_omega
            : kFeedforwardDefaultCoulombSmoothOmega;
    ff->config.kg_gravity = FeedforwardSanitize(ff->config.kg_gravity);
    ff->config.gravity_zero_angle =
        FeedforwardSanitize(ff->config.gravity_zero_angle);
    ff->config.omega_ff_limit =
        (isfinite(ff->config.omega_ff_limit) && ff->config.omega_ff_limit > 0.0f)
            ? ff->config.omega_ff_limit
            : 0.0f;
    ff->config.acc_ff_limit =
        (isfinite(ff->config.acc_ff_limit) && ff->config.acc_ff_limit > 0.0f)
            ? ff->config.acc_ff_limit
            : 0.0f;
    ff->config.output_ff_limit = (isfinite(ff->config.output_ff_limit) &&
                                  ff->config.output_ff_limit > 0.0f)
                                     ? ff->config.output_ff_limit
                                     : 0.0f;
    ff->config.lpf_cutoff_hz =
        (isfinite(ff->config.lpf_cutoff_hz) && ff->config.lpf_cutoff_hz > 0.0f)
            ? ff->config.lpf_cutoff_hz
            : 0.0f;
  }
}

/**
 * @brief 复位前馈内部低通滤波状态与输出（用于模式切换或视觉断联）
 * @param ff 前馈对象指针
 */
void GimbalFeedforwardReset(GimbalFeedforward *ff)
{
  if (ff == NULL)
  {
    return;
  }
  ff->omega_filtered = 0.0f;
  ff->accel_filtered = 0.0f;
  ff->last_output.omega_ff = 0.0f;
  ff->last_output.output_ff = 0.0f;
}

/**
 * @brief 运行期动态切换前馈总开关（关闭时自动清空滤波状态实现无冲击切换）
 * @param ff 前馈对象指针
 * @param enabled true 开启; false 关闭
 */
void GimbalFeedforwardSetEnabled(GimbalFeedforward *ff, bool enabled)
{
  if (ff == NULL)
  {
    return;
  }
  if (!enabled)
  {
    GimbalFeedforwardReset(ff);
  }
  ff->config.enabled = enabled;
}

/**
 * @brief 从系统辨识模型导入动力学前馈参数
 * @param ff 前馈对象指针
 * @param model 已辨识的电流坐标机械模型（SysidModel）
 * @param torque_constant 转矩常数 Kt（Nm/A）；<=0 时直接输出电流域参数（适用 GM6020），
 *                        >0 时乘以 Kt 换算为力矩域参数（适用 DM MIT 力矩模式）
 * @return true 导入成功; false 参数无效或模型未就绪
 */
bool GimbalFeedforwardLoadSysidModel(GimbalFeedforward *ff,
                                     const SysidModel *model,
                                     float torque_constant)
{
  if (ff == NULL || model == NULL || model->status != kSysidModelReady ||
      !isfinite(model->inertia) || !isfinite(model->viscous) ||
      !isfinite(model->friction) || !isfinite(torque_constant))
  {
    return false;
  }

  float scale = (torque_constant > 0.0f) ? torque_constant : 1.0f;
  ff->config.ka_acc = fmaxf(model->inertia, 0.0f) * scale;
  ff->config.kv_viscous = fmaxf(model->viscous, 0.0f) * scale;
  ff->config.kc_coulomb = fmaxf(model->friction, 0.0f) * scale;
  return true;
}

/**
 * @brief 计算云台双级前馈输出（速度前馈 + 动力学/重力前馈）
 * @param ff 前馈对象指针
 * @param target_omega 上位机期望角速度（rad/s）
 * @param target_accel 上位机期望角加速度（rad/s^2）
 * @param current_angle 当前云台角度（rad，用于 Pitch 重力前馈）
 * @param dt 控制周期（s）
 * @return 双级前馈输出结构体，关闭或输入无效时输出 0
 */
GimbalFeedforwardOutput GimbalFeedforwardCalculate(GimbalFeedforward *ff,
                                                   float target_omega,
                                                   float target_accel,
                                                   float current_angle,
                                                   float dt)
{
  GimbalFeedforwardOutput out = {0.0f, 0.0f};
  if (ff == NULL || !ff->config.enabled)
  {
    return out;
  }

  float omega_in =
      FeedforwardSymmetricClamp(target_omega, ff->config.omega_ff_limit);
  float accel_in =
      FeedforwardSymmetricClamp(target_accel, ff->config.acc_ff_limit);

  if (ff->config.lpf_cutoff_hz > 0.0f && isfinite(dt) && dt > 0.0f)
  {
    float rc_omega = kFeedforwardTwoPi * ff->config.lpf_cutoff_hz * dt;
    float alpha = rc_omega / (1.0f + rc_omega);
    ff->omega_filtered += alpha * (omega_in - ff->omega_filtered);
    ff->accel_filtered += alpha * (accel_in - ff->accel_filtered);
  }
  else
  {
    ff->omega_filtered = omega_in;
    ff->accel_filtered = accel_in;
  }

  if (ff->config.enable_velocity_ff)
  {
    out.omega_ff = FeedforwardSymmetricClamp(
        ff->config.kv_vel * ff->omega_filtered, ff->config.omega_ff_limit);
  }

  float total_output_ff = 0.0f;
  if (ff->config.enable_dynamic_ff)
  {
    float smooth = (ff->config.coulomb_smooth_omega > 0.0f)
                       ? ff->config.coulomb_smooth_omega
                       : kFeedforwardDefaultCoulombSmoothOmega;
    float coulomb_term =
        ff->config.kc_coulomb * tanhf(ff->omega_filtered / smooth);
    total_output_ff += ff->config.ka_acc * ff->accel_filtered +
                       ff->config.kv_viscous * ff->omega_filtered +
                       coulomb_term;
  }

  if (ff->config.enable_gravity_ff && isfinite(current_angle))
  {
    total_output_ff +=
        ff->config.kg_gravity *
        cosf(current_angle + ff->config.gravity_zero_angle);
  }

  out.output_ff =
      FeedforwardSymmetricClamp(total_output_ff, ff->config.output_ff_limit);
  ff->last_output = out;
  return out;
}
