#ifndef ALG_FEEDFORWARD_H
#define ALG_FEEDFORWARD_H

#include <stdbool.h>

#include "alg_sysid.h"

// 云台双级前馈配置结构体（支持运行时总开关与按项开关）
typedef struct
{
  bool enabled;            // 前馈总开关（false 时全部前馈输出为 0）
  bool enable_velocity_ff; // 速度前馈开关（注入外环位置输出/速度环目标）
  bool enable_dynamic_ff;  // 动力学前馈开关（加速度/粘滞阻尼/库仑摩擦）
  bool enable_gravity_ff;  // 重力前馈开关（Pitch 轴适用，Yaw 轴关闭）

  float kv_vel;               // 速度前馈系数（无量纲，典型 0.8~1.0）
  float ka_acc;               // 加速度前馈系数（J/Kt [A/(rad/s^2)] 或 J [Nm/(rad/s^2)]）
  float kv_viscous;           // 粘滞阻尼前馈系数（b/Kt [A/(rad/s)] 或 b [Nm/(rad/s)]）
  float kc_coulomb;           // 库仑摩擦前馈幅值（A 或 Nm）
  float coulomb_smooth_omega; // tanh 摩擦平滑过渡角速度（rad/s，<=0 默认 0.1）

  float kg_gravity;         // 重力前馈幅值（A 或 Nm，Yaw 轴填 0）
  float gravity_zero_angle; // 重力零位偏置角（rad，u_grav = kg * cos(angle + zero)）

  float omega_ff_limit;  // 速度前馈限幅（rad/s，<=0 不限幅）
  float acc_ff_limit;    // 加速度前馈输入限幅（rad/s^2，<=0 不限幅）
  float output_ff_limit; // 动力学+重力总前馈输出限幅（A 或 Nm，<=0 不限幅）
  float lpf_cutoff_hz;   // 速度/加速度前馈一阶低通截止频率（Hz，<=0 关闭滤波）
} GimbalFeedforwardConfig;

// 云台双级前馈输出结构体
typedef struct
{
  float omega_ff;  // 速度环前馈输入（rad/s，叠加到位置环输出或 MIT given_omega）
  float output_ff; // 电流/力矩前馈输出（A 或 Nm，叠加到速度环输出或 MIT given_torque）
} GimbalFeedforwardOutput;

// 云台双级前馈对象结构体（纯 C11、调用方分配、无全局状态）
typedef struct
{
  GimbalFeedforwardConfig config;      // 前馈参数配置
  float omega_filtered;                // 滤波后速度前馈（rad/s）
  float accel_filtered;                // 滤波后加速度前馈（rad/s^2）
  GimbalFeedforwardOutput last_output; // 最近一次前馈输出
} GimbalFeedforward;

void GimbalFeedforwardInit(GimbalFeedforward *ff,
                           const GimbalFeedforwardConfig *config);
void GimbalFeedforwardReset(GimbalFeedforward *ff);
void GimbalFeedforwardSetEnabled(GimbalFeedforward *ff, bool enabled);
bool GimbalFeedforwardLoadSysidModel(GimbalFeedforward *ff,
                                     const SysidModel *model,
                                     float torque_constant);
GimbalFeedforwardOutput GimbalFeedforwardCalculate(GimbalFeedforward *ff,
                                                   float target_omega,
                                                   float target_accel,
                                                   float current_angle,
                                                   float dt);

#endif // ALG_FEEDFORWARD_H
