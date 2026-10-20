#ifndef ALG_AUTOTUNE_H
#define ALG_AUTOTUNE_H

#include "alg_dob.h"
#include "alg_pid.h"
#include "alg_sysid.h"

// 自动整定状态
typedef enum
{
  kAutotuneIdle,
  kAutotuneIdentify,
  kAutotuneSpeed,
  kAutotunePosition,
  kAutotuneDob,
  kAutotuneComplete,
  kAutotuneFailed,
  kAutotuneCancelled
} AutotuneState;

// 自动整定失败原因
typedef enum
{
  kAutotuneNoError,
  kAutotuneInvalidSample,
  kAutotuneMotionLimit,
  kAutotuneInsufficientModel,
  kAutotuneTrackingError
} AutotuneError;

// 自动整定配置结构体，运动量与控制器反馈使用相同坐标
typedef struct
{
  float sampling_time; // 采样时间（s）
  float current_limit; // 测试电流上限（A）
  float speed_limit;   // 测试角速度上限（rad/s）
  float position_min;  // 连续角度下限（rad）
  float position_max;  // 连续角度上限（rad）
  float speed_offset;  // 速度环工作点（rad/s）
  float response_time; // 期望响应时间（s），0表示自动选择
  bool tune_position;  // 整定位置外环
  bool adaptive;       // 正常运行时缓慢更新模型及增益
} AutotuneConfig;

// 自动整定结果结构体
typedef struct
{
  SysidModel model;
  float speed_kp;
  float speed_ki;
  float position_kp;
  float bandwidth;  // 速度环设计自然频率（rad/s）
  float rms_error;  // 当前验证归一化均方根误差
  float peak_error; // 当前验证归一化峰值误差
  float dob_cutoff_hz;
  uint32_t revision; // 已应用的自适应更新次数
} AutotuneResult;

// 自动整定结构体
typedef struct
{
  AutotuneConfig config;
  AutotuneResult result;
  AutotuneState state;
  AutotuneError error;
  SysidObject sysid;
  SysidObject rotor_sysid;
  SysidModel validated_model;
  PidObject speed_pid;
  PidObject position_pid;
  DobObject dob;
  float elapsed;
  float total_time;
  float center;
  float test_amplitude;
  float requested_current;
  float sum_error;
  float peak_error;
  float adaptive_time;
  float feedforward;
  float velocity_sine;
  float velocity_cosine;
  float rotor_inertia;
  float excitation_time;
  float excitation_peak;
  float external_difference; // 正常控制中DOB限幅丢失的电流请求（A）
  uint32_t samples;
  uint32_t random;
  uint32_t pulse_ticks;
  uint32_t adaptive_count;
  uint8_t attempt;
  bool has_dob;
  bool applied;
} AutotuneObject;

bool AutotuneInit(AutotuneObject *tune, const AutotuneConfig *config);
bool AutotuneStart(AutotuneObject *tune, float position, float omega,
                   const DobObject *dob);
bool AutotuneIsActive(const AutotuneObject *tune);
void AutotuneCancel(AutotuneObject *tune);
float AutotuneUpdate(AutotuneObject *tune, float position, float omega,
                     float actual_current, float rotor_omega);
void AutotuneTrackOutput(AutotuneObject *tune, float executed_current);
bool AutotuneApply(AutotuneObject *tune, PidObject *speed, PidObject *position,
                   DobObject *dob);
bool AutotuneRestore(AutotuneObject *tune, const SysidModel *model,
                     float bandwidth, const PidObject *speed,
                     const PidObject *position, const DobObject *dob);
float AutotuneAdapt(AutotuneObject *tune, PidObject *speed, float omega,
                    float actual_current);

#endif // ALG_AUTOTUNE_H
