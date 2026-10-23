#ifndef ALG_PID_TUNE_H
#define ALG_PID_TUNE_H

#include "alg_pid.h"
#include "alg_sysid.h"

// PI性能优化状态
typedef enum
{
  kPidTuneIdle = 0,
  kPidTuneRunning,
  kPidTuneComplete,
  kPidTuneLimited,
  kPidTuneInvalidSample,
} PidTuneState;

// 低维敏感度优化配置，输入与反馈坐标同辨识模型
typedef struct
{
  SysidModel model;     // 已验证的机械模型
  float sampling_time;  // 采样时间（s）
  float settling_time;  // 每轮不计入损失的过渡时间（s）
  float feedback_scale; // 跟踪误差归一化尺度
  float output_weight;  // 归一化执行量代价权重
  float change_weight;  // 归一化执行量变化代价权重
} PidTuneConfig;

// 已验证测试窗口中的最优参数
typedef struct
{
  float kp;
  float ki;
  float initial_loss;
  float loss;
  uint32_t trials; // 已完成测试轮数
} PidTuneResult;

// 固定尺寸PI优化对象，参数采用对数坐标
typedef struct
{
  PidTuneConfig config;
  PidTuneResult result;
  PidTuneState state;
  float velocity_sensitivity[2];
  float integral_sensitivity[2];
  float error_sensitivity[2];
  float output_sensitivity[2];
  float initial_gain[2];
  float decay;
  float input_gain;
  float elapsed;
  float previous_output;
  float requested_output;
  float output_scale;
  double gradient[2];
  double curvature[2][2];
  double loss;
  uint32_t samples;
  bool limited;
} PidTuneObject;

bool PidTuneInit(PidTuneObject *tune, const PidTuneConfig *config);
bool PidTuneStart(PidTuneObject *tune, const PidObject *pid);
float PidTuneCalculate(PidTuneObject *tune, PidObject *pid, float reference,
                       float feedback);
void PidTuneTrackOutput(PidTuneObject *tune, float executed_output);
bool PidTuneFinish(PidTuneObject *tune, PidObject *pid);

#endif // ALG_PID_TUNE_H
