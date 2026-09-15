#ifndef ALG_PID_H
#define ALG_PID_H
#include "stdbool.h"
// PID控制器结构体
typedef struct {
  bool is_enabled;                       // PID使能标志
  float kp;                              // 比例增益
  float ki;                              // 积分增益
  float kd;                              // 微分增益
  float sampling_time;                   // 采样时间
  float reference;                       // 目标值
  float feedback;                        // 反馈值
  float error;                           // 当前误差
  float error_sum;                       // 误差累计
  float previous_error;                  // 上一次误差
  float previous_reference;              // 上一次目标值
  float previous_feedback;               // 上一次反馈值
  float output;                          // 输出值
  bool is_derivativeonfeedback_enabled;  // 微分先行使能标志
  bool is_feedforward_enabled;           // 前馈使能标志
  float kff;                             // 前馈增益
  bool is_antiwindup_enabled;            // 抗积分饱和使能标志
  float output_max;                      // 最大输出
  float integral_max;                    // 最大积分
  bool is_deadband_enabled;              // 死区使能标志
  float deadband_threshold;              // 死区阈值
  bool is_integralseparate_enabled;      // 积分分离使能标志
  float integral_separate_threshold;     // 积分分离阈值
} PidObject;
// PID控制器配置结构体
typedef struct {
  float kp;                              // 比例增益
  float ki;                              // 积分增益
  float kd;                              // 微分增益
  float sampling_time;                   // 采样时间
  bool is_derivativeonfeedback_enabled;  // 微分先行使能标志
  bool is_feedforward_enabled;           // 前馈使能标志
  float kff;                             // 前馈增益
  bool is_antiwindup_enabled;            // 抗积分饱和使能标志
  float output_max;                      // 最大输出
  float integral_max;                    // 最大积分
  bool is_deadband_enabled;              // 死区使能标志
  float deadband_threshold;              // 死区阈值
  bool is_integralseparate_enabled;      // 积分分离使能标志
  float integral_separate_threshold;     // 积分分离阈值
} PidConfig;
void PidReset(PidObject *pid_object);
void PidInit(PidObject *pid_object, PidConfig *pid_config);
float PidCalculate(PidObject *pid_object, float reference, float feedback);
#endif  // ALG_PID_H