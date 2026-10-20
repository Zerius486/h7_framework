#ifndef ALG_SYSID_H
#define ALG_SYSID_H

#include <stdbool.h>
#include <stdint.h>

// 积分机械模型配置结构体
typedef struct
{
  float sampling_time; // 采样时间（s）
  float window_time;   // 积分窗口（s）
  float current_scale; // 电流归一化尺度（A）
  float omega_scale;   // 角速度归一化尺度（rad/s）
  float memory_time;   // 递推记忆时间（s），0表示不遗忘
} SysidConfig;

// 电流坐标机械模型结构体
typedef struct
{
  float inertia;  // 等效J/Kt
  float viscous;  // 等效b/Kt
  float friction; // 库仑摩擦电流（A）
  float bias;     // 当前工况恒定负载电流（A）
  float fitness;  // 一步预测拟合度
  uint32_t count; // 有效积分窗口数
} SysidModel;

// 固定尺寸递推辨识结构体
typedef struct
{
  SysidConfig config;
  float coefficient[4];
  float covariance[4][4];
  float integral[3];
  float elapsed;
  float start_omega;
  float previous_omega;
  float previous_current;
  float energy;
  float residual;
  float excitation;
  uint32_t count;
  bool has_sample;
} SysidObject;

bool SysidInit(SysidObject *sysid, const SysidConfig *config);
bool SysidUpdate(SysidObject *sysid, float current, float omega);
bool SysidGetModel(const SysidObject *sysid, SysidModel *model);

#endif // ALG_SYSID_H
