#ifndef ALG_SYSID_H
#define ALG_SYSID_H

#include <stdbool.h>
#include <stdint.h>

#define SYSID_MAX_CURRENT_DELAY 32U
#define SYSID_VALIDATION_WINDOWS 40U

// 辨识更新方法
typedef enum
{
  kSysidExponential = 0, // 指数遗忘RLS
  kSysidSubspace,        // 信息子空间遗忘RLS
} SysidMethod;

// 模型可用性检查结果
typedef enum
{
  kSysidInsufficientSamples,
  kSysidInsufficientExcitation,
  kSysidInvalidModel,
  kSysidInconsistentModel,
  kSysidModelReady
} SysidStatus;

// 积分机械模型配置结构体
typedef struct
{
  float sampling_time;           // 采样时间（s）
  float window_time;             // 积分窗口（s）
  float current_scale;           // 电流归一化尺度（A）
  float omega_scale;             // 角速度归一化尺度（rad/s）
  float memory_time;             // 递推记忆时间（s），0表示不遗忘
  SysidMethod method;            // 辨识更新方法
  float minimum_speed;           // 滑动辨识最低速度（rad/s），0使用尺度的1%
  uint8_t current_delay_samples; // 电流历史配对延迟，0表示不延迟
} SysidConfig;

// 电流坐标机械模型结构体
typedef struct
{
  float inertia;            // 等效J/Kt
  float viscous;            // 等效b/Kt
  float friction;           // 库仑摩擦电流（A）
  float bias;               // 当前工况恒定负载电流（A）
  float fitness;            // 返回模型在有效窗口上的拟合度
  uint32_t count;           // 有效积分窗口数
  float validation_fitness; // 相邻独立数据段的最差交叉拟合度
  SysidStatus status;       // 模型读取状态
  float velocity_rmse;      // 独立速度预测均方根误差（rad/s）
  float velocity_fitness;   // 相对片段起始速度的动态拟合度
} SysidModel;

// 固定尺寸回归统计结构体
typedef struct
{
  double information[4][4];
  double output[4];
  double energy;
  uint32_t count;
} SysidStatistics;

// 固定尺寸递推辨识结构体
typedef struct
{
  SysidConfig config;
  float coefficient[4];
  float covariance[4][4];
  double information[4][4]; // SIFt-RLS信息矩阵，保留弱激励方向的精度
  float integral[3];
  float elapsed;
  float start_omega;
  float previous_omega;
  float previous_current;
  float excitation;
  uint32_t count;
  bool has_sample;
  SysidStatistics statistics;
  SysidStatistics block;
  SysidStatistics previous_block;
  float current_history[SYSID_MAX_CURRENT_DELAY + 1U];
  float validation_fitness;
  uint32_t rejected_windows;
  uint8_t history_index;
  uint8_t history_count;
  bool sliding_window;
  SysidStatus validation_status;
} SysidObject;

// 固定模型的分段自由速度预测，验证数据不回写辨识器
typedef struct
{
  SysidConfig config;
  SysidModel model;
  float current_history[SYSID_MAX_CURRENT_DELAY + 1U];
  float predicted;
  float initial;
  float elapsed;
  float previous_current;
  double error;
  double energy;
  uint32_t samples;
  uint32_t segments;
  uint8_t history_index;
  uint8_t history_count;
  bool active;
} SysidPrediction;

bool SysidInit(SysidObject *sysid, const SysidConfig *config);
bool SysidUpdate(SysidObject *sysid, float current, float omega);
bool SysidGetModel(const SysidObject *sysid, SysidModel *model);
void SysidResetWindow(SysidObject *sysid);
bool SysidPredictionInit(SysidPrediction *prediction, const SysidConfig *config,
                         const SysidModel *model);
bool SysidPredictionUpdate(SysidPrediction *prediction, float current,
                           float omega);
bool SysidPredictionGetResult(const SysidPrediction *prediction,
                              SysidModel *model);

#endif // ALG_SYSID_H
