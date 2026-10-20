#ifndef ALG_CHASSIS_TUNE_H
#define ALG_CHASSIS_TUNE_H

#include "alg_autotune.h"
#include "alg_kinetics.h"

// 整车落地测试阶段
typedef enum
{
  kChassisTuneIdle,
  kChassisTunePrepare,
  kChassisTuneYaw,
  kChassisTuneBrake,
  kChassisTuneForward,
  kChassisTuneLateral,
  kChassisTuneComplete,
  kChassisTuneFailed,
  kChassisTuneCancelled
} ChassisTuneState;

// 落地测试配置，距离由轮速里程计检查
typedef struct
{
  float sampling_time; // 采样时间（s）
  float speed;         // 平移测试幅度（m/s）
  float omega;         // 旋转测试指令上限（rad/s）
  float distance;      // 相对起点轮速里程计半径（m）
  float angle;         // 相对起点旋转范围（rad）
  float response_time; // 角速度环期望响应时间（s）
} ChassisTuneConfig;

// 平移响应统计，只反映轮速反馈下的响应
typedef struct
{
  float rms;
  float peak;
  float gain;
  float lag;      // 相位滞后（rad）
  float coupling; // 交叉方向归一化RMS
} ChassisTuneResponse;

// 固定内存整车测试对象，旋转模型输入为运动学角速度指令
typedef struct
{
  ChassisTuneConfig config;
  AutotuneObject yaw;
  ChassisTuneResponse forward;
  ChassisTuneResponse lateral;
  ChassisTuneState state;
  ChassisTuneState next;
  AutotuneError error;
  ChassisVelocity command;
  float elapsed;
  float total_time;
  float heading;
  float x;
  float y;
  float gyro_bias;
  float sum_gyro;
  float sum_gyro_square;
  float sum_error;
  float sum_cross;
  float sine;
  float cosine;
  float still_time;
  float frequency;
  uint32_t samples;
  bool aligning;
} ChassisTuneObject;

bool ChassisTuneInit(ChassisTuneObject *tune, const ChassisTuneConfig *config);
bool ChassisTuneStart(ChassisTuneObject *tune);
bool ChassisTuneIsActive(const ChassisTuneObject *tune);
void ChassisTuneCancel(ChassisTuneObject *tune);
ChassisVelocity ChassisTuneUpdate(ChassisTuneObject *tune,
                                  const ChassisVelocity *velocity, float gyro,
                                  float executed_omega);

#endif // ALG_CHASSIS_TUNE_H
