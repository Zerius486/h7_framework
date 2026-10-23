#ifndef ALG_MOTION_H
#define ALG_MOTION_H

#include "alg_kinetics.h"

// 标量变化率限制状态
typedef struct
{
  float value; // 当前输出
} MotionRamp;

// 五次轨迹状态，目标变化时保持位置、速度和加速度连续
typedef struct
{
  float coefficient[6]; // 归一化时间多项式系数
  float target;         // 目标位置
  float elapsed;        // 已运行时间（s）
  float duration;       // 轨迹时长（s）
  float position;       // 当前目标位置
  float velocity;       // 当前目标速度
  float acceleration;   // 当前目标加速度
} MotionTrajectory;

// 舵轮目标生成状态
typedef struct
{
  ChassisVelocity velocity; // 上一次可实现的车体速度
  ChassisState wheels;      // 上一次轮速及舵角
  bool has_sample;          // 已初始化
} SteerSetpoint;

// 电机功率模型，功率为max(k_i*i*i + k_w*omega*i, 0)
typedef struct
{
  float copper_loss;     // 电流平方损耗系数（W/A²）
  float torque_constant; // 转矩常数（N·m/A）
} MotorPowerModel;

float MotionRampUpdate(MotionRamp *ramp, float target, float rise_rate,
                       float fall_rate, float sampling_time);
void MotionTrajectoryReset(MotionTrajectory *trajectory, float position,
                           float duration);
void MotionTrajectoryUpdate(MotionTrajectory *trajectory, float target,
                            float sampling_time);
void SteerSetpointReset(SteerSetpoint *setpoint, const float angle[4]);
void SteerSetpointUpdate(SteerSetpoint *setpoint, Chassis *chassis,
                         float wheel_acceleration, float steer_rate,
                         float switch_hysteresis, float sampling_time);
void SteerSpeedCorrect(const ChassisState *target, const float angle[4],
                       float angle_scale, float speed[4]);
float MotorPowerScale(const MotorPowerModel *model, const float *current,
                      const float *omega, unsigned int count, float budget);
float MotorPowerEstimate(const MotorPowerModel *model, const float *current,
                         const float *omega, unsigned int count, float scale);
bool ChassisEstimateVelocity(const Chassis *chassis, const float speed[4],
                             const float angle[4], float gyro_omega,
                             float gyro_weight, ChassisVelocity *velocity,
                             float *residual);

#endif // ALG_MOTION_H
