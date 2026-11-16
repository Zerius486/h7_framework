#ifndef ALG_MOTION_H
#define ALG_MOTION_H

#include "alg_kf.h"
#include "alg_kinetics.h"

#define CHASSIS_SPEED_KF_WORKSPACE_SIZE 131U

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

// IMU刚体杆臂效应补偿器（d为机体系下由IMU指向质心COM的向量）
typedef struct
{
  float lever_arm[3];          // 杆臂向量 d（m），机体系下 IMU -> COM
  float prev_lever_arm[3];     // 上一拍杆臂向量（m）
  float lever_arm_vel[3];      // 滤波后的杆臂相对速度 d_dot（m/s）
  float prev_lever_arm_vel[3]; // 上一拍杆臂相对速度（m/s）
  float lever_arm_acc[3];      // 滤波后的杆臂相对加速度 d_ddot（m/s²）
  float prev_omega[3];         // 上一拍角速度（rad/s）
  float alpha[3];              // 滤波后的机体角加速度（rad/s²）
  float diff_lpf_tau;          // 微分一阶低通时间常数（s），<=0表示不滤波
  bool enabled;                // 是否启用补偿
  bool has_sample;             // 是否已有历史采样
} LeverArmCompensator;

// 3状态底盘速度卡尔曼滤波器配置（状态：[速度, 加速度, 加速度计零偏]）
typedef struct
{
  float sampling_time; // 采样时间（s）
  float accel_q;       // 加速度随机游走过程噪声强度 q
  float bias_q;        // 加速度计零偏过程噪声强度 q_b
  float vel_r;         // 轮速观测噪声方差
  float acc_r;         // IMU前向加速度观测噪声方差
} ChassisSpeedKfConfig;

// 3状态底盘速度卡尔曼滤波器（支持腾空/打滑时伪观测无扰切换）
typedef struct
{
  KfObject kf;
  float workspace[CHASSIS_SPEED_KF_WORKSPACE_SIZE];
  ChassisSpeedKfConfig config;
  float velocity;     // 估计前向速度（m/s）
  float acceleration; // 估计真实前向加速度（m/s²）
  float accel_bias;   // 估计加速度计零偏（m/s²）
  float position;     // 积分位置（m）
  bool initialized;   // 初始化标志
} ChassisSpeedKf;

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
void LeverArmCompensatorInit(LeverArmCompensator *comp,
                             const float lever_arm[3], float diff_lpf_tau);
void LeverArmCompensatorReset(LeverArmCompensator *comp);
void LeverArmCompensatorSetArm(LeverArmCompensator *comp,
                               const float lever_arm[3]);
void LeverArmCompensatorUpdate(LeverArmCompensator *comp,
                                 const float accel_imu[3], const float omega[3],
                                 float dt, float accel_com[3]);
void ChassisSpeedKfConfigInit(ChassisSpeedKfConfig *config,
                              float sampling_time);
bool ChassisSpeedKfInit(ChassisSpeedKf *speed_kf,
                        const ChassisSpeedKfConfig *config);
void ChassisSpeedKfReset(ChassisSpeedKf *speed_kf);
float ChassisSpeedKfUpdate(ChassisSpeedKf *speed_kf, float wheel_velocity,
                           float accel_imu, bool wheel_valid);

#endif // ALG_MOTION_H
