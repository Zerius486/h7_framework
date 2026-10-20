#include "alg_motion.h"

#include <math.h>
#include <string.h>

/**
 * @brief 按上升和下降变化率更新标量目标
 * @param ramp 状态指针
 * @param target 目标值
 * @param rise_rate 上升速率，单位为输入单位每秒
 * @param fall_rate 下降速率，单位为输入单位每秒
 * @param sampling_time 采样时间（s）
 * @return 限制后的目标
 */
float MotionRampUpdate(MotionRamp *ramp, float target, float rise_rate,
                       float fall_rate, float sampling_time)
{
  float delta = target - ramp->value;
  delta = fminf(fmaxf(delta, -fall_rate * sampling_time),
                rise_rate * sampling_time);
  ramp->value += delta;
  return ramp->value;
}

/**
 * @brief 将五次轨迹重置为指定静止位置
 * @param trajectory 状态指针
 * @param position 初始位置
 * @param duration 轨迹时长（s），应为正值
 */
void MotionTrajectoryReset(MotionTrajectory *trajectory, float position,
                           float duration)
{
  memset(trajectory, 0, sizeof(*trajectory));
  trajectory->position = position;
  trajectory->target = position;
  trajectory->duration = duration;
  trajectory->coefficient[0] = position;
}

/**
 * @brief 更新五次轨迹，目标变化时从当前运动状态重新规划
 * @param trajectory 状态指针
 * @param target 目标位置
 * @param sampling_time 采样时间（s）
 */
void MotionTrajectoryUpdate(MotionTrajectory *trajectory, float target,
                            float sampling_time)
{
  float *c = trajectory->coefficient;
  float duration = trajectory->duration;
  if (target != trajectory->target)
  {
    float delta = target - trajectory->position;
    float velocity = trajectory->velocity * duration;
    float acceleration = trajectory->acceleration * duration * duration;
    c[0] = trajectory->position;
    c[1] = velocity;
    c[2] = acceleration / 2.0f;
    c[3] = 10.0f * delta - 6.0f * velocity - 1.5f * acceleration;
    c[4] = -15.0f * delta + 8.0f * velocity + 1.5f * acceleration;
    c[5] = 6.0f * delta - 3.0f * velocity - 0.5f * acceleration;
    trajectory->target = target;
    trajectory->elapsed = 0.0f;
  }
  trajectory->elapsed = fminf(trajectory->elapsed + sampling_time, duration);
  float u = trajectory->elapsed / duration;
  trajectory->position =
      ((((c[5] * u + c[4]) * u + c[3]) * u + c[2]) * u + c[1]) * u + c[0];
  trajectory->velocity =
      (((5.0f * c[5] * u + 4.0f * c[4]) * u + 3.0f * c[3]) * u + 2.0f * c[2]) *
          u +
      c[1];
  trajectory->velocity /= duration;
  trajectory->acceleration =
      ((20.0f * c[5] * u + 12.0f * c[4]) * u + 6.0f * c[3]) * u + 2.0f * c[2];
  trajectory->acceleration /= duration * duration;
  if (trajectory->elapsed >= duration)
  {
    trajectory->position = trajectory->target;
    trajectory->velocity = 0.0f;
    trajectory->acceleration = 0.0f;
  }
}

/**
 * @brief 将舵轮目标状态重置为零速及指定舵角
 * @param setpoint 状态指针
 * @param angle 四轮舵角（rad）
 */
void SteerSetpointReset(SteerSetpoint *setpoint, const float angle[4])
{
  memset(setpoint, 0, sizeof(*setpoint));
  memcpy(setpoint->wheels.wheel_angle, angle,
         sizeof(setpoint->wheels.wheel_angle));
  setpoint->has_sample = true;
}

/**
 * @brief 计算候选速度的轮速和舵角，并检查每周期变化约束
 * @param setpoint 上周期目标
 * @param chassis 底盘参数和目标
 * @param fraction 车体速度插值系数
 * @param speed_step 本周期轮速变化上限（m/s）
 * @param angle_step 本周期转角变化上限（rad）
 * @param hysteresis 选向滞回角度（rad）
 * @param candidate 候选底盘
 * @return true表示满足约束
 */
static bool SteerCandidate(const SteerSetpoint *setpoint,
                           const Chassis *chassis, float fraction,
                           float speed_step, float angle_step, float hysteresis,
                           Chassis *candidate)
{
  *candidate = *chassis;
  candidate->velocity.vx =
      setpoint->velocity.vx +
      fraction * (chassis->velocity.vx - setpoint->velocity.vx);
  candidate->velocity.vy =
      setpoint->velocity.vy +
      fraction * (chassis->velocity.vy - setpoint->velocity.vy);
  candidate->velocity.omega =
      setpoint->velocity.omega +
      fraction * (chassis->velocity.omega - setpoint->velocity.omega);
  candidate->state = setpoint->wheels;
  ChassisSetWheelAngleFeedback(candidate, setpoint->wheels.wheel_angle);
  ChassisUpdateState(candidate);
  bool feasible = true;
  for (int i = 0; i < 4; i++)
  {
    float delta = remainderf(candidate->state.wheel_angle[i] -
                                 setpoint->wheels.wheel_angle[i],
                             2.0f * (float)M_PI);
    float previous = setpoint->wheels.wheel_speed[i];
    float speed = candidate->state.wheel_speed[i];
    // 选向边界附近保留上周期轮速符号，减少反复切换。
    if (previous * speed < 0.0f &&
        fabsf(delta) > (float)M_PI / 2.0f - hysteresis)
    {
      candidate->state.wheel_speed[i] = -speed;
      candidate->state.wheel_angle[i] = remainderf(
          candidate->state.wheel_angle[i] + (float)M_PI, 2.0f * (float)M_PI);
      delta = remainderf(candidate->state.wheel_angle[i] -
                             setpoint->wheels.wheel_angle[i],
                         2.0f * (float)M_PI);
    }
    feasible &= fabsf(candidate->state.wheel_speed[i] - previous) <=
                    speed_step + 1e-6f &&
                fabsf(delta) <= angle_step + 1e-6f;
  }
  return feasible;
}

/**
 * @brief 在轮速加速度和舵向速度约束下生成协调的四轮目标
 * @param setpoint 目标状态指针
 * @param chassis 底盘目标及输出，轮速应在调用前统一限制
 * @param wheel_acceleration 轮速加速度上限（m/s²）
 * @param steer_rate 舵向速度上限（rad/s）
 * @param switch_hysteresis 选向滞回（rad）
 * @param sampling_time 采样时间（s）
 */
void SteerSetpointUpdate(SteerSetpoint *setpoint, Chassis *chassis,
                         float wheel_acceleration, float steer_rate,
                         float switch_hysteresis, float sampling_time)
{
  if (!setpoint->has_sample)
  {
    SteerSetpointReset(setpoint, chassis->wheel_angle_feedback);
  }
  float speed_step = wheel_acceleration * sampling_time;
  float angle_step = steer_rate * sampling_time;
  Chassis candidate;
  float fraction = 1.0f;
  if (!SteerCandidate(setpoint, chassis, fraction, speed_step, angle_step,
                      switch_hysteresis, &candidate))
  {
    float low = 0.0f;
    float high = 1.0f;
    for (int i = 0; i < 16; i++)
    {
      float mid = (low + high) / 2.0f;
      if (SteerCandidate(setpoint, chassis, mid, speed_step, angle_step,
                         switch_hysteresis, &candidate))
      {
        low = mid;
      }
      else
      {
        high = mid;
      }
    }
    fraction = low;
    SteerCandidate(setpoint, chassis, fraction, speed_step, angle_step,
                   switch_hysteresis, &candidate);
  }
  if (fraction < 1e-4f && fabsf(setpoint->velocity.vx) +
                                  fabsf(setpoint->velocity.vy) +
                                  fabsf(setpoint->velocity.omega) <
                              1e-5f)
  {
    // 静止时先转向，避免非零目标在零速处无法开始插值。
    Chassis desired;
    SteerCandidate(setpoint, chassis, 1.0f, speed_step, angle_step,
                   switch_hysteresis, &desired);
    candidate.velocity = setpoint->velocity;
    for (int i = 0; i < 4; i++)
    {
      float delta = remainderf(desired.state.wheel_angle[i] -
                                   setpoint->wheels.wheel_angle[i],
                               2.0f * (float)M_PI);
      candidate.state.wheel_angle[i] =
          remainderf(setpoint->wheels.wheel_angle[i] +
                         fminf(fmaxf(delta, -angle_step), angle_step),
                     2.0f * (float)M_PI);
      candidate.state.wheel_speed[i] = 0.0f;
    }
  }
  setpoint->velocity = candidate.velocity;
  setpoint->wheels = candidate.state;
  chassis->velocity = candidate.velocity;
  chassis->state = candidate.state;
}

/**
 * @brief 按电机模型计算非回馈功率
 * @param model 各电机功率模型
 * @param current 电流数组（A）
 * @param omega 转子角速度数组（rad/s）
 * @param count 电机数量
 * @param scale 电流缩放比例
 * @return 总功率（W）
 */
float MotorPowerEstimate(const MotorPowerModel *model, const float *current,
                         const float *omega, unsigned int count, float scale)
{
  float power = 0.0f;
  for (unsigned int i = 0; i < count; i++)
  {
    float value = current[i] * scale;
    power += fmaxf(model[i].copper_loss * value * value +
                       model[i].torque_constant * omega[i] * value,
                   0.0f);
  }
  return power;
}

/**
 * @brief 计算满足功率预算的统一电流缩放比例
 * @param model 各电机模型
 * @param current 电流数组（A）
 * @param omega 转子角速度数组（rad/s）
 * @param count 电机数量
 * @param budget 可用功率（W）
 * @return 电流缩放比例，范围[0,1]
 */
float MotorPowerScale(const MotorPowerModel *model, const float *current,
                      const float *omega, unsigned int count, float budget)
{
  if (MotorPowerEstimate(model, current, omega, count, 1.0f) <= budget)
  {
    return 1.0f;
  }
  float low = 0.0f;
  float high = 1.0f;
  for (int i = 0; i < 24; i++)
  {
    float mid = (low + high) / 2.0f;
    if (MotorPowerEstimate(model, current, omega, count, mid) <= budget)
    {
      low = mid;
    }
    else
    {
      high = mid;
    }
  }
  return low;
}

/**
 * @brief 从四轮反馈和可选陀螺仪约束估计车体速度
 * @param chassis 底盘几何及构型
 * @param speed 四轮线速度（m/s），使用构型定义的正向
 * @param angle 四轮实际舵角（rad），固定轮不使用
 * @param gyro_omega 车体Z轴角速度（rad/s）
 * @param gyro_weight 陀螺仪方程权重，0表示仅使用轮速
 * @param velocity 估计速度输出
 * @param residual 轮速方程均方根残差输出（m/s）
 * @return true表示方程可解
 */
bool ChassisEstimateVelocity(const Chassis *chassis, const float speed[4],
                             const float angle[4], float gyro_omega,
                             float gyro_weight, ChassisVelocity *velocity,
                             float *residual)
{
  float matrix[3][4] = {0};
  float rows[8][4] = {0};
  int count = chassis->type == kSteerWheel ? 8 : 4;
  const float x[4] = {1.0f, -1.0f, -1.0f, 1.0f};
  const float y[4] = {1.0f, 1.0f, -1.0f, -1.0f};
  for (int i = 0; i < 4; i++)
  {
    if (chassis->type == kSteerWheel)
    {
      rows[2 * i][0] = 1.0f;
      rows[2 * i][2] = -y[i] * chassis->params.ly;
      rows[2 * i][3] = speed[i] * cosf(angle[i]);
      rows[2 * i + 1][1] = 1.0f;
      rows[2 * i + 1][2] = x[i] * chassis->params.lx;
      rows[2 * i + 1][3] = speed[i] * sinf(angle[i]);
    }
    else if (chassis->type == kMecanumWheel)
    {
      rows[i][0] = 1.0f;
      rows[i][1] = i % 2 == 0 ? -1.0f : 1.0f;
      rows[i][2] =
          (i < 2 ? -1.0f : 1.0f) * (chassis->params.lx + chassis->params.ly);
      rows[i][3] = speed[i];
    }
    else
    {
      rows[i][0] = (i < 2 ? -1.0f : 1.0f) * kSqrt2 / 2.0f;
      rows[i][1] = (i == 0 || i == 3 ? 1.0f : -1.0f) * kSqrt2 / 2.0f;
      rows[i][2] = kSqrt2 / 2.0f * (chassis->params.lx + chassis->params.ly);
      rows[i][3] = speed[i];
    }
  }
  for (int i = 0; i < count; i++)
  {
    for (int j = 0; j < 3; j++)
    {
      for (int k = 0; k < 4; k++)
      {
        matrix[j][k] += rows[i][j] * rows[i][k];
      }
    }
  }
  matrix[2][2] += gyro_weight;
  matrix[2][3] += gyro_weight * gyro_omega;
  for (int i = 0; i < 3; i++)
  {
    if (fabsf(matrix[i][i]) < 1e-8f)
    {
      return false;
    }
    float divisor = matrix[i][i];
    for (int k = i; k < 4; k++)
    {
      matrix[i][k] /= divisor;
    }
    for (int j = 0; j < 3; j++)
    {
      if (j != i)
      {
        float factor = matrix[j][i];
        for (int k = i; k < 4; k++)
        {
          matrix[j][k] -= factor * matrix[i][k];
        }
      }
    }
  }
  velocity->vx = matrix[0][3];
  velocity->vy = matrix[1][3];
  velocity->omega = matrix[2][3];
  float error = 0.0f;
  for (int i = 0; i < count; i++)
  {
    float value = rows[i][0] * velocity->vx + rows[i][1] * velocity->vy +
                  rows[i][2] * velocity->omega - rows[i][3];
    error += value * value;
  }
  *residual = sqrtf(error / (float)count);
  return true;
}
