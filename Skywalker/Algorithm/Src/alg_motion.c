#include "alg_motion.h"

#include <math.h>
#include <string.h>

#include "alg_conversion.h"

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
  if (ramp == NULL)
  {
    return target;
  }
  if (!isfinite(target) || !isfinite(rise_rate) || !isfinite(fall_rate) ||
      !isfinite(sampling_time) || sampling_time <= 0.0f || rise_rate < 0.0f ||
      fall_rate < 0.0f)
  {
    return ramp->value;
  }
  float delta = target - ramp->value;
  delta = MathClamp(delta, -fall_rate * sampling_time, rise_rate * sampling_time);
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
  if (trajectory == NULL)
  {
    return;
  }
  memset(trajectory, 0, sizeof(*trajectory));
  float safe_pos = isfinite(position) ? position : 0.0f;
  float safe_dur = (isfinite(duration) && duration > 0.0f) ? duration : 0.0f;
  trajectory->position = safe_pos;
  trajectory->target = safe_pos;
  trajectory->duration = safe_dur;
  trajectory->coefficient[0] = safe_pos;
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
  if (trajectory == NULL || !isfinite(target) || !isfinite(sampling_time) ||
      sampling_time <= 0.0f)
  {
    return;
  }
  float *c = trajectory->coefficient;
  float duration = trajectory->duration;
  if (!isfinite(duration) || duration <= 0.0f)
  {
    trajectory->position = target;
    trajectory->target = target;
    trajectory->velocity = 0.0f;
    trajectory->acceleration = 0.0f;
    return;
  }
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
    float delta = MathAngleDiff(candidate->state.wheel_angle[i],
                                setpoint->wheels.wheel_angle[i]);
    float previous = setpoint->wheels.wheel_speed[i];
    float speed = candidate->state.wheel_speed[i];
    // 选向边界附近保留上周期轮速符号，减少反复切换。
    if (previous * speed < 0.0f &&
        fabsf(delta) > (float)M_PI / 2.0f - hysteresis)
    {
      candidate->state.wheel_speed[i] = -speed;
      candidate->state.wheel_angle[i] =
          MathWrapPi(candidate->state.wheel_angle[i] + (float)M_PI);
      delta = MathAngleDiff(candidate->state.wheel_angle[i],
                            setpoint->wheels.wheel_angle[i]);
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
  if (setpoint == NULL || chassis == NULL || !isfinite(wheel_acceleration) ||
      !isfinite(steer_rate) || !isfinite(switch_hysteresis) ||
      !isfinite(sampling_time) || sampling_time <= 0.0f ||
      wheel_acceleration < 0.0f || steer_rate < 0.0f)
  {
    return;
  }
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
    Chassis desired = candidate;
    Chassis best_candidate;
    Chassis trial;
    bool has_best = false;
    float low = 0.0f;
    float high = 1.0f;
    for (int i = 0; i < 8; i++)
    {
      float mid = (low + high) / 2.0f;
      if (SteerCandidate(setpoint, chassis, mid, speed_step, angle_step,
                         switch_hysteresis, &trial))
      {
        low = mid;
        best_candidate = trial;
        has_best = true;
      }
      else
      {
        high = mid;
      }
    }
    fraction = low;
    if (has_best)
    {
      candidate = best_candidate;
    }
    else
    {
      SteerCandidate(setpoint, chassis, 0.0f, speed_step, angle_step,
                     switch_hysteresis, &candidate);
    }

    if (fraction < 1e-4f)
    {
      // 插值受阻时，仅预转向尚未对准的近零速轮。
      bool is_presteering = false;
      for (int i = 0; i < 4; i++)
      {
        if (fabsf(setpoint->wheels.wheel_speed[i]) > kSteerWheelSpeedEpsilon)
        {
          continue;
        }
        float delta = MathAngleDiff(desired.state.wheel_angle[i],
                                    setpoint->wheels.wheel_angle[i]);
        if (fabsf(delta) <= angle_step + 1e-6f)
        {
          continue;
        }
        if (!is_presteering)
        {
          candidate.velocity = setpoint->velocity;
          candidate.state = setpoint->wheels;
          is_presteering = true;
        }
        candidate.state.wheel_angle[i] =
            MathWrapPi(setpoint->wheels.wheel_angle[i] +
                       MathClamp(delta, -angle_step, angle_step));
      }
    }
  }
  setpoint->velocity = candidate.velocity;
  setpoint->wheels = candidate.state;
  chassis->velocity = candidate.velocity;
  chassis->state = candidate.state;
}

/**
 * @brief 按实际舵角误差进行三次余弦轮速修正
 * @param target 未修正的四轮目标
 * @param angle 四轮实际舵角（rad）
 * @param angle_scale 角度系数a，非负，0表示不衰减
 * @param speed 修正后的四轮线速度（m/s）
 */
void SteerSpeedCorrect(const ChassisState *target, const float angle[4],
                       float angle_scale, float speed[4])
{
  for (int i = 0; i < 4; i++)
  {
    float delta = fabsf(MathAngleDiff(target->wheel_angle[i], angle[i]));
    float phase = fminf(fmaxf(angle_scale, 0.0f) * delta, (float)M_PI / 2.0f);
    float correction = fmaxf(cosf(phase), 0.0f);
    speed[i] = target->wheel_speed[i] * correction * correction * correction;
  }
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
  if (model == NULL || current == NULL || omega == NULL || !isfinite(scale))
  {
    return 0.0f;
  }
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
  if (model == NULL || current == NULL || omega == NULL || !isfinite(budget) ||
      budget <= 0.0f)
  {
    return 0.0f;
  }
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
  if (chassis == NULL || speed == NULL || velocity == NULL ||
      residual == NULL || (chassis->type == kSteerWheel && angle == NULL) ||
      !isfinite(gyro_omega) || !isfinite(gyro_weight) || gyro_weight < 0.0f ||
      (chassis->type != kSteerWheel && chassis->type != kMecanumWheel &&
       chassis->type != kOmniWheel))
  {
    return false;
  }

  for (int i = 0; i < 4; i++)
  {
    if (!isfinite(speed[i]) ||
        (chassis->type == kSteerWheel && !isfinite(angle[i])))
    {
      return false;
    }
  }

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

  // 采用带列主元选择的高斯-约旦消元法（Gauss-Jordan with Partial Pivoting）求解 3x3 正规方程
  for (int i = 0; i < 3; i++)
  {
    // 列主元选取：寻找当前列 i 中绝对值最大的行，降低浮点舍入误差
    int pivot_row = i;
    float max_val = fabsf(matrix[i][i]);
    for (int r = i + 1; r < 3; r++)
    {
      float val = fabsf(matrix[r][i]);
      if (val > max_val)
      {
        max_val = val;
        pivot_row = r;
      }
    }

    // 奇异性检查：最大主元过小或 NaN 说明方程病态或不可解（如舵轮打滑或几何退化）
    if (!(max_val >= 1e-6f))
    {
      return false;
    }

    // 交换主元行
    if (pivot_row != i)
    {
      for (int k = i; k < 4; k++)
      {
        float temp = matrix[i][k];
        matrix[i][k] = matrix[pivot_row][k];
        matrix[pivot_row][k] = temp;
      }
    }

    // 归一化当前主元行
    float divisor = matrix[i][i];
    for (int k = i; k < 4; k++)
    {
      matrix[i][k] /= divisor;
    }

    // 消去其他行的当前列元素
    for (int j = 0; j < 3; j++)
    {
      if (j != i)
      {
        float factor = matrix[j][i];
        if (fabsf(factor) > 1e-9f)
        {
          for (int k = i; k < 4; k++)
          {
            matrix[j][k] -= factor * matrix[i][k];
          }
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

static void VectorCross3(const float a[3], const float b[3], float out[3])
{
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}

static bool VectorFinite3(const float v[3])
{
  return isfinite(v[0]) && isfinite(v[1]) && isfinite(v[2]);
}

/**
 * @brief 初始化IMU刚体杆臂效应补偿器
 * @param comp 补偿器指针
 * @param lever_arm 机体系下从IMU指向质心的杆臂向量（m），NULL或全零时禁用
 * @param diff_lpf_tau 角加速度与杆臂微分的一阶低通时间常数（s）
 */
void LeverArmCompensatorInit(LeverArmCompensator *comp,
                             const float lever_arm[3], float diff_lpf_tau)
{
  if (comp == NULL)
  {
    return;
  }
  memset(comp, 0, sizeof(*comp));
  comp->diff_lpf_tau =
      (isfinite(diff_lpf_tau) && diff_lpf_tau > 0.0f) ? diff_lpf_tau : 0.0f;
  if (lever_arm != NULL)
  {
    LeverArmCompensatorSetArm(comp, lever_arm);
  }
}

/**
 * @brief 重置杆臂补偿器历史微分状态
 * @param comp 补偿器指针
 */
void LeverArmCompensatorReset(LeverArmCompensator *comp)
{
  if (comp == NULL)
  {
    return;
  }
  memset(comp->prev_lever_arm, 0, sizeof(comp->prev_lever_arm));
  memset(comp->lever_arm_vel, 0, sizeof(comp->lever_arm_vel));
  memset(comp->prev_lever_arm_vel, 0, sizeof(comp->prev_lever_arm_vel));
  memset(comp->lever_arm_acc, 0, sizeof(comp->lever_arm_acc));
  memset(comp->prev_omega, 0, sizeof(comp->prev_omega));
  memset(comp->alpha, 0, sizeof(comp->alpha));
  comp->has_sample = false;
}

/**
 * @brief 更新机体系下的杆臂向量（支持云台旋转等动态杆臂场景）
 * @param comp 补偿器指针
 * @param lever_arm 机体系下从IMU指向质心的杆臂向量（m）
 */
void LeverArmCompensatorSetArm(LeverArmCompensator *comp,
                               const float lever_arm[3])
{
  if (comp == NULL || lever_arm == NULL || !VectorFinite3(lever_arm))
  {
    return;
  }
  comp->lever_arm[0] = lever_arm[0];
  comp->lever_arm[1] = lever_arm[1];
  comp->lever_arm[2] = lever_arm[2];
  comp->enabled = (fabsf(lever_arm[0]) > 1e-6f) ||
                  (fabsf(lever_arm[1]) > 1e-6f) ||
                  (fabsf(lever_arm[2]) > 1e-6f);
  if (!comp->enabled)
  {
    LeverArmCompensatorReset(comp);
  }
}

/**
 * @brief 计算杆臂效应补偿后的质心加速度:
 *        a_com = a_imu + alpha x d + omega x (omega x d) + 2*omega x d_dot + d_ddot
 * @param comp 补偿器指针
 * @param accel_imu IMU安装处加速度（m/s²）
 * @param omega 机体角速度（rad/s）
 * @param dt 采样时间（s）
 * @param accel_com 输出质心加速度（m/s²）
 */
void LeverArmCompensatorUpdate(LeverArmCompensator *comp,
                                 const float accel_imu[3], const float omega[3],
                                 float dt, float accel_com[3])
{
  if (accel_imu == NULL || accel_com == NULL)
  {
    return;
  }
  if (comp == NULL || !comp->enabled || omega == NULL || !isfinite(dt) ||
      dt <= 0.0f || !VectorFinite3(accel_imu) || !VectorFinite3(omega))
  {
    accel_com[0] = accel_imu[0];
    accel_com[1] = accel_imu[1];
    accel_com[2] = accel_imu[2];
    return;
  }

  if (!comp->has_sample)
  {
    for (int i = 0; i < 3; i++)
    {
      comp->prev_omega[i] = omega[i];
      comp->prev_lever_arm[i] = comp->lever_arm[i];
      comp->lever_arm_vel[i] = 0.0f;
      comp->prev_lever_arm_vel[i] = 0.0f;
      comp->lever_arm_acc[i] = 0.0f;
      comp->alpha[i] = 0.0f;
    }
    comp->has_sample = true;
  }
  else
  {
    float inv_dt = 1.0f / dt;
    float lpf_alpha =
        (comp->diff_lpf_tau > 0.0f) ? (dt / (comp->diff_lpf_tau + dt)) : 1.0f;
    for (int i = 0; i < 3; i++)
    {
      float raw_alpha = (omega[i] - comp->prev_omega[i]) * inv_dt;
      comp->alpha[i] += lpf_alpha * (raw_alpha - comp->alpha[i]);
      comp->prev_omega[i] = omega[i];

      float raw_vel = (comp->lever_arm[i] - comp->prev_lever_arm[i]) * inv_dt;
      comp->lever_arm_vel[i] += lpf_alpha * (raw_vel - comp->lever_arm_vel[i]);
      comp->prev_lever_arm[i] = comp->lever_arm[i];

      float raw_acc =
          (comp->lever_arm_vel[i] - comp->prev_lever_arm_vel[i]) * inv_dt;
      comp->lever_arm_acc[i] += lpf_alpha * (raw_acc - comp->lever_arm_acc[i]);
      comp->prev_lever_arm_vel[i] = comp->lever_arm_vel[i];
    }
  }

  float tangential[3];
  float omega_cross_d[3];
  float centripetal[3];
  float coriolis[3];
  VectorCross3(comp->alpha, comp->lever_arm, tangential);
  VectorCross3(omega, comp->lever_arm, omega_cross_d);
  VectorCross3(omega, omega_cross_d, centripetal);
  VectorCross3(omega, comp->lever_arm_vel, coriolis);

  for (int i = 0; i < 3; i++)
  {
    accel_com[i] = accel_imu[i] + tangential[i] + centripetal[i] +
                   2.0f * coriolis[i] + comp->lever_arm_acc[i];
  }
}

/**
 * @brief 使用默认参数初始化3状态速度卡尔曼滤波器配置
 * @param config 配置结构体指针
 * @param sampling_time 采样时间（s）
 */
void ChassisSpeedKfConfigInit(ChassisSpeedKfConfig *config, float sampling_time)
{
  if (config == NULL)
  {
    return;
  }
  config->sampling_time = sampling_time;
  config->accel_q = 2000.0f;
  config->bias_q = 0.05f;
  config->vel_r = 1.0f;
  config->acc_r = 2.0f;
}

static void ChassisSpeedKfApplyMatrices(ChassisSpeedKf *speed_kf)
{
  float dt = speed_kf->config.sampling_time;
  float dt2 = dt * dt;
  float dt3 = dt2 * dt;
  float dt4 = dt2 * dt2;
  float q = speed_kf->config.accel_q;
  float qb = speed_kf->config.bias_q;

  memset(speed_kf->kf.f_data, 0, 9U * sizeof(float));
  speed_kf->kf.f_data[0] = 1.0f;
  speed_kf->kf.f_data[1] = dt;
  speed_kf->kf.f_data[4] = 1.0f;
  speed_kf->kf.f_data[8] = 1.0f;

  memset(speed_kf->kf.q_data, 0, 9U * sizeof(float));
  speed_kf->kf.q_data[0] = 0.25f * dt4 * q;
  speed_kf->kf.q_data[1] = 0.5f * dt3 * q;
  speed_kf->kf.q_data[3] = 0.5f * dt3 * q;
  speed_kf->kf.q_data[4] = dt2 * q;
  speed_kf->kf.q_data[8] = dt * qb;

  memset(speed_kf->kf.h_data, 0, 6U * sizeof(float));
  speed_kf->kf.h_data[0] = 1.0f;
  speed_kf->kf.h_data[4] = 1.0f;
  speed_kf->kf.h_data[5] = 1.0f;

  memset(speed_kf->kf.r_data, 0, 4U * sizeof(float));
  speed_kf->kf.r_data[0] = speed_kf->config.vel_r;
  speed_kf->kf.r_data[3] = speed_kf->config.acc_r;
}

/**
 * @brief 初始化3状态底盘速度卡尔曼滤波器
 * @param speed_kf 滤波器对象指针
 * @param config 配置结构体指针
 * @return true表示初始化成功
 */
bool ChassisSpeedKfInit(ChassisSpeedKf *speed_kf,
                        const ChassisSpeedKfConfig *config)
{
  if (speed_kf == NULL || config == NULL || !isfinite(config->sampling_time) ||
      config->sampling_time <= 0.0f || !isfinite(config->accel_q) ||
      config->accel_q <= 0.0f || !isfinite(config->bias_q) ||
      config->bias_q < 0.0f || !isfinite(config->vel_r) ||
      config->vel_r <= 0.0f || !isfinite(config->acc_r) ||
      config->acc_r <= 0.0f)
  {
    return false;
  }
  if (KfWorkspaceSize(3U, 0U, 2U) > CHASSIS_SPEED_KF_WORKSPACE_SIZE)
  {
    return false;
  }

  memset(speed_kf, 0, sizeof(*speed_kf));
  speed_kf->config = *config;
  if (KfInit(&speed_kf->kf, 3U, 0U, 2U, speed_kf->workspace,
             CHASSIS_SPEED_KF_WORKSPACE_SIZE) != 0)
  {
    return false;
  }
  ChassisSpeedKfApplyMatrices(speed_kf);
  speed_kf->initialized = true;
  ChassisSpeedKfReset(speed_kf);
  return true;
}

/**
 * @brief 复位底盘速度卡尔曼滤波器状态与协方差
 * @param speed_kf 滤波器对象指针
 */
void ChassisSpeedKfReset(ChassisSpeedKf *speed_kf)
{
  if (speed_kf == NULL || !speed_kf->initialized)
  {
    return;
  }
  memset(speed_kf->kf.xhat_data, 0, 3U * sizeof(float));
  memset(speed_kf->kf.xhat_minus_data, 0, 3U * sizeof(float));
  memset(speed_kf->kf.p_data, 0, 9U * sizeof(float));
  speed_kf->kf.p_data[0] = 1.0f;
  speed_kf->kf.p_data[4] = 1.0f;
  speed_kf->kf.p_data[8] = 1.0f;
  speed_kf->velocity = 0.0f;
  speed_kf->acceleration = 0.0f;
  speed_kf->accel_bias = 0.0f;
  speed_kf->position = 0.0f;
}

/**
 * @brief 单步更新底盘速度卡尔曼滤波器（腾空/打滑时构造零残差伪观测）
 * @param speed_kf 滤波器对象指针
 * @param wheel_velocity 轮速计估计的前向速度（m/s）
 * @param accel_imu 经过杆臂补偿及重力扣除后的IMU前向加速度（m/s²）
 * @param wheel_valid 轮速观测是否可信（触地未打滑为true，腾空/打滑为false）
 * @return 融合估计的底盘前向速度（m/s）
 */
float ChassisSpeedKfUpdate(ChassisSpeedKf *speed_kf, float wheel_velocity,
                           float accel_imu, bool wheel_valid)
{
  if (speed_kf == NULL || !speed_kf->initialized || !isfinite(accel_imu) ||
      (wheel_valid && !isfinite(wheel_velocity)))
  {
    return (speed_kf != NULL) ? speed_kf->velocity : 0.0f;
  }

  // 腾空/打滑时极大放大轮速观测方差 R[0]，让速度协方差自由增长并在落地后快速重收敛
  speed_kf->kf.r_data[0] = wheel_valid ? speed_kf->config.vel_r : 1.0e6f;
  float z_vel = wheel_valid
                    ? wheel_velocity
                    : (speed_kf->velocity +
                       speed_kf->acceleration * speed_kf->config.sampling_time);
  float z[2] = {z_vel, accel_imu};
  if (KfStep(&speed_kf->kf, NULL, z) == ARM_MATH_SUCCESS)
  {
    speed_kf->velocity = speed_kf->kf.xhat_data[0];
    speed_kf->acceleration = speed_kf->kf.xhat_data[1];
    speed_kf->accel_bias = speed_kf->kf.xhat_data[2];
    speed_kf->position += speed_kf->velocity * speed_kf->config.sampling_time;
  }
  return speed_kf->velocity;
}

/**
 * @brief 初始化2D平面底盘速度卡尔曼滤波器
 * @param kf2d 2D滤波器对象指针
 * @param config 单轴滤波器配置指针（XY共用相同噪声参数）
 * @return true表示初始化成功
 */
bool ChassisSpeedKf2DInit(ChassisSpeedKf2D *kf2d,
                          const ChassisSpeedKfConfig *config)
{
  if (kf2d == NULL || config == NULL)
  {
    return false;
  }
  memset(kf2d, 0, sizeof(*kf2d));
  if (!ChassisSpeedKfInit(&kf2d->kf_x, config) ||
      !ChassisSpeedKfInit(&kf2d->kf_y, config))
  {
    return false;
  }
  kf2d->initialized = true;
  return true;
}

/**
 * @brief 复位2D平面底盘速度卡尔曼滤波器
 * @param kf2d 2D滤波器对象指针
 */
void ChassisSpeedKf2DReset(ChassisSpeedKf2D *kf2d)
{
  if (kf2d == NULL || !kf2d->initialized)
  {
    return;
  }
  ChassisSpeedKfReset(&kf2d->kf_x);
  ChassisSpeedKfReset(&kf2d->kf_y);
  kf2d->vx = 0.0f;
  kf2d->vy = 0.0f;
  kf2d->ax = 0.0f;
  kf2d->ay = 0.0f;
  kf2d->px = 0.0f;
  kf2d->py = 0.0f;
}

/**
 * @brief 单步更新2D底盘速度滤波器（含小陀螺旋转科里奥利耦合补偿）
 * @param kf2d 2D滤波器对象指针
 * @param wheel_vx 轮速计前向速度（m/s）
 * @param wheel_vy 轮速计横向速度（m/s）
 * @param accel_imu_x 机体系前向净加速度（m/s²）
 * @param accel_imu_y 机体系横向净加速度（m/s²）
 * @param omega_z 车体Z轴角速度（rad/s，逆时针为正）
 * @param wheel_valid 轮速观测是否可信
 */
void ChassisSpeedKf2DUpdate(ChassisSpeedKf2D *kf2d, float wheel_vx,
                            float wheel_vy, float accel_imu_x,
                            float accel_imu_y, float omega_z, bool wheel_valid)
{
  if (kf2d == NULL || !kf2d->initialized || !isfinite(accel_imu_x) ||
      !isfinite(accel_imu_y) || !isfinite(omega_z) ||
      (wheel_valid && (!isfinite(wheel_vx) || !isfinite(wheel_vy))))
  {
    return;
  }

  // 刚体平面运动学：a_imu_x = dv_x/dt - omega_z * v_y, a_imu_y = dv_y/dt + omega_z * v_x
  float ax_kin = accel_imu_x + omega_z * kf2d->vy;
  float ay_kin = accel_imu_y - omega_z * kf2d->vx;
  kf2d->vx = ChassisSpeedKfUpdate(&kf2d->kf_x, wheel_vx, ax_kin, wheel_valid);
  kf2d->vy = ChassisSpeedKfUpdate(&kf2d->kf_y, wheel_vy, ay_kin, wheel_valid);
  kf2d->ax = kf2d->kf_x.acceleration;
  kf2d->ay = kf2d->kf_y.acceleration;
  kf2d->px = kf2d->kf_x.position;
  kf2d->py = kf2d->kf_y.position;
}
