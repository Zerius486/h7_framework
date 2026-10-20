#include "dev_dji_motor.h"

#include <math.h>
#include <string.h>

#include "alg_conversion.h"

static float DjiMotorCurrentLimitA(DjiMotorType type)
{
  switch (type)
  {
  case kDjiMotorM2006:
    return kDjiM2006MotorCurrentLimitA;
  case kDjiMotorM3508:
    return kDjiM3508MotorCurrentLimitA;
  case kDjiMotorGm6020:
    return kDjiGm6020MotorCurrentLimitA;
  default:
    return 0.0f;
  }
}

static float DjiMotorClamp(float value, float min, float max)
{
  if (value > max)
  {
    return max;
  }
  if (value < min)
  {
    return min;
  }
  return value;
}

static float DjiMotorWrapPi(float angle)
{
  while (angle > (float)M_PI)
  {
    angle -= 2.0f * (float)M_PI;
  }
  while (angle < -(float)M_PI)
  {
    angle += 2.0f * (float)M_PI;
  }
  return angle;
}

/**
 * @brief 计算GM6020位置环目标角度分支
 * @param motor DJI电机对象指针
 * @param position_feedback 连续位置反馈（rad）
 * @param omega_feedback 角速度反馈（rad/s）
 * @return 位置目标角度（rad）
 */
static float DjiMotorPositionReference(const DjiMotor *motor,
                                       float position_feedback,
                                       float omega_feedback)
{
  float candidate_angle =
      position_feedback +
      omega_feedback * motor->state.position_loop.sampling_time;
  float turn_count = roundf((candidate_angle - motor->state.given_angle) /
                            (2.0f * (float)M_PI));
  return motor->state.given_angle + turn_count * 2.0f * (float)M_PI;
}

/**
 * @brief 将DJI电机电流值从单位值转换为安培
 * @param type DJI电机类型
 * @param current_units 电流值（单位值）
 * @return 电流值（安培）
 */
float DjiMotorCurrentUnitsToAmperes(DjiMotorType type, int16_t current_units)
{
  switch (type)
  {
  case kDjiMotorM2006:
    return (float)current_units * kDjiM2006MotorCurrentLimitA /
           kDjiM2006MotorCurrentLimitU;
  case kDjiMotorM3508:
    return (float)current_units * kDjiM3508MotorCurrentLimitA /
           kDjiM3508MotorCurrentLimitU;
  case kDjiMotorGm6020:
    return (float)current_units * kDjiGm6020MotorCurrentLimitA /
           kDjiGm6020MotorCurrentLimitU;
  default:
    return 0.0f;
  }
}

/**
 * @brief 将DJI电机电流值从安培转换为单位值
 * @param type DJI电机类型
 * @param current_a 电流值（安培）
 * @return 电流值（单位值）
 */
int16_t DjiMotorCurrentAmperesToUnits(DjiMotorType type, float current_a)
{
  switch (type)
  {
  case kDjiMotorM2006:
    current_a = DjiMotorClamp(current_a, -kDjiM2006MotorCurrentLimitA,
                              kDjiM2006MotorCurrentLimitA);
    return (int16_t)(current_a * kDjiM2006MotorCurrentLimitU /
                     kDjiM2006MotorCurrentLimitA);
  case kDjiMotorM3508:
    current_a = DjiMotorClamp(current_a, -kDjiM3508MotorCurrentLimitA,
                              kDjiM3508MotorCurrentLimitA);
    return (int16_t)(current_a * kDjiM3508MotorCurrentLimitU /
                     kDjiM3508MotorCurrentLimitA);
  case kDjiMotorGm6020:
    current_a = DjiMotorClamp(current_a, -kDjiGm6020MotorCurrentLimitA,
                              kDjiGm6020MotorCurrentLimitA);
    return (int16_t)(current_a * kDjiGm6020MotorCurrentLimitU /
                     kDjiGm6020MotorCurrentLimitA);
  default:
    return 0;
  }
}

/**
 * @brief 初始化DJI电机对象，配置PID控制器和初始状态
 * @param motor DJI电机结构体指针
 * @param type DJI电机类型
 * @param mode DJI电机控制模式
 * @param speed_pid_config 速度环PID配置结构体指针
 * @param position_pid_config 位置环PID配置结构体指针
 */
void DjiMotorInit(DjiMotor *motor, DjiMotorType type, DjiMotorMode mode,
                  PidConfig *speed_pid_config, PidConfig *position_pid_config)
{
  if (motor == NULL || speed_pid_config == NULL)
  {
    return;
  }

  memset(motor, 0, sizeof(*motor));
  PidInit(&motor->state.speed_loop, speed_pid_config);
  if (position_pid_config != NULL)
  {
    PidInit(&motor->state.position_loop, position_pid_config);
  }

  motor->state.is_enabled = true;
  motor->type = type;
  motor->mode = mode;
}

/**
 * @brief 更新DJI电机状态，从CAN接收数据解析角度、角速度、电流和温度
 * @param motor DJI电机结构体指针
 * @param rx_data 接收数据缓冲区指针
 */
void DjiMotorUpdate(DjiMotor *motor, uint8_t *rx_data)
{
  if (rx_data == NULL || motor == NULL)
  {
    return;
  }

  uint16_t encoder = (uint16_t)(((uint16_t)rx_data[0] << 8) | rx_data[1]);
  int16_t rpm = (int16_t)(((uint16_t)rx_data[2] << 8) | rx_data[3]);
  int16_t current_units = (int16_t)(((uint16_t)rx_data[4] << 8) | rx_data[5]);

  motor->state.angle = EncoderToRadian(encoder);
  motor->state.omega = RpmToRadS((float)rpm);
  if (!motor->state.is_angle_initialized)
  {
    motor->state.continuous_angle = motor->state.angle;
    motor->state.last_angle = motor->state.angle;
    motor->state.is_angle_initialized = true;
  }
  else
  {
    motor->state.continuous_angle +=
        DjiMotorWrapPi(motor->state.angle - motor->state.last_angle);
    motor->state.last_angle = motor->state.angle;
  }
  motor->state.current =
      DjiMotorCurrentUnitsToAmperes(motor->type, current_units);

  if (motor->type == kDjiMotorM3508 || motor->type == kDjiMotorGm6020)
  {
    motor->state.temperature =
        (float)(motor->type == kDjiMotorGm6020 ? rx_data[7] : rx_data[6]);
  }
}

/**
 * @brief 根据控制模式计算DJI电机目标电流（级联PID控制）
 * @param motor DJI电机结构体指针
 */
void DjiMotorCurrentCalculate(DjiMotor *motor)
{
  if (motor == NULL)
  {
    return;
  }
  DjiMotorCurrentCalculateWithFeedback(
      motor, motor->type == kDjiMotorGm6020 ? motor->state.continuous_angle
                                           : motor->state.angle,
      motor->state.omega);
}

/**
 * @brief 根据指定反馈计算目标电流，供反馈滤波链调用
 * @param motor DJI电机结构体指针
 * @param position_feedback 位置反馈，GM6020使用连续角度（rad）
 * @param omega_feedback 角速度反馈（rad/s）
 */
void DjiMotorCurrentCalculateWithFeedback(DjiMotor *motor,
                                          float position_feedback,
                                          float omega_feedback)
{
  if (motor == NULL || !motor->state.is_enabled)
  {
    return;
  }

  switch (motor->mode)
  {
  case kDjiMotorModeSpeed:
  {
    motor->state.given_current = PidCalculate(
        &motor->state.speed_loop, motor->state.given_omega, omega_feedback);
    break;
  }
  case kDjiMotorModePosition:
  {
    float position_reference = motor->state.given_angle;
    if (motor->type == kDjiMotorGm6020)
    {
      position_reference = DjiMotorPositionReference(
          motor, position_feedback, omega_feedback);
    }

    motor->state.given_omega = PidCalculate(
        &motor->state.position_loop, position_reference,
        position_feedback);
    motor->state.given_current = PidCalculate(
        &motor->state.speed_loop, motor->state.given_omega, omega_feedback);
    break;
  }
  default:
  {
    motor->state.given_current = 0.0f;
    break;
  }
  }

  float current_limit = DjiMotorCurrentLimitA(motor->type);
  if (current_limit > 0.0f)
  {
    motor->state.given_current = DjiMotorClamp(motor->state.given_current,
                                               -current_limit, current_limit);
  }
}

/**
 * @brief 按DJI指令组ID封装四路电流，未分配或未使能的通道为零
 * @param motors 电机对象指针数组
 * @param motor_count 电机数量，最多四个
 * @param tx_id 指令组CAN ID
 * @param tx_data 八字节发送缓冲区
 */
void DjiMotorPackCurrentGroup(DjiMotor *const motors[], uint8_t motor_count,
                               uint32_t tx_id, uint8_t tx_data[8])
{
  if (tx_data == NULL)
  {
    return;
  }
  memset(tx_data, 0, 8U);
  if (motors == NULL)
  {
    return;
  }
  for (uint8_t i = 0; i < motor_count && i < 4U; i++)
  {
    DjiMotor *motor = motors[i];
    if (motor == NULL || !motor->state.is_enabled)
    {
      continue;
    }
    uint32_t first_id;
    uint32_t last_id;
    if (motor->type == kDjiMotorGm6020 &&
        (tx_id == 0x1FFU || tx_id == 0x2FFU))
    {
      first_id = tx_id == 0x1FFU ? 0x205U : 0x209U;
      last_id = tx_id == 0x1FFU ? 0x208U : 0x20BU;
    }
    else if ((motor->type == kDjiMotorM2006 ||
              motor->type == kDjiMotorM3508) &&
             (tx_id == 0x200U || tx_id == 0x1FFU))
    {
      first_id = tx_id == 0x200U ? 0x201U : 0x205U;
      last_id = first_id + 3U;
    }
    else
    {
      continue;
    }
    if (motor->state.id < first_id || motor->state.id > last_id)
    {
      continue;
    }
    uint32_t slot = motor->state.id - first_id;
    uint16_t current = (uint16_t)DjiMotorCurrentAmperesToUnits(
        motor->type, motor->state.given_current);
    tx_data[2U * slot] = (uint8_t)(current >> 8);
    tx_data[2U * slot + 1U] = (uint8_t)current;
  }
}
