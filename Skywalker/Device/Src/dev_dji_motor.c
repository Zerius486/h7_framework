#include "dev_dji_motor.h"

#include <math.h>
#include <string.h>

#include "alg_conversion.h"

static float DjiMotorCurrentLimitA(DjiMotorType type) {
  switch (type) {
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

static float DjiMotorClamp(float value, float min, float max) {
  if (value > max) {
    return max;
  }
  if (value < min) {
    return min;
  }
  return value;
}

static float DjiMotorWrapPi(float angle) {
  while (angle > (float)M_PI) {
    angle -= 2.0f * (float)M_PI;
  }
  while (angle < -(float)M_PI) {
    angle += 2.0f * (float)M_PI;
  }
  return angle;
}

/**
 * @brief 将DJI电机电流值从单位值转换为安培
 * @param type DJI电机类型
 * @param current_units 电流值（单位值）
 * @return 电流值（安培）
 */
float DjiMotorCurrentUnitsToAmperes(DjiMotorType type, int16_t current_units) {
  switch (type) {
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
int16_t DjiMotorCurrentAmperesToUnits(DjiMotorType type, float current_a) {
  switch (type) {
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
                  PidConfig *speed_pid_config, PidConfig *position_pid_config) {
  if (motor == NULL || speed_pid_config == NULL) {
    return;
  }

  memset(motor, 0, sizeof(*motor));
  PidInit(&motor->state.speed_loop, speed_pid_config);
  if (position_pid_config != NULL) {
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
void DjiMotorUpdate(DjiMotor *motor, uint8_t *rx_data) {
  if (rx_data == NULL || motor == NULL) {
    return;
  }

  uint16_t encoder = (uint16_t)(((uint16_t)rx_data[0] << 8) | rx_data[1]);
  int16_t rpm = (int16_t)(((uint16_t)rx_data[2] << 8) | rx_data[3]);
  int16_t current_units = (int16_t)(((uint16_t)rx_data[4] << 8) | rx_data[5]);

  motor->state.angle = EncoderToRadian(encoder);
  motor->state.omega = RpmToRadS((float)rpm);
  motor->state.current =
      DjiMotorCurrentUnitsToAmperes(motor->type, current_units);

  if (motor->type == kDjiMotorM3508 || motor->type == kDjiMotorGm6020) {
    motor->state.temperature = (float)rx_data[6];
  }
}

/**
 * @brief 根据控制模式计算DJI电机目标电流（级联PID控制）
 * @param motor DJI电机结构体指针
 */
void DjiMotorCurrentCalculate(DjiMotor *motor) {
  if (motor == NULL || !motor->state.is_enabled) {
    return;
  }

  switch (motor->mode) {
    case kDjiMotorModeSpeed: {
      motor->state.given_current =
          PidCalculate(&motor->state.speed_loop, motor->state.given_omega,
                       motor->state.omega);
      break;
    }
    case kDjiMotorModePosition: {
      float position_reference = motor->state.given_angle;
      if (motor->type == kDjiMotorGm6020) {
        position_reference =
            motor->state.angle +
            DjiMotorWrapPi(motor->state.given_angle - motor->state.angle);
      }

      motor->state.given_omega = PidCalculate(
          &motor->state.position_loop, position_reference, motor->state.angle);
      motor->state.given_current =
          PidCalculate(&motor->state.speed_loop, motor->state.given_omega,
                       motor->state.omega);
      break;
    }
    default: {
      motor->state.given_current = 0.0f;
      break;
    }
  }

  float current_limit = DjiMotorCurrentLimitA(motor->type);
  if (current_limit > 0.0f) {
    motor->state.given_current = DjiMotorClamp(motor->state.given_current,
                                               -current_limit, current_limit);
  }
}
