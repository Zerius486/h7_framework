#include "dev_lk_motor.h"

#include <string.h>

/**
 * @brief 限制LK电机电流单位值
 * @param current_units 输入电流单位值
 * @return 限幅后的电流单位值
 */
static int16_t LkMotorClampCurrent(int16_t current_units) {
  const int16_t kMinCurrent = -2000;
  const int16_t kMaxCurrent = 2000;

  if (current_units > kMaxCurrent) {
    return kMaxCurrent;
  }
  if (current_units < kMinCurrent) {
    return kMinCurrent;
  }
  return current_units;
}

/**
 * @brief 获取LK电机反馈CAN ID
 * @param motor LK电机对象指针
 * @return 反馈CAN ID，参数无效时返回0
 */
uint32_t LkMotorFeedbackId(const LkMotor *motor) {
  if (motor == NULL) {
    return 0U;
  }
  return kLkMotorFeedbackBaseId + (uint32_t)motor->state.id;
}

/**
 * @brief 初始化LK电机对象
 * @param motor LK电机对象指针
 * @param id 电机ID，范围1~4
 */
void LkMotorInit(LkMotor *motor, uint8_t id) {
  if (motor == NULL) {
    return;
  }

  memset(motor, 0, sizeof(*motor));
  motor->state.id = id;
  motor->state.is_enabled = true;
}

/**
 * @brief 更新LK电机反馈数据
 * @param motor LK电机对象指针
 * @param rx_data CAN反馈数据，长度8字节
 */
void LkMotorUpdate(LkMotor *motor, const uint8_t rx_data[8]) {
  if (motor == NULL || rx_data == NULL) {
    return;
  }

  motor->state.last_encoder = motor->state.encoder;
  motor->state.encoder =
      (uint16_t)(((uint16_t)rx_data[7] << 8) | (uint16_t)rx_data[6]);

  // 通过编码器跨半圈阈值判断溢出方向，得到连续累计角度。
  int32_t delta =
      (int32_t)motor->state.encoder - (int32_t)motor->state.last_encoder;
  if (delta > 32768) {
    motor->state.total_round--;
  } else if (delta < -32768) {
    motor->state.total_round++;
  }

  int16_t speed_dps =
      (int16_t)(((uint16_t)rx_data[5] << 8) | (uint16_t)rx_data[4]);
  motor->state.current_units =
      (int16_t)(((uint16_t)rx_data[3] << 8) | (uint16_t)rx_data[2]);
  motor->state.temperature = rx_data[1];
  motor->state.angle = (float)motor->state.encoder * kLkMotorEncoderToDegree *
                       kLkMotorDegreeToRadian;
  motor->state.total_angle =
      ((float)motor->state.total_round * 360.0f +
       (float)motor->state.encoder * kLkMotorEncoderToDegree) *
      kLkMotorDegreeToRadian;
  motor->state.omega = (float)speed_dps * kLkMotorDegreeToRadian;
  motor->state.torque =
      (float)motor->state.current_units * kLkMotorCurrentToTorque;
}

/**
 * @brief 设置LK电机目标电流单位值
 * @param motor LK电机对象指针
 * @param current_units 目标电流单位值
 */
void LkMotorSetCurrentUnits(LkMotor *motor, int16_t current_units) {
  if (motor == NULL) {
    return;
  }
  motor->state.given_current_units = LkMotorClampCurrent(current_units);
}

/**
 * @brief 使能LK电机控制输出
 * @param motor LK电机对象指针
 */
void LkMotorEnable(LkMotor *motor) {
  if (motor != NULL) {
    motor->state.is_enabled = true;
  }
}

/**
 * @brief 失能LK电机控制输出并清零目标电流
 * @param motor LK电机对象指针
 */
void LkMotorDisable(LkMotor *motor) {
  if (motor != NULL) {
    motor->state.is_enabled = false;
    motor->state.given_current_units = 0;
  }
}

/**
 * @brief 按电机ID打包LK四电机电流控制帧
 * @param motors LK电机对象指针数组
 * @param motor_count 数组长度
 * @param tx_data 输出数据缓冲区，长度8字节
 */
void LkMotorPackCurrentGroup(LkMotor *const motors[], uint8_t motor_count,
                             uint8_t tx_data[8]) {
  if (tx_data == NULL) {
    return;
  }

  memset(tx_data, 0, 8);
  if (motors == NULL) {
    return;
  }

  for (uint8_t i = 0; i < motor_count; i++) {
    LkMotor *motor = motors[i];
    if (motor == NULL || motor->state.id == 0U ||
        motor->state.id > kLkMotorMaxGroupCount) {
      continue;
    }

    int16_t current =
        motor->state.is_enabled
            ? LkMotorClampCurrent(motor->state.given_current_units)
            : 0;
    uint8_t offset = (uint8_t)((motor->state.id - 1U) * 2U);
    tx_data[offset] = (uint8_t)(current & 0xFF);
    tx_data[offset + 1] = (uint8_t)((uint16_t)current >> 8);
  }
}

/**
 * @brief 直接按顺序打包4路LK电机电流单位值
 * @param current_units 4路电流单位值，顺序对应ID 1~4
 * @param tx_data 输出数据缓冲区，长度8字节
 */
void LkMotorPackCurrentUnits4(const int16_t current_units[4],
                              uint8_t tx_data[8]) {
  if (current_units == NULL || tx_data == NULL) {
    return;
  }

  for (uint8_t i = 0; i < kLkMotorMaxGroupCount; i++) {
    int16_t current = LkMotorClampCurrent(current_units[i]);
    tx_data[i * 2U] = (uint8_t)(current & 0xFF);
    tx_data[i * 2U + 1U] = (uint8_t)((uint16_t)current >> 8);
  }
}
