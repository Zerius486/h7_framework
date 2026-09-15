#include "dev_dm_motor.h"

#include <string.h>

/**
 * @brief 将浮点数限制在指定范围内
 * @param value 输入值
 * @param min 最小值
 * @param max 最大值
 * @return 限幅后的数值
 */
static float DmMotorClamp(float value, float min, float max) {
  if (value > max) {
    return max;
  }
  if (value < min) {
    return min;
  }
  return value;
}

/**
 * @brief 将浮点物理量映射为指定bit宽度的无符号整数
 * @param value 输入物理量
 * @param min 物理量下限
 * @param max 物理量上限
 * @param bits 目标整数位宽
 * @return 映射后的整数值
 */
static uint16_t DmMotorFloatToUint(float value, float min, float max,
                                   uint8_t bits) {
  value = DmMotorClamp(value, min, max);
  float span = max - min;
  uint32_t max_int = (1UL << bits) - 1UL;
  return (uint16_t)((value - min) * (float)max_int / span);
}

/**
 * @brief 将无符号整数反映射为浮点物理量
 * @param value 输入整数值
 * @param min 物理量下限
 * @param max 物理量上限
 * @param bits 输入整数位宽
 * @return 反映射后的物理量
 */
static float DmMotorUintToFloat(uint16_t value, float min, float max,
                                uint8_t bits) {
  float span = max - min;
  uint32_t max_int = (1UL << bits) - 1UL;
  return (float)value * span / (float)max_int + min;
}

/**
 * @brief 按达妙普通模式协议打包float
 * @param value 输入浮点数
 * @param tx_data 输出数据缓冲区，长度至少4字节
 */
static void DmMotorPackFloat(float value, uint8_t tx_data[4]) {
  union {
    float value;
    uint8_t bytes[4];
  } converter = {.value = value};

  tx_data[0] = converter.bytes[0];
  tx_data[1] = converter.bytes[1];
  tx_data[2] = converter.bytes[2];
  tx_data[3] = converter.bytes[3];
}

/**
 * @brief 将目标力矩转换为达妙电流单位值
 * @param torque_nm 目标力矩（Nm）
 * @return 电流单位值
 */
float DmMotorTorqueToCurrentUnits(float torque_nm) {
  float torque = DmMotorClamp(torque_nm, kDmMotorTorqueMin, kDmMotorTorqueMax);
  return torque * kDmMotorCurrentUnitsMax / kDmMotorTorqueMax;
}

/**
 * @brief 将达妙电流单位值转换为估计力矩
 * @param current_units 电流单位值
 * @return 估计力矩（Nm）
 */
float DmMotorCurrentUnitsToTorque(float current_units) {
  float current = DmMotorClamp(current_units, -kDmMotorCurrentUnitsMax,
                               kDmMotorCurrentUnitsMax);
  return current * kDmMotorTorqueMax / kDmMotorCurrentUnitsMax;
}

/**
 * @brief 获取当前控制模式对应的发送CAN ID
 * @param motor 达妙电机对象指针
 * @return 发送CAN ID，参数无效时返回0
 */
uint32_t DmMotorTxId(const DmMotor *motor) {
  if (motor == NULL) {
    return 0U;
  }
  return motor->state.id + (uint32_t)motor->mode;
}

/**
 * @brief 初始化达妙电机对象
 * @param motor 达妙电机对象指针
 * @param type 电机类型
 * @param mode 控制模式
 * @param id 电机发送ID
 * @param master_id 电机反馈ID
 */
void DmMotorInit(DmMotor *motor, DmMotorType type, DmMotorMode mode,
                 uint32_t id, uint32_t master_id) {
  if (motor == NULL) {
    return;
  }

  memset(motor, 0, sizeof(*motor));
  motor->type = type;
  motor->mode = mode;
  motor->state.id = id;
  motor->state.master_id = master_id;
  motor->state.velocity_limit = kDmForcePositionDefaultVelocityLimit;
  motor->state.current_limit = (uint16_t)kDmMotorCurrentUnitsMax;
}

/**
 * @brief 更新达妙电机反馈数据
 * @param motor 达妙电机对象指针
 * @param rx_data CAN反馈数据，长度8字节
 */
void DmMotorUpdate(DmMotor *motor, const uint8_t rx_data[8]) {
  if (motor == NULL || rx_data == NULL) {
    return;
  }

  uint16_t position =
      (uint16_t)(((uint16_t)rx_data[1] << 8) | (uint16_t)rx_data[2]);
  uint16_t velocity =
      (uint16_t)(((uint16_t)rx_data[3] << 4) | ((uint16_t)rx_data[4] >> 4));
  uint16_t torque =
      (uint16_t)((((uint16_t)rx_data[4] & 0x0FU) << 8) | (uint16_t)rx_data[5]);

  motor->state.id = (uint32_t)(rx_data[0] & 0x0FU);
  motor->state.error_code = (uint8_t)(rx_data[0] >> 4);
  motor->state.angle = DmMotorUintToFloat(position, kDmMotorPositionMin,
                                          kDmMotorPositionMax, 16);
  motor->state.omega = DmMotorUintToFloat(velocity, kDmMotorVelocityMin,
                                          kDmMotorVelocityMax, 12);
  motor->state.torque =
      DmMotorUintToFloat(torque, kDmMotorTorqueMin, kDmMotorTorqueMax, 12);
  motor->state.temperature_mos = (float)rx_data[6];
  motor->state.temperature_rotor = (float)rx_data[7];
}

/**
 * @brief 打包达妙特殊控制命令
 * @param motor 达妙电机对象指针
 * @param command 控制命令
 * @param tx_data 输出数据缓冲区，长度8字节
 */
void DmMotorPackCommand(DmMotor *motor, DmMotorCommand command,
                        uint8_t tx_data[8]) {
  if (motor == NULL || tx_data == NULL) {
    return;
  }

  memset(tx_data, 0xFF, 7);
  tx_data[7] = (uint8_t)command;

  if (command == kDmMotorCommandEnable) {
    motor->state.is_enabled = true;
  } else if (command == kDmMotorCommandDisable) {
    motor->state.is_enabled = false;
  }
}

/**
 * @brief 打包进入电机模式命令
 * @param motor 达妙电机对象指针
 * @param tx_data 输出数据缓冲区，长度8字节
 */
void DmMotorPackEnable(DmMotor *motor, uint8_t tx_data[8]) {
  DmMotorPackCommand(motor, kDmMotorCommandEnable, tx_data);
}

/**
 * @brief 打包退出电机模式命令
 * @param motor 达妙电机对象指针
 * @param tx_data 输出数据缓冲区，长度8字节
 */
void DmMotorPackDisable(DmMotor *motor, uint8_t tx_data[8]) {
  DmMotorPackCommand(motor, kDmMotorCommandDisable, tx_data);
}

/**
 * @brief 打包保存当前位置为零点命令
 * @param motor 达妙电机对象指针
 * @param tx_data 输出数据缓冲区，长度8字节
 */
void DmMotorPackZeroOffset(DmMotor *motor, uint8_t tx_data[8]) {
  DmMotorPackCommand(motor, kDmMotorCommandZeroOffset, tx_data);
}

/**
 * @brief 打包清除错误命令
 * @param motor 达妙电机对象指针
 * @param tx_data 输出数据缓冲区，长度8字节
 */
void DmMotorPackClearError(DmMotor *motor, uint8_t tx_data[8]) {
  DmMotorPackCommand(motor, kDmMotorCommandClearError, tx_data);
}

/**
 * @brief 打包MIT模式控制帧
 * @param motor 达妙电机对象指针
 * @param tx_data 输出数据缓冲区，长度8字节
 */
void DmMotorPackMit(const DmMotor *motor, uint8_t tx_data[8]) {
  if (motor == NULL || tx_data == NULL) {
    return;
  }

  uint16_t position = DmMotorFloatToUint(
      motor->state.given_angle, kDmMotorPositionMin, kDmMotorPositionMax, 16);
  uint16_t velocity = DmMotorFloatToUint(
      motor->state.given_omega, kDmMotorVelocityMin, kDmMotorVelocityMax, 12);
  uint16_t kp =
      DmMotorFloatToUint(motor->state.kp, kDmMotorKpMin, kDmMotorKpMax, 12);
  uint16_t kd =
      DmMotorFloatToUint(motor->state.kd, kDmMotorKdMin, kDmMotorKdMax, 12);
  uint16_t torque = DmMotorFloatToUint(
      motor->state.given_torque, kDmMotorTorqueMin, kDmMotorTorqueMax, 12);

  tx_data[0] = (uint8_t)(position >> 8);
  tx_data[1] = (uint8_t)(position & 0xFFU);
  tx_data[2] = (uint8_t)(velocity >> 4);
  tx_data[3] = (uint8_t)(((velocity & 0x0FU) << 4) | (kp >> 8));
  tx_data[4] = (uint8_t)(kp & 0xFFU);
  tx_data[5] = (uint8_t)(kd >> 4);
  tx_data[6] = (uint8_t)(((kd & 0x0FU) << 4) | (torque >> 8));
  tx_data[7] = (uint8_t)(torque & 0xFFU);
}

/**
 * @brief 打包位置速度模式控制帧
 * @param motor 达妙电机对象指针
 * @param tx_data 输出数据缓冲区，长度8字节
 */
void DmMotorPackPositionVelocity(const DmMotor *motor, uint8_t tx_data[8]) {
  if (motor == NULL || tx_data == NULL) {
    return;
  }

  DmMotorPackFloat(motor->state.given_angle, &tx_data[0]);
  DmMotorPackFloat(motor->state.given_omega, &tx_data[4]);
}

/**
 * @brief 打包速度模式控制帧
 * @param motor 达妙电机对象指针
 * @param tx_data 输出数据缓冲区，长度8字节
 */
void DmMotorPackVelocity(const DmMotor *motor, uint8_t tx_data[8]) {
  if (motor == NULL || tx_data == NULL) {
    return;
  }

  DmMotorPackFloat(motor->state.given_omega, &tx_data[0]);
  memset(&tx_data[4], 0, 4);
}

/**
 * @brief 打包力位混控模式控制帧
 * @param motor 达妙电机对象指针
 * @param tx_data 输出数据缓冲区，长度8字节
 */
void DmMotorPackForcePosition(const DmMotor *motor, uint8_t tx_data[8]) {
  if (motor == NULL || tx_data == NULL) {
    return;
  }

  DmMotorPackFloat(motor->state.given_angle, &tx_data[0]);
  tx_data[4] = (uint8_t)(motor->state.velocity_limit & 0xFFU);
  tx_data[5] = (uint8_t)(motor->state.velocity_limit >> 8);
  tx_data[6] = (uint8_t)(motor->state.current_limit & 0xFFU);
  tx_data[7] = (uint8_t)(motor->state.current_limit >> 8);
}
