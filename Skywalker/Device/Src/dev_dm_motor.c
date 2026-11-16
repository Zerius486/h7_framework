#include "dev_dm_motor.h"

#include <math.h>
#include <string.h>

#include "alg_conversion.h"
#include "stm32h7xx_hal.h"

static bool DmMotorGuardEnableHook(void *device_ptr)
{
  DmMotor *motor = (DmMotor *)device_ptr;
  if (motor == NULL)
  {
    return false;
  }
  motor->state.is_enabled = true;
  return true;
}

static void DmMotorGuardDisableHook(void *device_ptr)
{
  DmMotor *motor = (DmMotor *)device_ptr;
  if (motor == NULL)
  {
    return;
  }
  motor->state.is_enabled = false;
  motor->state.given_torque = 0.0f;
  motor->state.kp = 0.0f;
  motor->state.kd = 0.0f;
}

static void DmMotorGuardResyncHook(void *device_ptr)
{
  DmMotor *motor = (DmMotor *)device_ptr;
  if (motor == NULL)
  {
    return;
  }
  motor->state.is_enabled = false;
  motor->state.given_torque = 0.0f;
  motor->state.kp = 0.0f;
  motor->state.kd = 0.0f;
  DmMotorResetUnwrap(motor);
}

static void DmMotorGuardOfflineHook(void *device_ptr)
{
  DmMotorGuardResyncHook(device_ptr);
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
                                   uint8_t bits)
{
  value = MathClamp(value, min, max);
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
                                uint8_t bits)
{
  float span = max - min;
  uint32_t max_int = (1UL << bits) - 1UL;
  return (float)value * span / (float)max_int + min;
}

/**
 * @brief 按达妙普通模式协议打包float
 * @param value 输入浮点数
 * @param tx_data 输出数据缓冲区，长度至少4字节
 */
static void DmMotorPackFloat(float value, uint8_t tx_data[4])
{
  union
  {
    float value;
    uint8_t bytes[4];
  } converter = {.value = value};

  tx_data[0] = converter.bytes[0];
  tx_data[1] = converter.bytes[1];
  tx_data[2] = converter.bytes[2];
  tx_data[3] = converter.bytes[3];
}

/**
 * @brief 将J4310目标力矩转换为电流单位值
 * @param torque_nm 目标力矩（Nm）
 * @return 电流单位值
 */
float DmMotorTorqueToCurrentUnits(float torque_nm)
{
  float torque = MathClamp(torque_nm, kDmMotorTorqueMin, kDmMotorTorqueMax);
  return torque * kDmMotorCurrentUnitsMax / kDmMotorTorqueMax;
}

/**
 * @brief 将J4310电流单位值转换为估计力矩
 * @param current_units 电流单位值
 * @return 估计力矩（Nm）
 */
float DmMotorCurrentUnitsToTorque(float current_units)
{
  float current = MathClamp(current_units, -kDmMotorCurrentUnitsMax,
                            kDmMotorCurrentUnitsMax);
  return current * kDmMotorTorqueMax / kDmMotorCurrentUnitsMax;
}

/**
 * @brief 获取当前控制模式对应的发送CAN ID
 * @param motor 达妙电机对象指针
 * @return 发送CAN ID，参数无效时返回0
 */
uint32_t DmMotorTxId(const DmMotor *motor)
{
  if (motor == NULL)
  {
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
                 uint32_t id, uint32_t master_id)
{
  if (motor == NULL)
  {
    return;
  }

  memset(motor, 0, sizeof(*motor));
  motor->type = type;
  motor->mode = mode;
  motor->state.id = id;
  motor->state.master_id = master_id;
  motor->state.velocity_limit = kDmForcePositionDefaultVelocityLimit;
  motor->state.current_limit = (uint16_t)kDmMotorCurrentUnitsMax;

  motor->limits.position_max = kDmMotorPositionMax;
  motor->limits.velocity_max = kDmMotorVelocityMax;
  motor->limits.torque_max = kDmMotorTorqueMax;
  if (type == kDmMotorTypeJ8009P)
  {
    motor->limits.position_max = kDmMotorJ8009PPositionMax;
    motor->limits.velocity_max = kDmMotorJ8009PVelocityMax;
    motor->limits.torque_max = kDmMotorJ8009PTorqueMax;
  }

  PeripheralOps ops = {
      .on_enable = DmMotorGuardEnableHook,
      .on_disable = DmMotorGuardDisableHook,
      .on_offline = DmMotorGuardOfflineHook,
      .on_resync = DmMotorGuardResyncHook,
      .on_clear_fault = NULL,
      .on_reconnect_probe = NULL,
  };
  PeripheralGuardInit(&motor->guard, "dm_motor", kPeripheralTypeMotorDm,
                      kPeripheralDefaultTimeoutMotorMs, motor, &ops);
}

/**
 * @brief 重置达妙电机多圈展开状态（离线重连或重设零点后调用）
 * @param motor 达妙电机对象指针
 */
void DmMotorResetUnwrap(DmMotor *motor)
{
  if (motor == NULL)
  {
    return;
  }
  motor->state.wraps = 0;
  motor->state.previous_raw_angle = 0.0f;
  motor->state.continuous_angle = 0.0;
  motor->state.angle_initialized = false;
}

/**
 * @brief 更新达妙电机反馈数据并按协议跨度 (2 * position_max) 做多圈展开
 * @param motor 达妙电机对象指针
 * @param rx_data CAN反馈数据，长度8字节
 */
void DmMotorUpdate(DmMotor *motor, const uint8_t rx_data[8])
{
  if (motor == NULL || rx_data == NULL)
  {
    return;
  }

  uint16_t position =
      (uint16_t)(((uint16_t)rx_data[1] << 8) | (uint16_t)rx_data[2]);
  uint16_t velocity =
      (uint16_t)(((uint16_t)rx_data[3] << 4) | ((uint16_t)rx_data[4] >> 4));
  uint16_t torque =
      (uint16_t)((((uint16_t)rx_data[4] & 0x0FU) << 8) | (uint16_t)rx_data[5]);

  motor->state.error_code = (uint8_t)(rx_data[0] >> 4);
  float raw_angle = DmMotorUintToFloat(position, -motor->limits.position_max,
                                       motor->limits.position_max, 16);
  motor->state.omega = DmMotorUintToFloat(velocity, -motor->limits.velocity_max,
                                          motor->limits.velocity_max, 12);
  motor->state.torque = DmMotorUintToFloat(torque, -motor->limits.torque_max,
                                           motor->limits.torque_max, 12);
  motor->state.temperature_mos = (float)rx_data[6];
  motor->state.temperature_rotor = (float)rx_data[7];

  // 刷新通信心跳
  PeripheralGuardFeed(&motor->guard, HAL_GetTick());

  // 硬件错误码联动状态机（达妙协议：0=失能，1=使能，>=0x08 为过压/欠压/过流/过温等硬件故障）
  if (motor->state.error_code >= 0x08U)
  {
    PeripheralGuardTriggerFault(&motor->guard, motor->state.error_code);
    // 故障帧不更新多圈展开计数，避免坏帧污染 wraps
    return;
  }
  else if (PeripheralIsFault(&motor->guard))
  {
    PeripheralGuardClearFault(&motor->guard);
  }

  float half_span = motor->limits.position_max;
  if (!isfinite(half_span) || half_span <= 0.0f || !isfinite(raw_angle) ||
      raw_angle < -half_span || raw_angle > half_span)
  {
    return;
  }

  motor->state.raw_angle = raw_angle;
  if (!motor->state.angle_initialized)
  {
    motor->state.wraps = 0;
    motor->state.previous_raw_angle = raw_angle;
    motor->state.continuous_angle = (double)raw_angle;
    motor->state.angle_initialized = true;
  }
  else
  {
    double span = 2.0 * (double)half_span;
    double delta = (double)raw_angle - (double)motor->state.previous_raw_angle;
    if (delta < -(double)half_span)
    {
      motor->state.wraps += 1;
    }
    else if (delta > (double)half_span)
    {
      motor->state.wraps -= 1;
    }
    motor->state.continuous_angle =
        (double)raw_angle + (double)motor->state.wraps * span;
    motor->state.previous_raw_angle = raw_angle;
  }
  motor->state.angle = (float)motor->state.continuous_angle;
}

/**
 * @brief 打包达妙特殊控制命令
 * @param motor 达妙电机对象指针
 * @param command 控制命令
 * @param tx_data 输出数据缓冲区，长度8字节
 */
void DmMotorPackCommand(DmMotor *motor, DmMotorCommand command,
                        uint8_t tx_data[8])
{
  if (motor == NULL || tx_data == NULL)
  {
    return;
  }

  memset(tx_data, 0xFF, 7);
  tx_data[7] = (uint8_t)command;

  if (command == kDmMotorCommandEnable)
  {
    (void)PeripheralGuardEnable(&motor->guard);
  }
  else if (command == kDmMotorCommandDisable)
  {
    (void)PeripheralGuardDisable(&motor->guard);
  }
}

/**
 * @brief 打包进入电机模式命令
 * @param motor 达妙电机对象指针
 * @param tx_data 输出数据缓冲区，长度8字节
 */
void DmMotorPackEnable(DmMotor *motor, uint8_t tx_data[8])
{
  DmMotorPackCommand(motor, kDmMotorCommandEnable, tx_data);
}

/**
 * @brief 打包退出电机模式命令
 * @param motor 达妙电机对象指针
 * @param tx_data 输出数据缓冲区，长度8字节
 */
void DmMotorPackDisable(DmMotor *motor, uint8_t tx_data[8])
{
  DmMotorPackCommand(motor, kDmMotorCommandDisable, tx_data);
}

/**
 * @brief 打包保存当前位置为零点命令
 * @param motor 达妙电机对象指针
 * @param tx_data 输出数据缓冲区，长度8字节
 */
void DmMotorPackZeroOffset(DmMotor *motor, uint8_t tx_data[8])
{
  DmMotorPackCommand(motor, kDmMotorCommandZeroOffset, tx_data);
}

/**
 * @brief 打包清除错误命令
 * @param motor 达妙电机对象指针
 * @param tx_data 输出数据缓冲区，长度8字节
 */
void DmMotorPackClearError(DmMotor *motor, uint8_t tx_data[8])
{
  DmMotorPackCommand(motor, kDmMotorCommandClearError, tx_data);
}

/**
 * @brief 打包MIT模式控制帧
 * @param motor 达妙电机对象指针
 * @param tx_data 输出数据缓冲区，长度8字节
 */
void DmMotorPackMit(const DmMotor *motor, uint8_t tx_data[8])
{
  if (motor == NULL || tx_data == NULL)
  {
    return;
  }

  float given_torque = motor->state.given_torque;
  float kp = motor->state.kp;
  float kd = motor->state.kd;
  if (!motor->state.is_enabled || !PeripheralIsEnabled(&motor->guard))
  {
    given_torque = 0.0f;
    kp = 0.0f;
    kd = 0.0f;
  }

  uint16_t position =
      DmMotorFloatToUint(motor->state.given_angle, -motor->limits.position_max,
                         motor->limits.position_max, 16);
  uint16_t velocity =
      DmMotorFloatToUint(motor->state.given_omega, -motor->limits.velocity_max,
                         motor->limits.velocity_max, 12);
  uint16_t kp_uint =
      DmMotorFloatToUint(kp, kDmMotorKpMin, kDmMotorKpMax, 12);
  uint16_t kd_uint =
      DmMotorFloatToUint(kd, kDmMotorKdMin, kDmMotorKdMax, 12);
  uint16_t torque =
      DmMotorFloatToUint(given_torque, -motor->limits.torque_max,
                         motor->limits.torque_max, 12);

  tx_data[0] = (uint8_t)(position >> 8);
  tx_data[1] = (uint8_t)(position & 0xFFU);
  tx_data[2] = (uint8_t)(velocity >> 4);
  tx_data[3] = (uint8_t)(((velocity & 0x0FU) << 4) | (kp_uint >> 8));
  tx_data[4] = (uint8_t)(kp_uint & 0xFFU);
  tx_data[5] = (uint8_t)(kd_uint >> 4);
  tx_data[6] = (uint8_t)(((kd_uint & 0x0FU) << 4) | (torque >> 8));
  tx_data[7] = (uint8_t)(torque & 0xFFU);
}

/**
 * @brief 打包位置速度模式控制帧
 * @param motor 达妙电机对象指针
 * @param tx_data 输出数据缓冲区，长度8字节
 */
void DmMotorPackPositionVelocity(const DmMotor *motor, uint8_t tx_data[8])
{
  if (motor == NULL || tx_data == NULL)
  {
    return;
  }

  float angle = motor->state.given_angle;
  float omega = motor->state.given_omega;
  if (!motor->state.is_enabled || !PeripheralIsEnabled(&motor->guard))
  {
    angle = motor->state.angle;
    omega = 0.0f;
  }

  DmMotorPackFloat(angle, &tx_data[0]);
  DmMotorPackFloat(omega, &tx_data[4]);
}

/**
 * @brief 打包速度模式控制帧
 * @param motor 达妙电机对象指针
 * @param tx_data 输出数据缓冲区，长度8字节
 */
void DmMotorPackVelocity(const DmMotor *motor, uint8_t tx_data[8])
{
  if (motor == NULL || tx_data == NULL)
  {
    return;
  }

  float omega = (!motor->state.is_enabled || !PeripheralIsEnabled(&motor->guard))
                    ? 0.0f
                    : motor->state.given_omega;
  DmMotorPackFloat(omega, &tx_data[0]);
  memset(&tx_data[4], 0, 4);
}

/**
 * @brief 打包力位混控模式控制帧
 * @param motor 达妙电机对象指针
 * @param tx_data 输出数据缓冲区，长度8字节
 */
void DmMotorPackForcePosition(const DmMotor *motor, uint8_t tx_data[8])
{
  if (motor == NULL || tx_data == NULL)
  {
    return;
  }

  float angle = motor->state.given_angle;
  uint16_t velocity_limit = motor->state.velocity_limit;
  uint16_t current_limit = motor->state.current_limit;
  if (!motor->state.is_enabled || !PeripheralIsEnabled(&motor->guard))
  {
    angle = motor->state.angle;
    velocity_limit = 0U;
    current_limit = 0U;
  }

  DmMotorPackFloat(angle, &tx_data[0]);
  tx_data[4] = (uint8_t)(velocity_limit & 0xFFU);
  tx_data[5] = (uint8_t)(velocity_limit >> 8);
  tx_data[6] = (uint8_t)(current_limit & 0xFFU);
  tx_data[7] = (uint8_t)(current_limit >> 8);
}
