#include "dev_lk_motor.h"

#include <string.h>
#include "stm32h7xx_hal.h"

static bool LkMotorGuardEnableHook(void *device_ptr)
{
  LkMotor *motor = (LkMotor *)device_ptr;
  if (motor == NULL)
  {
    return false;
  }
  motor->state.is_enabled = true;
  return true;
}

static void LkMotorGuardDisableHook(void *device_ptr)
{
  LkMotor *motor = (LkMotor *)device_ptr;
  if (motor == NULL)
  {
    return;
  }
  motor->state.is_enabled = false;
  motor->state.given_current_units = 0;
}

static void LkMotorGuardResyncHook(void *device_ptr)
{
  LkMotor *motor = (LkMotor *)device_ptr;
  if (motor == NULL)
  {
    return;
  }
  // 离线恢复防暴冲：重置多圈累计初始化标志与圈数计数
  motor->state.is_encoder_initialized = false;
  motor->state.total_round = 0;
  motor->state.is_enabled = false;
  motor->state.given_current_units = 0;
}

/**
 * @brief 限制LK电机电流单位值
 * @param current_units 输入电流单位值
 * @return 限幅后的电流单位值
 */
static int16_t LkMotorClampCurrent(int16_t current_units)
{
  const int16_t kMinCurrent = -2000;
  const int16_t kMaxCurrent = 2000;

  if (current_units > kMaxCurrent)
  {
    return kMaxCurrent;
  }
  if (current_units < kMinCurrent)
  {
    return kMinCurrent;
  }
  return current_units;
}

/**
 * @brief 获取LK电机反馈CAN ID
 * @param motor LK电机对象指针
 * @return 反馈CAN ID，参数无效时返回0
 */
uint32_t LkMotorFeedbackId(const LkMotor *motor)
{
  if (motor == NULL)
  {
    return 0U;
  }
  return kLkMotorFeedbackBaseId + (uint32_t)motor->state.id;
}

/**
 * @brief 初始化LK电机对象
 * @param motor LK电机对象指针
 * @param id 电机ID，范围1~4
 */
void LkMotorInit(LkMotor *motor, uint8_t id)
{
  if (motor == NULL)
  {
    return;
  }

  memset(motor, 0, sizeof(*motor));
  motor->state.id = id;
  motor->state.is_enabled = false;
  motor->direction = 1;

  PeripheralOps ops = {
      .on_enable = LkMotorGuardEnableHook,
      .on_disable = LkMotorGuardDisableHook,
      .on_offline = LkMotorGuardDisableHook,
      .on_resync = LkMotorGuardResyncHook,
      .on_clear_fault = NULL,
      .on_reconnect_probe = NULL,
  };
  PeripheralGuardInit(&motor->guard, "lk_motor", kPeripheralTypeMotorLk,
                      kPeripheralDefaultTimeoutMotorMs, motor, &ops);
}

/**
 * @brief 设置LK电机方向标志（+1 正向，-1 反向）
 * @param motor LK电机对象指针
 * @param direction 方向标志（>=0 视为 +1，<0 视为 -1）
 */
void LkMotorSetDirection(LkMotor *motor, int8_t direction)
{
  if (motor == NULL)
  {
    return;
  }
  int8_t normalized = (direction < 0) ? -1 : 1;
  if (motor->direction != normalized)
  {
    motor->direction = normalized;
    motor->state.is_encoder_initialized = false;
    motor->state.total_round = 0;
  }
}

/**
 * @brief 更新LK电机反馈数据
 * @param motor LK电机对象指针
 * @param rx_data CAN反馈数据，长度8字节
 */
void LkMotorUpdate(LkMotor *motor, const uint8_t rx_data[8])
{
  if (motor == NULL || rx_data == NULL)
  {
    return;
  }

  // 先刷新外设心跳喂狗，若刚从离线态恢复则先触发 on_resync 重置多圈展开基准
  PeripheralGuardFeed(&motor->guard, HAL_GetTick());

  // 仅解析电机实时状态反馈帧（0x9C 读取状态2，0xA1~0xA8 闭环控制回复）
  if (rx_data[0] != 0x9CU && (rx_data[0] < 0xA1U || rx_data[0] > 0xA8U))
  {
    return;
  }

  int8_t dir = (motor->direction < 0) ? -1 : 1;
  float dir_f = (float)dir;
  uint16_t raw_encoder =
      (uint16_t)(((uint16_t)rx_data[7] << 8) | (uint16_t)rx_data[6]);

  motor->state.last_encoder = motor->state.encoder;
  motor->state.encoder =
      (dir < 0) ? (uint16_t)(65536U - (uint32_t)raw_encoder) : raw_encoder;

  // 通过编码器跨半圈阈值判断溢出方向，得到连续累计角度。
  if (!motor->state.is_encoder_initialized)
  {
    motor->state.last_encoder = motor->state.encoder;
    motor->state.is_encoder_initialized = true;
  }
  else
  {
    int32_t delta =
        (int32_t)motor->state.encoder - (int32_t)motor->state.last_encoder;
    if (delta > 32768)
    {
      motor->state.total_round--;
    }
    else if (delta < -32768)
    {
      motor->state.total_round++;
    }
    motor->state.last_encoder = motor->state.encoder;
  }

  int16_t speed_dps =
      (int16_t)(((uint16_t)rx_data[5] << 8) | (uint16_t)rx_data[4]);
  int16_t raw_current_units =
      (int16_t)(((uint16_t)rx_data[3] << 8) | (uint16_t)rx_data[2]);
  motor->state.current_units =
      (int16_t)((int32_t)dir * (int32_t)LkMotorClampCurrent(raw_current_units));
  motor->state.temperature = rx_data[1];
  motor->state.angle = (float)motor->state.encoder * kLkMotorEncoderToDegree *
                       kLkMotorDegreeToRadian;
  motor->state.total_angle =
      ((float)motor->state.total_round * 360.0f +
       (float)motor->state.encoder * kLkMotorEncoderToDegree) *
      kLkMotorDegreeToRadian;
  motor->state.omega = dir_f * (float)speed_dps * kLkMotorDegreeToRadian;
  motor->state.torque =
      (float)motor->state.current_units * kLkMotorCurrentToTorque;
}

/**
 * @brief 设置LK电机目标电流单位值
 * @param motor LK电机对象指针
 * @param current_units 目标电流单位值
 */
void LkMotorSetCurrentUnits(LkMotor *motor, int16_t current_units)
{
  if (motor == NULL)
  {
    return;
  }
  motor->state.given_current_units = LkMotorClampCurrent(current_units);
}

/**
 * @brief 使能LK电机控制输出
 * @param motor LK电机对象指针
 */
void LkMotorEnable(LkMotor *motor)
{
  if (motor != NULL)
  {
    (void)PeripheralGuardEnable(&motor->guard);
  }
}

/**
 * @brief 失能LK电机控制输出并清零目标电流
 * @param motor LK电机对象指针
 */
void LkMotorDisable(LkMotor *motor)
{
  if (motor != NULL)
  {
    (void)PeripheralGuardDisable(&motor->guard);
  }
}

/**
 * @brief 按电机ID打包LK四电机电流控制帧
 * @param motors LK电机对象指针数组
 * @param motor_count 数组长度
 * @param tx_data 输出数据缓冲区，长度8字节
 */
void LkMotorPackCurrentGroup(LkMotor *const motors[], uint8_t motor_count,
                             uint8_t tx_data[8])
{
  if (tx_data == NULL)
  {
    return;
  }

  memset(tx_data, 0, 8);
  if (motors == NULL)
  {
    return;
  }

  for (uint8_t i = 0; i < motor_count; i++)
  {
    LkMotor *motor = motors[i];
    if (motor == NULL || motor->state.id == 0U ||
        motor->state.id > kLkMotorMaxGroupCount)
    {
      continue;
    }

    int8_t dir = (motor->direction < 0) ? -1 : 1;
    int16_t current =
        (motor->state.is_enabled && PeripheralIsEnabled(&motor->guard))
            ? (int16_t)(dir * LkMotorClampCurrent(motor->state.given_current_units))
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
                              uint8_t tx_data[8])
{
  if (current_units == NULL || tx_data == NULL)
  {
    return;
  }

  for (uint8_t i = 0; i < kLkMotorMaxGroupCount; i++)
  {
    int16_t current = LkMotorClampCurrent(current_units[i]);
    tx_data[i * 2U] = (uint8_t)(current & 0xFF);
    tx_data[i * 2U + 1U] = (uint8_t)((uint16_t)current >> 8);
  }
}
