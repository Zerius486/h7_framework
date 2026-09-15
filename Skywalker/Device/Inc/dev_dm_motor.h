#ifndef DEV_DM_MOTOR_H
#define DEV_DM_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

enum {
  kDmMotorCanDataLength = 8,                    // 达妙CAN数据帧长度
  kDmForcePositionDefaultVelocityLimit = 1200,  // 力位混控默认速度限制
};

// 达妙MIT模式参数范围，应与上位机电机配置保持一致。
static const float kDmMotorPositionMin = -12.5f;
static const float kDmMotorPositionMax = 12.5f;
static const float kDmMotorVelocityMin = -30.0f;
static const float kDmMotorVelocityMax = 30.0f;
static const float kDmMotorKpMin = 0.0f;
static const float kDmMotorKpMax = 500.0f;
static const float kDmMotorKdMin = 0.0f;
static const float kDmMotorKdMax = 5.0f;
static const float kDmMotorTorqueMin = -10.0f;
static const float kDmMotorTorqueMax = 10.0f;
static const float kDmMotorCurrentUnitsMax = 10000.0f;

// 达妙电机类型枚举
typedef enum {
  kDmMotorTypeNone = 0,  // 未指定型号
  kDmMotorTypeJ4310,     // J4310系列
} DmMotorType;

// 达妙电机控制模式枚举，数值为发送CAN ID偏移量。
typedef enum {
  kDmMotorModeMit = 0x000,               // MIT模式
  kDmMotorModePositionVelocity = 0x100,  // 位置速度模式
  kDmMotorModeVelocity = 0x200,          // 速度模式
  kDmMotorModeForcePosition = 0x300,     // 力位混控模式
} DmMotorMode;

// 达妙电机特殊控制命令
typedef enum {
  kDmMotorCommandEnable = 0xFC,      // 进入电机模式
  kDmMotorCommandDisable = 0xFD,     // 退出电机模式
  kDmMotorCommandZeroOffset = 0xFE,  // 保存当前位置为零点
  kDmMotorCommandClearError = 0xFB,  // 清除电机错误
} DmMotorCommand;

// 达妙电机状态结构体
typedef struct {
  bool is_enabled;     // 电机使能标志
  uint32_t id;         // 电机发送ID
  uint32_t master_id;  // 电机反馈ID
  uint8_t error_code;  // 电机错误码

  float angle;              // 当前角度（rad）
  float omega;              // 当前角速度（rad/s）
  float torque;             // 当前力矩（Nm）
  float temperature_mos;    // MOS温度（℃）
  float temperature_rotor;  // 转子温度（℃）

  float given_angle;   // 目标角度（rad）
  float given_omega;   // 目标角速度（rad/s）
  float given_torque;  // 目标力矩（Nm）
  float kp;            // MIT模式比例增益
  float kd;            // MIT模式微分增益

  uint16_t velocity_limit;  // 力位混控速度限制
  uint16_t current_limit;   // 力位混控电流限制
} DmMotorState;

// 达妙电机对象结构体
typedef struct {
  DmMotorState state;  // 电机状态
  DmMotorType type;    // 电机类型
  DmMotorMode mode;    // 控制模式
} DmMotor;

// API函数声明
float DmMotorTorqueToCurrentUnits(float torque_nm);
float DmMotorCurrentUnitsToTorque(float current_units);
uint32_t DmMotorTxId(const DmMotor *motor);
void DmMotorInit(DmMotor *motor, DmMotorType type, DmMotorMode mode,
                 uint32_t id, uint32_t master_id);
void DmMotorUpdate(DmMotor *motor, const uint8_t rx_data[8]);
void DmMotorPackCommand(DmMotor *motor, DmMotorCommand command,
                        uint8_t tx_data[8]);
void DmMotorPackEnable(DmMotor *motor, uint8_t tx_data[8]);
void DmMotorPackDisable(DmMotor *motor, uint8_t tx_data[8]);
void DmMotorPackZeroOffset(DmMotor *motor, uint8_t tx_data[8]);
void DmMotorPackClearError(DmMotor *motor, uint8_t tx_data[8]);
void DmMotorPackMit(const DmMotor *motor, uint8_t tx_data[8]);
void DmMotorPackPositionVelocity(const DmMotor *motor, uint8_t tx_data[8]);
void DmMotorPackVelocity(const DmMotor *motor, uint8_t tx_data[8]);
void DmMotorPackForcePosition(const DmMotor *motor, uint8_t tx_data[8]);

#endif  // DEV_DM_MOTOR_H
