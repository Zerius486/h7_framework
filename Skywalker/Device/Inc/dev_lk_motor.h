#ifndef DEV_LK_MOTOR_H
#define DEV_LK_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

enum {
  kLkMotorMaxGroupCount = 4,  // LK多电机电流指令最多控制4个电机
  kLkMotorCurrentCommandId = 0x280,  // LK多电机电流控制帧ID
  kLkMotorFeedbackBaseId = 0x140,    // LK电机反馈帧基础ID
};

// LK电机反馈数据换算系数
static const float kLkMotorEncoderToDegree = 360.0f / 65536.0f;
static const float kLkMotorDegreeToRadian = 0.01745329251994329577f;
static const float kLkMotorCurrentToTorque = 0.003645f;

// LK电机状态结构体
typedef struct {
  bool is_enabled;              // 电机使能标志
  uint8_t id;                   // 电机ID，范围1~4
  uint16_t encoder;             // 当前编码器值
  uint16_t last_encoder;        // 上一次编码器值
  int32_t total_round;          // 累计圈数
  float angle;                  // 单圈角度（rad）
  float total_angle;            // 累计角度（rad）
  float omega;                  // 当前角速度（rad/s）
  int16_t current_units;        // 当前电流单位值
  float torque;                 // 估计力矩（Nm）
  uint8_t temperature;          // 电机温度（℃）
  int16_t given_current_units;  // 目标电流单位值
} LkMotorState;

// LK电机对象结构体
typedef struct {
  LkMotorState state;  // 电机状态
} LkMotor;

// API函数声明
uint32_t LkMotorFeedbackId(const LkMotor *motor);
void LkMotorInit(LkMotor *motor, uint8_t id);
void LkMotorUpdate(LkMotor *motor, const uint8_t rx_data[8]);
void LkMotorSetCurrentUnits(LkMotor *motor, int16_t current_units);
void LkMotorEnable(LkMotor *motor);
void LkMotorDisable(LkMotor *motor);
void LkMotorPackCurrentGroup(LkMotor *const motors[], uint8_t motor_count,
                             uint8_t tx_data[8]);
void LkMotorPackCurrentUnits4(const int16_t current_units[4],
                              uint8_t tx_data[8]);

#endif  // DEV_LK_MOTOR_H
