#ifndef DEV_DJI_MOTOR_H
#define DEV_DJI_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#include "alg_pid.h"

// DJI电机电流值限幅（单位值，原始CAN数据范围）
enum {
  kDjiM2006MotorCurrentLimitU = 10000,
  kDjiM3508MotorCurrentLimitU = 16384,
  kDjiGm6020MotorCurrentLimitU = 16384,
};

// DJI电机电流值限幅（安培，SI单位制）
static const float kDjiM2006MotorCurrentLimitA = 10.0000f;
static const float kDjiM3508MotorCurrentLimitA = 20.0000f;
static const float kDjiGm6020MotorCurrentLimitA = 3.0000f;

// DJI电机状态结构体
typedef struct {
  bool is_enabled;          // 电机使能标志
  uint32_t id;              // 电机ID
  float angle;              // 当前角度（rad）
  float given_angle;        // 目标角度（rad）
  float omega;              // 当前角速度（rad/s）
  float given_omega;        // 目标角速度（rad/s）
  float current;            // 当前电流（A）
  float given_current;      // 目标电流（A）
  float temperature;        // 温度（℃）
  PidObject speed_loop;     // 速度环PID控制器
  PidObject position_loop;  // 位置环PID控制器
} DjiMotorState;

// DJI电机类型枚举
typedef enum {
  kDjiMotorNone = 0,  // 无类型
  kDjiMotorM2006,     // M2006电机
  kDjiMotorM3508,     // M3508电机
  kDjiMotorGm6020,    // GM6020电机
} DjiMotorType;

// DJI电机控制模式枚举
typedef enum {
  kDjiMotorModeSpeed = 1,     // 速度控制模式
  kDjiMotorModePosition = 2,  // 位置控制模式
} DjiMotorMode;

// DJI电机对象结构体
typedef struct {
  DjiMotorState state;  // 电机状态
  DjiMotorType type;    // 电机类型
  DjiMotorMode mode;    // 控制模式
} DjiMotor;

// API函数声明
float DjiMotorCurrentUnitsToAmperes(DjiMotorType type, int16_t current_units);
int16_t DjiMotorCurrentAmperesToUnits(DjiMotorType type, float current_a);
void DjiMotorInit(DjiMotor *motor, DjiMotorType type, DjiMotorMode mode,
                  PidConfig *speed_pid_config, PidConfig *position_pid_config);
void DjiMotorUpdate(DjiMotor *motor, uint8_t *rx_data);
void DjiMotorCurrentCalculate(DjiMotor *motor);

#endif  // DEV_DJI_MOTOR_H
