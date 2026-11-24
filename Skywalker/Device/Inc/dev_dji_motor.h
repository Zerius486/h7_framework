#ifndef DEV_DJI_MOTOR_H
#define DEV_DJI_MOTOR_H

#include <stdbool.h>
#include <stdint.h>

#include "alg_pid.h"
#include "dev_peripheral.h"

// DJI电机电流值限幅（单位值，原始CAN数据范围）
enum
{
  kDjiM2006MotorCurrentLimitU = 10000,
  kDjiM3508MotorCurrentLimitU = 16384,
  kDjiGm6020MotorCurrentLimitU = 16384,
};

// DJI电机电流值限幅（安培，SI单位制）
// GM6020需通过RoboMaster Assistant开启电流模式。
static const float kDjiM2006MotorCurrentLimitA = 10.0000f;
static const float kDjiM3508MotorCurrentLimitA = 20.0000f;
static const float kDjiGm6020MotorCurrentLimitA = 3.0000f;

// DJI电机状态结构体
typedef struct
{
  bool is_enabled;           // 电机使能标志
  uint32_t id;               // 电机ID
  float angle;               // 当前单圈角度（rad）
  int32_t total_round;       // GM6020多圈计数
  float continuous_angle;    // GM6020当前连续角度（rad）
  float last_angle;          // GM6020上一次单圈角度（rad）
  bool is_angle_initialized; // GM6020连续角度初始化标志
  float given_angle;         // GM6020目标角度（rad）
  float ff_omega;            // GM6020速度前馈（rad/s，可选，默认0）
  float ff_current;          // GM6020电流/动力学前馈（A，可选，默认0）
  float omega;               // 当前角速度（rad/s）
  float given_omega;         // 目标角速度（rad/s）
  float current;             // 当前电流（A）
  float given_current;       // 目标电流（A）
  float temperature;         // 温度（℃）
  PidObject speed_loop;      // 速度环PID控制器
  PidObject position_loop;   // GM6020位置环PID控制器
} DjiMotorState;

// DJI电机类型枚举
typedef enum
{
  kDjiMotorNone = 0, // 无类型
  kDjiMotorM2006,    // M2006电机
  kDjiMotorM3508,    // M3508电机
  kDjiMotorGm6020,   // GM6020电机
} DjiMotorType;

// DJI电机控制模式枚举
typedef enum
{
  kDjiMotorModeSpeed = 1,    // 速度控制模式
  kDjiMotorModePosition = 2, // 位置控制模式
} DjiMotorMode;

// DJI电机对象结构体
typedef struct
{
  DjiMotorState state;   // 电机状态
  DjiMotorType type;     // 电机类型
  DjiMotorMode mode;     // 控制模式
  int8_t direction;      // 电机方向标志（+1 正向，-1 反向）
  PeripheralGuard guard; // 统一外设生命周期守护对象
} DjiMotor;

// API函数声明
float DjiMotorCurrentUnitsToAmperes(DjiMotorType type, int16_t current_units);
int16_t DjiMotorCurrentAmperesToUnits(DjiMotorType type, float current_a);
void DjiMotorInit(DjiMotor *motor, DjiMotorType type, DjiMotorMode mode,
                  PidConfig *speed_pid_config, PidConfig *position_pid_config);
bool DjiMotorEnable(DjiMotor *motor);
void DjiMotorDisable(DjiMotor *motor);
void DjiMotorSetDirection(DjiMotor *motor, int8_t direction);
void DjiMotorSetFeedforward(DjiMotor *motor, float ff_omega, float ff_current);
void DjiMotorUpdate(DjiMotor *motor, uint8_t *rx_data);
void DjiMotorCurrentCalculate(DjiMotor *motor);
void DjiMotorCurrentCalculateWithFeedback(DjiMotor *motor,
                                          float position_feedback,
                                          float omega_feedback);
void DjiMotorPackCurrentGroup(DjiMotor *const motors[], uint8_t motor_count,
                               uint32_t tx_id, uint8_t tx_data[8]);

#endif // DEV_DJI_MOTOR_H
