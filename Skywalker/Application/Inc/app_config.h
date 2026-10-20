/**
 * @file app_config.h
 * @brief 机器人配置定义
 * @attention
 * 本文件与 app_robot.c、app_protocol.h/.c 平级，按机器人需求定义配置项。
 */

#ifndef APP_CONFIG_H
#define APP_CONFIG_H

#include <stdint.h>

#include "alg_kinetics.h"
#include "stm32h7xx_hal.h"

// 板卡角色枚举
typedef enum
{
  kAppBoardSingle = 0, // 单板控制
  kAppBoardGimbal,     // 双板云台端
  kAppBoardChassis,    // 双板底盘端
} AppBoardRole;

// 控制来源枚举
typedef enum
{
  kAppControlRemote = 0, // 遥控器
  kAppControlHost,       // 上位机
  kAppControlVision,     // 视觉
  kAppControlBoardLink,  // 板间通信
} AppControlSource;

// 机器人角色和默认控制来源配置结构体
typedef struct
{
  AppBoardRole board_role;         // 本板角色
  AppControlSource chassis_source; // 底盘默认控制来源
  AppControlSource gimbal_source;   // 云台默认控制来源
  AppControlSource shoot_source;    // 发射默认控制来源
} AppRobotConfig;

// 任务周期配置结构体
typedef struct
{
  uint32_t manager_period_ms; // 管理任务周期（ms）
  uint32_t gimbal_period_ms;  // 云台任务周期（ms）
  uint32_t chassis_period_ms; // 底盘任务周期（ms）
  uint32_t shoot_period_ms;   // 发射任务周期（ms）
} AppTaskConfig;

// 通信外设配置结构体
typedef struct
{
  UART_HandleTypeDef *remote_huart;       // 遥控器串口
  UART_HandleTypeDef *host_huart;         // 上位机串口
  UART_HandleTypeDef *vision_huart;       // 视觉串口
  UART_HandleTypeDef *referee_rx_huart;   // 裁判系统接收串口
  UART_HandleTypeDef *referee_tx_huart;   // 裁判系统发送串口
  FDCAN_HandleTypeDef *board_link_hfdcan; // 板间通信CAN
} AppCommunicationConfig;

// 电机映射和标定配置结构体
typedef struct
{
  FDCAN_HandleTypeDef *hfdcan; // 电机CAN
  uint32_t tx_id;             // 指令CAN ID，组帧电机填写组ID
  uint32_t rx_id;             // 反馈CAN ID
  float reduction_ratio;      // 电机转速与输出轴转速之比
  float zero_angle;           // 输出轴零位对应的电机角度（rad）
  float direction;            // 电机正向与机构正向的关系，取1或-1
} AppMotorConfig;

// 底盘机械配置结构体
typedef struct
{
  ChassisType type;     // 底盘类型
  ChassisParams params; // 底盘半轮距（m）
  float wheel_radius;  // 驱动轮半径（m）
  float max_speed;     // 底盘平移速度上限（m/s）
  float max_omega;     // 底盘旋转角速度上限（rad/s）
} AppChassisConfig;

// 云台配置结构体，角度相对机构零位
typedef struct
{
  float yaw_min;   // yaw最小角度（rad）
  float yaw_max;   // yaw最大角度（rad）
  float pitch_min; // pitch最小角度（rad）
  float pitch_max; // pitch最大角度（rad）
} AppGimbalConfig;

// 发射机构配置结构体
typedef struct
{
  float friction_omega; // 摩擦轮目标角速度（rad/s）
  float feed_angle;     // 拨弹盘单发转角（rad）
  float max_frequency;  // 最大发射频率（Hz）
} AppShootConfig;

extern const AppRobotConfig g_app_robot_config;
extern const AppTaskConfig g_app_task_config;
extern const AppCommunicationConfig g_app_communication_config;

extern const AppChassisConfig g_app_chassis_config;
extern const AppGimbalConfig g_app_gimbal_config;
extern const AppShootConfig g_app_shoot_config;

void AppConfigInit(void);

#endif // APP_CONFIG_H
