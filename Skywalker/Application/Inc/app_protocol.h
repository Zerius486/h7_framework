/**
 * @file app_protocol.h
 * @brief 机器人通信协议定义
 * @attention
 * 本文件与 app_robot.c 平级，按机器人需求配置协议字段和通信接口。
 */

#ifndef APP_PROTOCOL_H
#define APP_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

#include "alg_quaternion.h"
#include "dev_peripheral.h"
#include "stm32h7xx_hal.h"

enum
{
  kVisionProtocolBaudrate = 921600U, // 与 rm_auto_aim 默认串口波特率对齐（bps）
};

#pragma pack(push, 1)

typedef struct
{
  uint8_t head[2];       // 帧头，固定为AB
  uint8_t mode;          // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
  float q[4];            // 姿态四元数，wxyz顺序
  float yaw;             // 云台yaw角度（rad）
  float yaw_velocity;    // 云台yaw角速度（rad/s）
  float pitch;           // 云台pitch角度（rad）
  float pitch_velocity;  // 云台pitch角速度（rad/s）
  float bullet_speed;    // 弹速（m/s）
  uint16_t bullet_count; // 累计发弹数
  uint16_t crc16;        // CRC16，低字节在前
} GimbalToVision;

typedef struct
{
  uint8_t head[2];          // 帧头，固定为AB
  uint8_t mode;             // 0: 不控制, 1: 控制云台, 2: 控制云台并开火
  float yaw;                // 视觉期望yaw角度或速度，按业务模式解释
  float yaw_velocity;       // 视觉期望yaw速度（rad/s）
  float yaw_acceleration;   // 视觉期望yaw加速度（rad/s^2）
  float pitch;              // 视觉期望pitch角度（rad）
  float pitch_velocity;     // 视觉期望pitch速度（rad/s）
  float pitch_acceleration; // 视觉期望pitch加速度（rad/s^2）
  uint8_t fire_rate_hz;     // 视觉建议射频（Hz，0表示停火/默认；下位机保留热量上限裁决权）
  uint16_t crc16;           // CRC16，低字节在前
} VisionToGimbal;

#pragma pack(pop)

// 视觉通信链路诊断统计结构体
typedef struct
{
  uint32_t rx_frame_count;      // 校验通过的接收帧数
  uint32_t crc_error_count;     // CRC16 校验失败次数
  uint32_t invalid_frame_count; // 字段范围/有限值非法次数
  uint32_t tx_frame_count;      // 成功提交发送的帧数
  uint32_t tx_drop_count;       // 串口忙或未就绪导致的发送丢包数
} VisionProtocolStats;

extern GimbalToVision g_gimbal_to_vision;
extern VisionToGimbal g_vision_to_gimbal;
extern PeripheralGuard g_vision_guard;

void VisionProtocolInit(void);
bool VisionProtocolInitWithUart(UART_HandleTypeDef *huart);
void VisionProtocolRxCallback(uint8_t *rx_buffer, uint16_t rx_length);
bool VisionProtocolGetCommand(VisionToGimbal *out);
bool VisionProtocolSampleCommand(VisionToGimbal *out, float max_extrapolate_s);
float VisionProtocolGetFireFrequency(const VisionToGimbal *cmd, float max_frequency_hz);
void VisionProtocolUpdateGimbalState(uint8_t mode, const float q[4], float yaw,
                                     float yaw_velocity, float pitch,
                                     float pitch_velocity, float bullet_speed,
                                     uint16_t bullet_count);
void VisionProtocolUpdateFromFluImu(uint8_t mode, Quaternion q_i2w,
                                    float gyro_y_rad_s, float gyro_z_rad_s,
                                    float bullet_speed, uint16_t bullet_count);
void VisionProtocolTransmitGimbalState(void);
void VisionProtocolGetStats(VisionProtocolStats *out);

#endif // APP_PROTOCOL_H
