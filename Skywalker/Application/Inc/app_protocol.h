#ifndef APP_PROTOCOL_H
#define APP_PROTOCOL_H

#include <stdint.h>

#pragma pack(push, 1)

typedef struct {
  uint8_t head[2];        // 帧头，固定为AB
  uint8_t mode;           // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
  float q[4];             // 姿态四元数，wxyz顺序
  float yaw;              // 云台yaw角度（rad）
  float yaw_velocity;     // 云台yaw角速度（rad/s）
  float pitch;            // 云台pitch角度（rad）
  float pitch_velocity;   // 云台pitch角速度（rad/s）
  float bullet_speed;     // 弹速（m/s）
  uint16_t bullet_count;  // 累计发弹数
  uint16_t crc16;         // CRC16，低字节在前
} GimbalToVision;

typedef struct {
  uint8_t head[2];           // 帧头，固定为AB
  uint8_t mode;              // 0: 不控制, 1: 控制云台, 2: 控制云台并开火
  float yaw;                 // 视觉期望yaw角度或速度，按业务模式解释
  float yaw_velocity;        // 视觉期望yaw速度（rad/s）
  float yaw_acceleration;    // 视觉期望yaw加速度（rad/s^2）
  float pitch;               // 视觉期望pitch角度（rad）
  float pitch_velocity;      // 视觉期望pitch速度（rad/s）
  float pitch_acceleration;  // 视觉期望pitch加速度（rad/s^2）
  uint16_t crc16;            // CRC16，低字节在前
} VisionToGimbal;

#pragma pack(pop)

extern GimbalToVision g_gimbal_to_vision;
extern VisionToGimbal g_vision_to_gimbal;

void VisionProtocolRxCallback(uint8_t *rx_buffer, uint16_t rx_length);
void VisionProtocolTransmitGimbalState(void);

#endif  // APP_PROTOCOL_H
