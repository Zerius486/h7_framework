/**
 * @file app_protocol.c
 * @brief 机器人通信协议实现
 * @attention
 * 本文件与 app_robot.c 平级，按机器人需求配置协议编解码和通信外设。
 */

#include "app_protocol.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "alg_crc.h"
#include "alg_euler.h"
#include "app_config.h"
#include "bsp_stream.h"
#include "bsp_uart.h"
#include "stm32h7xx_hal.h"

static const float kVisionDefaultBulletSpeed = 24.0f;
static const float kVisionBulletSpeedMin = 8.0f;
static const float kVisionBulletSpeedMax = 40.0f;

// 编译期与 rm_auto_aim/include/rm_aim/hal/protocol.hpp 严格字节对齐校验
_Static_assert(sizeof(GimbalToVision) == 43U, "GimbalToVision size mismatch");
_Static_assert(sizeof(VisionToGimbal) == 30U, "VisionToGimbal size mismatch");

GimbalToVision g_gimbal_to_vision = {
    .head = {'A', 'B'},
    .q = {1.0f, 0.0f, 0.0f, 0.0f},
    .bullet_speed = 24.0f,
};

VisionToGimbal g_vision_to_gimbal = {
    .head = {'A', 'B'},
};

PeripheralGuard g_vision_guard = {0};

static UART_HandleTypeDef *s_vision_huart = NULL;
static uint32_t s_last_rx_tick_ms = 0U;
static VisionProtocolStats s_vision_stats = {0};
static uint8_t g_vision_rx_stream[sizeof(VisionToGimbal) * 4U];
static StreamBuffer g_vision_rx_stream_buffer;
static bool g_vision_rx_stream_initialized = false;

static void VisionGuardOfflineHook(void *device_ptr)
{
  (void)device_ptr;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  g_vision_to_gimbal.mode = 0U;
  g_vision_to_gimbal.yaw_velocity = 0.0f;
  g_vision_to_gimbal.yaw_acceleration = 0.0f;
  g_vision_to_gimbal.pitch_velocity = 0.0f;
  g_vision_to_gimbal.pitch_acceleration = 0.0f;
  g_vision_to_gimbal.fire_rate_hz = 0U;
  if (primask == 0U)
  {
    __enable_irq();
  }
}

void VisionProtocolInit(void)
{
  (void)VisionProtocolInitWithUart(g_app_communication_config.vision_huart);
}

/**
 * @brief 手动指定视觉串口，自动对齐 921600 波特率并启动环形 DMA 接收
 * @param huart 视觉通信串口句柄（禁止使用 RS485 串口 USART2/USART3）
 * @return true 串口绑定成功; false 串口为空或为 RS485 串口（仍初始化看门狗）
 */
bool VisionProtocolInitWithUart(UART_HandleTypeDef *huart)
{
  PeripheralOps ops = {
      .on_enable = NULL,
      .on_disable = VisionGuardOfflineHook,
      .on_offline = VisionGuardOfflineHook,
      .on_resync = NULL,
      .on_clear_fault = NULL,
      .on_reconnect_probe = NULL,
  };
  PeripheralGuardInit(&g_vision_guard, "vision", kPeripheralTypeVision,
                      kPeripheralDefaultTimeoutVisionMs, &g_vision_to_gimbal,
                      &ops);
  (void)PeripheralManagerRegister(&g_vision_guard, false);

  StreamBufferInit(&g_vision_rx_stream_buffer, g_vision_rx_stream,
                   sizeof(g_vision_rx_stream));
  g_vision_rx_stream_initialized = true;
  s_last_rx_tick_ms = 0U;
  memset(&s_vision_stats, 0, sizeof(s_vision_stats));

  if (huart == NULL || UartIsRs485(huart))
  {
    s_vision_huart = NULL;
    return false;
  }

  s_vision_huart = huart;
  (void)UartSetBaudrate(huart, kVisionProtocolBaudrate);
  UartInit(huart, VisionProtocolRxCallback);
  return true;
}

static bool VisionProtocolHeaderIsValid(const uint8_t *data)
{
  return data != NULL && data[0] == 'A' && data[1] == 'B';
}

static bool VisionProtocolFrameIsValid(const VisionToGimbal *frame)
{
  if (frame == NULL || frame->mode > 2U || frame->fire_rate_hz > 40U)
  {
    return false;
  }
  float yaw = frame->yaw;
  float yaw_vel = frame->yaw_velocity;
  float yaw_acc = frame->yaw_acceleration;
  float pitch = frame->pitch;
  float pitch_vel = frame->pitch_velocity;
  float pitch_acc = frame->pitch_acceleration;
  return isfinite(yaw) && isfinite(yaw_vel) && isfinite(yaw_acc) &&
         isfinite(pitch) && isfinite(pitch_vel) && isfinite(pitch_acc);
}

void VisionProtocolRxCallback(uint8_t *rx_buffer, uint16_t rx_length)
{
  if (rx_buffer == NULL || rx_length == 0U)
  {
    return;
  }

  if (g_vision_guard.state == kPeripheralStateUninit)
  {
    VisionProtocolInit();
  }

  if (!g_vision_rx_stream_initialized)
  {
    StreamBufferInit(&g_vision_rx_stream_buffer, g_vision_rx_stream,
                     sizeof(g_vision_rx_stream));
    g_vision_rx_stream_initialized = true;
  }
  StreamBufferAppend(&g_vision_rx_stream_buffer, rx_buffer, rx_length);

  uint16_t offset = 0U;
  while ((uint16_t)(g_vision_rx_stream_buffer.length - offset) >=
         sizeof(VisionToGimbal))
  {
    if (!VisionProtocolHeaderIsValid(&g_vision_rx_stream_buffer.buffer[offset]))
    {
      offset++;
      continue;
    }

    if (!CheckCrc16(&g_vision_rx_stream_buffer.buffer[offset],
                    sizeof(VisionToGimbal)))
    {
      s_vision_stats.crc_error_count++;
      offset++;
      continue;
    }

    VisionToGimbal frame;
    memcpy(&frame, &g_vision_rx_stream_buffer.buffer[offset], sizeof(frame));
    offset = (uint16_t)(offset + sizeof(VisionToGimbal));
    if (!VisionProtocolFrameIsValid(&frame))
    {
      s_vision_stats.invalid_frame_count++;
      continue;
    }

    uint32_t now_tick = HAL_GetTick();
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    memcpy(&g_vision_to_gimbal, &frame, sizeof(g_vision_to_gimbal));
    s_last_rx_tick_ms = now_tick;
    s_vision_stats.rx_frame_count++;
    if (primask == 0U)
    {
      __enable_irq();
    }
    PeripheralGuardFeed(&g_vision_guard, now_tick);
  }

  StreamBufferConsume(&g_vision_rx_stream_buffer, offset);
}

/**
 * @brief 获取经看门狗与原子快照保护的视觉控制指令
 * @param out 对齐输出结构体指针
 * @return true 视觉已收到合法首帧且在线有效; false 未就绪/离线/无效（out 已置安全零值）
 */
bool VisionProtocolGetCommand(VisionToGimbal *out)
{
  return VisionProtocolSampleCommand(out, 0.0f);
}

/**
 * @brief 获取视觉控制指令并可选执行 200Hz->1kHz 帧间运动学外推
 * @param out 对齐输出结构体指针
 * @param max_extrapolate_s 最大允许外推时间（s，<=0 关闭外推，推荐 0.015f）
 * @return true 视觉已就绪在线且有效; false 未就绪/离线/无效（out 已置安全零值）
 */
bool VisionProtocolSampleCommand(VisionToGimbal *out, float max_extrapolate_s)
{
  if (out == NULL)
  {
    return false;
  }
  if (!PeripheralIsReady(&g_vision_guard))
  {
    memset(out, 0, sizeof(*out));
    out->head[0] = 'A';
    out->head[1] = 'B';
    return false;
  }

  uint32_t rx_tick = 0U;
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  memcpy(out, &g_vision_to_gimbal, sizeof(*out));
  rx_tick = s_last_rx_tick_ms;
  if (primask == 0U)
  {
    __enable_irq();
  }

  if (!VisionProtocolFrameIsValid(out))
  {
    memset(out, 0, sizeof(*out));
    out->head[0] = 'A';
    out->head[1] = 'B';
    return false;
  }

  if (out->mode != 0U && isfinite(max_extrapolate_s) &&
      max_extrapolate_s > 0.0f)
  {
    int32_t elapsed_ms = (int32_t)(HAL_GetTick() - rx_tick);
    if (elapsed_ms > 0)
    {
      float dt = (float)elapsed_ms * 0.001f;
      if (dt > max_extrapolate_s)
      {
        dt = max_extrapolate_s;
      }
      out->yaw += out->yaw_velocity * dt +
                  0.5f * out->yaw_acceleration * dt * dt;
      out->yaw_velocity += out->yaw_acceleration * dt;
      out->pitch += out->pitch_velocity * dt +
                    0.5f * out->pitch_acceleration * dt * dt;
      out->pitch_velocity += out->pitch_acceleration * dt;
    }
  }
  return true;
}

/**
 * @brief 根据视觉指令与下位机最大允许射频计算最终拨弹频率（Hz）
 * @param cmd 视觉控制指令指针
 * @param max_frequency_hz 下位机当前热量/机构允许的最大射频（Hz）
 * @return 期望拨弹频率（Hz）；非开火模式或无效指令返回 0.0f
 */
float VisionProtocolGetFireFrequency(const VisionToGimbal *cmd, float max_frequency_hz)
{
  if (cmd == NULL || cmd->mode != 2U || !isfinite(max_frequency_hz) || max_frequency_hz <= 0.0f)
  {
    return 0.0f;
  }
  if (cmd->fire_rate_hz == 0U)
  {
    // 上位机未指定射频时，按电控默认最大允许射频执行
    return max_frequency_hz;
  }
  float req = (float)cmd->fire_rate_hz;
  return (req < max_frequency_hz) ? req : max_frequency_hz;
}

/**
 * @brief 原子更新上行云台状态（遵循 FLU 世界系：yaw 左转为正，pitch 抬头为正）
 */
void VisionProtocolUpdateGimbalState(uint8_t mode, const float q[4], float yaw,
                                     float yaw_velocity, float pitch,
                                     float pitch_velocity, float bullet_speed,
                                     uint16_t bullet_count)
{
  float q_norm[4] = {1.0f, 0.0f, 0.0f, 0.0f};
  if (q != NULL && isfinite(q[0]) && isfinite(q[1]) && isfinite(q[2]) &&
      isfinite(q[3]))
  {
    float n2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    if (n2 > 1e-6f)
    {
      float inv_n = 1.0f / sqrtf(n2);
      q_norm[0] = q[0] * inv_n;
      q_norm[1] = q[1] * inv_n;
      q_norm[2] = q[2] * inv_n;
      q_norm[3] = q[3] * inv_n;
    }
  }

  uint8_t safe_mode = (mode <= 3U) ? mode : 0U;
  float safe_yaw = isfinite(yaw) ? yaw : 0.0f;
  float safe_yaw_vel = isfinite(yaw_velocity) ? yaw_velocity : 0.0f;
  float safe_pitch = isfinite(pitch) ? pitch : 0.0f;
  float safe_pitch_vel = isfinite(pitch_velocity) ? pitch_velocity : 0.0f;

  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  g_gimbal_to_vision.head[0] = 'A';
  g_gimbal_to_vision.head[1] = 'B';
  g_gimbal_to_vision.mode = safe_mode;
  memcpy(g_gimbal_to_vision.q, q_norm, sizeof(q_norm));
  g_gimbal_to_vision.yaw = safe_yaw;
  g_gimbal_to_vision.yaw_velocity = safe_yaw_vel;
  g_gimbal_to_vision.pitch = safe_pitch;
  g_gimbal_to_vision.pitch_velocity = safe_pitch_vel;
  if (isfinite(bullet_speed) && bullet_speed > kVisionBulletSpeedMin &&
      bullet_speed < kVisionBulletSpeedMax)
  {
    g_gimbal_to_vision.bullet_speed = bullet_speed;
  }
  else if (!isfinite(g_gimbal_to_vision.bullet_speed) ||
           g_gimbal_to_vision.bullet_speed <= kVisionBulletSpeedMin ||
           g_gimbal_to_vision.bullet_speed >= kVisionBulletSpeedMax)
  {
    g_gimbal_to_vision.bullet_speed = kVisionDefaultBulletSpeed;
  }
  g_gimbal_to_vision.bullet_count = bullet_count;
  if (primask == 0U)
  {
    __enable_irq();
  }
}

/**
 * @brief 直接从 FLU 机体系 IMU 四元数与角速度更新上行状态（自动转为 pitch 抬头为正）
 * @param mode 视觉模式（0: 空闲, 1: 自瞄, 2: 小符, 3: 大符）
 * @param q_i2w IMU 机体到 FLU 世界系四元数 (w, x, y, z)
 * @param gyro_y_rad_s FLU Y 轴（左轴）角速度（rad/s），抬头为负，内部自动取反
 * @param gyro_z_rad_s FLU Z 轴（上轴）角速度（rad/s），左转为正
 * @param bullet_speed 弹速（m/s，非法/未发弹时保持上次有效值或默认 24.0）
 * @param bullet_count 累计发弹数
 */
void VisionProtocolUpdateFromFluImu(uint8_t mode, Quaternion q_i2w,
                                    float gyro_y_rad_s, float gyro_z_rad_s,
                                    float bullet_speed, uint16_t bullet_count)
{
  float q_arr[4] = {q_i2w.w, q_i2w.x, q_i2w.y, q_i2w.z};
  Euler euler = QuaternionToEuler(q_i2w);
  // FLU 下右手 ZYX 欧拉角绕 +Y(左轴) 旋转使枪口下压，故 pitch 抬头为正需取反
  VisionProtocolUpdateGimbalState(mode, q_arr, euler.yaw, gyro_z_rad_s,
                                  -euler.pitch, -gyro_y_rad_s, bullet_speed,
                                  bullet_count);
}

/**
 * @brief 通过配置的视觉串口发送云台状态
 */
void VisionProtocolTransmitGimbalState(void)
{
  UART_HandleTypeDef *huart =
      (s_vision_huart != NULL) ? s_vision_huart
                               : g_app_communication_config.vision_huart;
  if (huart == NULL)
  {
    s_vision_stats.tx_drop_count++;
    return;
  }

  uint8_t tx_buffer[sizeof(GimbalToVision)];
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  g_gimbal_to_vision.head[0] = 'A';
  g_gimbal_to_vision.head[1] = 'B';
  memcpy(tx_buffer, &g_gimbal_to_vision, sizeof(tx_buffer));
  if (primask == 0U)
  {
    __enable_irq();
  }

  uint16_t crc16 =
      GetCrc16(tx_buffer, (uint16_t)(sizeof(tx_buffer) - sizeof(uint16_t)));
  tx_buffer[sizeof(tx_buffer) - 2U] = (uint8_t)(crc16 & 0xFFU);
  tx_buffer[sizeof(tx_buffer) - 1U] = (uint8_t)(crc16 >> 8);
  if (UartTransmitTry(huart, tx_buffer, sizeof(tx_buffer)))
  {
    s_vision_stats.tx_frame_count++;
  }
  else
  {
    s_vision_stats.tx_drop_count++;
  }
}

/**
 * @brief 获取视觉通信链路诊断计数快照
 * @param out 输出统计结构体指针
 */
void VisionProtocolGetStats(VisionProtocolStats *out)
{
  if (out == NULL)
  {
    return;
  }
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  *out = s_vision_stats;
  if (primask == 0U)
  {
    __enable_irq();
  }
}
