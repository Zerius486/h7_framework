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
#include "app_config.h"
#include "bsp_stream.h"
#include "bsp_uart.h"
#include "stm32h7xx_hal.h"

GimbalToVision g_gimbal_to_vision = {
    .head = {'A', 'B'},
};

VisionToGimbal g_vision_to_gimbal = {
    .head = {'A', 'B'},
};

PeripheralGuard g_vision_guard = {0};

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
      offset++;
      continue;
    }

    VisionToGimbal frame;
    memcpy(&frame, &g_vision_rx_stream_buffer.buffer[offset], sizeof(frame));
    offset = (uint16_t)(offset + sizeof(VisionToGimbal));
    if (!VisionProtocolFrameIsValid(&frame))
    {
      continue;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    memcpy(&g_vision_to_gimbal, &frame, sizeof(g_vision_to_gimbal));
    if (primask == 0U)
    {
      __enable_irq();
    }
    PeripheralGuardFeed(&g_vision_guard, HAL_GetTick());
  }

  StreamBufferConsume(&g_vision_rx_stream_buffer, offset);
}

/**
 * @brief 获取经看门狗与原子快照保护的视觉控制指令
 * @param out 对齐输出结构体指针
 * @return true 视觉在线且指令有效; false 离线/无效（out 已置安全零值）
 */
bool VisionProtocolGetCommand(VisionToGimbal *out)
{
  if (out == NULL)
  {
    return false;
  }
  if (!PeripheralIsOnline(&g_vision_guard))
  {
    memset(out, 0, sizeof(*out));
    out->head[0] = 'A';
    out->head[1] = 'B';
    return false;
  }

  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  memcpy(out, &g_vision_to_gimbal, sizeof(*out));
  if (primask == 0U)
  {
    __enable_irq();
  }
  return VisionProtocolFrameIsValid(out);
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
 * @brief 通过配置的视觉串口发送云台状态
 */
void VisionProtocolTransmitGimbalState(void)
{
  g_gimbal_to_vision.head[0] = 'A';
  g_gimbal_to_vision.head[1] = 'B';

  uint8_t tx_buffer[sizeof(GimbalToVision)];
  memcpy(tx_buffer, &g_gimbal_to_vision, sizeof(tx_buffer));
  uint16_t crc16 =
      GetCrc16(tx_buffer, (uint16_t)(sizeof(tx_buffer) - sizeof(uint16_t)));
  tx_buffer[sizeof(tx_buffer) - 2U] = (uint8_t)(crc16 & 0xFFU);
  tx_buffer[sizeof(tx_buffer) - 1U] = (uint8_t)(crc16 >> 8);
  UartTransmit(g_app_communication_config.vision_huart, tx_buffer,
               sizeof(tx_buffer));
}
