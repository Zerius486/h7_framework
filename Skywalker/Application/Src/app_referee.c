#include "app_referee.h"

#include <string.h>

#include "alg_crc.h"
#include "stm32h7xx_hal.h"

// 编译期校验 2027 V1.0.0 协议所有线协议结构体字节长度
_Static_assert(sizeof(RefereeFrameHeader) == 5U, "RefereeFrameHeader size mismatch");
_Static_assert(sizeof(RefereeGameState) == 11U, "RefereeGameState size mismatch");
_Static_assert(sizeof(RefereeGameResult) == 1U, "RefereeGameResult size mismatch");
_Static_assert(sizeof(RefereeGameRobotHp) == 24U, "RefereeGameRobotHp size mismatch");
_Static_assert(sizeof(RefereeEventData) == 8U, "RefereeEventData size mismatch");
_Static_assert(sizeof(RefereeWarning) == 3U, "RefereeWarning size mismatch");
_Static_assert(sizeof(RefereeGameRobotState) == 17U, "RefereeGameRobotState size mismatch");
_Static_assert(sizeof(RefereePowerHeatData) == 14U, "RefereePowerHeatData size mismatch");
_Static_assert(sizeof(RefereeGameRobotPosition) == 12U, "RefereeGameRobotPosition size mismatch");
_Static_assert(sizeof(RefereeBuff) == 7U, "RefereeBuff size mismatch");
_Static_assert(sizeof(RefereeRobotHurt) == 7U, "RefereeRobotHurt size mismatch");
_Static_assert(sizeof(RefereeShootData) == 7U, "RefereeShootData size mismatch");
_Static_assert(sizeof(RefereeProjectileAllowance) == 8U, "RefereeProjectileAllowance size mismatch");
_Static_assert(sizeof(RefereeRfidStatus) == 6U, "RefereeRfidStatus size mismatch");
_Static_assert(sizeof(RefereeGroundRobotPosition) == 40U, "RefereeGroundRobotPosition size mismatch");
_Static_assert(sizeof(RefereeSentryInfo) == 14U, "RefereeSentryInfo size mismatch");
_Static_assert(sizeof(RefereeInteractiveHeader) == 6U, "RefereeInteractiveHeader size mismatch");
_Static_assert(sizeof(RefereeInteractionLayerDelete) == 2U, "RefereeInteractionLayerDelete size mismatch");
_Static_assert(sizeof(RefereeGraphicDataStruct) == 15U, "RefereeGraphicDataStruct size mismatch");
_Static_assert(sizeof(RefereeInteractionFigure2) == 30U, "RefereeInteractionFigure2 size mismatch");
_Static_assert(sizeof(RefereeInteractionFigure5) == 75U, "RefereeInteractionFigure5 size mismatch");
_Static_assert(sizeof(RefereeInteractionFigure7) == 105U, "RefereeInteractionFigure7 size mismatch");
_Static_assert(sizeof(RefereeInteractionCharacter) == 45U, "RefereeInteractionCharacter size mismatch");
_Static_assert(sizeof(RefereeSentryCmd) == 4U, "RefereeSentryCmd size mismatch");
_Static_assert(sizeof(RefereeRadarCmd) == 13U, "RefereeRadarCmd size mismatch");
_Static_assert(sizeof(RefereeMapCommand) == 12U, "RefereeMapCommand size mismatch");
_Static_assert(sizeof(RefereeMapRobotData) == 48U, "RefereeMapRobotData size mismatch");
_Static_assert(sizeof(RefereeMapData) == 105U, "RefereeMapData size mismatch");
_Static_assert(sizeof(RefereeCustomInfo) == 34U, "RefereeCustomInfo size mismatch");
#if defined(REFEREE_MODE_RMUC) && (REFEREE_MODE_RMUC != 0)
_Static_assert(sizeof(RefereeDartInfo) == 3U, "RefereeDartInfo size mismatch");
_Static_assert(sizeof(RefereeDartClientCmd) == 6U, "RefereeDartClientCmd size mismatch");
_Static_assert(sizeof(RefereeRadarMarkData) == 2U, "RefereeRadarMarkData size mismatch");
_Static_assert(sizeof(RefereeRadarInfo) == 8U, "RefereeRadarInfo size mismatch");
#endif

RefereeObject g_referee = {0};

/**
 * @brief 从完整帧中读取数据段长度
 * @param frame 裁判系统帧指针
 * @return 数据段长度
 */
static uint16_t RefereeFrameDataLength(const uint8_t *frame)
{
  return (uint16_t)((uint16_t)frame[1] | ((uint16_t)frame[2] << 8));
}

/**
 * @brief 从完整帧中读取命令码
 * @param frame 裁判系统帧指针
 * @return 命令码
 */
static uint16_t RefereeFrameCommandId(const uint8_t *frame)
{
  return (uint16_t)((uint16_t)frame[5] | ((uint16_t)frame[6] << 8));
}

/**
 * @brief 判断裁判系统帧队列是否已满
 * @param queue 队列指针
 * @return true表示队列已满
 */
static bool RefereeQueueIsFull(const RefereeQueue *queue)
{
  return (uint16_t)((queue->tail + 1U) % kRefereeQueueSize) == queue->head;
}

/**
 * @brief 将一帧有效裁判系统数据写入队列
 * @param referee 裁判系统对象指针
 * @param frame 完整帧数据
 * @param frame_length 完整帧长度
 */
static void RefereeQueuePush(RefereeObject *referee, const uint8_t *frame,
                             uint16_t frame_length)
{
  if (RefereeQueueIsFull(&referee->queue))
  {
    referee->dropped_frame_count++;
    return;
  }

  RefereePacket *packet = &referee->queue.packets[referee->queue.tail];
  memcpy(packet->buffer, frame, frame_length);
  packet->length = frame_length;
  // 确保数据包内容写入完成后再发布 tail 索引，保证 SPSC 无锁队列并发可见性
  __DMB();
  referee->queue.tail =
      (uint16_t)((referee->queue.tail + 1U) % kRefereeQueueSize);
}

/**
 * @brief 校验裁判系统帧头CRC8和整帧CRC16
 * @param frame 完整帧数据
 * @param frame_length 完整帧长度
 * @return true表示校验通过
 */
static bool RefereeFrameIsValid(uint8_t *frame, uint16_t frame_length)
{
  if (frame_length < kRefereeFrameMinLength || frame[0] != kRefereeSof)
  {
    return false;
  }
  return CheckCrc8(frame, kRefereeHeaderLength) &&
         CheckCrc16(frame, frame_length);
}

/**
 * @brief 在长度足够时复制裁判系统数据段
 * @param destination 目标结构体指针
 * @param destination_size 目标结构体长度
 * @param source 数据段指针
 * @param data_length 数据段长度
 */
static void RefereeCopyData(void *destination, uint16_t destination_size,
                            const uint8_t *source, uint16_t data_length)
{
  if (destination == NULL || source == NULL || data_length < destination_size)
  {
    return;
  }
  memcpy(destination, source, destination_size);
}

/**
 * @brief 初始化裁判系统对象
 * @param referee 裁判系统对象指针
 * @param rx_huart 接收串口句柄
 * @param tx_huart 发送串口句柄
 */
void RefereeInit(RefereeObject *referee, UART_HandleTypeDef *rx_huart,
                 UART_HandleTypeDef *tx_huart)
{
  if (referee == NULL)
  {
    return;
  }

  memset(referee, 0, sizeof(*referee));
  referee->rx_huart = rx_huart;
  referee->tx_huart = tx_huart;
  StreamBufferInit(&referee->rx_stream_buffer, referee->rx_stream,
                   sizeof(referee->rx_stream));
  PeripheralGuardInit(&referee->guard, "referee", kPeripheralTypeReferee,
                      kPeripheralDefaultTimeoutRefereeMs, referee, NULL);

  if (referee == &g_referee && rx_huart != NULL)
  {
    UartInit(rx_huart, RefereeUartRxCallback);
  }
}

/**
 * @brief 裁判系统UART接收回调，支持粘包和半包
 * @param referee 裁判系统对象指针
 * @param rx_buffer UART接收数据
 * @param rx_length UART接收长度
 */
void RefereeRxCallback(RefereeObject *referee, uint8_t *rx_buffer,
                       uint16_t rx_length)
{
  if (referee == NULL || rx_buffer == NULL || rx_length == 0U)
  {
    return;
  }

  StreamBufferAppend(&referee->rx_stream_buffer, rx_buffer, rx_length);

  // 从流式缓存中按SOF和data_length拆出完整帧。
  uint16_t offset = 0;
  while ((uint16_t)(referee->rx_stream_buffer.length - offset) >=
         kRefereeHeaderLength)
  {
    if (referee->rx_stream[offset] != kRefereeSof)
    {
      offset++;
      continue;
    }

    uint8_t *frame = &referee->rx_stream_buffer.buffer[offset];
    if (!CheckCrc8(frame, kRefereeHeaderLength))
    {
      offset++;
      continue;
    }

    uint16_t data_length = RefereeFrameDataLength(frame);
    uint16_t frame_length = (uint16_t)(data_length + kRefereeFrameMinLength);

    if (frame_length < kRefereeFrameMinLength || frame_length > kUartBufferSize)
    {
      offset++;
      continue;
    }
    if ((uint16_t)(referee->rx_stream_buffer.length - offset) < frame_length)
    {
      break;
    }

    if (RefereeFrameIsValid(frame, frame_length))
    {
      RefereeQueuePush(referee, frame, frame_length);
      referee->received_frame_count++;
      PeripheralGuardFeed(&referee->guard, HAL_GetTick());
    }
    else
    {
      referee->dropped_frame_count++;
      offset++;
      continue;
    }
    offset = (uint16_t)(offset + frame_length);
  }

  StreamBufferConsume(&referee->rx_stream_buffer, offset);
}

/**
 * @brief 默认全局裁判系统对象的UART接收回调
 * @param rx_buffer UART接收数据
 * @param rx_length UART接收长度
 */
void RefereeUartRxCallback(uint8_t *rx_buffer, uint16_t rx_length)
{
  RefereeRxCallback(&g_referee, rx_buffer, rx_length);
}

/**
 * @brief 解析队列中的裁判系统完整帧
 * @param referee 裁判系统对象指针
 */
void RefereeUpdate(RefereeObject *referee)
{
  if (referee == NULL)
  {
    return;
  }

  while (referee->queue.head != referee->queue.tail)
  {
    // 采用零拷贝指针读取已就绪帧，避免在任务栈上分配大结构体
    const RefereePacket *packet = &referee->queue.packets[referee->queue.head];
    uint16_t data_length = RefereeFrameDataLength(packet->buffer);
    uint16_t cmd_id = RefereeFrameCommandId(packet->buffer);
    const uint8_t *data = &packet->buffer[kRefereeDataOffset];

    switch ((RefereeCommandId)cmd_id)
    {
    case kRefereeCmdGameState:
      RefereeCopyData(&referee->game_state, sizeof(referee->game_state), data,
                      data_length);
      break;
    case kRefereeCmdGameResult:
      RefereeCopyData(&referee->game_result, sizeof(referee->game_result), data,
                      data_length);
      break;
    case kRefereeCmdGameRobotHp:
      RefereeCopyData(&referee->game_robot_hp, sizeof(referee->game_robot_hp),
                      data, data_length);
      break;
    case kRefereeCmdEventData:
      RefereeCopyData(&referee->event_data, sizeof(referee->event_data), data,
                      data_length);
      break;
    case kRefereeCmdRefereeWarning:
      RefereeCopyData(&referee->referee_warning,
                      sizeof(referee->referee_warning), data, data_length);
      break;
#if defined(REFEREE_MODE_RMUC) && (REFEREE_MODE_RMUC != 0)
    case kRefereeCmdDartInfo:
      RefereeCopyData(&referee->dart_info, sizeof(referee->dart_info), data,
                      data_length);
      break;
#endif
    case kRefereeCmdGameRobotState:
      RefereeCopyData(&referee->game_robot_state,
                      sizeof(referee->game_robot_state), data, data_length);
      break;
    case kRefereeCmdPowerHeatData:
      RefereeCopyData(&referee->power_heat_data,
                      sizeof(referee->power_heat_data), data, data_length);
      break;
    case kRefereeCmdGameRobotPosition:
      RefereeCopyData(&referee->game_robot_position,
                      sizeof(referee->game_robot_position), data, data_length);
      break;
    case kRefereeCmdBuff:
      RefereeCopyData(&referee->buff, sizeof(referee->buff), data, data_length);
      break;
    case kRefereeCmdRobotHurt:
      RefereeCopyData(&referee->robot_hurt, sizeof(referee->robot_hurt), data,
                      data_length);
      break;
    case kRefereeCmdShootData:
      RefereeCopyData(&referee->shoot_data, sizeof(referee->shoot_data), data,
                      data_length);
      break;
    case kRefereeCmdProjectileAllowance:
      RefereeCopyData(&referee->projectile_allowance,
                      sizeof(referee->projectile_allowance), data, data_length);
      break;
    case kRefereeCmdRfidStatus:
      RefereeCopyData(&referee->rfid_status, sizeof(referee->rfid_status), data,
                      data_length);
      break;
#if defined(REFEREE_MODE_RMUC) && (REFEREE_MODE_RMUC != 0)
    case kRefereeCmdDartClientCmd:
      RefereeCopyData(&referee->dart_client_cmd,
                      sizeof(referee->dart_client_cmd), data, data_length);
      break;
#endif
    case kRefereeCmdGroundRobotPosition:
      RefereeCopyData(&referee->ground_robot_position,
                      sizeof(referee->ground_robot_position), data,
                      data_length);
      break;
#if defined(REFEREE_MODE_RMUC) && (REFEREE_MODE_RMUC != 0)
    case kRefereeCmdRadarMarkData:
      RefereeCopyData(&referee->radar_mark_data,
                      sizeof(referee->radar_mark_data), data, data_length);
      break;
#endif
    case kRefereeCmdSentryInfo:
      RefereeCopyData(&referee->sentry_info, sizeof(referee->sentry_info), data,
                      data_length);
      break;
#if defined(REFEREE_MODE_RMUC) && (REFEREE_MODE_RMUC != 0)
    case kRefereeCmdRadarInfo:
      RefereeCopyData(&referee->radar_info, sizeof(referee->radar_info), data,
                      data_length);
      break;
#endif
    case kRefereeCmdStudentInteractive:
    {
      if (data_length >= sizeof(referee->interactive_header))
      {
        RefereeCopyData(&referee->interactive_header,
                        sizeof(referee->interactive_header), data, data_length);
        uint16_t payload_length =
            (uint16_t)(data_length - sizeof(referee->interactive_header));
        if (payload_length > kRefereeMaxInteractiveDataLength)
        {
          payload_length = kRefereeMaxInteractiveDataLength;
        }
        memcpy(referee->interactive_data.data,
               &data[sizeof(referee->interactive_header)], payload_length);
        referee->interactive_data.length = payload_length;
      }
      break;
    }
    case kRefereeCmdMapCommand:
      RefereeCopyData(&referee->map_command, sizeof(referee->map_command), data,
                      data_length);
      break;
    case kRefereeCmdMapRobotData:
      RefereeCopyData(&referee->map_robot_data, sizeof(referee->map_robot_data),
                      data, data_length);
      break;
    case kRefereeCmdMapData:
      RefereeCopyData(&referee->map_data, sizeof(referee->map_data), data,
                      data_length);
      break;
    case kRefereeCmdCustomInfo:
      RefereeCopyData(&referee->custom_info, sizeof(referee->custom_info), data,
                      data_length);
      break;
    case kRefereeCmdCustomRobotData:
    {
      uint16_t copy_length = data_length;
      if (copy_length > kRefereeCustomDataMaxLength)
      {
        copy_length = kRefereeCustomDataMaxLength;
      }
      memcpy(referee->custom_robot_data.data, data, copy_length);
      referee->custom_robot_data.length = copy_length;
      break;
    }
    case kRefereeCmdCustomRobotCmd:
    {
      uint16_t copy_length = data_length;
      if (copy_length > kRefereeCustomCmdMaxLength)
      {
        copy_length = kRefereeCustomCmdMaxLength;
      }
      memcpy(referee->custom_robot_cmd.data, data, copy_length);
      referee->custom_robot_cmd.length = copy_length;
      break;
    }
    default:
      break;
    }

    // 确保在推进 head 释放槽位前，所有对 packet 的读取均已完成
    __DMB();
    // 解析完成后单步推进head索引，SPSC无锁环形队列保证中断与任务并发安全
    referee->queue.head =
        (uint16_t)((referee->queue.head + 1U) % kRefereeQueueSize);
  }
}

/**
 * @brief 按裁判系统协议打包发送帧
 * @param referee 裁判系统对象指针
 * @param cmd_id 命令码
 * @param data 数据段指针
 * @param data_length 数据段长度
 * @param tx_buffer 输出帧缓冲区
 * @param tx_buffer_length 输出帧缓冲区长度
 * @param frame_length 输出完整帧长度
 * @return true表示打包成功
 */
bool RefereePackFrame(RefereeObject *referee, uint16_t cmd_id,
                      const uint8_t *data, uint16_t data_length,
                      uint8_t *tx_buffer, uint16_t tx_buffer_length,
                      uint16_t *frame_length)
{
  if (referee == NULL || tx_buffer == NULL || frame_length == NULL ||
      (data == NULL && data_length > 0U))
  {
    return false;
  }

  if ((uint32_t)data_length + kRefereeFrameMinLength > tx_buffer_length)
  {
    return false;
  }
  uint16_t length = (uint16_t)(data_length + kRefereeFrameMinLength);

  tx_buffer[0] = kRefereeSof;
  tx_buffer[1] = (uint8_t)(data_length & 0xFFU);
  tx_buffer[2] = (uint8_t)(data_length >> 8);
  tx_buffer[3] = referee->tx_sequence++;
  tx_buffer[4] = GetCrc8(tx_buffer, kRefereeHeaderLength - 1U);
  tx_buffer[5] = (uint8_t)(cmd_id & 0xFFU);
  tx_buffer[6] = (uint8_t)(cmd_id >> 8);
  if (data_length > 0U)
  {
    memcpy(&tx_buffer[kRefereeDataOffset], data, data_length);
  }

  uint16_t crc16 = GetCrc16(tx_buffer, (uint16_t)(length - kRefereeTailLength));
  tx_buffer[length - 2U] = (uint8_t)(crc16 & 0xFFU);
  tx_buffer[length - 1U] = (uint8_t)(crc16 >> 8);
  *frame_length = length;
  return true;
}

/**
 * @brief 通过裁判系统发送串口发送一帧数据
 * @param referee 裁判系统对象指针
 * @param cmd_id 命令码
 * @param data 数据段指针
 * @param data_length 数据段长度
 * @return true表示成功提交到UART发送接口
 */
bool RefereeSend(RefereeObject *referee, uint16_t cmd_id, const uint8_t *data,
                 uint16_t data_length)
{
  if (referee == NULL || referee->tx_huart == NULL)
  {
    return false;
  }

  uint16_t frame_length = 0;
  if (!RefereePackFrame(referee, cmd_id, data, data_length, referee->tx_buffer,
                        sizeof(referee->tx_buffer), &frame_length))
  {
    return false;
  }

  return UartTransmitTry(referee->tx_huart, referee->tx_buffer, frame_length);
}
