#include "app_referee.h"

#include <string.h>

#include "alg_crc.h"

RefereeObject g_referee = {0};

/**
 * @brief 从完整帧中读取数据段长度
 * @param frame 裁判系统帧指针
 * @return 数据段长度
 */
static uint16_t RefereeFrameDataLength(const uint8_t *frame) {
  return (uint16_t)((uint16_t)frame[1] | ((uint16_t)frame[2] << 8));
}

/**
 * @brief 从完整帧中读取命令码
 * @param frame 裁判系统帧指针
 * @return 命令码
 */
static uint16_t RefereeFrameCommandId(const uint8_t *frame) {
  return (uint16_t)((uint16_t)frame[5] | ((uint16_t)frame[6] << 8));
}

/**
 * @brief 判断裁判系统帧队列是否已满
 * @param queue 队列指针
 * @return true表示队列已满
 */
static bool RefereeQueueIsFull(const RefereeQueue *queue) {
  return (uint16_t)((queue->tail + 1U) % kRefereeQueueSize) == queue->head;
}

/**
 * @brief 将一帧有效裁判系统数据写入队列
 * @param referee 裁判系统对象指针
 * @param frame 完整帧数据
 * @param frame_length 完整帧长度
 */
static void RefereeQueuePush(RefereeObject *referee, const uint8_t *frame,
                             uint16_t frame_length) {
  if (RefereeQueueIsFull(&referee->queue)) {
    referee->dropped_frame_count++;
    return;
  }

  RefereePacket *packet = &referee->queue.packets[referee->queue.tail];
  memcpy(packet->buffer, frame, frame_length);
  packet->length = frame_length;
  referee->queue.tail =
      (uint16_t)((referee->queue.tail + 1U) % kRefereeQueueSize);
}

/**
 * @brief 校验裁判系统帧头CRC8和整帧CRC16
 * @param frame 完整帧数据
 * @param frame_length 完整帧长度
 * @return true表示校验通过
 */
static bool RefereeFrameIsValid(uint8_t *frame, uint16_t frame_length) {
  if (frame_length < kRefereeFrameMinLength || frame[0] != kRefereeSof) {
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
                            const uint8_t *source, uint16_t data_length) {
  if (destination == NULL || source == NULL || data_length < destination_size) {
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
                 UART_HandleTypeDef *tx_huart) {
  if (referee == NULL) {
    return;
  }

  memset(referee, 0, sizeof(*referee));
  referee->rx_huart = rx_huart;
  referee->tx_huart = tx_huart;

  if (referee == &g_referee && rx_huart != NULL) {
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
                       uint16_t rx_length) {
  if (referee == NULL || rx_buffer == NULL || rx_length == 0U) {
    return;
  }

  if ((uint32_t)referee->rx_stream_length + rx_length > kRefereeRxStreamSize) {
    // 缓存空间不足时保留尾部数据，避免半包在两次回调之间丢失。
    if (rx_length >= kRefereeRxStreamSize) {
      memcpy(referee->rx_stream, &rx_buffer[rx_length - kRefereeRxStreamSize],
             kRefereeRxStreamSize);
      referee->rx_stream_length = kRefereeRxStreamSize;
    } else {
      uint16_t keep_capacity = (uint16_t)(kRefereeRxStreamSize - rx_length);
      uint16_t keep_length = referee->rx_stream_length < keep_capacity
                                 ? referee->rx_stream_length
                                 : keep_capacity;
      memmove(referee->rx_stream,
              &referee->rx_stream[referee->rx_stream_length - keep_length],
              keep_length);
      memcpy(&referee->rx_stream[keep_length], rx_buffer, rx_length);
      referee->rx_stream_length = (uint16_t)(keep_length + rx_length);
    }
  } else {
    memcpy(&referee->rx_stream[referee->rx_stream_length], rx_buffer,
           rx_length);
    referee->rx_stream_length =
        (uint16_t)(referee->rx_stream_length + rx_length);
  }

  // 从流式缓存中按SOF和data_length拆出完整帧。
  uint16_t offset = 0;
  while ((uint16_t)(referee->rx_stream_length - offset) >=
         kRefereeFrameMinLength) {
    if (referee->rx_stream[offset] != kRefereeSof) {
      offset++;
      continue;
    }

    uint8_t *frame = &referee->rx_stream[offset];
    uint16_t data_length = RefereeFrameDataLength(frame);
    uint16_t frame_length = (uint16_t)(data_length + kRefereeFrameMinLength);

    if (frame_length < kRefereeFrameMinLength ||
        frame_length > kUartBufferSize) {
      offset++;
      continue;
    }
    if ((uint16_t)(referee->rx_stream_length - offset) < frame_length) {
      break;
    }

    if (RefereeFrameIsValid(frame, frame_length)) {
      RefereeQueuePush(referee, frame, frame_length);
      referee->received_frame_count++;
    } else {
      referee->dropped_frame_count++;
    }
    offset = (uint16_t)(offset + frame_length);
  }

  if (offset > 0U) {
    if (offset < referee->rx_stream_length) {
      memmove(referee->rx_stream, &referee->rx_stream[offset],
              referee->rx_stream_length - offset);
      referee->rx_stream_length =
          (uint16_t)(referee->rx_stream_length - offset);
    } else {
      referee->rx_stream_length = 0;
    }
  }
}

/**
 * @brief 默认全局裁判系统对象的UART接收回调
 * @param rx_buffer UART接收数据
 * @param rx_length UART接收长度
 */
void RefereeUartRxCallback(uint8_t *rx_buffer, uint16_t rx_length) {
  RefereeRxCallback(&g_referee, rx_buffer, rx_length);
}

/**
 * @brief 解析队列中的裁判系统完整帧
 * @param referee 裁判系统对象指针
 */
void RefereeUpdate(RefereeObject *referee) {
  if (referee == NULL) {
    return;
  }

  while (referee->queue.head != referee->queue.tail) {
    RefereePacket *packet = &referee->queue.packets[referee->queue.head];
    uint16_t data_length = RefereeFrameDataLength(packet->buffer);
    uint16_t cmd_id = RefereeFrameCommandId(packet->buffer);
    const uint8_t *data = &packet->buffer[kRefereeDataOffset];

    switch ((RefereeCommandId)cmd_id) {
      case kRefereeCmdGameState:
        RefereeCopyData(&referee->game_state, sizeof(referee->game_state), data,
                        data_length);
        break;
      case kRefereeCmdGameResult:
        RefereeCopyData(&referee->game_result, sizeof(referee->game_result),
                        data, data_length);
        break;
      case kRefereeCmdGameRobotHp:
        RefereeCopyData(&referee->game_robot_hp, sizeof(referee->game_robot_hp),
                        data, data_length);
        break;
      case kRefereeCmdEventData:
        RefereeCopyData(&referee->event_data, sizeof(referee->event_data), data,
                        data_length);
        break;
      case kRefereeCmdSupplyProjectileAction:
        RefereeCopyData(&referee->supply_projectile_action,
                        sizeof(referee->supply_projectile_action), data,
                        data_length);
        break;
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
                        sizeof(referee->game_robot_position), data,
                        data_length);
        break;
      case kRefereeCmdBuff:
        RefereeCopyData(&referee->buff, sizeof(referee->buff), data,
                        data_length);
        break;
      case kRefereeCmdAerialRobotEnergy:
        RefereeCopyData(&referee->aerial_robot_energy,
                        sizeof(referee->aerial_robot_energy), data,
                        data_length);
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
                        sizeof(referee->projectile_allowance), data,
                        data_length);
        break;
      case kRefereeCmdStudentInteractive: {
        if (data_length >= sizeof(referee->interactive_header)) {
          RefereeCopyData(&referee->interactive_header,
                          sizeof(referee->interactive_header), data,
                          data_length);
          uint16_t payload_length =
              (uint16_t)(data_length - sizeof(referee->interactive_header));
          if (payload_length > kRefereeMaxInteractiveDataLength) {
            payload_length = kRefereeMaxInteractiveDataLength;
          }
          memcpy(referee->interactive_data.data,
                 &data[sizeof(referee->interactive_header)], payload_length);
          referee->interactive_data.length = payload_length;
        }
        break;
      }
      default:
        break;
    }

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
                      uint16_t *frame_length) {
  if (referee == NULL || tx_buffer == NULL || frame_length == NULL ||
      (data == NULL && data_length > 0U)) {
    return false;
  }

  uint16_t length = (uint16_t)(data_length + kRefereeFrameMinLength);
  if (length > tx_buffer_length) {
    return false;
  }

  tx_buffer[0] = kRefereeSof;
  tx_buffer[1] = (uint8_t)(data_length & 0xFFU);
  tx_buffer[2] = (uint8_t)(data_length >> 8);
  tx_buffer[3] = referee->tx_sequence++;
  tx_buffer[4] = GetCrc8(tx_buffer, kRefereeHeaderLength - 1U);
  tx_buffer[5] = (uint8_t)(cmd_id & 0xFFU);
  tx_buffer[6] = (uint8_t)(cmd_id >> 8);
  if (data_length > 0U) {
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
                 uint16_t data_length) {
  if (referee == NULL || referee->tx_huart == NULL) {
    return false;
  }

  uint8_t tx_buffer[kUartBufferSize];
  uint16_t frame_length = 0;
  if (!RefereePackFrame(referee, cmd_id, data, data_length, tx_buffer,
                        sizeof(tx_buffer), &frame_length)) {
    return false;
  }

  UartTransmit(referee->tx_huart, tx_buffer, frame_length);
  return true;
}
