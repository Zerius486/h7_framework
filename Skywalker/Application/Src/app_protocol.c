#include "app_protocol.h"

#include <stdbool.h>
#include <string.h>

#include "alg_crc.h"
#include "bsp_stream.h"
#include "bsp_uart.h"
#include "usart.h"

GimbalToVision g_gimbal_to_vision = {
    .head = {'A', 'B'},
};

VisionToGimbal g_vision_to_gimbal = {
    .head = {'A', 'B'},
};

static uint8_t g_vision_rx_stream[sizeof(VisionToGimbal) * 4U];
static StreamBuffer g_vision_rx_stream_buffer;
static bool g_vision_rx_stream_initialized = false;

static bool VisionProtocolHeaderIsValid(const uint8_t *data)
{
  return data != NULL && data[0] == 'A' && data[1] == 'B';
}

void VisionProtocolRxCallback(uint8_t *rx_buffer, uint16_t rx_length)
{
  if (rx_buffer == NULL || rx_length == 0U)
  {
    return;
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

    if (CheckCrc16(&g_vision_rx_stream_buffer.buffer[offset],
                   sizeof(VisionToGimbal)))
    {
      memcpy(&g_vision_to_gimbal, &g_vision_rx_stream_buffer.buffer[offset],
             sizeof(g_vision_to_gimbal));
    }
    offset = (uint16_t)(offset + sizeof(VisionToGimbal));
  }

  StreamBufferConsume(&g_vision_rx_stream_buffer, offset);
}

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
  UartTransmit(&huart10, tx_buffer, sizeof(tx_buffer));
}
