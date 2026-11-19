#include "dev_dji_dt7.h"

#include <stdbool.h>
#include <stddef.h>
#include <string.h>

#include "stm32h7xx_hal.h"

enum
{
  kDt7FrameLength = 18U
};

// DT7 遥控器对象实例
Dt7Object g_dt7_object;

static bool Dt7ChannelValid(uint16_t raw)
{
  return raw >= kRcChValueMin && raw <= kRcChValueMax;
}

static bool Dt7SwitchValid(uint8_t sw)
{
  return sw == kRcSwUp || sw == kRcSwMid || sw == kRcSwDown;
}

static void Dt7GuardOfflineHook(void *device_ptr)
{
  Dt7Object *dt7 = (Dt7Object *)device_ptr;
  if (dt7 == NULL)
  {
    return;
  }
  // 离线失联保护：摇杆归中、开关置下位安全态、鼠标键盘清零
  dt7->rocker.ch0 = 0;
  dt7->rocker.ch1 = 0;
  dt7->rocker.ch2 = 0;
  dt7->rocker.ch3 = 0;
  dt7->rocker.wheel = 0;
  dt7->rocker.sw1 = kRcSwDown;
  dt7->rocker.sw2 = kRcSwDown;
  dt7->mouse.x = 0;
  dt7->mouse.y = 0;
  dt7->mouse.z = 0;
  dt7->mouse.l = 0;
  dt7->mouse.r = 0;
  memset(&dt7->key, 0, sizeof(dt7->key));
}

void Dt7Init(void)
{
  memset(&g_dt7_object, 0, sizeof(g_dt7_object));
  g_dt7_object.rocker.sw1 = kRcSwDown;
  g_dt7_object.rocker.sw2 = kRcSwDown;

  PeripheralOps ops = {
      .on_enable = NULL,
      .on_disable = Dt7GuardOfflineHook,
      .on_offline = Dt7GuardOfflineHook,
      .on_resync = NULL,
      .on_clear_fault = NULL,
      .on_reconnect_probe = NULL,
  };
  PeripheralGuardInit(&g_dt7_object.guard, "dji_dt7", kPeripheralTypeRemote,
                      kPeripheralDefaultTimeoutRemoteMs, &g_dt7_object, &ops);
}

/**
 * @brief DT7 遥控器接收回调函数，解析并校验18字节DBUS数据包
 * @param rx_buffer 接收缓冲区指针
 * @param length 接收数据长度
 */
void Dt7RxCallback(uint8_t *rx_buffer, uint16_t length)
{
  if (rx_buffer == NULL || length != kDt7FrameLength)
  {
    return;
  }

  uint16_t raw_ch0 =
      ((uint16_t)rx_buffer[0] | ((uint16_t)rx_buffer[1] << 8)) & 0x07FFU;
  uint16_t raw_ch1 =
      (((uint16_t)rx_buffer[1] >> 3) | ((uint16_t)rx_buffer[2] << 5)) &
      0x07FFU;
  uint16_t raw_ch2 =
      (((uint16_t)rx_buffer[2] >> 6) | ((uint16_t)rx_buffer[3] << 2) |
       ((uint16_t)rx_buffer[4] << 10)) &
      0x07FFU;
  uint16_t raw_ch3 =
      (((uint16_t)rx_buffer[4] >> 1) | ((uint16_t)rx_buffer[5] << 7)) &
      0x07FFU;
  uint8_t sw1 = (uint8_t)(((rx_buffer[5] >> 4) & 0x0CU) >> 2);
  uint8_t sw2 = (uint8_t)((rx_buffer[5] >> 4) & 0x03U);
  uint16_t raw_wheel =
      ((uint16_t)rx_buffer[16] | ((uint16_t)rx_buffer[17] << 8)) & 0x07FFU;

  if (!Dt7ChannelValid(raw_ch0) || !Dt7ChannelValid(raw_ch1) ||
      !Dt7ChannelValid(raw_ch2) || !Dt7ChannelValid(raw_ch3) ||
      !Dt7SwitchValid(sw1) || !Dt7SwitchValid(sw2) ||
      (raw_wheel != 0U && !Dt7ChannelValid(raw_wheel)))
  {
    return;
  }

  g_dt7_object.rocker.ch0 = (int16_t)raw_ch0 - (int16_t)kRcChValueOffset;
  g_dt7_object.rocker.ch1 = (int16_t)raw_ch1 - (int16_t)kRcChValueOffset;
  g_dt7_object.rocker.ch2 = (int16_t)raw_ch2 - (int16_t)kRcChValueOffset;
  g_dt7_object.rocker.ch3 = (int16_t)raw_ch3 - (int16_t)kRcChValueOffset;
  g_dt7_object.rocker.sw1 = sw1;
  g_dt7_object.rocker.sw2 = sw2;

  g_dt7_object.mouse.x =
      (int16_t)((uint16_t)rx_buffer[6] | ((uint16_t)rx_buffer[7] << 8));
  g_dt7_object.mouse.y =
      (int16_t)((uint16_t)rx_buffer[8] | ((uint16_t)rx_buffer[9] << 8));
  g_dt7_object.mouse.z =
      (int16_t)((uint16_t)rx_buffer[10] | ((uint16_t)rx_buffer[11] << 8));
  g_dt7_object.mouse.l = rx_buffer[12];
  g_dt7_object.mouse.r = rx_buffer[13];

  uint16_t key = (uint16_t)rx_buffer[14] | ((uint16_t)rx_buffer[15] << 8);
  g_dt7_object.key.w = (uint8_t)((key & kKeyPressedOffsetW) != 0U);
  g_dt7_object.key.s = (uint8_t)((key & kKeyPressedOffsetS) != 0U);
  g_dt7_object.key.a = (uint8_t)((key & kKeyPressedOffsetA) != 0U);
  g_dt7_object.key.d = (uint8_t)((key & kKeyPressedOffsetD) != 0U);
  g_dt7_object.key.shift = (uint8_t)((key & kKeyPressedOffsetShift) != 0U);
  g_dt7_object.key.ctrl = (uint8_t)((key & kKeyPressedOffsetCtrl) != 0U);
  g_dt7_object.key.q = (uint8_t)((key & kKeyPressedOffsetQ) != 0U);
  g_dt7_object.key.e = (uint8_t)((key & kKeyPressedOffsetE) != 0U);
  g_dt7_object.key.r = (uint8_t)((key & kKeyPressedOffsetR) != 0U);
  g_dt7_object.key.f = (uint8_t)((key & kKeyPressedOffsetF) != 0U);
  g_dt7_object.key.g = (uint8_t)((key & kKeyPressedOffsetG) != 0U);
  g_dt7_object.key.z = (uint8_t)((key & kKeyPressedOffsetZ) != 0U);
  g_dt7_object.key.x = (uint8_t)((key & kKeyPressedOffsetX) != 0U);
  g_dt7_object.key.c = (uint8_t)((key & kKeyPressedOffsetC) != 0U);
  g_dt7_object.key.v = (uint8_t)((key & kKeyPressedOffsetV) != 0U);
  g_dt7_object.key.b = (uint8_t)((key & kKeyPressedOffsetB) != 0U);

  g_dt7_object.rocker.wheel =
      (raw_wheel == 0U) ? 0
                        : ((int16_t)raw_wheel - (int16_t)kRcChValueOffset);
  uint32_t now_tick = HAL_GetTick();
  g_dt7_object.last_update_tick = now_tick;
  g_dt7_object.frame_count++;
  PeripheralGuardFeed(&g_dt7_object.guard, now_tick);
}

uint32_t Dt7FrameCount(void)
{
  return g_dt7_object.frame_count;
}

uint32_t Dt7LastUpdateTick(void)
{
  return g_dt7_object.last_update_tick;
}
