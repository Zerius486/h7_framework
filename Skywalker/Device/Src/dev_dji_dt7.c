#include "dev_dji_dt7.h"

#include <stddef.h>

#include "cmsis_os2.h"

enum
{
    kDt7FrameLength = 18U
};

// DT7 遥控器对象实例
Dt7Object g_dt7_object;

/**
 * @brief DT7 遥控器接收回调函数，解析18字节数据包
 * @param rx_buffer 接收缓冲区指针
 * @param length 接收数据长度
 */
void Dt7RxCallback(uint8_t *rx_buffer, uint16_t length)
{
    if (length != kDt7FrameLength || rx_buffer == NULL)
    {
        return;
    }

    g_dt7_object.rocker.ch0 =
        (int16_t)(((uint16_t)rx_buffer[0] | ((uint16_t)rx_buffer[1] << 8)) &
                  0x07FFU) -
        (int16_t)kRcChValueOffset;
    g_dt7_object.rocker.ch1 = (int16_t)((((uint16_t)rx_buffer[1] >> 3) |
                                         ((uint16_t)rx_buffer[2] << 5)) &
                                        0x07FFU) -
                              (int16_t)kRcChValueOffset;
    g_dt7_object.rocker.ch2 =
        (int16_t)((((uint16_t)rx_buffer[2] >> 6) | ((uint16_t)rx_buffer[3] << 2) |
                   ((uint16_t)rx_buffer[4] << 10)) &
                  0x07FFU) -
        (int16_t)kRcChValueOffset;
    g_dt7_object.rocker.ch3 = (int16_t)((((uint16_t)rx_buffer[4] >> 1) |
                                         ((uint16_t)rx_buffer[5] << 7)) &
                                        0x07FFU) -
                              (int16_t)kRcChValueOffset;

    g_dt7_object.rocker.sw1 = (uint8_t)(((rx_buffer[5] >> 4) & 0x0CU) >> 2);
    g_dt7_object.rocker.sw2 = (uint8_t)((rx_buffer[5] >> 4) & 0x03U);

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
        (int16_t)(((uint16_t)rx_buffer[16] | ((uint16_t)rx_buffer[17] << 8)) &
                  0x07FFU) -
        (int16_t)kRcChValueOffset;
    g_dt7_object.last_update_tick = osKernelGetTickCount();
    g_dt7_object.frame_count++;
}

uint32_t Dt7FrameCount(void) { return g_dt7_object.frame_count; }

uint32_t Dt7LastUpdateTick(void) { return g_dt7_object.last_update_tick; }
