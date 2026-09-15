#ifndef DEV_DT7_H
#define DEV_DT7_H

#include <stdint.h>

// 遥控器通道值范围和开关位置定义
enum {
  kRcChValueMin = 364,
  kRcChValueOffset = 1024,
  kRcChValueMax = 1684,
  kRcSwUp = 1,
  kRcSwMid = 3,
  kRcSwDown = 2,
};

// 遥控器开关状态判断宏
#define SwitchIsDown(s) ((s) == kRcSwDown)  // 开关是否处于下位
#define SwitchIsMid(s) ((s) == kRcSwMid)    // 开关是否处于中位
#define SwitchIsUp(s) ((s) == kRcSwUp)      // 开关是否处于上位

// 键盘按键掩码定义
enum {
  kKeyPressedOffsetW = (1U << 0),
  kKeyPressedOffsetS = (1U << 1),
  kKeyPressedOffsetA = (1U << 2),
  kKeyPressedOffsetD = (1U << 3),
  kKeyPressedOffsetShift = (1U << 4),
  kKeyPressedOffsetCtrl = (1U << 5),
  kKeyPressedOffsetQ = (1U << 6),
  kKeyPressedOffsetE = (1U << 7),
  kKeyPressedOffsetR = (1U << 8),
  kKeyPressedOffsetF = (1U << 9),
  kKeyPressedOffsetG = (1U << 10),
  kKeyPressedOffsetZ = (1U << 11),
  kKeyPressedOffsetX = (1U << 12),
  kKeyPressedOffsetC = (1U << 13),
  kKeyPressedOffsetV = (1U << 14),
  kKeyPressedOffsetB = (1U << 15),
};

// DT7 遥控器对象结构体
typedef struct {
  struct {
    int16_t ch0;    // 摇杆通道0（右水平）
    int16_t ch1;    // 摇杆通道1（右垂直）
    int16_t ch2;    // 摇杆通道2（左垂直）
    int16_t ch3;    // 摇杆通道3（左水平）
    uint8_t sw1;    // 开关1状态
    uint8_t sw2;    // 开关2状态
    int16_t wheel;  // 拨轮值
  } rocker;         // 摇杆及开关数据
  struct {
    int16_t x;  // 鼠标X轴移动量
    int16_t y;  // 鼠标Y轴移动量
    int16_t z;  // 鼠标Z轴（滚轮）移动量
    uint8_t l;  // 鼠标左键状态
    uint8_t r;  // 鼠标右键状态
  } mouse;      // 鼠标数据
  struct {
    uint8_t w;      // W键状态
    uint8_t s;      // S键状态
    uint8_t a;      // A键状态
    uint8_t d;      // D键状态
    uint8_t shift;  // Shift键状态
    uint8_t ctrl;   // Ctrl键状态
    uint8_t q;      // Q键状态
    uint8_t e;      // E键状态
    uint8_t r;      // R键状态
    uint8_t f;      // F键状态
    uint8_t g;      // G键状态
    uint8_t z;      // Z键状态
    uint8_t x;      // X键状态
    uint8_t c;      // C键状态
    uint8_t v;      // V键状态
    uint8_t b;      // B键状态
  } key;            // 键盘按键数据
  volatile uint32_t frame_count;       // 有效遥控帧计数
  volatile uint32_t last_update_tick;  // 最近一次有效帧的系统tick
} Dt7Object;

extern Dt7Object g_dt7_object;

void Dt7RxCallback(uint8_t *rx_buffer, uint16_t length);
uint32_t Dt7FrameCount(void);
uint32_t Dt7LastUpdateTick(void);

#endif  // DEV_DT7_H
