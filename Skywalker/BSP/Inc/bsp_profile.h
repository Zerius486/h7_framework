#ifndef BSP_PROFILE_H
#define BSP_PROFILE_H

#include <stdint.h>

// DWT用时统计，包含测量期间的中断和任务抢占
typedef struct
{
  uint32_t begin;   // 开始周期计数
  uint32_t last;    // 最近用时（cycle）
  uint32_t minimum; // 最小用时（cycle）
  uint32_t maximum; // 最大用时（cycle）
  uint32_t count;   // 采样数量
  uint64_t total;   // 累计用时（cycle）
} ProfileObject;

void ProfileInit(void);
void ProfileReset(ProfileObject *profile);
void ProfileBegin(ProfileObject *profile);
void ProfileEnd(ProfileObject *profile);

#endif // BSP_PROFILE_H
