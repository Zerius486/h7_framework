#ifndef ALG_MEDIAN_H
#define ALG_MEDIAN_H
#include "stdint.h"

enum { kMedianWindowSize = 5 };

// 中值滤波器结构体
typedef struct {
  float buffer[kMedianWindowSize];  // 环形缓冲区
  uint32_t index;                   // 当前写入位置
} MedianFilterObject;
void MedianFilterInit(MedianFilterObject *filter, float initial_value);
float MedianFilterUpdate(MedianFilterObject *filter, float value);
#endif  // ALG_MEDIAN_H
