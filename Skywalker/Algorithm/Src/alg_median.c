#include "alg_median.h"

#include <math.h>
#include <stddef.h>
/**
 * @brief 对两个值进行排序
 * @param a 第一个值指针
 * @param b 第二个值指针
 */
static void MedianSort2(float *a, float *b)
{
  if (*a > *b)
  {
    float t = *a;
    *a = *b;
    *b = t;
  }
}
/**
 * @brief 初始化中值滤波器
 * @param filter 滤波器对象指针
 * @param initial_value 初始值
 */
void MedianFilterInit(MedianFilterObject *filter, float initial_value)
{
  if (filter == NULL)
  {
    return;
  }
  float init_val = isfinite(initial_value) ? initial_value : 0.0f;
  filter->index = 0;
  for (uint8_t i = 0; i < kMedianWindowSize; ++i)
  {
    filter->buffer[i] = init_val;
  }
}
/**
 * @brief 更新中值滤波器并获取当前中值
 * @param filter 滤波器对象指针
 * @param value 新输入值
 * @return 当前窗口的中值
 */
float MedianFilterUpdate(MedianFilterObject *filter, float value)
{
  if (filter == NULL)
  {
    return value;
  }
  if (filter->index >= kMedianWindowSize)
  {
    filter->index = 0;
  }
  if (isfinite(value))
  {
    filter->buffer[filter->index] = value;
    if (++filter->index >= kMedianWindowSize)
    {
      filter->index = 0;
    }
  }
  float s0 = filter->buffer[0];
  float s1 = filter->buffer[1];
  float s2 = filter->buffer[2];
  float s3 = filter->buffer[3];
  float s4 = filter->buffer[4];
  MedianSort2(&s0, &s1);
  MedianSort2(&s3, &s4);
  MedianSort2(&s2, &s4);
  MedianSort2(&s2, &s3);
  MedianSort2(&s1, &s4);
  MedianSort2(&s0, &s3);
  MedianSort2(&s0, &s2);
  MedianSort2(&s1, &s3);
  MedianSort2(&s1, &s2);
  return s2;
}