#include "alg_median.h"

#include "string.h"
/**
 * @brief 对两个值进行排序
 * @param a 第一个值指针
 * @param b 第二个值指针
 */
static void MedianSort2(float *a, float *b) {
  if (*a > *b) {
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
void MedianFilterInit(MedianFilterObject *filter, float initial_value) {
  filter->index = 0;
  memset(filter->buffer, 0, sizeof(filter->buffer));
  filter->buffer[0] = initial_value;
  filter->buffer[1] = initial_value;
  filter->buffer[2] = initial_value;
  filter->buffer[3] = initial_value;
  filter->buffer[4] = initial_value;
}
/**
 * @brief 更新中值滤波器并获取当前中值
 * @param filter 滤波器对象指针
 * @param value 新输入值
 * @return 当前窗口的中值
 */
float MedianFilterUpdate(MedianFilterObject *filter, float value) {
  filter->buffer[filter->index] = value;
  filter->index = (filter->index + 1) % kMedianWindowSize;
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