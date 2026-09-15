#include "alg_notch.h"

#include "math.h"
/**
 * @brief 初始化陷波滤波器
 * @param notch_filter 陷波滤波器指针
 * @param f0 陷波频率
 * @param q 品质因数
 * @param fs 采样频率
 */
void NotchFilterInit(NotchFilterObject *notch_filter, float f0, float q,
                     float fs) {
  float w0 = 2.0f * M_PI * f0 / fs;
  float alpha = sinf(w0) / (2.0f * q);
  float cos_w0 = cosf(w0);
  float a0 = 1.0f + alpha;
  notch_filter->a0 = 1.0f / a0;
  notch_filter->a1 = -2.0f * cos_w0 / a0;
  notch_filter->a2 = (1.0f - alpha) / a0;
  notch_filter->b1 = 2.0f * cos_w0 / a0;
  notch_filter->b2 = -(1.0f - alpha) / a0;
  notch_filter->x1 = 0.0f;
  notch_filter->x2 = 0.0f;
  notch_filter->y1 = 0.0f;
  notch_filter->y2 = 0.0f;
}
/**
 * @brief 处理输入信号
 * @param notch_filter 陷波滤波器指针
 * @param input 当前输入信号
 * @return 滤波后的输出信号
 */
float NotchFilterProcess(NotchFilterObject *notch_filter, float input) {
  // 计算当前输出
  float output =
      notch_filter->a0 * input + notch_filter->a1 * notch_filter->x1 +
      notch_filter->a2 * notch_filter->x2 -
      notch_filter->b1 * notch_filter->y1 - notch_filter->b2 * notch_filter->y2;
  // 更新历史值
  notch_filter->x2 = notch_filter->x1;
  notch_filter->x1 = input;
  notch_filter->y2 = notch_filter->y1;
  notch_filter->y1 = output;
  return output;
}