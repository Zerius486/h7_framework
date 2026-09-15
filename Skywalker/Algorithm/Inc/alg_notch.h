#ifndef ALG_NOTCH_H
#define ALG_NOTCH_H
// 陷波滤波器结构体
typedef struct {
  float a0;  // 前馈系数0
  float a1;  // 前馈系数1
  float a2;  // 前馈系数2
  float b1;  // 反馈系数1
  float b2;  // 反馈系数2
  float x1;  // 上一次输入
  float x2;  // 上上次输入
  float y1;  // 上一次输出
  float y2;  // 上上次输出
} NotchFilterObject;
void NotchFilterInit(NotchFilterObject *notch_filter, float f0, float Q,
                     float fs);
float NotchFilterProcess(NotchFilterObject *notch_filter, float input);
#endif  // ALG_NOTCH_H