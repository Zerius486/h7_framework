#ifndef ALG_NOTCH_H
#define ALG_NOTCH_H
// 陷波滤波器结构体
typedef struct
{
  float a0; // 当前输入系数
  float a1; // 上一次输入系数
  float a2; // 上上次输入系数
  float b1; // 上一次输出反馈系数
  float b2; // 上上次输出反馈系数
  float x1; // 上一次输入
  float x2; // 上上次输入
  float y1; // 上一次输出
  float y2; // 上上次输出
} NotchFilterObject;
void NotchFilterInit(NotchFilterObject *notch_filter, float f0, float Q,
                     float fs);
float NotchFilterProcess(NotchFilterObject *notch_filter, float input);
#endif // ALG_NOTCH_H
