#ifndef ALG_MULTISINE_H
#define ALG_MULTISINE_H

#include <stdbool.h>
#include <stdint.h>

#define MULTISINE_MAX_TONES 8U

// 闭环参考激励配置，幅度和范围与参考信号使用相同坐标
typedef struct
{
  float sampling_time;  // 调用周期（s）
  float fundamental_hz; // 基频（Hz），按整数采样点量化
  float amplitude;      // 所有分量幅值之和
  float offset;         // 参考工作点
  float lower;
  float upper;
  uint16_t harmonic[MULTISINE_MAX_TONES]; // 不重复的正整数谐波
  uint8_t tone_count;
  uint8_t settling_periods;    // 过渡周期数，至少1
  uint8_t measurement_periods; // 测量周期数，至少2
} MultisineConfig;

// 多正弦激励对象，不持有电机或控制器
typedef struct
{
  MultisineConfig config;
  float fundamental_hz; // 量化后的实际基频，分析时使用此值
  float reference;
  uint32_t period_ticks;
  uint32_t tick;
  uint32_t sample; // 当前输出在周期内的采样下标
  uint16_t period; // 当前输出的周期下标
  bool measuring;
  bool active;
  bool initialized;
} MultisineObject;

bool MultisineInit(MultisineObject *multisine, const MultisineConfig *config);
bool MultisineStart(MultisineObject *multisine);
float MultisineCalculate(MultisineObject *multisine);
void MultisineCancel(MultisineObject *multisine);

#endif // ALG_MULTISINE_H
