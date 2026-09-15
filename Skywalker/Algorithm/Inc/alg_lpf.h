#ifndef ALG_LPF_H
#define ALG_LPF_H
// 一阶低通滤波器结构体
typedef struct {
  float alpha;       // 滤波系数
  float prev_input;  // 上一次的输入值
} Lpf1stObject;
// 二阶低通滤波器结构体
typedef struct {
  float alpha_1;       // 滤波系数1
  float alpha_2;       // 滤波系数2
  float prev_input_1;  // 上一次的输入值
  float prev_input_2;  // 上两次的输入值
} Lpf2ndObject;
void Lpf1stInit(Lpf1stObject *lpf_1st, float alpha);
float Lpf1stUpdate(Lpf1stObject *lpf_1st, float input);
void Lpf2ndInit(Lpf2ndObject *lpf_2nd, float alpha_1, float alpha_2);
float Lpf2ndUpdate(Lpf2ndObject *lpf_2nd, float input);
#endif  // ALG_LPF_H