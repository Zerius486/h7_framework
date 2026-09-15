#ifndef ALG_TD_NL_H
#define ALG_TD_NL_H
// 非线性微分追踪器结构体
typedef struct {
  float x1;  // 跟踪信号
  float x2;  // 微分信号
  float r;   // 速度因子
  float h;   // 滤波因子
} TdNlObject;
void TdNlInit(TdNlObject *td_nl, float r, float h);
void TdNlUpdate(TdNlObject *td_nl, float input, float dt);
#endif  // ALG_TD_NL_H