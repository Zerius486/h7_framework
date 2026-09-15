#ifndef ALG_TD_H
#define ALG_TD_H
// 微分追踪器结构体
typedef struct {
  float x1;  // 跟踪信号
  float x2;  // 微分信号
  float r;   // 跟踪速度因子
} TdObject;
void TdInit(TdObject *td, float r);
void TdUpdate(TdObject *td, float input, float dt);
#endif  // ALG_TD_H