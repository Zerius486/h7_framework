#ifndef ALG_QUATERNION_H
#define ALG_QUATERNION_H
// 四元数结构体
typedef struct
{
  float w; // 标量分量
  float x; // x轴分量
  float y; // y轴分量
  float z; // z轴分量
} Quaternion;
#include "alg_euler.h"
Quaternion QuaternionMultiply(Quaternion q1, Quaternion q2);
Quaternion QuaternionNormalize(Quaternion q);
Quaternion EulerToQuaternion(Euler euler);
#endif // ALG_QUATERNION_H
