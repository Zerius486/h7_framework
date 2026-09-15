#ifndef ALG_QUATERNION_H
#define ALG_QUATERNION_H
// 四元数结构体
typedef struct {
  float w;
  float x;
  float y;
  float z;
} Quaternion;
#include "alg_euler.h"
Quaternion QuaternionMultiply(Quaternion q1, Quaternion q2);
Quaternion QuaternionNormalize(Quaternion q);
Quaternion EulerToQuaternion(Euler euler);
#endif  // ALG_QUATERNION_H
