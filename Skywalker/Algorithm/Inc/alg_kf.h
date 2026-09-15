#ifndef ALG_KF_H
#define ALG_KF_H
#include "arm_math.h"
#include "stdint.h"
#include "stdlib.h"
// 卡尔曼滤波器结构体
typedef struct {
  uint8_t xhat_size;
  uint8_t u_size;
  uint8_t z_size;
  arm_matrix_instance_f32 xhat;        // 状态估计
  arm_matrix_instance_f32 xhat_minus;  // 先验状态估计
  arm_matrix_instance_f32 u;           // 控制输入
  arm_matrix_instance_f32 z;           // 观测值
  arm_matrix_instance_f32 k;           // 卡尔曼增益
  arm_matrix_instance_f32 p;           // 误差协方差矩阵
  arm_matrix_instance_f32 p_minus;     // 先验误差协方差矩阵
  arm_matrix_instance_f32 f;           // 状态转移矩阵
  arm_matrix_instance_f32 f_t;         // 状态转移矩阵的转置
  arm_matrix_instance_f32 b;           // 控制输入矩阵
  arm_matrix_instance_f32 h;           // 观测矩阵
  arm_matrix_instance_f32 h_t;         // 观测矩阵的转置
  arm_matrix_instance_f32 r;           // 观测噪声协方差矩阵
  arm_matrix_instance_f32 q;           // 过程噪声协方差矩阵
  arm_matrix_instance_f32 s;           // 创新协方差
  arm_matrix_instance_f32 i;           // 单位阵 I
  arm_matrix_instance_f32 temp_xx_1;   // x*x 临时矩阵1
  arm_matrix_instance_f32 temp_xx_2;   // x*x 临时矩阵2
  arm_matrix_instance_f32 temp_xz;     // x*z 临时矩阵
  arm_matrix_instance_f32 temp_zx;     // z*x 临时矩阵
  arm_matrix_instance_f32 temp_zz;     // z*z 临时矩阵
  arm_matrix_instance_f32 temp_x1;     // x*1 临时向量
  arm_matrix_instance_f32 temp_z1;     // z*1 临时向量
  float *xhat_data;
  float *xhat_minus_data;
  float *u_data;
  float *z_data;
  float *k_data;
  float *p_data;
  float *p_minus_data;
  float *f_data;
  float *f_t_data;
  float *b_data;
  float *h_data;
  float *h_t_data;
  float *r_data;
  float *q_data;
  float *s_data;
  float *i_data;
  float *temp_xx_1_data;
  float *temp_xx_2_data;
  float *temp_xz_data;
  float *temp_zx_data;
  float *temp_zz_data;
  float *temp_x1_data;
  float *temp_z1_data;
  uint8_t is_inited;
} KfObject;
int8_t KfInit(KfObject *kf, uint8_t xhat_size, uint8_t u_size, uint8_t z_size);
void KfDeinit(KfObject *kf);
arm_status KfPredict(KfObject *kf);
arm_status KfUpdate(KfObject *kf, const float *z_measurement);
arm_status KfStep(KfObject *kf, const float *u, const float *z_measurement);
#endif  // ALG_KF_H