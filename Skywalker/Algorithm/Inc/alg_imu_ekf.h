#ifndef ALG_IMU_EKF_H
#define ALG_IMU_EKF_H
#include "alg_euler.h"
#include "arm_math.h"

enum {
  kEkfImuStateSize = 6U,
  kEkfImuMeasSize = 3U,
  kEkfImuInputSize = 3U,
};

// imu_ekf结构体
typedef struct {
  float dt;
  float gravity;
  uint8_t is_inited;
  arm_matrix_instance_f32 x;        // 当前状态
  arm_matrix_instance_f32 x_minus;  // 预测状态
  arm_matrix_instance_f32 p;        // 当前协方差
  arm_matrix_instance_f32 p_minus;  // 预测协方差
  arm_matrix_instance_f32 f;        // 状态雅可比 F
  arm_matrix_instance_f32 f_t;      // F 转置
  arm_matrix_instance_f32 h;        // 观测雅可比 H
  arm_matrix_instance_f32 h_t;      // H 转置
  arm_matrix_instance_f32 q;        // 过程噪声
  arm_matrix_instance_f32 r;        // 观测噪声
  arm_matrix_instance_f32 k;        // 卡尔曼增益
  arm_matrix_instance_f32 s;        // 创新协方差
  arm_matrix_instance_f32 i;        // 单位阵
  arm_matrix_instance_f32 z;        // 观测向量
  arm_matrix_instance_f32 z_pred;   // 预测观测
  arm_matrix_instance_f32 y;        // 创新向量
  arm_matrix_instance_f32 temp_xx_1;
  arm_matrix_instance_f32 temp_xx_2;
  arm_matrix_instance_f32 temp_xz;
  arm_matrix_instance_f32 temp_zx;
  arm_matrix_instance_f32 temp_zz;
  arm_matrix_instance_f32 temp_x1;
  float x_data[kEkfImuStateSize];
  float x_minus_data[kEkfImuStateSize];
  float p_data[kEkfImuStateSize * kEkfImuStateSize];
  float p_minus_data[kEkfImuStateSize * kEkfImuStateSize];
  float f_data[kEkfImuStateSize * kEkfImuStateSize];
  float f_t_data[kEkfImuStateSize * kEkfImuStateSize];
  float h_data[kEkfImuMeasSize * kEkfImuStateSize];
  float h_t_data[kEkfImuStateSize * kEkfImuMeasSize];
  float q_data[kEkfImuStateSize * kEkfImuStateSize];
  float r_data[kEkfImuMeasSize * kEkfImuMeasSize];
  float k_data[kEkfImuStateSize * kEkfImuMeasSize];
  float s_data[kEkfImuMeasSize * kEkfImuMeasSize];
  float i_data[kEkfImuStateSize * kEkfImuStateSize];
  float z_data[kEkfImuMeasSize];
  float z_pred_data[kEkfImuMeasSize];
  float y_data[kEkfImuMeasSize];
  float temp_xx_1_data[kEkfImuStateSize * kEkfImuStateSize];
  float temp_xx_2_data[kEkfImuStateSize * kEkfImuStateSize];
  float temp_xz_data[kEkfImuStateSize * kEkfImuMeasSize];
  float temp_zx_data[kEkfImuMeasSize * kEkfImuStateSize];
  float temp_zz_data[kEkfImuMeasSize * kEkfImuMeasSize];
  float temp_x1_data[kEkfImuStateSize];
  float corrected_gyro_data[kEkfImuInputSize];
} ImuEkfObject;
int8_t ImuEkfInit(ImuEkfObject *ekf, float dt, float gravity);
void ImuEkfReset(ImuEkfObject *ekf);
void ImuEkfSetQr(ImuEkfObject *ekf, const float q_diag[kEkfImuStateSize],
                 const float r_diag[kEkfImuMeasSize]);
void ImuEkfSetEuler(ImuEkfObject *ekf, float roll, float pitch, float yaw);
arm_status ImuEkfStep(ImuEkfObject *ekf, const float gyro[kEkfImuInputSize],
                      const float accel[kEkfImuMeasSize]);
void ImuEkfGetEuler(const ImuEkfObject *ekf, float *roll, float *pitch,
                    float *yaw);
void ImuEkfGetGyroBias(const ImuEkfObject *ekf, float bias[kEkfImuInputSize]);
void ImuEkfGetState(const ImuEkfObject *ekf, Euler *euler,
                    EulerRate *euler_rate);
#endif  // ALG_IMU_EKF_H
