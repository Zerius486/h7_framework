#include "alg_imu_ekf.h"

#include "math.h"
#include "string.h"
/**
 * @brief 生成单位阵
 * @param data 目标矩阵数据指针
 * @param size 矩阵阶数
 */
static void EkfSetIdentity(float *data, uint16_t size) {
  uint16_t i;
  memset(data, 0, (uint32_t)size * size * sizeof(float));
  for (i = 0; i < size; i++) {
    data[(uint32_t)i * size + i] = 1.0f;
  }
}
/**
 * @brief 将角度归一化到[-pi, pi]
 * @param x 输入角度
 * @return 归一化后的角度
 */
static float EkfWrapPi(float x) {
  while (x > M_PI) {
    x -= 2.0f * M_PI;
  }
  while (x < -M_PI) {
    x += 2.0f * M_PI;
  }
  return x;
}
/**
 * @brief 安全的cos函数，避免分母接近0
 * @param x 输入角度
 * @return 保护后的cos值
 */
static float EkfSafeCos(float x) {
  float c = cosf(x);
  if (c > 0.0f && c < 1e-4f) {
    c = 1e-4f;
  } else if (c < 0.0f && c > -1e-4f) {
    c = -1e-4f;
  }
  return c;
}
/**
 * @brief 过程模型与状态雅可比
 * @param ekf ekf对象指针
 * @param gyro 陀螺仪输入
 */
static void EkfPredictModelAndJacobian(ImuEkfObject *ekf,
                                       const float gyro[kEkfImuInputSize]) {
  float *x = ekf->x_data;
  float *x_minus = ekf->x_minus_data;
  float *f = ekf->f_data;
  const float dt = ekf->dt;
  const float phi = x[0];
  const float theta = x[1];
  const float sin_phi = sinf(phi);
  const float cos_phi = cosf(phi);
  const float cos_theta = EkfSafeCos(theta);
  const float tan_theta = tanf(theta);
  const float sec_theta = 1.0f / cos_theta;
  const float sec_theta2 = sec_theta * sec_theta;
  const float p = gyro[0] - x[3];
  const float q = gyro[1] - x[4];
  const float r = gyro[2] - x[5];
  const float phi_dot = p + q * sin_phi * tan_theta + r * cos_phi * tan_theta;
  const float theta_dot = q * cos_phi - r * sin_phi;
  const float psi_dot = q * sin_phi * sec_theta + r * cos_phi * sec_theta;
  x_minus[0] = EkfWrapPi(phi + dt * phi_dot);
  x_minus[1] = EkfWrapPi(theta + dt * theta_dot);
  x_minus[2] = EkfWrapPi(x[2] + dt * psi_dot);
  x_minus[3] = x[3];
  x_minus[4] = x[4];
  x_minus[5] = x[5];
  EkfSetIdentity(f, kEkfImuStateSize);
  f[0 * kEkfImuStateSize + 0] =
      1.0f + dt * (q * cos_phi * tan_theta - r * sin_phi * tan_theta);
  f[0 * kEkfImuStateSize + 1] = dt * (q * sin_phi + r * cos_phi) * sec_theta2;
  f[0 * kEkfImuStateSize + 3] = -dt;
  f[0 * kEkfImuStateSize + 4] = -dt * sin_phi * tan_theta;
  f[0 * kEkfImuStateSize + 5] = -dt * cos_phi * tan_theta;
  f[1 * kEkfImuStateSize + 0] = dt * (-q * sin_phi - r * cos_phi);
  f[1 * kEkfImuStateSize + 4] = -dt * cos_phi;
  f[1 * kEkfImuStateSize + 5] = dt * sin_phi;
  f[2 * kEkfImuStateSize + 0] = dt * (q * cos_phi - r * sin_phi) * sec_theta;
  f[2 * kEkfImuStateSize + 1] =
      dt * (q * sin_phi + r * cos_phi) * sec_theta * tan_theta;
  f[2 * kEkfImuStateSize + 4] = -dt * sin_phi * sec_theta;
  f[2 * kEkfImuStateSize + 5] = -dt * cos_phi * sec_theta;
}
/**
 * @brief 观测模型与观测雅可比
 * @param ekf ekf对象指针
 */
static void EkfMeasurementModelAndJacobian(ImuEkfObject *ekf) {
  float *x_minus = ekf->x_minus_data;
  float *z_pred = ekf->z_pred_data;
  float *h = ekf->h_data;
  const float phi = x_minus[0];
  const float theta = x_minus[1];
  const float g = ekf->gravity;
  const float sin_phi = sinf(phi);
  const float cos_phi = cosf(phi);
  const float sin_theta = sinf(theta);
  const float cos_theta = cosf(theta);
  z_pred[0] = -g * sin_theta;
  z_pred[1] = g * sin_phi * cos_theta;
  z_pred[2] = g * cos_phi * cos_theta;
  memset(h, 0, kEkfImuMeasSize * kEkfImuStateSize * sizeof(float));
  h[0 * kEkfImuStateSize + 1] = -g * cos_theta;
  h[1 * kEkfImuStateSize + 0] = g * cos_phi * cos_theta;
  h[1 * kEkfImuStateSize + 1] = -g * sin_phi * sin_theta;
  h[2 * kEkfImuStateSize + 0] = -g * sin_phi * cos_theta;
  h[2 * kEkfImuStateSize + 1] = -g * cos_phi * sin_theta;
}
/**
 * @brief 初始化imu_ekf
 * @param ekf ekf对象指针
 * @param dt 采样周期
 * @param gravity 重力常数
 * @return 0表示成功，-1表示失败
 */
int8_t ImuEkfInit(ImuEkfObject *ekf, float dt, float gravity) {
  uint8_t i;
  if ((ekf == NULL) || (dt <= 0.0f)) {
    return -1;
  }
  memset(ekf, 0, sizeof(*ekf));
  ekf->dt = dt;
  ekf->gravity = (gravity > 0.0f) ? gravity : 9.81f;
  arm_mat_init_f32(&ekf->x, kEkfImuStateSize, 1U, ekf->x_data);
  arm_mat_init_f32(&ekf->x_minus, kEkfImuStateSize, 1U, ekf->x_minus_data);
  arm_mat_init_f32(&ekf->p, kEkfImuStateSize, kEkfImuStateSize, ekf->p_data);
  arm_mat_init_f32(&ekf->p_minus, kEkfImuStateSize, kEkfImuStateSize,
                   ekf->p_minus_data);
  arm_mat_init_f32(&ekf->f, kEkfImuStateSize, kEkfImuStateSize, ekf->f_data);
  arm_mat_init_f32(&ekf->f_t, kEkfImuStateSize, kEkfImuStateSize,
                   ekf->f_t_data);
  arm_mat_init_f32(&ekf->h, kEkfImuMeasSize, kEkfImuStateSize, ekf->h_data);
  arm_mat_init_f32(&ekf->h_t, kEkfImuStateSize, kEkfImuMeasSize, ekf->h_t_data);
  arm_mat_init_f32(&ekf->q, kEkfImuStateSize, kEkfImuStateSize, ekf->q_data);
  arm_mat_init_f32(&ekf->r, kEkfImuMeasSize, kEkfImuMeasSize, ekf->r_data);
  arm_mat_init_f32(&ekf->k, kEkfImuStateSize, kEkfImuMeasSize, ekf->k_data);
  arm_mat_init_f32(&ekf->s, kEkfImuMeasSize, kEkfImuMeasSize, ekf->s_data);
  arm_mat_init_f32(&ekf->i, kEkfImuStateSize, kEkfImuStateSize, ekf->i_data);
  arm_mat_init_f32(&ekf->z, kEkfImuMeasSize, 1U, ekf->z_data);
  arm_mat_init_f32(&ekf->z_pred, kEkfImuMeasSize, 1U, ekf->z_pred_data);
  arm_mat_init_f32(&ekf->y, kEkfImuMeasSize, 1U, ekf->y_data);
  arm_mat_init_f32(&ekf->temp_xx_1, kEkfImuStateSize, kEkfImuStateSize,
                   ekf->temp_xx_1_data);
  arm_mat_init_f32(&ekf->temp_xx_2, kEkfImuStateSize, kEkfImuStateSize,
                   ekf->temp_xx_2_data);
  arm_mat_init_f32(&ekf->temp_xz, kEkfImuStateSize, kEkfImuMeasSize,
                   ekf->temp_xz_data);
  arm_mat_init_f32(&ekf->temp_zx, kEkfImuMeasSize, kEkfImuStateSize,
                   ekf->temp_zx_data);
  arm_mat_init_f32(&ekf->temp_zz, kEkfImuMeasSize, kEkfImuMeasSize,
                   ekf->temp_zz_data);
  arm_mat_init_f32(&ekf->temp_x1, kEkfImuStateSize, 1U, ekf->temp_x1_data);
  EkfSetIdentity(ekf->i_data, kEkfImuStateSize);
  EkfSetIdentity(ekf->p_data, kEkfImuStateSize);
  memset(ekf->q_data, 0, sizeof(ekf->q_data));
  memset(ekf->r_data, 0, sizeof(ekf->r_data));
  for (i = 0; i < kEkfImuStateSize; i++) {
    ekf->q_data[(uint32_t)i * kEkfImuStateSize + i] = (i < 3U) ? 1e-4f : 1e-6f;
  }
  for (i = 0; i < kEkfImuMeasSize; i++) {
    ekf->r_data[(uint32_t)i * kEkfImuMeasSize + i] = 0.5f;
  }
  memset(ekf->corrected_gyro_data, 0, sizeof(ekf->corrected_gyro_data));
  ekf->is_inited = 1U;
  return 0;
}
/**
 * @brief 重置EKF状态
 * @param ekf EKF对象指针
 */
void ImuEkfReset(ImuEkfObject *ekf) {
  if ((ekf == NULL) || (ekf->is_inited == 0U)) {
    return;
  }
  memset(ekf->x_data, 0, sizeof(ekf->x_data));
  memset(ekf->x_minus_data, 0, sizeof(ekf->x_minus_data));
  EkfSetIdentity(ekf->p_data, kEkfImuStateSize);
  memset(ekf->p_minus_data, 0, sizeof(ekf->p_minus_data));
  memset(ekf->corrected_gyro_data, 0, sizeof(ekf->corrected_gyro_data));
}
/**
 * @brief 设置Q/R对角线
 * @param ekf EKF对象指针
 * @param q_diag Q对角线，传NULL则不更新
 * @param r_diag R对角线，传NULL则不更新
 */
void ImuEkfSetQr(ImuEkfObject *ekf, const float q_diag[kEkfImuStateSize],
                 const float r_diag[kEkfImuMeasSize]) {
  uint8_t i;
  if ((ekf == NULL) || (ekf->is_inited == 0U)) {
    return;
  }
  if (q_diag != NULL) {
    memset(ekf->q_data, 0, sizeof(ekf->q_data));
    for (i = 0; i < kEkfImuStateSize; i++) {
      ekf->q_data[(uint32_t)i * kEkfImuStateSize + i] = q_diag[i];
    }
  }
  if (r_diag != NULL) {
    memset(ekf->r_data, 0, sizeof(ekf->r_data));
    for (i = 0; i < kEkfImuMeasSize; i++) {
      ekf->r_data[(uint32_t)i * kEkfImuMeasSize + i] = r_diag[i];
    }
  }
}

/**
 * @brief 设置EKF初始欧拉角
 * @param ekf EKF对象指针
 * @param roll 横滚角（rad）
 * @param pitch 俯仰角（rad）
 * @param yaw 偏航角（rad）
 */
void ImuEkfSetEuler(ImuEkfObject *ekf, float roll, float pitch, float yaw) {
  if ((ekf == NULL) || (ekf->is_inited == 0U)) {
    return;
  }

  ekf->x_data[0] = EkfWrapPi(roll);
  ekf->x_data[1] = EkfWrapPi(pitch);
  ekf->x_data[2] = EkfWrapPi(yaw);
  ekf->x_minus_data[0] = ekf->x_data[0];
  ekf->x_minus_data[1] = ekf->x_data[1];
  ekf->x_minus_data[2] = ekf->x_data[2];
}
/**
 * @brief 执行EKF
 * @param ekf EKF对象指针
 * @param gyro 陀螺仪输入
 * @param accel 加速度输入
 * @return ARM_MATH_SUCCESS表示成功
 */
arm_status ImuEkfStep(ImuEkfObject *ekf, const float gyro[kEkfImuInputSize],
                      const float accel[kEkfImuMeasSize]) {
  arm_status status;
  uint8_t i;
  if ((ekf == NULL) || (gyro == NULL) || (accel == NULL) ||
      (ekf->is_inited == 0U)) {
    return ARM_MATH_ARGUMENT_ERROR;
  }
  EkfPredictModelAndJacobian(ekf, gyro);
  status = arm_mat_trans_f32(&ekf->f, &ekf->f_t);
  if (status != ARM_MATH_SUCCESS) {
    return status;
  }
  status = arm_mat_mult_f32(&ekf->f, &ekf->p, &ekf->temp_xx_1);
  if (status != ARM_MATH_SUCCESS) {
    return status;
  }
  status = arm_mat_mult_f32(&ekf->temp_xx_1, &ekf->f_t, &ekf->p_minus);
  if (status != ARM_MATH_SUCCESS) {
    return status;
  }
  status = arm_mat_add_f32(&ekf->p_minus, &ekf->q, &ekf->p_minus);
  if (status != ARM_MATH_SUCCESS) {
    return status;
  }
  EkfMeasurementModelAndJacobian(ekf);
  for (i = 0; i < kEkfImuMeasSize; i++) {
    ekf->z_data[i] = accel[i];
  }
  status = arm_mat_trans_f32(&ekf->h, &ekf->h_t);
  if (status != ARM_MATH_SUCCESS) {
    return status;
  }
  status = arm_mat_mult_f32(&ekf->h, &ekf->p_minus, &ekf->temp_zx);
  if (status != ARM_MATH_SUCCESS) {
    return status;
  }
  status = arm_mat_mult_f32(&ekf->temp_zx, &ekf->h_t, &ekf->s);
  if (status != ARM_MATH_SUCCESS) {
    return status;
  }
  status = arm_mat_add_f32(&ekf->s, &ekf->r, &ekf->s);
  if (status != ARM_MATH_SUCCESS) {
    return status;
  }
  status = arm_mat_inverse_f32(&ekf->s, &ekf->temp_zz);
  if (status != ARM_MATH_SUCCESS) {
    return status;
  }
  status = arm_mat_mult_f32(&ekf->p_minus, &ekf->h_t, &ekf->temp_xz);
  if (status != ARM_MATH_SUCCESS) {
    return status;
  }
  status = arm_mat_mult_f32(&ekf->temp_xz, &ekf->temp_zz, &ekf->k);
  if (status != ARM_MATH_SUCCESS) {
    return status;
  }
  status = arm_mat_sub_f32(&ekf->z, &ekf->z_pred, &ekf->y);
  if (status != ARM_MATH_SUCCESS) {
    return status;
  }
  status = arm_mat_mult_f32(&ekf->k, &ekf->y, &ekf->temp_x1);
  if (status != ARM_MATH_SUCCESS) {
    return status;
  }
  status = arm_mat_add_f32(&ekf->x_minus, &ekf->temp_x1, &ekf->x);
  if (status != ARM_MATH_SUCCESS) {
    return status;
  }
  ekf->x_data[0] = EkfWrapPi(ekf->x_data[0]);
  ekf->x_data[1] = EkfWrapPi(ekf->x_data[1]);
  ekf->x_data[2] = EkfWrapPi(ekf->x_data[2]);
  status = arm_mat_mult_f32(&ekf->k, &ekf->h, &ekf->temp_xx_1);
  if (status != ARM_MATH_SUCCESS) {
    return status;
  }
  status = arm_mat_sub_f32(&ekf->i, &ekf->temp_xx_1, &ekf->temp_xx_2);
  if (status != ARM_MATH_SUCCESS) {
    return status;
  }
  status = arm_mat_mult_f32(&ekf->temp_xx_2, &ekf->p_minus, &ekf->p);
  if (status == ARM_MATH_SUCCESS) {
    ekf->corrected_gyro_data[0] = gyro[0] - ekf->x_data[3];
    ekf->corrected_gyro_data[1] = gyro[1] - ekf->x_data[4];
    ekf->corrected_gyro_data[2] = gyro[2] - ekf->x_data[5];
  }
  return status;
}
/**
 * @brief 获取欧拉角估计
 * @param ekf EKF对象指针
 * @param roll 横滚角输出
 * @param pitch 俯仰角输出
 * @param yaw 偏航角输出
 */
void ImuEkfGetEuler(const ImuEkfObject *ekf, float *roll, float *pitch,
                    float *yaw) {
  if ((ekf == NULL) || (ekf->is_inited == 0U)) {
    return;
  }
  if (roll != NULL) {
    *roll = ekf->x_data[0];
  }
  if (pitch != NULL) {
    *pitch = ekf->x_data[1];
  }
  if (yaw != NULL) {
    *yaw = ekf->x_data[2];
  }
}
/**
 * @brief 获取陀螺零偏估计
 * @param ekf EKF对象指针
 * @param bias 零偏输出
 */
void ImuEkfGetGyroBias(const ImuEkfObject *ekf, float bias[kEkfImuInputSize]) {
  if ((ekf == NULL) || (bias == NULL) || (ekf->is_inited == 0U)) {
    return;
  }
  bias[0] = ekf->x_data[3];
  bias[1] = ekf->x_data[4];
  bias[2] = ekf->x_data[5];
}
/**
 * @brief 获取状态估计
 * @param ekf EKF对象指针
 * @param euler 欧拉角输出
 * @param euler_rate 欧拉角速度输出
 */
void ImuEkfGetState(const ImuEkfObject *ekf, Euler *euler,
                    EulerRate *euler_rate) {
  if ((ekf == NULL) || (ekf->is_inited == 0U)) {
    return;
  }
  if (euler != NULL) {
    euler->roll = ekf->x_data[0];
    euler->pitch = ekf->x_data[1];
    euler->yaw = ekf->x_data[2];
  }
  if (euler_rate != NULL) {
    const float phi = ekf->x_data[0];
    const float theta = ekf->x_data[1];
    const float p = ekf->corrected_gyro_data[0];
    const float q = ekf->corrected_gyro_data[1];
    const float r = ekf->corrected_gyro_data[2];
    const float sin_phi = sinf(phi);
    const float cos_phi = cosf(phi);
    const float tan_theta = tanf(theta);
    const float sec_theta = 1.0f / EkfSafeCos(theta);
    euler_rate->roll_rate =
        p + q * sin_phi * tan_theta + r * cos_phi * tan_theta;
    euler_rate->pitch_rate = q * cos_phi - r * sin_phi;
    euler_rate->yaw_rate = q * sin_phi * sec_theta + r * cos_phi * sec_theta;
  }
}
