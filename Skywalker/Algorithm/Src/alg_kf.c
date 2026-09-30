#include "alg_kf.h"

#include "arm_math.h"
#include "string.h"
/**
 * @brief 获取卡尔曼滤波器工作区大小
 * @param xhat_size 状态向量大小
 * @param u_size 控制向量大小
 * @param z_size 观测向量大小
 * @return 工作区所需float元素个数
 */
uint32_t KfWorkspaceSize(uint8_t xhat_size, uint8_t u_size, uint8_t z_size)
{
  uint32_t x = xhat_size;
  uint32_t u = u_size > 0U ? u_size : 1U;
  uint32_t z = z_size;
  return (2U * x + u + z + 8U * x * x + 5U * x * z + 3U * z * z + x + z);
}
/**
 * @brief 设置单位矩阵
 * @param data 矩阵数据指针
 * @param rows 矩阵行数
 * @param cols 矩阵列数
 */
static void KfSetIdentity(float *data, uint16_t rows, uint16_t cols)
{
  uint16_t i;
  uint16_t diag_size = rows < cols ? rows : cols;
  memset(data, 0, (uint32_t)rows * (uint32_t)cols * sizeof(float));
  for (i = 0; i < diag_size; i++)
  {
    data[(uint32_t)i * cols + i] = 1.0f;
  }
}
/**
 * @brief 复制向量数据
 * @param dst 目标向量指针
 * @param src 源向量指针
 * @param size 向量元素个数
 */
static void KfCopyVector(float *dst, const float *src, uint16_t size)
{
  if ((dst == NULL) || (src == NULL) || (size == 0U))
  {
    return;
  }
  memcpy(dst, src, (uint32_t)size * sizeof(float));
}
/**
 * @brief 释放卡尔曼滤波器资源
 * @param kf 卡尔曼滤波器对象指针
 */
void KfDeinit(KfObject *kf)
{
  if (kf == NULL)
  {
    return;
  }
  memset(kf, 0, sizeof(*kf));
}
/**
 * @brief 卡尔曼滤波器初始化函数
 * @param kf 卡尔曼滤波器对象指针
 * @param xhat_size 状态向量大小
 * @param u_size 控制向量大小
 * @param z_size 观测向量大小
 * @param workspace 工作区指针
 * @param workspace_size 工作区大小
 * @return 0表示成功，-1表示失败
 */
int8_t KfInit(KfObject *kf, uint8_t xhat_size, uint8_t u_size, uint8_t z_size,
              float *workspace, uint32_t workspace_size)
{
  uint8_t i;
  uint32_t offset = 0U;
  if ((kf == NULL) || (xhat_size == 0U) || (z_size == 0U))
  {
    return -1;
  }
  if (workspace == NULL || workspace_size <
                              KfWorkspaceSize(xhat_size, u_size, z_size))
  {
    return -1;
  }
  memset(kf, 0, sizeof(*kf));
  kf->xhat_size = xhat_size;
  kf->u_size = u_size;
  kf->z_size = z_size;
#define KF_ALLOC(pointer, count)                                             \
  do                                                                         \
  {                                                                          \
    (pointer) = &workspace[offset];                                          \
    memset((pointer), 0, (count) * sizeof(float));                           \
    offset += (count);                                                       \
  } while (0)
  KF_ALLOC(kf->xhat_data, xhat_size);
  KF_ALLOC(kf->xhat_minus_data, xhat_size);
  KF_ALLOC(kf->u_data, u_size > 0U ? u_size : 1U);
  KF_ALLOC(kf->z_data, z_size);
  KF_ALLOC(kf->k_data, (uint32_t)xhat_size * z_size);
  KF_ALLOC(kf->p_data, (uint32_t)xhat_size * xhat_size);
  KF_ALLOC(kf->p_minus_data, (uint32_t)xhat_size * xhat_size);
  KF_ALLOC(kf->f_data, (uint32_t)xhat_size * xhat_size);
  KF_ALLOC(kf->f_t_data, (uint32_t)xhat_size * xhat_size);
  KF_ALLOC(kf->b_data, (uint32_t)xhat_size * (u_size > 0U ? u_size : 1U));
  KF_ALLOC(kf->h_data, (uint32_t)z_size * xhat_size);
  KF_ALLOC(kf->h_t_data, (uint32_t)xhat_size * z_size);
  KF_ALLOC(kf->r_data, (uint32_t)z_size * z_size);
  KF_ALLOC(kf->q_data, (uint32_t)xhat_size * xhat_size);
  KF_ALLOC(kf->s_data, (uint32_t)z_size * z_size);
  KF_ALLOC(kf->i_data, (uint32_t)xhat_size * xhat_size);
  KF_ALLOC(kf->temp_xx_1_data, (uint32_t)xhat_size * xhat_size);
  KF_ALLOC(kf->temp_xx_2_data, (uint32_t)xhat_size * xhat_size);
  KF_ALLOC(kf->temp_xz_data, (uint32_t)xhat_size * z_size);
  KF_ALLOC(kf->temp_zx_data, (uint32_t)z_size * xhat_size);
  KF_ALLOC(kf->temp_zz_data, (uint32_t)z_size * z_size);
  KF_ALLOC(kf->temp_x1_data, xhat_size);
  KF_ALLOC(kf->temp_z1_data, z_size);
#undef KF_ALLOC
  arm_mat_init_f32(&kf->xhat, xhat_size, 1U, kf->xhat_data);
  arm_mat_init_f32(&kf->xhat_minus, xhat_size, 1U, kf->xhat_minus_data);
  arm_mat_init_f32(&kf->u, u_size > 0U ? u_size : 1U, 1U, kf->u_data);
  arm_mat_init_f32(&kf->z, z_size, 1U, kf->z_data);
  arm_mat_init_f32(&kf->k, xhat_size, z_size, kf->k_data);
  arm_mat_init_f32(&kf->p, xhat_size, xhat_size, kf->p_data);
  arm_mat_init_f32(&kf->p_minus, xhat_size, xhat_size, kf->p_minus_data);
  arm_mat_init_f32(&kf->f, xhat_size, xhat_size, kf->f_data);
  arm_mat_init_f32(&kf->f_t, xhat_size, xhat_size, kf->f_t_data);
  arm_mat_init_f32(&kf->b, xhat_size, u_size > 0U ? u_size : 1U, kf->b_data);
  arm_mat_init_f32(&kf->h, z_size, xhat_size, kf->h_data);
  arm_mat_init_f32(&kf->h_t, xhat_size, z_size, kf->h_t_data);
  arm_mat_init_f32(&kf->r, z_size, z_size, kf->r_data);
  arm_mat_init_f32(&kf->q, xhat_size, xhat_size, kf->q_data);
  arm_mat_init_f32(&kf->s, z_size, z_size, kf->s_data);
  arm_mat_init_f32(&kf->i, xhat_size, xhat_size, kf->i_data);
  arm_mat_init_f32(&kf->temp_xx_1, xhat_size, xhat_size, kf->temp_xx_1_data);
  arm_mat_init_f32(&kf->temp_xx_2, xhat_size, xhat_size, kf->temp_xx_2_data);
  arm_mat_init_f32(&kf->temp_xz, xhat_size, z_size, kf->temp_xz_data);
  arm_mat_init_f32(&kf->temp_zx, z_size, xhat_size, kf->temp_zx_data);
  arm_mat_init_f32(&kf->temp_zz, z_size, z_size, kf->temp_zz_data);
  arm_mat_init_f32(&kf->temp_x1, xhat_size, 1U, kf->temp_x1_data);
  arm_mat_init_f32(&kf->temp_z1, z_size, 1U, kf->temp_z1_data);
  KfSetIdentity(kf->f_data, xhat_size, xhat_size);
  KfSetIdentity(kf->i_data, xhat_size, xhat_size);
  memset(kf->h_data, 0, (uint32_t)z_size * xhat_size * sizeof(float));
  for (i = 0; (i < z_size) && (i < xhat_size); i++)
  {
    kf->h_data[(uint32_t)i * xhat_size + i] = 1.0f;
  }
  memset(kf->q_data, 0, (uint32_t)xhat_size * xhat_size * sizeof(float));
  memset(kf->r_data, 0, (uint32_t)z_size * z_size * sizeof(float));
  memset(kf->p_data, 0, (uint32_t)xhat_size * xhat_size * sizeof(float));
  for (i = 0; i < xhat_size; i++)
  {
    kf->q_data[(uint32_t)i * xhat_size + i] = 1e-3f;
    kf->p_data[(uint32_t)i * xhat_size + i] = 1.0f;
  }
  for (i = 0; i < z_size; i++)
  {
    kf->r_data[(uint32_t)i * z_size + i] = 1e-2f;
  }
  kf->is_inited = 1U;
  return 0;
}
/**
 * @brief 卡尔曼滤波器预测步骤
 * @param kf 卡尔曼滤波器对象指针
 * @return 执行状态，ARM_MATH_SUCCESS表示成功，其他值表示失败
 */
arm_status KfPredict(KfObject *kf)
{
  arm_status status;
  if ((kf == NULL) || (kf->is_inited == 0U))
  {
    return ARM_MATH_ARGUMENT_ERROR;
  }
  status = arm_mat_mult_f32(&kf->f, &kf->xhat, &kf->xhat_minus);
  if (status != ARM_MATH_SUCCESS)
  {
    return status;
  }
  if (kf->u_size > 0U)
  {
    status = arm_mat_mult_f32(&kf->b, &kf->u, &kf->temp_x1);
    if (status != ARM_MATH_SUCCESS)
    {
      return status;
    }
    status = arm_mat_add_f32(&kf->xhat_minus, &kf->temp_x1, &kf->xhat_minus);
    if (status != ARM_MATH_SUCCESS)
    {
      return status;
    }
  }
  status = arm_mat_trans_f32(&kf->f, &kf->f_t);
  if (status != ARM_MATH_SUCCESS)
  {
    return status;
  }
  status = arm_mat_mult_f32(&kf->f, &kf->p, &kf->temp_xx_1);
  if (status != ARM_MATH_SUCCESS)
  {
    return status;
  }
  status = arm_mat_mult_f32(&kf->temp_xx_1, &kf->f_t, &kf->p_minus);
  if (status != ARM_MATH_SUCCESS)
  {
    return status;
  }
  status = arm_mat_add_f32(&kf->p_minus, &kf->q, &kf->p_minus);
  return status;
}
/**
 * @brief 卡尔曼滤波器更新步骤
 * @param kf 卡尔曼滤波器对象指针
 * @param z_measurement 观测值指针
 * @return 执行状态，ARM_MATH_SUCCESS表示成功，其他值表示失败
 */
arm_status KfUpdate(KfObject *kf, const float *z_measurement)
{
  arm_status status;
  if ((kf == NULL) || (kf->is_inited == 0U) || (z_measurement == NULL))
  {
    return ARM_MATH_ARGUMENT_ERROR;
  }
  KfCopyVector(kf->z_data, z_measurement, kf->z_size);
  status = arm_mat_trans_f32(&kf->h, &kf->h_t);
  if (status != ARM_MATH_SUCCESS)
  {
    return status;
  }
  status = arm_mat_mult_f32(&kf->h, &kf->p_minus, &kf->temp_zx);
  if (status != ARM_MATH_SUCCESS)
  {
    return status;
  }
  status = arm_mat_mult_f32(&kf->temp_zx, &kf->h_t, &kf->s);
  if (status != ARM_MATH_SUCCESS)
  {
    return status;
  }
  status = arm_mat_add_f32(&kf->s, &kf->r, &kf->s);
  if (status != ARM_MATH_SUCCESS)
  {
    return status;
  }
  status = arm_mat_inverse_f32(&kf->s, &kf->temp_zz);
  if (status != ARM_MATH_SUCCESS)
  {
    return status;
  }
  status = arm_mat_mult_f32(&kf->p_minus, &kf->h_t, &kf->temp_xz);
  if (status != ARM_MATH_SUCCESS)
  {
    return status;
  }
  status = arm_mat_mult_f32(&kf->temp_xz, &kf->temp_zz, &kf->k);
  if (status != ARM_MATH_SUCCESS)
  {
    return status;
  }
  status = arm_mat_mult_f32(&kf->h, &kf->xhat_minus, &kf->temp_z1);
  if (status != ARM_MATH_SUCCESS)
  {
    return status;
  }
  status = arm_mat_sub_f32(&kf->z, &kf->temp_z1, &kf->temp_z1);
  if (status != ARM_MATH_SUCCESS)
  {
    return status;
  }
  status = arm_mat_mult_f32(&kf->k, &kf->temp_z1, &kf->temp_x1);
  if (status != ARM_MATH_SUCCESS)
  {
    return status;
  }
  status = arm_mat_add_f32(&kf->xhat_minus, &kf->temp_x1, &kf->xhat);
  if (status != ARM_MATH_SUCCESS)
  {
    return status;
  }
  status = arm_mat_mult_f32(&kf->k, &kf->h, &kf->temp_xx_1);
  if (status != ARM_MATH_SUCCESS)
  {
    return status;
  }
  status = arm_mat_sub_f32(&kf->i, &kf->temp_xx_1, &kf->temp_xx_2);
  if (status != ARM_MATH_SUCCESS)
  {
    return status;
  }
  status = arm_mat_mult_f32(&kf->temp_xx_2, &kf->p_minus, &kf->p);
  return status;
}
/**
 * @brief 卡尔曼滤波器单步执行函数
 * @param kf 卡尔曼滤波器对象指针
 * @param u 控制输入指针，如果为NULL则不更新控制输入
 * @param z_measurement 观测值指针
 * @return 执行状态，ARM_MATH_SUCCESS表示成功，其他值表示失败
 */
arm_status KfStep(KfObject *kf, const float *u, const float *z_measurement)
{
  arm_status status;
  if ((kf == NULL) || (kf->is_inited == 0U))
  {
    return ARM_MATH_ARGUMENT_ERROR;
  }
  if ((u != NULL) && (kf->u_size > 0U))
  {
    KfCopyVector(kf->u_data, u, kf->u_size);
  }
  status = KfPredict(kf);
  if (status != ARM_MATH_SUCCESS)
  {
    return status;
  }
  return KfUpdate(kf, z_measurement);
}
