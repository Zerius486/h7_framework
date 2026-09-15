#include "dev_bmi088.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

#include "bsp_pwm.h"
#include "bsp_spi.h"
#include "main.h"

enum {
  kBmi088SpiReadMask = 0x80U,
  kBmi088SpiDummyData = 0x55U,
  kBmi088SpiTimeoutMs = 2U,
  kBmi088LongDelayMs = 80U,
  kBmi088ComWaitMs = 1U,
  kBmi088AccelConfigCount = 6U,
  kBmi088GyroConfigCount = 6U,
};

enum {
  kBmi088AccelChipIdReg = 0x00U,
  kBmi088AccelChipIdValue = 0x1EU,
  kBmi088AccelXoutLReg = 0x12U,
  kBmi088TempMReg = 0x22U,
  kBmi088AccelConfigReg = 0x40U,
  kBmi088AccelRangeReg = 0x41U,
  kBmi088Int1IoCtrlReg = 0x53U,
  kBmi088IntMapDataReg = 0x58U,
  kBmi088AccelPowerConfigReg = 0x7CU,
  kBmi088AccelPowerCtrlReg = 0x7DU,
  kBmi088AccelSoftResetReg = 0x7EU,
};

enum {
  kBmi088GyroChipIdReg = 0x00U,
  kBmi088GyroChipIdValue = 0x0FU,
  kBmi088GyroXoutLReg = 0x02U,
  kBmi088GyroRangeReg = 0x0FU,
  kBmi088GyroBandwidthReg = 0x10U,
  kBmi088GyroPowerModeReg = 0x11U,
  kBmi088GyroSoftResetReg = 0x14U,
  kBmi088GyroCtrlReg = 0x15U,
  kBmi088GyroIntIoConfigReg = 0x16U,
  kBmi088GyroIntIoMapReg = 0x18U,
};

enum {
  kBmi088AccelPowerOn = 0x04U,
  kBmi088AccelPowerActive = 0x00U,
  kBmi088AccelConfigNormal1600Hz = 0x8CU,
  kBmi088AccelRange6gConfig = 0x01U,
  kBmi088AccelInt1DrdyLowPushPull = 0x08U,
  kBmi088AccelMapDrdyToInt1 = 0x04U,
  kBmi088AccelSoftReset = 0xB6U,
  kBmi088GyroRange2000DpsConfig = 0x00U,
  kBmi088GyroBandwidth2000Hz230Hz = 0x81U,
  kBmi088GyroNormalMode = 0x00U,
  kBmi088GyroDataReadyOn = 0x80U,
  kBmi088GyroInt3LowPushPull = 0x00U,
  kBmi088GyroMapDrdyToInt3 = 0x01U,
  kBmi088GyroSoftReset = 0xB6U,
};

static const float kBmi088TempFactor = 0.125f;
static const float kBmi088TempOffset = 23.0f;
static const float kBmi088AccelSensitivity6g = 0.00179443359375f;
static const float kBmi088GyroSensitivity2000Dps = 0.0010652644360316953f;
static const float kBmi088DefaultGravity = 9.81f;
static const float kBmi088DefaultMotionAccelLpf = 0.0085f;
static const float kBmi088DefaultGNormMotionThreshold = 0.5f;
static const float kBmi088DefaultGyroMotionThreshold = 0.15f;

typedef struct {
  uint8_t reg;         // 寄存器地址
  uint8_t value;       // 目标配置值
  Bmi088Status error;  // 配置失败时返回的错误码
} Bmi088RegisterConfig;

Bmi088Object g_bmi088 = {0};

static const Bmi088RegisterConfig kBmi088AccelConfig[kBmi088AccelConfigCount] =
    {
        {kBmi088AccelPowerCtrlReg, kBmi088AccelPowerOn,
         kBmi088AccelPowerControlError},
        {kBmi088AccelPowerConfigReg, kBmi088AccelPowerActive,
         kBmi088AccelPowerConfigError},
        {kBmi088AccelConfigReg, kBmi088AccelConfigNormal1600Hz,
         kBmi088AccelConfigError},
        {kBmi088AccelRangeReg, kBmi088AccelRange6gConfig,
         kBmi088AccelRangeError},
        {kBmi088Int1IoCtrlReg, kBmi088AccelInt1DrdyLowPushPull,
         kBmi088AccelIntConfigError},
        {kBmi088IntMapDataReg, kBmi088AccelMapDrdyToInt1,
         kBmi088AccelIntMapError},
};

static const Bmi088RegisterConfig kBmi088GyroConfig[kBmi088GyroConfigCount] = {
    {kBmi088GyroRangeReg, kBmi088GyroRange2000DpsConfig, kBmi088GyroRangeError},
    {kBmi088GyroBandwidthReg, kBmi088GyroBandwidth2000Hz230Hz,
     kBmi088GyroBandwidthError},
    {kBmi088GyroPowerModeReg, kBmi088GyroNormalMode, kBmi088GyroPowerModeError},
    {kBmi088GyroCtrlReg, kBmi088GyroDataReadyOn, kBmi088GyroDataReadyError},
    {kBmi088GyroIntIoConfigReg, kBmi088GyroInt3LowPushPull,
     kBmi088GyroIntConfigError},
    {kBmi088GyroIntIoMapReg, kBmi088GyroMapDrdyToInt3, kBmi088GyroIntMapError},
};

/**
 * @brief 初始化BMI088 IMU默认配置
 * @param config IMU配置结构体指针
 * @param dt EKF采样周期（s）
 */
void Bmi088ImuConfigInit(Bmi088ImuConfig *config, float dt) {
  if (config == NULL) {
    return;
  }

  memset(config, 0, sizeof(*config));
  config->dt = dt;
  config->gravity = kBmi088DefaultGravity;
  config->is_init_euler_from_accel_enabled = true;
  config->init_euler_sample_count = 100U;
  config->init_euler_sample_delay_ms = 1U;
  config->motion_accel_lpf = kBmi088DefaultMotionAccelLpf;
  config->g_norm_motion_threshold = kBmi088DefaultGNormMotionThreshold;
  config->gyro_motion_threshold = kBmi088DefaultGyroMotionThreshold;
}

/**
 * @brief 选中BMI088加速度计片选
 */
static void Bmi088AccelSelect(void) {
  HAL_GPIO_WritePin(BMI088_ACCEL_CS_GPIO_Port, BMI088_ACCEL_CS_Pin,
                    GPIO_PIN_RESET);
}

/**
 * @brief 释放BMI088加速度计片选
 */
static void Bmi088AccelDeselect(void) {
  HAL_GPIO_WritePin(BMI088_ACCEL_CS_GPIO_Port, BMI088_ACCEL_CS_Pin,
                    GPIO_PIN_SET);
}

/**
 * @brief 选中BMI088陀螺仪片选
 */
static void Bmi088GyroSelect(void) {
  HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin,
                    GPIO_PIN_RESET);
}

/**
 * @brief 释放BMI088陀螺仪片选
 */
static void Bmi088GyroDeselect(void) {
  HAL_GPIO_WritePin(BMI088_GYRO_CS_GPIO_Port, BMI088_GYRO_CS_Pin, GPIO_PIN_SET);
}

/**
 * @brief 通过SPI阻塞传输单字节
 * @param hspi SPI句柄
 * @param data 发送字节
 * @return 接收字节
 */
static uint8_t Bmi088TransferByte(SPI_HandleTypeDef *hspi, uint8_t data) {
  uint8_t rx_data = 0;
  (void)SpiTransmitReceiveBlocking(hspi, &data, &rx_data, 1U,
                                   kBmi088SpiTimeoutMs);
  return rx_data;
}

/**
 * @brief 写BMI088寄存器
 * @param hspi SPI句柄
 * @param reg 寄存器地址
 * @param data 写入数据
 */
static void Bmi088WriteRegister(SPI_HandleTypeDef *hspi, uint8_t reg,
                                uint8_t data) {
  uint8_t tx_data[2] = {reg, data};
  uint8_t rx_data[2] = {0};
  (void)SpiTransmitReceiveBlocking(hspi, tx_data, rx_data, sizeof(tx_data),
                                   kBmi088SpiTimeoutMs);
}

/**
 * @brief 读取陀螺仪寄存器
 * @param hspi SPI句柄
 * @param reg 寄存器地址
 * @return 寄存器值
 */
static uint8_t Bmi088ReadGyroRegister(SPI_HandleTypeDef *hspi, uint8_t reg) {
  uint8_t tx_data[2] = {reg | kBmi088SpiReadMask, kBmi088SpiDummyData};
  uint8_t rx_data[2] = {0};
  (void)SpiTransmitReceiveBlocking(hspi, tx_data, rx_data, sizeof(tx_data),
                                   kBmi088SpiTimeoutMs);
  return rx_data[1];
}

/**
 * @brief 读取加速度计寄存器
 * @param hspi SPI句柄
 * @param reg 寄存器地址
 * @return 寄存器值
 */
static uint8_t Bmi088ReadAccelRegister(SPI_HandleTypeDef *hspi, uint8_t reg) {
  uint8_t value = 0;

  (void)Bmi088TransferByte(hspi, reg | kBmi088SpiReadMask);
  (void)Bmi088TransferByte(hspi, kBmi088SpiDummyData);
  value = Bmi088TransferByte(hspi, kBmi088SpiDummyData);

  return value;
}

/**
 * @brief 连续读取陀螺仪寄存器
 * @param hspi SPI句柄
 * @param reg 起始寄存器地址
 * @param data 输出数据缓冲区
 * @param length 读取长度
 */
static void Bmi088ReadGyroRegisters(SPI_HandleTypeDef *hspi, uint8_t reg,
                                    uint8_t *data, uint8_t length) {
  if (data == NULL || length == 0U) {
    return;
  }

  (void)Bmi088TransferByte(hspi, reg | kBmi088SpiReadMask);
  for (uint8_t i = 0; i < length; i++) {
    data[i] = Bmi088TransferByte(hspi, kBmi088SpiDummyData);
  }
}

/**
 * @brief 连续读取加速度计寄存器
 * @param hspi SPI句柄
 * @param reg 起始寄存器地址
 * @param data 输出数据缓冲区
 * @param length 读取长度
 */
static void Bmi088ReadAccelRegisters(SPI_HandleTypeDef *hspi, uint8_t reg,
                                     uint8_t *data, uint8_t length) {
  if (data == NULL || length == 0U) {
    return;
  }

  (void)Bmi088TransferByte(hspi, reg | kBmi088SpiReadMask);
  (void)Bmi088TransferByte(hspi, kBmi088SpiDummyData);
  for (uint8_t i = 0; i < length; i++) {
    data[i] = Bmi088TransferByte(hspi, kBmi088SpiDummyData);
  }
}

/**
 * @brief 带片选控制写加速度计寄存器
 */
static void Bmi088AccelWrite(SPI_HandleTypeDef *hspi, uint8_t reg,
                             uint8_t data) {
  Bmi088AccelSelect();
  Bmi088WriteRegister(hspi, reg, data);
  Bmi088AccelDeselect();
}

/**
 * @brief 带片选控制读加速度计寄存器
 */
static uint8_t Bmi088AccelRead(SPI_HandleTypeDef *hspi, uint8_t reg) {
  Bmi088AccelSelect();
  uint8_t value = Bmi088ReadAccelRegister(hspi, reg);
  Bmi088AccelDeselect();
  return value;
}

/**
 * @brief 带片选控制连续读加速度计寄存器
 */
static void Bmi088AccelReadMulti(SPI_HandleTypeDef *hspi, uint8_t reg,
                                 uint8_t *data, uint8_t length) {
  Bmi088AccelSelect();
  Bmi088ReadAccelRegisters(hspi, reg, data, length);
  Bmi088AccelDeselect();
}

/**
 * @brief 带片选控制写陀螺仪寄存器
 */
static void Bmi088GyroWrite(SPI_HandleTypeDef *hspi, uint8_t reg,
                            uint8_t data) {
  Bmi088GyroSelect();
  Bmi088WriteRegister(hspi, reg, data);
  Bmi088GyroDeselect();
}

/**
 * @brief 带片选控制读陀螺仪寄存器
 */
static uint8_t Bmi088GyroRead(SPI_HandleTypeDef *hspi, uint8_t reg) {
  Bmi088GyroSelect();
  uint8_t value = Bmi088ReadGyroRegister(hspi, reg);
  Bmi088GyroDeselect();
  return value;
}

/**
 * @brief 带片选控制连续读陀螺仪寄存器
 */
static void Bmi088GyroReadMulti(SPI_HandleTypeDef *hspi, uint8_t reg,
                                uint8_t *data, uint8_t length) {
  Bmi088GyroSelect();
  Bmi088ReadGyroRegisters(hspi, reg, data, length);
  Bmi088GyroDeselect();
}

/**
 * @brief 初始化BMI088加速度计
 * @param hspi SPI句柄
 * @return 初始化状态
 */
static Bmi088Status Bmi088InitAccel(SPI_HandleTypeDef *hspi) {
  uint8_t chip_id = Bmi088AccelRead(hspi, kBmi088AccelChipIdReg);
  HAL_Delay(kBmi088ComWaitMs);
  chip_id = Bmi088AccelRead(hspi, kBmi088AccelChipIdReg);
  HAL_Delay(kBmi088ComWaitMs);

  Bmi088AccelWrite(hspi, kBmi088AccelSoftResetReg, kBmi088AccelSoftReset);
  HAL_Delay(kBmi088LongDelayMs);

  chip_id = Bmi088AccelRead(hspi, kBmi088AccelChipIdReg);
  HAL_Delay(kBmi088ComWaitMs);
  chip_id = Bmi088AccelRead(hspi, kBmi088AccelChipIdReg);
  HAL_Delay(kBmi088ComWaitMs);
  if (chip_id != kBmi088AccelChipIdValue) {
    return kBmi088NoSensor;
  }

  for (uint8_t i = 0; i < kBmi088AccelConfigCount; i++) {
    Bmi088AccelWrite(hspi, kBmi088AccelConfig[i].reg,
                     kBmi088AccelConfig[i].value);
    HAL_Delay(kBmi088ComWaitMs);
    if (Bmi088AccelRead(hspi, kBmi088AccelConfig[i].reg) !=
        kBmi088AccelConfig[i].value) {
      return kBmi088AccelConfig[i].error;
    }
  }

  return kBmi088NoError;
}

/**
 * @brief 初始化BMI088陀螺仪
 * @param hspi SPI句柄
 * @return 初始化状态
 */
static Bmi088Status Bmi088InitGyro(SPI_HandleTypeDef *hspi) {
  uint8_t chip_id = Bmi088GyroRead(hspi, kBmi088GyroChipIdReg);
  HAL_Delay(kBmi088ComWaitMs);
  chip_id = Bmi088GyroRead(hspi, kBmi088GyroChipIdReg);
  HAL_Delay(kBmi088ComWaitMs);

  Bmi088GyroWrite(hspi, kBmi088GyroSoftResetReg, kBmi088GyroSoftReset);
  HAL_Delay(kBmi088LongDelayMs);

  chip_id = Bmi088GyroRead(hspi, kBmi088GyroChipIdReg);
  HAL_Delay(kBmi088ComWaitMs);
  chip_id = Bmi088GyroRead(hspi, kBmi088GyroChipIdReg);
  HAL_Delay(kBmi088ComWaitMs);
  if (chip_id != kBmi088GyroChipIdValue) {
    return kBmi088NoSensor;
  }

  for (uint8_t i = 0; i < kBmi088GyroConfigCount; i++) {
    Bmi088GyroWrite(hspi, kBmi088GyroConfig[i].reg, kBmi088GyroConfig[i].value);
    HAL_Delay(kBmi088ComWaitMs);
    if (Bmi088GyroRead(hspi, kBmi088GyroConfig[i].reg) !=
        kBmi088GyroConfig[i].value) {
      return kBmi088GyroConfig[i].error;
    }
  }

  return kBmi088NoError;
}

/**
 * @brief 将低字节和高字节合成为int16_t
 * @param low 低字节
 * @param high 高字节
 * @return 合成后的有符号整数
 */
static int16_t Bmi088BytesToInt16(uint8_t low, uint8_t high) {
  return (int16_t)((uint16_t)low | ((uint16_t)high << 8));
}

/**
 * @brief 计算三维向量模长
 * @param vector 三维向量
 * @return 向量模长
 */
static float Bmi088VectorNorm3(const float vector[3]) {
  return sqrtf(vector[0] * vector[0] + vector[1] * vector[1] +
               vector[2] * vector[2]);
}

/**
 * @brief 通过安装矩阵修正三轴数据
 * @param matrix 3x3安装矩阵
 * @param input 输入三轴数据
 * @param output 输出三轴数据
 */
static void Bmi088ApplyInstallMatrix(const float matrix[9],
                                     const float input[3], float output[3]) {
  float temp[3] = {0.0f, 0.0f, 0.0f};

  for (uint8_t row = 0; row < 3U; row++) {
    temp[row] = matrix[row * 3U + 0U] * input[0] +
                matrix[row * 3U + 1U] * input[1] +
                matrix[row * 3U + 2U] * input[2];
  }
  memcpy(output, temp, sizeof(temp));
}

/**
 * @brief 将机体系向量转换到地面系
 * @param euler 当前欧拉角
 * @param body_vector 机体系向量
 * @param earth_vector 地面系向量
 */
static void Bmi088BodyToEarth(const Euler *euler, const float body_vector[3],
                              float earth_vector[3]) {
  float sin_roll = sinf(euler->roll);
  float cos_roll = cosf(euler->roll);
  float sin_pitch = sinf(euler->pitch);
  float cos_pitch = cosf(euler->pitch);
  float sin_yaw = sinf(euler->yaw);
  float cos_yaw = cosf(euler->yaw);

  earth_vector[0] =
      cos_yaw * cos_pitch * body_vector[0] +
      (cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll) * body_vector[1] +
      (cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll) * body_vector[2];
  earth_vector[1] =
      sin_yaw * cos_pitch * body_vector[0] +
      (sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll) * body_vector[1] +
      (sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll) * body_vector[2];
  earth_vector[2] = -sin_pitch * body_vector[0] +
                    cos_pitch * sin_roll * body_vector[1] +
                    cos_pitch * cos_roll * body_vector[2];
}

/**
 * @brief 将地面系向量转换到机体系
 * @param euler 当前欧拉角
 * @param earth_vector 地面系向量
 * @param body_vector 机体系向量
 */
static void Bmi088EarthToBody(const Euler *euler, const float earth_vector[3],
                              float body_vector[3]) {
  float sin_roll = sinf(euler->roll);
  float cos_roll = cosf(euler->roll);
  float sin_pitch = sinf(euler->pitch);
  float cos_pitch = cosf(euler->pitch);
  float sin_yaw = sinf(euler->yaw);
  float cos_yaw = cosf(euler->yaw);

  body_vector[0] = cos_yaw * cos_pitch * earth_vector[0] +
                   sin_yaw * cos_pitch * earth_vector[1] -
                   sin_pitch * earth_vector[2];
  body_vector[1] =
      (cos_yaw * sin_pitch * sin_roll - sin_yaw * cos_roll) * earth_vector[0] +
      (sin_yaw * sin_pitch * sin_roll + cos_yaw * cos_roll) * earth_vector[1] +
      cos_pitch * sin_roll * earth_vector[2];
  body_vector[2] =
      (cos_yaw * sin_pitch * cos_roll + sin_yaw * sin_roll) * earth_vector[0] +
      (sin_yaw * sin_pitch * cos_roll - cos_yaw * sin_roll) * earth_vector[1] +
      cos_pitch * cos_roll * earth_vector[2];
}

/**
 * @brief 读取加速度计原始数据
 * @param bmi088 BMI088对象指针
 */
static void Bmi088ReadRawAccel(Bmi088Object *bmi088) {
  uint8_t data[6] = {0};
  Bmi088AccelReadMulti(bmi088->hspi, kBmi088AccelXoutLReg, data, sizeof(data));
  bmi088->raw.accel[0] = Bmi088BytesToInt16(data[0], data[1]);
  bmi088->raw.accel[1] = Bmi088BytesToInt16(data[2], data[3]);
  bmi088->raw.accel[2] = Bmi088BytesToInt16(data[4], data[5]);
}

/**
 * @brief 读取陀螺仪原始数据
 * @param bmi088 BMI088对象指针
 */
static void Bmi088ReadRawGyro(Bmi088Object *bmi088) {
  uint8_t data[8] = {0};
  Bmi088GyroReadMulti(bmi088->hspi, kBmi088GyroChipIdReg, data, sizeof(data));
  if (data[0] != kBmi088GyroChipIdValue) {
    return;
  }

  bmi088->raw.gyro[0] = Bmi088BytesToInt16(data[2], data[3]);
  bmi088->raw.gyro[1] = Bmi088BytesToInt16(data[4], data[5]);
  bmi088->raw.gyro[2] = Bmi088BytesToInt16(data[6], data[7]);
}

/**
 * @brief 读取温度原始数据
 * @param bmi088 BMI088对象指针
 */
static void Bmi088ReadRawTemperature(Bmi088Object *bmi088) {
  uint8_t data[2] = {0};
  int16_t temperature = 0;

  Bmi088AccelReadMulti(bmi088->hspi, kBmi088TempMReg, data, sizeof(data));
  temperature = (int16_t)(((uint16_t)data[0] << 3) | ((uint16_t)data[1] >> 5));
  if (temperature > 1023) {
    temperature -= 2048;
  }
  bmi088->raw.temperature = temperature;
}

/**
 * @brief 将BMI088原始数据转换为物理量
 * @param bmi088 BMI088对象指针
 */
static void Bmi088UpdateScaledData(Bmi088Object *bmi088) {
  for (uint8_t i = 0; i < 3U; i++) {
    bmi088->accel[i] =
        bmi088->accel_sensitivity * bmi088->accel_scale * bmi088->raw.accel[i];
    bmi088->gyro[i] =
        bmi088->gyro_sensitivity * bmi088->raw.gyro[i] - bmi088->gyro_offset[i];
  }
  bmi088->temperature =
      bmi088->raw.temperature * kBmi088TempFactor + kBmi088TempOffset;
}

/**
 * @brief 更新安装误差修正后的IMU数据
 * @param bmi088 BMI088对象指针
 */
static void Bmi088UpdateCorrectedData(Bmi088Object *bmi088) {
  Bmi088ApplyInstallMatrix(bmi088->install_matrix, bmi088->accel,
                           bmi088->corrected_accel);
  Bmi088ApplyInstallMatrix(bmi088->install_matrix, bmi088->gyro,
                           bmi088->corrected_gyro);
}

/**
 * @brief 根据当前姿态扣除重力并更新运动加速度
 * @param bmi088 BMI088对象指针
 */
static void Bmi088UpdateMotionAccel(Bmi088Object *bmi088) {
  float gravity_earth[3] = {0.0f, 0.0f, bmi088->imu_ekf.gravity};
  float gravity_body[3] = {0.0f, 0.0f, 0.0f};
  float motion_body_raw[3] = {0.0f, 0.0f, 0.0f};
  float alpha = 1.0f;

  Bmi088EarthToBody(&bmi088->euler, gravity_earth, gravity_body);
  if (bmi088->motion_accel_lpf > 0.0f && bmi088->imu_ekf.dt > 0.0f) {
    alpha =
        bmi088->imu_ekf.dt / (bmi088->motion_accel_lpf + bmi088->imu_ekf.dt);
  }

  for (uint8_t i = 0; i < 3U; i++) {
    motion_body_raw[i] = bmi088->corrected_accel[i] - gravity_body[i];
    bmi088->motion_accel_body[i] =
        alpha * motion_body_raw[i] +
        (1.0f - alpha) * bmi088->motion_accel_body[i];
  }
  Bmi088BodyToEarth(&bmi088->euler, bmi088->motion_accel_body,
                    bmi088->motion_accel_earth);
}

/**
 * @brief 从多次加速度均值估计初始roll/pitch
 * @param bmi088 BMI088对象指针
 * @param sample_count 采样次数
 * @param sample_delay_ms 采样间隔（ms）
 */
static void Bmi088InitEulerFromAccel(Bmi088Object *bmi088,
                                     uint16_t sample_count,
                                     uint32_t sample_delay_ms) {
  float accel_sum[3] = {0.0f, 0.0f, 0.0f};

  if (sample_count == 0U) {
    return;
  }

  for (uint16_t sample = 0; sample < sample_count; sample++) {
    (void)Bmi088Update(bmi088);
    for (uint8_t i = 0; i < 3U; i++) {
      accel_sum[i] += bmi088->corrected_accel[i];
    }
    if (sample_delay_ms > 0U) {
      HAL_Delay(sample_delay_ms);
    }
  }
  for (uint8_t i = 0; i < 3U; i++) {
    accel_sum[i] /= (float)sample_count;
  }

  float roll = atan2f(accel_sum[1], accel_sum[2]);
  float pitch = atan2f(-accel_sum[0], sqrtf(accel_sum[1] * accel_sum[1] +
                                            accel_sum[2] * accel_sum[2]));
  ImuEkfSetEuler(&bmi088->imu_ekf, roll, pitch, 0.0f);
  ImuEkfGetState(&bmi088->imu_ekf, &bmi088->euler, &bmi088->euler_rate);
}

/**
 * @brief 初始化BMI088对象并配置加速度计、陀螺仪
 * @param bmi088 BMI088对象指针
 * @param hspi SPI句柄
 * @return 初始化状态
 */
Bmi088Status Bmi088Init(Bmi088Object *bmi088, SPI_HandleTypeDef *hspi) {
  if (bmi088 == NULL || hspi == NULL) {
    return kBmi088NoSensor;
  }

  memset(bmi088, 0, sizeof(*bmi088));
  bmi088->hspi = hspi;
  bmi088->accel_range = kBmi088AccelRange6g;
  bmi088->gyro_range = kBmi088GyroRange2000Dps;
  bmi088->accel_sensitivity = kBmi088AccelSensitivity6g;
  bmi088->gyro_sensitivity = kBmi088GyroSensitivity2000Dps;
  bmi088->accel_scale = 1.0f;
  bmi088->g_norm = kBmi088DefaultGravity;
  bmi088->motion_accel_lpf = kBmi088DefaultMotionAccelLpf;
  Bmi088ImuSetInstallMatrix(bmi088, NULL);

  Bmi088AccelDeselect();
  Bmi088GyroDeselect();

  bmi088->status = Bmi088InitAccel(hspi);
  if (bmi088->status != kBmi088NoError) {
    return bmi088->status;
  }

  bmi088->status = Bmi088InitGyro(hspi);
  if (bmi088->status != kBmi088NoError) {
    return bmi088->status;
  }

  bmi088->is_inited = true;
  return kBmi088NoError;
}

/**
 * @brief 更新BMI088加速度、角速度和温度数据
 * @param bmi088 BMI088对象指针
 * @return 更新状态
 */
Bmi088Status Bmi088Update(Bmi088Object *bmi088) {
  if (bmi088 == NULL || !bmi088->is_inited) {
    return kBmi088NoSensor;
  }

  Bmi088ReadRawAccel(bmi088);
  Bmi088ReadRawTemperature(bmi088);
  Bmi088ReadRawGyro(bmi088);
  Bmi088UpdateScaledData(bmi088);
  Bmi088UpdateCorrectedData(bmi088);

  return kBmi088NoError;
}

/**
 * @brief 初始化BMI088 IMU封装，内部包含传感器、EKF和可选恒温控制
 * @param bmi088 BMI088对象指针
 * @param hspi SPI句柄
 * @param config IMU封装配置
 * @return 初始化状态
 */
Bmi088Status Bmi088ImuInit(Bmi088Object *bmi088, SPI_HandleTypeDef *hspi,
                           const Bmi088ImuConfig *config) {
  if (bmi088 == NULL || hspi == NULL || config == NULL) {
    return kBmi088NoSensor;
  }

  Bmi088Status status = Bmi088Init(bmi088, hspi);
  if (status != kBmi088NoError) {
    return status;
  }

  if (ImuEkfInit(&bmi088->imu_ekf, config->dt, config->gravity) != 0) {
    bmi088->status = kBmi088ImuEkfInitError;
    return bmi088->status;
  }
  ImuEkfSetQr(&bmi088->imu_ekf, config->q_diag, config->r_diag);
  Bmi088ImuSetInstallMatrix(bmi088, config->install_matrix);
  bmi088->is_imu_enabled = true;
  bmi088->imu_ekf_status = ARM_MATH_SUCCESS;
  bmi088->motion_accel_lpf = (config->motion_accel_lpf > 0.0f)
                                 ? config->motion_accel_lpf
                                 : kBmi088DefaultMotionAccelLpf;

  if (config->is_auto_calibrate_enabled) {
    uint16_t sample_count = (config->calibration_sample_count > 0U)
                                ? config->calibration_sample_count
                                : 600U;
    float g_threshold = (config->g_norm_motion_threshold > 0.0f)
                            ? config->g_norm_motion_threshold
                            : kBmi088DefaultGNormMotionThreshold;
    float gyro_threshold = (config->gyro_motion_threshold > 0.0f)
                               ? config->gyro_motion_threshold
                               : kBmi088DefaultGyroMotionThreshold;
    status = Bmi088ImuCalibrate(bmi088, sample_count,
                                config->calibration_sample_delay_ms,
                                g_threshold, gyro_threshold);
    if (status != kBmi088NoError) {
      bmi088->status = status;
      return status;
    }
  }

  if (config->is_init_euler_from_accel_enabled) {
    uint16_t sample_count = (config->init_euler_sample_count > 0U)
                                ? config->init_euler_sample_count
                                : 100U;
    Bmi088InitEulerFromAccel(bmi088, sample_count,
                             config->init_euler_sample_delay_ms);
  }

  if (config->is_heater_enabled) {
    if (config->heater_htim == NULL || config->heater_pid_config == NULL) {
      bmi088->status = kBmi088HeaterConfigError;
      return bmi088->status;
    }
    Bmi088HeaterInit(bmi088, config->heater_htim, config->heater_channel,
                     config->heater_pid_config,
                     config->heater_target_temperature);
  }

  bmi088->status = kBmi088NoError;
  return bmi088->status;
}

/**
 * @brief 更新BMI088 IMU封装，内部完成传感器读取、恒温控制和EKF
 * @param bmi088 BMI088对象指针
 * @return 更新状态
 */
Bmi088Status Bmi088ImuUpdate(Bmi088Object *bmi088) {
  if (bmi088 == NULL || !bmi088->is_inited || !bmi088->is_imu_enabled) {
    return kBmi088NoSensor;
  }

  Bmi088Status status = Bmi088Update(bmi088);
  if (status != kBmi088NoError) {
    bmi088->status = status;
    return status;
  }

  if (bmi088->is_heater_enabled) {
    bmi088->heater_duty_ratio = Bmi088HeaterControl(bmi088);
  }

  bmi088->imu_ekf_status = ImuEkfStep(&bmi088->imu_ekf, bmi088->corrected_gyro,
                                      bmi088->corrected_accel);
  if (bmi088->imu_ekf_status != ARM_MATH_SUCCESS) {
    bmi088->status = kBmi088ImuEkfUpdateError;
    return bmi088->status;
  }

  ImuEkfGetState(&bmi088->imu_ekf, &bmi088->euler, &bmi088->euler_rate);
  ImuEkfGetGyroBias(&bmi088->imu_ekf, bmi088->gyro_bias);
  Bmi088UpdateMotionAccel(bmi088);
  bmi088->update_count++;
  bmi088->is_data_ready = true;
  bmi088->status = kBmi088NoError;
  return bmi088->status;
}

/**
 * @brief 重置BMI088内部IMU EKF状态
 * @param bmi088 BMI088对象指针
 */
void Bmi088ImuReset(Bmi088Object *bmi088) {
  if (bmi088 == NULL || !bmi088->is_imu_enabled) {
    return;
  }

  ImuEkfReset(&bmi088->imu_ekf);
  memset(&bmi088->euler, 0, sizeof(bmi088->euler));
  memset(&bmi088->euler_rate, 0, sizeof(bmi088->euler_rate));
  memset(bmi088->gyro_bias, 0, sizeof(bmi088->gyro_bias));
  memset(bmi088->motion_accel_body, 0, sizeof(bmi088->motion_accel_body));
  memset(bmi088->motion_accel_earth, 0, sizeof(bmi088->motion_accel_earth));
  bmi088->update_count = 0U;
  bmi088->is_data_ready = false;
  bmi088->imu_ekf_status = ARM_MATH_SUCCESS;
}

/**
 * @brief 设置BMI088内部IMU EKF的Q/R对角线
 * @param bmi088 BMI088对象指针
 * @param q_diag Q对角线，传NULL则不更新
 * @param r_diag R对角线，传NULL则不更新
 */
void Bmi088ImuSetEkfQr(Bmi088Object *bmi088,
                       const float q_diag[kEkfImuStateSize],
                       const float r_diag[kEkfImuMeasSize]) {
  if (bmi088 == NULL || !bmi088->is_imu_enabled) {
    return;
  }

  ImuEkfSetQr(&bmi088->imu_ekf, q_diag, r_diag);
}

/**
 * @brief 设置传感器到机体系安装修正矩阵
 * @param bmi088 BMI088对象指针
 * @param install_matrix 3x3安装矩阵，传NULL则恢复单位阵
 */
void Bmi088ImuSetInstallMatrix(Bmi088Object *bmi088,
                               const float install_matrix[9]) {
  if (bmi088 == NULL) {
    return;
  }

  memset(bmi088->install_matrix, 0, sizeof(bmi088->install_matrix));
  if (install_matrix == NULL) {
    bmi088->install_matrix[0] = 1.0f;
    bmi088->install_matrix[4] = 1.0f;
    bmi088->install_matrix[8] = 1.0f;
  } else {
    memcpy(bmi088->install_matrix, install_matrix,
           sizeof(bmi088->install_matrix));
  }
}

/**
 * @brief 设置运动加速度低通时间常数
 * @param bmi088 BMI088对象指针
 * @param lpf 低通时间常数（s），小于等于0表示不滤波
 */
void Bmi088ImuSetMotionAccelLpf(Bmi088Object *bmi088, float lpf) {
  if (bmi088 == NULL) {
    return;
  }
  bmi088->motion_accel_lpf = lpf;
}

/**
 * @brief 在线标定陀螺零偏和加速度比例
 * @param bmi088 BMI088对象指针
 * @param sample_count 采样次数
 * @param sample_delay_ms 采样间隔（ms）
 * @param g_norm_motion_threshold 重力模长波动阈值
 * @param gyro_motion_threshold 陀螺波动阈值
 * @return 标定状态
 */
Bmi088Status Bmi088ImuCalibrate(Bmi088Object *bmi088, uint16_t sample_count,
                                uint32_t sample_delay_ms,
                                float g_norm_motion_threshold,
                                float gyro_motion_threshold) {
  float gyro_sum[3] = {0.0f, 0.0f, 0.0f};
  float gyro_min[3] = {0.0f, 0.0f, 0.0f};
  float gyro_max[3] = {0.0f, 0.0f, 0.0f};
  float g_norm_sum = 0.0f;
  float g_norm_min = 0.0f;
  float g_norm_max = 0.0f;

  if (bmi088 == NULL || !bmi088->is_inited || sample_count == 0U) {
    return kBmi088NoSensor;
  }

  bmi088->accel_scale = 1.0f;
  memset(bmi088->gyro_offset, 0, sizeof(bmi088->gyro_offset));
  memset(&bmi088->calibration, 0, sizeof(bmi088->calibration));

  for (uint16_t sample = 0; sample < sample_count; sample++) {
    (void)Bmi088Update(bmi088);
    float g_norm = Bmi088VectorNorm3(bmi088->corrected_accel);

    if (sample == 0U) {
      g_norm_min = g_norm;
      g_norm_max = g_norm;
      memcpy(gyro_min, bmi088->corrected_gyro, sizeof(gyro_min));
      memcpy(gyro_max, bmi088->corrected_gyro, sizeof(gyro_max));
    }

    g_norm_sum += g_norm;
    if (g_norm < g_norm_min) {
      g_norm_min = g_norm;
    } else if (g_norm > g_norm_max) {
      g_norm_max = g_norm;
    }

    for (uint8_t i = 0; i < 3U; i++) {
      gyro_sum[i] += bmi088->corrected_gyro[i];
      if (bmi088->corrected_gyro[i] < gyro_min[i]) {
        gyro_min[i] = bmi088->corrected_gyro[i];
      } else if (bmi088->corrected_gyro[i] > gyro_max[i]) {
        gyro_max[i] = bmi088->corrected_gyro[i];
      }
    }

    if (sample_delay_ms > 0U) {
      HAL_Delay(sample_delay_ms);
    }
  }

  bmi088->calibration.sample_count = sample_count;
  bmi088->calibration.g_norm = g_norm_sum / (float)sample_count;
  bmi088->calibration.g_norm_range = g_norm_max - g_norm_min;
  bmi088->calibration.temperature_when_cali = bmi088->temperature;
  for (uint8_t i = 0; i < 3U; i++) {
    bmi088->calibration.gyro_range[i] = gyro_max[i] - gyro_min[i];
  }

  bool is_moving = bmi088->calibration.g_norm_range > g_norm_motion_threshold ||
                   bmi088->calibration.gyro_range[0] > gyro_motion_threshold ||
                   bmi088->calibration.gyro_range[1] > gyro_motion_threshold ||
                   bmi088->calibration.gyro_range[2] > gyro_motion_threshold;
  if (is_moving || bmi088->calibration.g_norm <= 0.0f) {
    bmi088->status = kBmi088CalibrateMotionError;
    return bmi088->status;
  }

  for (uint8_t i = 0; i < 3U; i++) {
    bmi088->gyro_offset[i] = gyro_sum[i] / (float)sample_count;
  }
  bmi088->g_norm = bmi088->calibration.g_norm;
  bmi088->accel_scale = bmi088->imu_ekf.gravity / bmi088->g_norm;
  bmi088->calibration.is_valid = true;
  bmi088->is_gyro_offset_inited = true;
  bmi088->status = kBmi088NoError;

  return bmi088->status;
}

/**
 * @brief 手动设置陀螺仪零偏
 * @param bmi088 BMI088对象指针
 * @param gyro_offset 三轴陀螺仪零偏（rad/s）
 */
void Bmi088SetGyroOffset(Bmi088Object *bmi088, const float gyro_offset[3]) {
  if (bmi088 == NULL || gyro_offset == NULL) {
    return;
  }

  memcpy(bmi088->gyro_offset, gyro_offset, sizeof(bmi088->gyro_offset));
  bmi088->is_gyro_offset_inited = true;
}

/**
 * @brief 采样计算陀螺仪零偏，校准时应保持机器人静止
 * @param bmi088 BMI088对象指针
 * @param sample_count 采样次数
 * @param sample_delay_ms 采样间隔（ms）
 * @return 校准状态
 */
Bmi088Status Bmi088CalibrateGyroOffset(Bmi088Object *bmi088,
                                       uint16_t sample_count,
                                       uint32_t sample_delay_ms) {
  float gyro_sum[3] = {0.0f, 0.0f, 0.0f};

  if (bmi088 == NULL || !bmi088->is_inited || sample_count == 0U) {
    return kBmi088NoSensor;
  }

  memset(bmi088->gyro_offset, 0, sizeof(bmi088->gyro_offset));
  for (uint16_t sample = 0; sample < sample_count; sample++) {
    Bmi088ReadRawGyro(bmi088);
    for (uint8_t i = 0; i < 3U; i++) {
      gyro_sum[i] += bmi088->gyro_sensitivity * bmi088->raw.gyro[i];
    }
    if (sample_delay_ms > 0U) {
      HAL_Delay(sample_delay_ms);
    }
  }

  for (uint8_t i = 0; i < 3U; i++) {
    bmi088->gyro_offset[i] = gyro_sum[i] / (float)sample_count;
  }
  bmi088->is_gyro_offset_inited = true;

  return kBmi088NoError;
}

/**
 * @brief 初始化BMI088恒温加热控制
 * @param bmi088 BMI088对象指针
 * @param htim 加热片PWM定时器
 * @param channel 加热片PWM通道
 * @param pid_config 恒温PID配置
 * @param target_temperature 目标温度（℃）
 */
void Bmi088HeaterInit(Bmi088Object *bmi088, TIM_HandleTypeDef *htim,
                      uint32_t channel, PidConfig *pid_config,
                      float target_temperature) {
  if (bmi088 == NULL || htim == NULL || pid_config == NULL) {
    return;
  }

  PwmInit(htim, channel);
  PidInit(&bmi088->heater_pid, pid_config);
  bmi088->heater_htim = htim;
  bmi088->heater_channel = channel;
  bmi088->heater_target_temperature = target_temperature;
  bmi088->heater_duty_ratio = 0.0f;
  bmi088->is_heater_enabled = true;
}

/**
 * @brief 关闭BMI088恒温加热控制
 * @param bmi088 BMI088对象指针
 */
void Bmi088HeaterDisable(Bmi088Object *bmi088) {
  if (bmi088 == NULL) {
    return;
  }

  if (bmi088->heater_htim != NULL) {
    PwmSetDutyRatio(bmi088->heater_htim, bmi088->heater_channel, 0.0f);
  }
  bmi088->heater_duty_ratio = 0.0f;
  bmi088->is_heater_enabled = false;
}

/**
 * @brief 执行一次BMI088恒温控制
 * @param bmi088 BMI088对象指针
 * @return 输出PWM占空比
 */
float Bmi088HeaterControl(Bmi088Object *bmi088) {
  if (bmi088 == NULL || !bmi088->is_heater_enabled) {
    return 0.0f;
  }

  float duty_ratio =
      PidCalculate(&bmi088->heater_pid, bmi088->heater_target_temperature,
                   bmi088->temperature);
  if (duty_ratio < 0.0f) {
    duty_ratio = 0.0f;
  } else if (duty_ratio > 1.0f) {
    duty_ratio = 1.0f;
  }
  PwmSetDutyRatio(bmi088->heater_htim, bmi088->heater_channel, duty_ratio);

  bmi088->heater_duty_ratio = duty_ratio;
  return duty_ratio;
}
