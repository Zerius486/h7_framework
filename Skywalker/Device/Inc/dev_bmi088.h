#ifndef DEV_BMI088_H
#define DEV_BMI088_H

#include <stdbool.h>
#include <stdint.h>

#include "alg_imu_ekf.h"
#include "alg_pid.h"
#include "stm32h7xx_hal.h"

// BMI088初始化和运行状态
typedef enum {
  kBmi088NoError = 0x00,                 // 无错误
  kBmi088AccelPowerControlError = 0x01,  // 加速度计电源控制配置失败
  kBmi088AccelPowerConfigError = 0x02,   // 加速度计电源模式配置失败
  kBmi088AccelConfigError = 0x03,        // 加速度计采样配置失败
  kBmi088AccelRangeError = 0x04,         // 加速度计量程配置失败
  kBmi088AccelIntConfigError = 0x05,     // 加速度计中断电气配置失败
  kBmi088AccelIntMapError = 0x06,        // 加速度计中断映射失败
  kBmi088GyroRangeError = 0x07,          // 陀螺仪量程配置失败
  kBmi088GyroBandwidthError = 0x08,      // 陀螺仪带宽配置失败
  kBmi088GyroPowerModeError = 0x09,      // 陀螺仪电源模式配置失败
  kBmi088GyroDataReadyError = 0x0A,      // 陀螺仪数据就绪配置失败
  kBmi088GyroIntConfigError = 0x0B,      // 陀螺仪中断电气配置失败
  kBmi088GyroIntMapError = 0x0C,         // 陀螺仪中断映射失败
  kBmi088ImuEkfInitError = 0x20,         // IMU EKF初始化失败
  kBmi088ImuEkfUpdateError = 0x21,       // IMU EKF更新失败
  kBmi088HeaterConfigError = 0x22,       // 恒温控制配置失败
  kBmi088CalibrateMotionError = 0x23,    // 标定过程中检测到运动
  kBmi088NoSensor = 0xFF,                // 未检测到传感器或对象无效
} Bmi088Status;

// BMI088加速度计量程
typedef enum {
  kBmi088AccelRange3g = 0,  // +/-3g
  kBmi088AccelRange6g,      // +/-6g
  kBmi088AccelRange12g,     // +/-12g
  kBmi088AccelRange24g,     // +/-24g
} Bmi088AccelRange;

// BMI088陀螺仪量程
typedef enum {
  kBmi088GyroRange2000Dps = 0,  // +/-2000 deg/s
  kBmi088GyroRange1000Dps,      // +/-1000 deg/s
  kBmi088GyroRange500Dps,       // +/-500 deg/s
  kBmi088GyroRange250Dps,       // +/-250 deg/s
  kBmi088GyroRange125Dps,       // +/-125 deg/s
} Bmi088GyroRange;

typedef struct {
  int16_t accel[3];     // 加速度原始值
  int16_t gyro[3];      // 陀螺仪原始值
  int16_t temperature;  // 温度原始值
} Bmi088RawData;

// BMI088在线标定结果
typedef struct {
  bool is_valid;                // 标定结果有效标志
  uint16_t sample_count;        // 标定采样次数
  float g_norm;                 // 标定得到的重力模长（m/s^2）
  float g_norm_range;           // 标定期间重力模长波动
  float gyro_range[3];          // 标定期间三轴陀螺波动
  float temperature_when_cali;  // 标定时温度（℃）
} Bmi088Calibration;

// BMI088 IMU封装配置
typedef struct {
  float dt;                // EKF采样周期（s）
  float gravity;           // 重力加速度（m/s^2）
  const float *q_diag;     // EKF过程噪声对角线，传NULL使用默认值
  const float *r_diag;     // EKF观测噪声对角线，传NULL使用默认值
  bool is_heater_enabled;  // 是否启用内部恒温控制
  PidConfig *heater_pid_config;          // 恒温PID配置
  TIM_HandleTypeDef *heater_htim;        // 加热片PWM定时器
  uint32_t heater_channel;               // 加热片PWM通道
  float heater_target_temperature;       // 目标温度（℃）
  bool is_auto_calibrate_enabled;        // 初始化时是否在线标定
  uint16_t calibration_sample_count;     // 在线标定采样次数
  uint32_t calibration_sample_delay_ms;  // 在线标定采样间隔（ms）
  float g_norm_motion_threshold;         // 标定重力波动阈值
  float gyro_motion_threshold;           // 标定陀螺波动阈值
  bool is_init_euler_from_accel_enabled;  // 是否用加速度初始化roll/pitch
  uint16_t init_euler_sample_count;       // 初始姿态加速度采样次数
  uint32_t init_euler_sample_delay_ms;    // 初始姿态采样间隔（ms）
  const float *install_matrix;            // 传感器到机体系安装矩阵
  float motion_accel_lpf;  // 运动加速度低通时间常数（s）
} Bmi088ImuConfig;

typedef struct {
  SPI_HandleTypeDef *hspi;        // SPI句柄
  Bmi088Status status;            // 当前状态
  Bmi088AccelRange accel_range;   // 加速度计量程
  Bmi088GyroRange gyro_range;     // 陀螺仪量程
  float accel_sensitivity;        // 加速度换算系数
  float gyro_sensitivity;         // 角速度换算系数
  float accel[3];                 // m/s^2
  float gyro[3];                  // rad/s
  float gyro_offset[3];           // rad/s
  float accel_scale;              // 加速度整体比例修正
  float g_norm;                   // 标定得到的重力模长（m/s^2）
  float temperature;              // degC
  Bmi088RawData raw;              // 原始数据
  Bmi088Calibration calibration;  // 最近一次在线标定结果
  bool is_inited;                 // 初始化完成标志
  bool is_gyro_offset_inited;     // 陀螺仪零偏初始化标志
  ImuEkfObject imu_ekf;           // 内部IMU EKF对象
  Euler euler;                    // EKF输出欧拉角（rad）
  EulerRate euler_rate;           // EKF输出欧拉角速度（rad/s）
  float gyro_bias[3];             // EKF估计陀螺零偏（rad/s）
  float corrected_accel[3];     // 安装误差修正后的加速度（m/s^2）
  float corrected_gyro[3];      // 安装误差修正后的角速度（rad/s）
  float motion_accel_body[3];   // 机体系运动加速度（m/s^2）
  float motion_accel_earth[3];  // 地面系运动加速度（m/s^2）
  float install_matrix[9];      // 传感器到机体系安装修正矩阵
  float motion_accel_lpf;       // 运动加速度低通时间常数（s）
  uint32_t update_count;        // IMU更新计数
  arm_status imu_ekf_status;    // 最近一次EKF更新状态
  bool is_imu_enabled;          // IMU EKF使能标志
  bool is_data_ready;           // 已完成至少一次IMU更新
  PidObject heater_pid;         // 恒温控制PID
  TIM_HandleTypeDef *heater_htim;   // 加热片PWM定时器
  uint32_t heater_channel;          // 加热片PWM通道
  float heater_target_temperature;  // 目标温度（℃）
  float heater_duty_ratio;          // 最近一次恒温控制占空比
  bool is_heater_enabled;           // 恒温控制使能标志
} Bmi088Object;

extern Bmi088Object g_bmi088;

// API函数声明
void Bmi088ImuConfigInit(Bmi088ImuConfig *config, float dt);
Bmi088Status Bmi088Init(Bmi088Object *bmi088, SPI_HandleTypeDef *hspi);
Bmi088Status Bmi088Update(Bmi088Object *bmi088);
Bmi088Status Bmi088ImuInit(Bmi088Object *bmi088, SPI_HandleTypeDef *hspi,
                           const Bmi088ImuConfig *config);
Bmi088Status Bmi088ImuUpdate(Bmi088Object *bmi088);
void Bmi088ImuReset(Bmi088Object *bmi088);
void Bmi088ImuSetEkfQr(Bmi088Object *bmi088,
                       const float q_diag[kEkfImuStateSize],
                       const float r_diag[kEkfImuMeasSize]);
void Bmi088ImuSetInstallMatrix(Bmi088Object *bmi088,
                               const float install_matrix[9]);
void Bmi088ImuSetMotionAccelLpf(Bmi088Object *bmi088, float lpf);
Bmi088Status Bmi088ImuCalibrate(Bmi088Object *bmi088, uint16_t sample_count,
                                uint32_t sample_delay_ms,
                                float g_norm_motion_threshold,
                                float gyro_motion_threshold);
void Bmi088SetGyroOffset(Bmi088Object *bmi088, const float gyro_offset[3]);
Bmi088Status Bmi088CalibrateGyroOffset(Bmi088Object *bmi088,
                                       uint16_t sample_count,
                                       uint32_t sample_delay_ms);
void Bmi088HeaterInit(Bmi088Object *bmi088, TIM_HandleTypeDef *htim,
                      uint32_t channel, PidConfig *pid_config,
                      float target_temperature);
void Bmi088HeaterDisable(Bmi088Object *bmi088);
float Bmi088HeaterControl(Bmi088Object *bmi088);

#endif  // DEV_BMI088_H
