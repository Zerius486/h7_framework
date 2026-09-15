#include "app_task.h"

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "app_protocol.h"
#include "app_referee.h"
#include "bsp_fdcan.h"
#include "bsp_uart.h"
#include "cmsis_os2.h"
#include "app_unit_test.h"
#include "dev_bmi088.h"
#include "dev_dji_dt7.h"
#include "dev_dji_motor.h"
#include "dev_dm_motor.h"
#include "fdcan.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"

// 上赛季Standard机器人的Skywalker版重写。这个文件集中覆盖四个RTOS任务入口，
// 方便后续将整车行为作为模板或范例整体替换。

enum {
  // 1U: 接入裁判系统；0U: 不接入裁判系统，使用本文件内默认参数。
  kStandardRefereeEnabled = 0U,
  // 只接底盘时先关闭云台、发射和视觉链路，只跑DT7 + FDCAN3底盘。
  kStandardChassisEnabled = 1U,
  kStandardGimbalEnabled = 1U,
  kStandardShootEnabled = 1U,
  kStandardVisionEnabled = 0U,
  kStandardKeyboardMouseEnabled = 1U,
  kStandardRemoteSwitchSafeModeEnabled = 0U,
  kStandardUnitTestMask = kUnitTestNone,
  kStandardManagerPeriodMs = 10,
  kStandardGimbalPeriodMs = 2,
  kStandardChassisPeriodMs = 2,
  kStandardShootPeriodMs = 2,
  kStandardMotorCount = 4,
  kStandardCanDataLength = 8,
  kStandardDt7OfflineTimeoutMs = 100,
  kStandardImuGyroZAxis = 2,
};

static const float kStandardChassisRadiusM = 0.15f;
static const float kStandardWheelRadiusM = 0.01f;
static const float kStandardMaxVxMps = 5.0f;
static const float kStandardMaxVyMps = 5.0f;
static const float kStandardMaxOmegaRadps = 10.0f;
static const float kStandardNoRefereeCurrentScale = 0.5f;
static const float kStandardChassisYawRateFeedbackKp = 0.5f;
static const float kStandardChassisAutoSpinOmegaRadps = 4.0f;
static const float kStandardChassisYawWheelDeadband = 0.05f;
static const float kStandardImuYawRateSign = 1.0f;
static const float kStandardGimbalYawFieldKp = 6.0f;
static const float kStandardGimbalYawMotorSign = 1.0f;
static const float kStandardGimbalYawRcSign = -1.0f;
static const float kStandardGimbalYawMaxOmegaRadps = 3.0f;
static const float kStandardGimbalYawDeadband = 0.05f;
static const float kStandardGimbalPitchRcSign = 1.0f;
static const float kStandardGimbalPitchMinRad = -(float)M_PI / 4.0f;
static const float kStandardGimbalPitchMaxRad = (float)M_PI / 4.0f;
static const float kStandardGimbalPitchMaxOmegaRadps = 1.5f;
static const float kStandardGimbalPitchDeadband = 0.05f;
static const float kStandardMouseYawRadPerCount = 0.0015f;
static const float kStandardMousePitchRadPerCount = 0.0008f;
static const float kStandardImuHeaterTargetTemperatureC = 40.0f;
static const float kStandardPitchOffsetRad = -1.66286434f;
static const float kStandardBulletFeedOmegaRadps = 500.0f;
static const float kStandardFrictionOmegaRadps = 800.0f;
static const float kStandardDefaultChassisPowerW = 100.0f;

// M3508底盘功率估计参数，保留旧车测试得到的基础模型。
static const float kStandardPowerK0 = 0.30f;
static const float kStandardPowerK1 = 0.37f;
static const float kStandardPowerK2 = 8.30f;
static const float kStandardPowerK3 = 0.0f;

static PidConfig g_standard_bmi088_heater_pid = {
    .kp = 0.20f,
    .ki = 0.03f,
    .kd = 0.0f,
    .sampling_time = 0.002f,
    .is_antiwindup_enabled = true,
    .output_max = 1.0f,
    .integral_max = 10.0f,
};

typedef struct {
  float vx;
  float vy;
  float omega_z;
} StandardVelocity;

typedef struct {
  StandardVelocity desired_velocity;
  StandardVelocity actual_velocity;
  float estimated_power;
  float yaw_angle;
  float target_yaw_angle;
  float yaw_rate;
  float control_dt;
  Bmi088Status imu_status;
  bool imu_ready;
} StandardChassisState;

typedef struct {
  float yaw_angle;
  float target_yaw_angle;
  float pitch_angle;
  float yaw_rate;
  float pitch_rate;
} StandardGimbalState;

typedef struct {
  DjiMotor chassis_motor[kStandardMotorCount];
  DjiMotor friction_motor[2];
  DjiMotor bullet_feed_motor;
  DjiMotor pitch_motor;
  DmMotor yaw_motor;
  StandardChassisState chassis;
  StandardGimbalState gimbal;
  uint16_t bullet_count;
  uint32_t chassis_imu_last_tick;
  uint32_t gimbal_last_tick;
  uint32_t keyboard_mouse_last_frame_count;
  volatile bool is_initialized;
  volatile bool is_initializing;
} StandardApp;

static StandardApp g_standard;

static bool StandardIsUnitTestMode(void) {
  return (uint32_t)kStandardUnitTestMask != (uint32_t)kUnitTestNone;
}

static float StandardClamp(float value, float min, float max) {
  if (value > max) {
    return max;
  }
  if (value < min) {
    return min;
  }
  return value;
}

static float StandardRcNormalize(int16_t value) {
  return StandardClamp((float)value / 660.0f, -1.0f, 1.0f);
}

static float StandardApplyDeadband(float value, float deadband) {
  if (value > -deadband && value < deadband) {
    return 0.0f;
  }
  return value;
}

static bool StandardAutoSpinIsEnabled(void) {
  return SwitchIsUp(g_dt7_object.rocker.sw1);
}

static float StandardWrapPi(float angle) {
  while (angle > (float)M_PI) {
    angle -= 2.0f * (float)M_PI;
  }
  while (angle < -(float)M_PI) {
    angle += 2.0f * (float)M_PI;
  }
  return angle;
}

static void StandardUpdateKeyboardMouseInput(void) {
  if (kStandardKeyboardMouseEnabled == 0U) {
    g_standard.keyboard_mouse_last_frame_count = Dt7FrameCount();
    return;
  }

  uint32_t frame_count = Dt7FrameCount();
  if (frame_count == g_standard.keyboard_mouse_last_frame_count) {
    return;
  }
  g_standard.keyboard_mouse_last_frame_count = frame_count;

  g_standard.gimbal.target_yaw_angle = StandardWrapPi(
      g_standard.gimbal.target_yaw_angle +
      (float)g_dt7_object.mouse.x * kStandardMouseYawRadPerCount);
  g_standard.gimbal.pitch_angle -=
      (float)g_dt7_object.mouse.y * kStandardMousePitchRadPerCount;
}

static float StandardChassisYawRateCommand(void) {
  float wheel_input = StandardApplyDeadband(
      StandardRcNormalize(g_dt7_object.rocker.wheel),
      kStandardChassisYawWheelDeadband);
  float yaw_input = -wheel_input - (float)g_dt7_object.key.q +
                    (float)g_dt7_object.key.e;
  float yaw_rate =
      StandardClamp(yaw_input, -1.0f, 1.0f) * kStandardMaxOmegaRadps;
  return StandardClamp(yaw_rate, -kStandardMaxOmegaRadps,
                       kStandardMaxOmegaRadps);
}

static float StandardGimbalYawRateCommand(void) {
  float yaw_input = StandardApplyDeadband(
      StandardRcNormalize(g_dt7_object.rocker.ch0),
      kStandardGimbalYawDeadband);
  float yaw_rate =
      kStandardGimbalYawRcSign * yaw_input * kStandardGimbalYawMaxOmegaRadps;
  return StandardClamp(yaw_rate, -kStandardGimbalYawMaxOmegaRadps,
                       kStandardGimbalYawMaxOmegaRadps);
}

static float StandardGimbalPitchRateCommand(void) {
  float pitch_input = StandardApplyDeadband(
      StandardRcNormalize(g_dt7_object.rocker.ch1),
      kStandardGimbalPitchDeadband);
  float pitch_rate = kStandardGimbalPitchRcSign * pitch_input *
                     kStandardGimbalPitchMaxOmegaRadps;
  return StandardClamp(pitch_rate, -kStandardGimbalPitchMaxOmegaRadps,
                       kStandardGimbalPitchMaxOmegaRadps);
}

static void StandardPackDjiCurrents(const DjiMotor *motor_0,
                                    const DjiMotor *motor_1,
                                    const DjiMotor *motor_2,
                                    const DjiMotor *motor_3,
                                    uint8_t tx_data[8]) {
  const DjiMotor *motors[4] = {motor_0, motor_1, motor_2, motor_3};
  memset(tx_data, 0, 8);
  for (uint8_t i = 0; i < 4U; i++) {
    if (motors[i] == NULL) {
      continue;
    }
    int16_t current_units = DjiMotorCurrentAmperesToUnits(
        motors[i]->type, motors[i]->state.given_current);
    tx_data[i * 2U] = (uint8_t)(current_units >> 8);
    tx_data[i * 2U + 1U] = (uint8_t)(current_units & 0xFFU);
  }
}

static void StandardFdcan2RxCallback(uint32_t std_id, uint8_t *rx_buffer) {
  switch (std_id) {
    case 0x201U:
      DjiMotorUpdate(&g_standard.friction_motor[0], rx_buffer);
      break;
    case 0x202U:
      DjiMotorUpdate(&g_standard.friction_motor[1], rx_buffer);
      break;
    case 0x20BU:
      DjiMotorUpdate(&g_standard.pitch_motor, rx_buffer);
      break;
    default:
      if (std_id == g_standard.yaw_motor.state.id ||
          std_id == g_standard.yaw_motor.state.master_id) {
        DmMotorUpdate(&g_standard.yaw_motor, rx_buffer);
      }
      break;
  }
}

static void StandardFdcan3RxCallback(uint32_t std_id, uint8_t *rx_buffer) {
  if (std_id >= 0x201U && std_id <= 0x204U) {
    DjiMotorUpdate(&g_standard.chassis_motor[std_id - 0x201U], rx_buffer);
  } else if (std_id == 0x205U) {
    DjiMotorUpdate(&g_standard.bullet_feed_motor, rx_buffer);
  }
}

static void StandardMotorInit(void) {
  static PidConfig m2006_speed_pid = {
      .kp = 1.515502929687f,
      .ki = 1.165771484375f,
      .kd = 0.0f,
      .sampling_time = 0.002f,
      .is_antiwindup_enabled = true,
      .output_max = 5.0f,
      .integral_max = 6.103515625f,
  };
  static PidConfig m3508_speed_pid = {
      .kp = 0.5515502929687f,
      .ki = 0.0f,
      .kd = 0.0f,
      .sampling_time = 0.002f,
      .is_antiwindup_enabled = true,
      .output_max = 14.6484375f,
      .integral_max = 6.103515625f,
  };
  static PidConfig gm6020_speed_pid = {
      .kp = 0.35f,
      .ki = 6.75f,
      .kd = 0.0f,
      .sampling_time = 0.002f,
      .is_antiwindup_enabled = true,
      .output_max = 0.54f,
      .integral_max = 0.54f,
  };
  static PidConfig gm6020_position_pid = {
      .kp = 5.50f,
      .ki = 0.0f,
      .kd = 0.0f,
      .sampling_time = 0.002f,
      .is_feedforward_enabled = true,
      .kff = 0.1f,
      .is_antiwindup_enabled = true,
      .output_max = 10.0f,
      .integral_max = 10.0f,
  };
  for (uint8_t i = 0; i < kStandardMotorCount; i++) {
    DjiMotorInit(&g_standard.chassis_motor[i], kDjiMotorM3508,
                 kDjiMotorModeSpeed, &m3508_speed_pid, NULL);
  }
  DjiMotorInit(&g_standard.friction_motor[0], kDjiMotorM3508,
               kDjiMotorModeSpeed, &m3508_speed_pid, NULL);
  DjiMotorInit(&g_standard.friction_motor[1], kDjiMotorM3508,
               kDjiMotorModeSpeed, &m3508_speed_pid, NULL);
  DjiMotorInit(&g_standard.bullet_feed_motor, kDjiMotorM2006,
               kDjiMotorModeSpeed, &m2006_speed_pid, NULL);
  DjiMotorInit(&g_standard.pitch_motor, kDjiMotorGm6020,
               kDjiMotorModePosition, &gm6020_speed_pid,
               &gm6020_position_pid);
  DmMotorInit(&g_standard.yaw_motor, kDmMotorTypeJ4310, kDmMotorModeVelocity,
              0x01U, 0x11U);
}

static void StandardSharedInit(void) {
  if (g_standard.is_initialized) {
    return;
  }

  uint32_t lock = osKernelLock();
  if (g_standard.is_initialized || g_standard.is_initializing) {
    (void)osKernelRestoreLock(lock);
    while (!g_standard.is_initialized) {
      osDelay(1);
    }
    return;
  }
  memset(&g_standard, 0, sizeof(g_standard));
  g_standard.is_initializing = true;
  (void)osKernelRestoreLock(lock);

  StandardMotorInit();
  if (kStandardGimbalEnabled != 0U || kStandardShootEnabled != 0U) {
    FdcanInit(&hfdcan2, StandardFdcan2RxCallback);
  }
  if (kStandardChassisEnabled != 0U || kStandardShootEnabled != 0U) {
    FdcanInit(&hfdcan3, StandardFdcan3RxCallback);
  }
  UartInit(&huart5, Dt7RxCallback);
  if (kStandardVisionEnabled != 0U) {
    UartInit(&huart10, VisionProtocolRxCallback);
  }
  if (kStandardRefereeEnabled != 0U) {
    RefereeInit(&g_referee, &huart1, &huart1);
  }
  if (kStandardChassisEnabled != 0U) {
    g_standard.chassis.imu_status = Bmi088Init(&g_bmi088, &hspi2);
    if (g_standard.chassis.imu_status == kBmi088NoError) {
      Bmi088HeaterInit(&g_bmi088, &htim3, TIM_CHANNEL_4,
                       &g_standard_bmi088_heater_pid,
                       kStandardImuHeaterTargetTemperatureC);
      g_standard.chassis.imu_status =
          Bmi088CalibrateGyroOffset(&g_bmi088, 300U, 1U);
      g_standard.chassis.imu_ready =
          g_standard.chassis.imu_status == kBmi088NoError;
    }
    g_standard.chassis.target_yaw_angle = g_standard.chassis.yaw_angle;
    g_standard.gimbal.target_yaw_angle = g_standard.chassis.target_yaw_angle;
  }
  g_standard.gimbal.pitch_angle = kStandardPitchOffsetRad;

  if (kStandardGimbalEnabled != 0U) {
    uint8_t tx_data[kStandardCanDataLength] = {0};
    DmMotorPackEnable(&g_standard.yaw_motor, tx_data);
    FdcanTransmit(&hfdcan2, tx_data, kStandardCanDataLength,
                  DmMotorTxId(&g_standard.yaw_motor));
  }
  g_standard.chassis_imu_last_tick = osKernelGetTickCount();
  g_standard.gimbal_last_tick = osKernelGetTickCount();
  g_standard.is_initialized = true;
  g_standard.is_initializing = false;
}

static float StandardEstimateChassisPower(void) {
  float sum_power = 0.0f;
  for (uint8_t i = 0; i < kStandardMotorCount; i++) {
    float current = g_standard.chassis_motor[i].state.given_current;
    float omega = g_standard.chassis_motor[i].state.omega;
    float torque = kStandardPowerK0 * current;
    float wheel_power = torque * omega + kStandardPowerK1 * fabsf(omega) +
                        kStandardPowerK2 * torque * torque +
                        kStandardPowerK3 / (float)kStandardMotorCount;
    if (wheel_power > 0.0f) {
      sum_power += wheel_power;
    }
  }
  return sum_power;
}

static void StandardApplyChassisPowerLimit(float max_power) {
  float command_power = StandardEstimateChassisPower();
  if (max_power <= 0.0f) {
    for (uint8_t i = 0; i < kStandardMotorCount; i++) {
      g_standard.chassis_motor[i].state.given_current = 0.0f;
    }
    g_standard.chassis.estimated_power = 0.0f;
    return;
  }

  if (command_power > max_power) {
    float scale = max_power / command_power;
    for (uint8_t i = 0; i < kStandardMotorCount; i++) {
      g_standard.chassis_motor[i].state.given_current *= scale;
    }
    g_standard.chassis.estimated_power = max_power;
  } else {
    g_standard.chassis.estimated_power = command_power;
  }
}

static float StandardChassisPowerLimit(void) {
  if (kStandardRefereeEnabled != 0U &&
      g_referee.game_robot_state.chassis_power_limit > 0U) {
    return (float)g_referee.game_robot_state.chassis_power_limit;
  }
  return kStandardDefaultChassisPowerW;
}

static void StandardChassisImuUpdate(void) {
  uint32_t now = osKernelGetTickCount();
  uint32_t elapsed_ms = now - g_standard.chassis_imu_last_tick;
  g_standard.chassis_imu_last_tick = now;
  g_standard.chassis.control_dt = (float)elapsed_ms * 0.001f;

  if (!g_standard.chassis.imu_ready) {
    return;
  }

  Bmi088Status status = Bmi088Update(&g_bmi088);
  if (g_bmi088.is_heater_enabled) {
    (void)Bmi088HeaterControl(&g_bmi088);
  }
  g_standard.chassis.imu_status = status;
  if (status != kBmi088NoError) {
    g_standard.chassis.imu_ready = false;
    g_standard.chassis.yaw_rate = 0.0f;
    g_standard.chassis.target_yaw_angle = g_standard.chassis.yaw_angle;
    return;
  }

  float yaw_rate =
      kStandardImuYawRateSign * g_bmi088.corrected_gyro[kStandardImuGyroZAxis];
  g_standard.chassis.yaw_rate = yaw_rate;
  g_standard.chassis.yaw_angle = StandardWrapPi(
      g_standard.chassis.yaw_angle + yaw_rate * (float)elapsed_ms * 0.001f);
}

static void StandardChassisKinematics(void) {
  float vx_input = -StandardRcNormalize(g_dt7_object.rocker.ch3) -
                   (float)g_dt7_object.key.w + (float)g_dt7_object.key.s;
  float vy_input = StandardRcNormalize(g_dt7_object.rocker.ch2) +
                   (float)g_dt7_object.key.a - (float)g_dt7_object.key.d;
  float desired_yaw_rate = StandardChassisYawRateCommand();
  bool auto_spin_enabled = StandardAutoSpinIsEnabled();
  if (auto_spin_enabled) {
    if (g_standard.chassis.imu_ready) {
      g_standard.chassis.target_yaw_angle = g_standard.chassis.yaw_angle;
    }
    desired_yaw_rate += kStandardChassisAutoSpinOmegaRadps;
    desired_yaw_rate = StandardClamp(desired_yaw_rate, -kStandardMaxOmegaRadps,
                                     kStandardMaxOmegaRadps);
  }

  g_standard.chassis.desired_velocity.vx =
      StandardClamp(vx_input, -1.0f, 1.0f) * kStandardMaxVxMps;
  g_standard.chassis.desired_velocity.vy =
      StandardClamp(vy_input, -1.0f, 1.0f) * kStandardMaxVyMps;
  g_standard.chassis.desired_velocity.omega_z = desired_yaw_rate;

  float field_vx = g_standard.chassis.desired_velocity.vx;
  float field_vy = g_standard.chassis.desired_velocity.vy;
  float yaw = g_standard.chassis.imu_ready ? g_standard.chassis.yaw_angle : 0.0f;
  float cos_yaw = cosf(yaw);
  float sin_yaw = sinf(yaw);
  float vx = cos_yaw * field_vx + sin_yaw * field_vy;
  float vy = -sin_yaw * field_vx + cos_yaw * field_vy;
  float wz = desired_yaw_rate;
  if (g_standard.chassis.imu_ready) {
    wz += kStandardChassisYawRateFeedbackKp *
          (desired_yaw_rate - g_standard.chassis.yaw_rate);
  }
  wz = StandardClamp(wz, -kStandardMaxOmegaRadps, kStandardMaxOmegaRadps);
  g_standard.chassis.actual_velocity.vx = vx;
  g_standard.chassis.actual_velocity.vy = vy;
  g_standard.chassis.actual_velocity.omega_z = g_standard.chassis.yaw_rate;
  float rotation_speed = wz * kStandardChassisRadiusM;

  g_standard.chassis_motor[0].state.given_omega =
      (-0.707f * vx + 0.707f * vy + rotation_speed) /
      kStandardWheelRadiusM;
  g_standard.chassis_motor[1].state.given_omega =
      (-0.707f * vx - 0.707f * vy + rotation_speed) /
      kStandardWheelRadiusM;
  g_standard.chassis_motor[2].state.given_omega =
      (0.707f * vx - 0.707f * vy + rotation_speed) /
      kStandardWheelRadiusM;
  g_standard.chassis_motor[3].state.given_omega =
      (0.707f * vx + 0.707f * vy + rotation_speed) / kStandardWheelRadiusM;
}

static void StandardChassisControl(void) {
  StandardChassisImuUpdate();
  StandardChassisKinematics();

  for (uint8_t i = 0; i < kStandardMotorCount; i++) {
    DjiMotorCurrentCalculate(&g_standard.chassis_motor[i]);
  }
  if (kStandardRefereeEnabled != 0U) {
    StandardApplyChassisPowerLimit(StandardChassisPowerLimit());
  } else {
    for (uint8_t i = 0; i < kStandardMotorCount; i++) {
      g_standard.chassis_motor[i].state.given_current *=
          kStandardNoRefereeCurrentScale;
    }
    g_standard.chassis.estimated_power = StandardEstimateChassisPower();
  }
  uint8_t tx_data[kStandardCanDataLength] = {0};
  StandardPackDjiCurrents(&g_standard.chassis_motor[0],
                          &g_standard.chassis_motor[1],
                          &g_standard.chassis_motor[2],
                          &g_standard.chassis_motor[3], tx_data);
  FdcanTransmit(&hfdcan3, tx_data, kStandardCanDataLength, 0x200U);
}

static void StandardGimbalControl(void) {
  if (g_vision_to_gimbal.mode == 1U || g_vision_to_gimbal.mode == 2U) {
    g_standard.gimbal.yaw_rate = g_vision_to_gimbal.yaw_velocity;
    g_standard.gimbal.pitch_angle =
        g_vision_to_gimbal.pitch + kStandardPitchOffsetRad;
    g_standard.gimbal.pitch_rate = g_vision_to_gimbal.pitch_velocity;
  } else {
    uint32_t now = osKernelGetTickCount();
    uint32_t elapsed_ms = now - g_standard.gimbal_last_tick;
    g_standard.gimbal_last_tick = now;
    float dt = (float)elapsed_ms * 0.001f;

    g_standard.gimbal.target_yaw_angle = StandardWrapPi(
        g_standard.gimbal.target_yaw_angle +
        StandardGimbalYawRateCommand() * dt);
    if (StandardAutoSpinIsEnabled()) {
      g_standard.gimbal.target_yaw_angle =
          g_standard.chassis.target_yaw_angle;
    }

    float chassis_yaw = g_standard.chassis.imu_ready
                            ? g_standard.chassis.yaw_angle
                            : 0.0f;
    float yaw_error =
        StandardWrapPi(g_standard.gimbal.target_yaw_angle - chassis_yaw);
    g_standard.gimbal.yaw_rate =
        kStandardGimbalYawMotorSign *
        (kStandardGimbalYawFieldKp * yaw_error -
         g_standard.chassis.yaw_rate);

    g_standard.gimbal.pitch_rate = StandardGimbalPitchRateCommand();
    g_standard.gimbal.pitch_angle += g_standard.gimbal.pitch_rate * dt;
  }

  g_standard.gimbal.pitch_angle =
      StandardClamp(g_standard.gimbal.pitch_angle,
                    kStandardPitchOffsetRad + kStandardGimbalPitchMinRad,
                    kStandardPitchOffsetRad + kStandardGimbalPitchMaxRad);
  g_standard.pitch_motor.state.given_angle = g_standard.gimbal.pitch_angle;
  DjiMotorCurrentCalculate(&g_standard.pitch_motor);

  // GM6020 pitch 在 FDCAN2，反馈 ID 0x20B 为电机 ID 7，对应 0x2FE 第3路电流。
  uint8_t pitch_tx_data[kStandardCanDataLength] = {0};
  StandardPackDjiCurrents(NULL, NULL, &g_standard.pitch_motor, NULL,
                          pitch_tx_data);
  FdcanTransmit(&hfdcan2, pitch_tx_data, kStandardCanDataLength,
                0x2FEU);

  g_standard.yaw_motor.state.given_omega = g_standard.gimbal.yaw_rate;
  uint8_t yaw_tx_data[kStandardCanDataLength] = {0};
  DmMotorPackVelocity(&g_standard.yaw_motor, yaw_tx_data);
  FdcanTransmit(&hfdcan2, yaw_tx_data, kStandardCanDataLength,
                DmMotorTxId(&g_standard.yaw_motor));
  if (g_standard.yaw_motor.state.error_code != 0U) {
    DmMotorPackClearError(&g_standard.yaw_motor, yaw_tx_data);
    FdcanTransmit(&hfdcan2, yaw_tx_data, kStandardCanDataLength,
                  DmMotorTxId(&g_standard.yaw_motor));
  }
}

static void StandardShootControl(void) {
  bool should_feed =
      g_dt7_object.mouse.l != 0U || g_vision_to_gimbal.mode == 2U;
  bool should_spin = SwitchIsUp(g_dt7_object.rocker.sw2) || should_feed;

  g_standard.bullet_feed_motor.state.given_omega =
      should_feed ? kStandardBulletFeedOmegaRadps : 0.0f;
  g_standard.friction_motor[0].state.given_omega =
      should_spin ? -kStandardFrictionOmegaRadps : 0.0f;
  g_standard.friction_motor[1].state.given_omega =
      should_spin ? kStandardFrictionOmegaRadps : 0.0f;

  DjiMotorCurrentCalculate(&g_standard.bullet_feed_motor);
  uint8_t bullet_tx_data[kStandardCanDataLength] = {0};
  StandardPackDjiCurrents(&g_standard.bullet_feed_motor, NULL, NULL, NULL,
                          bullet_tx_data);
  FdcanTransmit(&hfdcan3, bullet_tx_data, kStandardCanDataLength, 0x1FFU);

  DjiMotorCurrentCalculate(&g_standard.friction_motor[0]);
  DjiMotorCurrentCalculate(&g_standard.friction_motor[1]);
  uint8_t friction_tx_data[kStandardCanDataLength] = {0};
  StandardPackDjiCurrents(&g_standard.friction_motor[0],
                          &g_standard.friction_motor[1], NULL, NULL,
                          friction_tx_data);
  FdcanTransmit(&hfdcan2, friction_tx_data, kStandardCanDataLength,
                0x200U);
}

static void StandardStopChassisMotors(void) {
  for (uint8_t i = 0; i < kStandardMotorCount; i++) {
    g_standard.chassis_motor[i].state.given_omega = 0.0f;
    g_standard.chassis_motor[i].state.given_current = 0.0f;
  }
  g_standard.chassis.target_yaw_angle = g_standard.chassis.yaw_angle;
  g_standard.chassis_imu_last_tick = osKernelGetTickCount();
}

static void StandardStopGimbalMotors(void) {
  g_standard.gimbal.target_yaw_angle = g_standard.chassis.target_yaw_angle;
  g_standard.gimbal.pitch_angle = kStandardPitchOffsetRad;
  g_standard.gimbal.pitch_rate = 0.0f;
  g_standard.pitch_motor.state.given_angle = g_standard.gimbal.pitch_angle;
  g_standard.pitch_motor.state.given_omega = 0.0f;
  g_standard.pitch_motor.state.given_current = 0.0f;
  g_standard.yaw_motor.state.given_omega = 0.0f;
  g_standard.gimbal_last_tick = osKernelGetTickCount();
}

static void StandardStopShootMotors(void) {
  g_standard.bullet_feed_motor.state.given_omega = 0.0f;
  g_standard.bullet_feed_motor.state.given_current = 0.0f;
  g_standard.friction_motor[0].state.given_omega = 0.0f;
  g_standard.friction_motor[0].state.given_current = 0.0f;
  g_standard.friction_motor[1].state.given_omega = 0.0f;
  g_standard.friction_motor[1].state.given_current = 0.0f;
}

static void StandardTransmitGimbalStopCurrents(void) {
  uint8_t pitch_tx_data[kStandardCanDataLength] = {0};
  uint8_t yaw_tx_data[kStandardCanDataLength] = {0};
  DmMotorPackVelocity(&g_standard.yaw_motor, yaw_tx_data);

  FdcanTransmit(&hfdcan2, pitch_tx_data, kStandardCanDataLength,
                0x2FEU);
  FdcanTransmit(&hfdcan2, yaw_tx_data, kStandardCanDataLength,
                DmMotorTxId(&g_standard.yaw_motor));
}

static void StandardTransmitChassisStopCurrents(void) {
  uint8_t tx_data[kStandardCanDataLength] = {0};

  FdcanTransmit(&hfdcan3, tx_data, kStandardCanDataLength, 0x200U);
}

static void StandardTransmitShootStopCurrents(void) {
  uint8_t friction_tx_data[kStandardCanDataLength] = {0};
  uint8_t bullet_tx_data[kStandardCanDataLength] = {0};

  FdcanTransmit(&hfdcan2, friction_tx_data, kStandardCanDataLength,
                0x200U);
  FdcanTransmit(&hfdcan3, bullet_tx_data, kStandardCanDataLength, 0x1FFU);
}

static bool StandardDt7IsOnline(void) {
  if (Dt7FrameCount() == 0U) {
    return false;
  }
  return (uint32_t)(osKernelGetTickCount() - Dt7LastUpdateTick()) <=
         kStandardDt7OfflineTimeoutMs;
}

static bool StandardIsSafeMode(void) {
  if (!StandardDt7IsOnline()) {
    return true;
  }
  return kStandardRemoteSwitchSafeModeEnabled != 0U &&
         SwitchIsDown(g_dt7_object.rocker.sw1);
}

static void StandardEnterGimbalSafeMode(void) {
  StandardStopGimbalMotors();
  StandardTransmitGimbalStopCurrents();
}

static void StandardEnterChassisSafeMode(void) {
  StandardStopChassisMotors();
  StandardTransmitChassisStopCurrents();
}

static void StandardEnterShootSafeMode(void) {
  StandardStopShootMotors();
  StandardTransmitShootStopCurrents();
}

static void StandardUpdateVisionTx(void) {
  g_gimbal_to_vision.mode = 0U;
  g_gimbal_to_vision.q[0] = 1.0f;
  g_gimbal_to_vision.q[1] = 0.0f;
  g_gimbal_to_vision.q[2] = 0.0f;
  g_gimbal_to_vision.q[3] = 0.0f;
  g_gimbal_to_vision.yaw = g_standard.yaw_motor.state.angle;
  g_gimbal_to_vision.yaw_velocity = g_standard.yaw_motor.state.omega;
  g_gimbal_to_vision.pitch = g_standard.pitch_motor.state.angle;
  g_gimbal_to_vision.pitch_velocity = g_standard.pitch_motor.state.omega;
  g_gimbal_to_vision.bullet_speed =
      (kStandardRefereeEnabled != 0U) ? g_referee.shoot_data.bullet_speed
                                      : 0.0f;
  g_gimbal_to_vision.bullet_count = g_standard.bullet_count;
}

uint32_t ManagerTaskInit(void) {
  if (StandardIsUnitTestMode()) {
    return UnitTestInit(kStandardUnitTestMask);
  }
  StandardSharedInit();
  return kStandardManagerPeriodMs;
}

void ManagerTaskLoop(void) {
  if (StandardIsUnitTestMode()) {
    UnitTestLoop();
    return;
  }
  if (kStandardRefereeEnabled != 0U) {
    RefereeUpdate(&g_referee);
  }
  StandardUpdateKeyboardMouseInput();
  if (kStandardVisionEnabled != 0U) {
    StandardUpdateVisionTx();
    VisionProtocolTransmitGimbalState();
  }
}

uint32_t GimbalTaskInit(void) {
  if (StandardIsUnitTestMode()) {
    return kStandardGimbalPeriodMs;
  }
  if (kStandardGimbalEnabled == 0U) {
    return kStandardGimbalPeriodMs;
  }
  StandardSharedInit();
  return kStandardGimbalPeriodMs;
}

void GimbalTaskLoop(void) {
  if (StandardIsUnitTestMode()) {
    return;
  }
  if (kStandardGimbalEnabled == 0U) {
    return;
  }
  if (StandardIsSafeMode()) {
    StandardEnterGimbalSafeMode();
    return;
  }
  StandardGimbalControl();
}

uint32_t ChassisTaskInit(void) {
  if (StandardIsUnitTestMode()) {
    return kStandardChassisPeriodMs;
  }
  if (kStandardChassisEnabled == 0U) {
    return kStandardChassisPeriodMs;
  }
  StandardSharedInit();
  return kStandardChassisPeriodMs;
}

void ChassisTaskLoop(void) {
  if (StandardIsUnitTestMode()) {
    return;
  }
  if (kStandardChassisEnabled == 0U) {
    return;
  }
  if (StandardIsSafeMode()) {
    StandardEnterChassisSafeMode();
    return;
  }
  StandardChassisControl();
}

uint32_t ShootTaskInit(void) {
  if (StandardIsUnitTestMode()) {
    return kStandardShootPeriodMs;
  }
  if (kStandardShootEnabled == 0U) {
    return kStandardShootPeriodMs;
  }
  StandardSharedInit();
  return kStandardShootPeriodMs;
}

void ShootTaskLoop(void) {
  if (StandardIsUnitTestMode()) {
    return;
  }
  if (kStandardShootEnabled == 0U) {
    return;
  }
  if (StandardIsSafeMode()) {
    StandardEnterShootSafeMode();
    return;
  }
  StandardShootControl();
}
