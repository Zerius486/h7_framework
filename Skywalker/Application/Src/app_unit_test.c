#include "app_unit_test.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>

#include "bsp_fdcan.h"
#include "bsp_uart.h"
#include "cmsis_os2.h"
#include "dev_dji_dt7.h"
#include "dev_dji_motor.h"
#include "fdcan.h"
#include "spi.h"
#include "usart.h"

enum {
  kUnitTestPeriodMs = 2U,
  kUnitTestRemoteTimeoutMs = 100U,
  kUnitTestFdcanTimeoutMs = 500U,
  kUnitTestUartTimeoutMs = 500U,
  kUnitTestMotorCount = 4U,
  kUnitTestCanDataLength = 8U,
};

static const float kUnitTestFdcan3MotorTargetOmegaRadps = 20.0f;
static const float kUnitTestFdcan3MotorRampRadps2 = 20.0f;
static const float kUnitTestFdcan3MotorCurrentLimitA = 2.0f;
static const float kUnitTestRemoteDriveMaxVxMps = 0.8f;
static const float kUnitTestRemoteDriveMaxVyMps = 0.8f;
static const float kUnitTestRemoteDriveWheelRadiusM = 0.01f;
static const float kUnitTestRemoteDriveWheelRampRadps2 = 80.0f;
static const float kUnitTestRemoteDriveDeadband = 0.05f;
static const float kUnitTestRemoteDriveCurrentLimitA = 2.0f;

volatile UnitTestReport g_unit_test_report = {0};

static uint32_t g_unit_test_mask = kUnitTestNone;
static bool g_unit_test_is_initialized = false;
static uint32_t g_test_start_tick = 0U;
static DjiMotor g_fdcan3_motor[kUnitTestMotorCount];
static uint8_t g_fdcan3_motor_tx_data[kUnitTestCanDataLength];
static float g_fdcan3_motor_ramp_omega = 0.0f;
static float g_fdcan3_motor_target_omega[kUnitTestMotorCount];
static uint32_t g_fdcan3_motor_last_tick = 0U;

static float UnitTestVectorNorm3(const float value[3]) {
  return sqrtf(value[0] * value[0] + value[1] * value[1] +
               value[2] * value[2]);
}

static bool UnitTestIsSelected(UnitTestMask mask) {
  return (g_unit_test_mask & (uint32_t)mask) != 0U;
}

static bool UnitTestIsRemoteChassisDriveSelected(void) {
  return UnitTestIsSelected(kUnitTestRemoteChassisDrive);
}

static bool UnitTestIsFdcan3MotorSelected(void) {
  return UnitTestIsSelected(kUnitTestFdcan3MotorSpin) ||
         UnitTestIsRemoteChassisDriveSelected();
}

static void UnitTestItemDisable(volatile UnitTestItemReport *item) {
  item->state = kUnitTestStateDisabled;
  item->last_error = 0U;
}

static void UnitTestItemRun(volatile UnitTestItemReport *item) {
  item->state = kUnitTestStateRunning;
  item->last_error = 0U;
}

static void UnitTestItemPass(volatile UnitTestItemReport *item) {
  item->state = kUnitTestStatePass;
  item->last_error = 0U;
  item->last_update_tick = osKernelGetTickCount();
}

static void UnitTestItemFail(volatile UnitTestItemReport *item,
                             uint32_t error) {
  item->state = kUnitTestStateFail;
  item->last_error = error;
}

static void UnitTestRecordUartRx(UART_HandleTypeDef *huart, uint16_t length);

static void UnitTestRemoteRxCallback(uint8_t *rx_buffer, uint16_t length) {
  UnitTestRecordUartRx(&huart5, length);
  Dt7RxCallback(rx_buffer, length);
}

static void UnitTestRecordUartRx(UART_HandleTypeDef *huart, uint16_t length) {
  uint8_t index = UartIndex(huart);
  if (index == 0xFFU) {
    return;
  }
  g_unit_test_report.uart_rx_count[index]++;
  g_unit_test_report.uart_last_length[index] = length;
  g_unit_test_report.uart_last_tick[index] = osKernelGetTickCount();
}

static void UnitTestUart1RxCallback(uint8_t *rx_buffer, uint16_t length) {
  (void)rx_buffer;
  UnitTestRecordUartRx(&huart1, length);
}

static void UnitTestUart2RxCallback(uint8_t *rx_buffer, uint16_t length) {
  (void)rx_buffer;
  UnitTestRecordUartRx(&huart2, length);
}

static void UnitTestUart3RxCallback(uint8_t *rx_buffer, uint16_t length) {
  (void)rx_buffer;
  UnitTestRecordUartRx(&huart3, length);
}

static void UnitTestUart7RxCallback(uint8_t *rx_buffer, uint16_t length) {
  (void)rx_buffer;
  UnitTestRecordUartRx(&huart7, length);
}

static void UnitTestUart10RxCallback(uint8_t *rx_buffer, uint16_t length) {
  (void)rx_buffer;
  UnitTestRecordUartRx(&huart10, length);
}

static void UnitTestFdcanRecord(uint8_t index, uint32_t std_id,
                                uint8_t *rx_buffer) {
  if (index >= 3U || rx_buffer == NULL) {
    return;
  }
  g_unit_test_report.fdcan_rx_count[index]++;
  g_unit_test_report.fdcan_last_id[index] = std_id;
  memcpy((void *)g_unit_test_report.fdcan_last_data[index], rx_buffer, 8);
}

static void UnitTestFdcan1RxCallback(uint32_t std_id, uint8_t *rx_buffer) {
  UnitTestFdcanRecord(0U, std_id, rx_buffer);
}

static void UnitTestFdcan2RxCallback(uint32_t std_id, uint8_t *rx_buffer) {
  UnitTestFdcanRecord(1U, std_id, rx_buffer);
}

static void UnitTestFdcan3RxCallback(uint32_t std_id, uint8_t *rx_buffer) {
  UnitTestFdcanRecord(2U, std_id, rx_buffer);
  if (UnitTestIsFdcan3MotorSelected() && std_id >= 0x201U &&
      std_id <= 0x204U) {
    uint8_t index = (uint8_t)(std_id - 0x201U);
    DjiMotorUpdate(&g_fdcan3_motor[index], rx_buffer);
    g_unit_test_report.fdcan3_motor_rx_count[index]++;
  }
}

static void UnitTestInitRemote(void) {
  if (UnitTestIsSelected(kUnitTestRemote) ||
      UnitTestIsRemoteChassisDriveSelected()) {
    UartInit(&huart5, UnitTestRemoteRxCallback);
    UnitTestItemRun(&g_unit_test_report.remote);
  } else {
    UnitTestItemDisable(&g_unit_test_report.remote);
  }
}

static void UnitTestInitFdcan(void) {
  if (UnitTestIsSelected(kUnitTestFdcan) ||
      UnitTestIsFdcan3MotorSelected()) {
    if (UnitTestIsSelected(kUnitTestFdcan)) {
      FdcanInit(&hfdcan1, UnitTestFdcan1RxCallback);
      FdcanInit(&hfdcan2, UnitTestFdcan2RxCallback);
    }
    FdcanInit(&hfdcan3, UnitTestFdcan3RxCallback);
    UnitTestItemRun(&g_unit_test_report.fdcan);
  } else {
    UnitTestItemDisable(&g_unit_test_report.fdcan);
  }
}

static void UnitTestInitFdcan3MotorSpin(void) {
  if (!UnitTestIsFdcan3MotorSelected()) {
    return;
  }

  static PidConfig speed_pid = {
      .kp = 0.5515502929687f,
      .ki = 0.0f,
      .kd = 0.0f,
      .sampling_time = 0.002f,
      .is_antiwindup_enabled = true,
      .output_max = 14.6484375f,
      .integral_max = 6.103515625f,
  };

  for (uint8_t i = 0; i < kUnitTestMotorCount; i++) {
    DjiMotorInit(&g_fdcan3_motor[i], kDjiMotorM3508, kDjiMotorModeSpeed,
                 &speed_pid, NULL);
    g_fdcan3_motor[i].state.given_omega = 0.0f;
    g_fdcan3_motor_target_omega[i] = 0.0f;
  }
  g_fdcan3_motor_ramp_omega = 0.0f;
  g_fdcan3_motor_last_tick = osKernelGetTickCount();
  g_unit_test_report.fdcan3_motor_target_omega =
      UnitTestIsSelected(kUnitTestFdcan3MotorSpin)
          ? kUnitTestFdcan3MotorTargetOmegaRadps
          : 0.0f;
}

static void UnitTestInitImu(void) {
  if (UnitTestIsSelected(kUnitTestImu)) {
    Bmi088Status status = Bmi088Init(&g_bmi088, &hspi2);
    g_unit_test_report.imu_status = status;
    if (status == kBmi088NoError) {
      UnitTestItemRun(&g_unit_test_report.imu);
    } else {
      UnitTestItemFail(&g_unit_test_report.imu, (uint32_t)status);
    }
  } else {
    UnitTestItemDisable(&g_unit_test_report.imu);
  }
}

static void UnitTestInitUart(void) {
  if (UnitTestIsSelected(kUnitTestUart)) {
    UartInit(&huart1, UnitTestUart1RxCallback);
    UartInit(&huart2, UnitTestUart2RxCallback);
    UartInit(&huart3, UnitTestUart3RxCallback);
    UartInit(&huart7, UnitTestUart7RxCallback);
    UartInit(&huart10, UnitTestUart10RxCallback);
    UnitTestItemRun(&g_unit_test_report.uart);
  } else {
    UnitTestItemDisable(&g_unit_test_report.uart);
  }
}

uint32_t UnitTestInit(uint32_t selected_mask) {
  if (g_unit_test_is_initialized) {
    return kUnitTestPeriodMs;
  }

  memset((void *)&g_unit_test_report, 0, sizeof(g_unit_test_report));
  g_unit_test_mask = selected_mask;
  g_unit_test_report.selected_mask = selected_mask;
  g_test_start_tick = osKernelGetTickCount();

  UnitTestInitRemote();
  UnitTestInitFdcan();
  UnitTestInitFdcan3MotorSpin();
  UnitTestInitImu();
  UnitTestInitUart();

  g_unit_test_is_initialized = true;
  return kUnitTestPeriodMs;
}

static void UnitTestUpdateRemote(void) {
  if (!UnitTestIsSelected(kUnitTestRemote) &&
      !UnitTestIsRemoteChassisDriveSelected()) {
    return;
  }

  uint32_t now = osKernelGetTickCount();
  g_unit_test_report.remote.update_count++;
  g_unit_test_report.remote_frame_count = Dt7FrameCount();
  g_unit_test_report.remote_sw1 = g_dt7_object.rocker.sw1;
  g_unit_test_report.remote_sw2 = g_dt7_object.rocker.sw2;
  g_unit_test_report.remote_ch[0] = g_dt7_object.rocker.ch0;
  g_unit_test_report.remote_ch[1] = g_dt7_object.rocker.ch1;
  g_unit_test_report.remote_ch[2] = g_dt7_object.rocker.ch2;
  g_unit_test_report.remote_ch[3] = g_dt7_object.rocker.ch3;
  g_unit_test_report.remote_ch[4] = g_dt7_object.rocker.wheel;

  bool is_online =
      Dt7FrameCount() > 0U &&
      (uint32_t)(now - Dt7LastUpdateTick()) <= kUnitTestRemoteTimeoutMs;
  g_unit_test_report.remote_online = is_online ? 1U : 0U;
  if (is_online) {
    UnitTestItemPass(&g_unit_test_report.remote);
  } else {
    UnitTestItemFail(&g_unit_test_report.remote, 1U);
  }
}

static void UnitTestUpdateFdcan(void) {
  if (!UnitTestIsSelected(kUnitTestFdcan) &&
      !UnitTestIsFdcan3MotorSelected()) {
    return;
  }

  g_unit_test_report.fdcan.update_count++;
  if (g_unit_test_report.fdcan_rx_count[0] > 0U ||
      g_unit_test_report.fdcan_rx_count[1] > 0U ||
      g_unit_test_report.fdcan_rx_count[2] > 0U) {
    UnitTestItemPass(&g_unit_test_report.fdcan);
  } else if ((uint32_t)(osKernelGetTickCount() - g_test_start_tick) >
             kUnitTestFdcanTimeoutMs) {
    UnitTestItemFail(&g_unit_test_report.fdcan, 1U);
  }
}

static void UnitTestPackFdcan3MotorCurrents(void) {
  memset(g_fdcan3_motor_tx_data, 0, sizeof(g_fdcan3_motor_tx_data));
  for (uint8_t i = 0; i < kUnitTestMotorCount; i++) {
    int16_t current_units = DjiMotorCurrentAmperesToUnits(
        g_fdcan3_motor[i].type, g_fdcan3_motor[i].state.given_current);
    g_fdcan3_motor_tx_data[i * 2U] = (uint8_t)(current_units >> 8);
    g_fdcan3_motor_tx_data[i * 2U + 1U] =
        (uint8_t)(current_units & 0xFFU);
  }
}

static float UnitTestClampFloat(float value, float min, float max) {
  if (value > max) {
    return max;
  }
  if (value < min) {
    return min;
  }
  return value;
}

static float UnitTestRcNormalize(int16_t value) {
  return UnitTestClampFloat((float)value / 660.0f, -1.0f, 1.0f);
}

static float UnitTestApplyDeadband(float value, float deadband) {
  if (value > -deadband && value < deadband) {
    return 0.0f;
  }
  return value;
}

static float UnitTestRampFloat(float current, float target, float step) {
  if (current < target) {
    current += step;
    return (current > target) ? target : current;
  }
  if (current > target) {
    current -= step;
    return (current < target) ? target : current;
  }
  return current;
}

static bool UnitTestFdcan3AllMotorsHaveFeedback(void) {
  for (uint8_t i = 0; i < kUnitTestMotorCount; i++) {
    if (g_unit_test_report.fdcan3_motor_rx_count[i] == 0U) {
      return false;
    }
  }
  return true;
}

static void UnitTestUpdateFdcan3MotorRamp(void) {
  uint32_t now = osKernelGetTickCount();
  uint32_t elapsed_ms = now - g_fdcan3_motor_last_tick;
  g_fdcan3_motor_last_tick = now;
  float step = kUnitTestFdcan3MotorRampRadps2 * (float)elapsed_ms * 0.001f;

  if (g_fdcan3_motor_ramp_omega < kUnitTestFdcan3MotorTargetOmegaRadps) {
    g_fdcan3_motor_ramp_omega += step;
    if (g_fdcan3_motor_ramp_omega > kUnitTestFdcan3MotorTargetOmegaRadps) {
      g_fdcan3_motor_ramp_omega = kUnitTestFdcan3MotorTargetOmegaRadps;
    }
  } else if (g_fdcan3_motor_ramp_omega >
             kUnitTestFdcan3MotorTargetOmegaRadps) {
    g_fdcan3_motor_ramp_omega -= step;
    if (g_fdcan3_motor_ramp_omega < kUnitTestFdcan3MotorTargetOmegaRadps) {
      g_fdcan3_motor_ramp_omega = kUnitTestFdcan3MotorTargetOmegaRadps;
    }
  }
}

static void UnitTestStopFdcan3Motors(void) {
  for (uint8_t i = 0; i < kUnitTestMotorCount; i++) {
    g_fdcan3_motor_target_omega[i] = 0.0f;
    g_fdcan3_motor[i].state.given_omega = 0.0f;
    g_fdcan3_motor[i].state.given_current = 0.0f;
  }
  memset(g_fdcan3_motor_tx_data, 0, sizeof(g_fdcan3_motor_tx_data));
  FdcanTransmit(&hfdcan3, g_fdcan3_motor_tx_data, kUnitTestCanDataLength,
                0x200U);
}

static bool UnitTestRemoteIsOnline(void) {
  uint32_t now = osKernelGetTickCount();
  return Dt7FrameCount() > 0U &&
         (uint32_t)(now - Dt7LastUpdateTick()) <= kUnitTestRemoteTimeoutMs;
}

static void UnitTestUpdateFdcan3MotorSpin(void) {
  if (!UnitTestIsSelected(kUnitTestFdcan3MotorSpin)) {
    return;
  }

  bool all_motors_have_feedback = UnitTestFdcan3AllMotorsHaveFeedback();
  if (all_motors_have_feedback) {
    UnitTestUpdateFdcan3MotorRamp();
  } else {
    g_fdcan3_motor_ramp_omega = 0.0f;
    g_fdcan3_motor_last_tick = osKernelGetTickCount();
  }

  for (uint8_t i = 0; i < kUnitTestMotorCount; i++) {
    g_fdcan3_motor[i].state.given_omega = g_fdcan3_motor_ramp_omega;
    if (all_motors_have_feedback) {
      DjiMotorCurrentCalculate(&g_fdcan3_motor[i]);
      g_fdcan3_motor[i].state.given_current = UnitTestClampFloat(
          g_fdcan3_motor[i].state.given_current,
          -kUnitTestFdcan3MotorCurrentLimitA,
          kUnitTestFdcan3MotorCurrentLimitA);
    } else {
      g_fdcan3_motor[i].state.given_current = 0.0f;
    }
    g_unit_test_report.fdcan3_motor_omega[i] =
        g_fdcan3_motor[i].state.omega;
    g_unit_test_report.fdcan3_motor_current[i] =
        g_fdcan3_motor[i].state.current;
    g_unit_test_report.fdcan3_motor_given_current[i] =
        g_fdcan3_motor[i].state.given_current;
    g_unit_test_report.fdcan3_motor_temperature[i] =
        (uint8_t)g_fdcan3_motor[i].state.temperature;
  }

  UnitTestPackFdcan3MotorCurrents();
  FdcanTransmit(&hfdcan3, g_fdcan3_motor_tx_data, kUnitTestCanDataLength,
                0x200U);

  if (all_motors_have_feedback) {
    UnitTestItemPass(&g_unit_test_report.fdcan);
  } else if ((uint32_t)(osKernelGetTickCount() - g_test_start_tick) >
             kUnitTestFdcanTimeoutMs) {
    UnitTestItemFail(&g_unit_test_report.fdcan, 2U);
  }
}

static void UnitTestUpdateRemoteChassisDriveTarget(float target_omega[4]) {
  float vx_input = UnitTestApplyDeadband(
      -UnitTestRcNormalize(g_dt7_object.rocker.ch3),
      kUnitTestRemoteDriveDeadband);
  float vy_input = UnitTestApplyDeadband(
      UnitTestRcNormalize(g_dt7_object.rocker.ch2),
      kUnitTestRemoteDriveDeadband);

  float vx = vx_input * kUnitTestRemoteDriveMaxVxMps;
  float vy = vy_input * kUnitTestRemoteDriveMaxVyMps;

  g_unit_test_report.remote_drive_vx = vx;
  g_unit_test_report.remote_drive_vy = vy;

  target_omega[0] = (-0.707f * vx + 0.707f * vy) /
                    kUnitTestRemoteDriveWheelRadiusM;
  target_omega[1] = (-0.707f * vx - 0.707f * vy) /
                    kUnitTestRemoteDriveWheelRadiusM;
  target_omega[2] = (0.707f * vx - 0.707f * vy) /
                    kUnitTestRemoteDriveWheelRadiusM;
  target_omega[3] = (0.707f * vx + 0.707f * vy) /
                    kUnitTestRemoteDriveWheelRadiusM;
}

static void UnitTestUpdateRemoteChassisDrive(void) {
  if (!UnitTestIsRemoteChassisDriveSelected()) {
    return;
  }

  bool ready = UnitTestRemoteIsOnline() && UnitTestFdcan3AllMotorsHaveFeedback();
  uint32_t now = osKernelGetTickCount();
  uint32_t elapsed_ms = now - g_fdcan3_motor_last_tick;
  g_fdcan3_motor_last_tick = now;

  float target_omega[kUnitTestMotorCount] = {0.0f, 0.0f, 0.0f, 0.0f};
  if (ready) {
    UnitTestUpdateRemoteChassisDriveTarget(target_omega);
  } else {
    g_unit_test_report.remote_drive_vx = 0.0f;
    g_unit_test_report.remote_drive_vy = 0.0f;
  }

  float step =
      kUnitTestRemoteDriveWheelRampRadps2 * (float)elapsed_ms * 0.001f;
  for (uint8_t i = 0; i < kUnitTestMotorCount; i++) {
    g_fdcan3_motor_target_omega[i] = target_omega[i];
    g_fdcan3_motor[i].state.given_omega = UnitTestRampFloat(
        g_fdcan3_motor[i].state.given_omega, target_omega[i], step);

    if (ready) {
      DjiMotorCurrentCalculate(&g_fdcan3_motor[i]);
      g_fdcan3_motor[i].state.given_current = UnitTestClampFloat(
          g_fdcan3_motor[i].state.given_current,
          -kUnitTestRemoteDriveCurrentLimitA,
          kUnitTestRemoteDriveCurrentLimitA);
    } else {
      g_fdcan3_motor[i].state.given_current = 0.0f;
    }

    g_unit_test_report.fdcan3_motor_given_omega[i] =
        g_fdcan3_motor[i].state.given_omega;
    g_unit_test_report.fdcan3_motor_omega[i] =
        g_fdcan3_motor[i].state.omega;
    g_unit_test_report.fdcan3_motor_current[i] =
        g_fdcan3_motor[i].state.current;
    g_unit_test_report.fdcan3_motor_given_current[i] =
        g_fdcan3_motor[i].state.given_current;
    g_unit_test_report.fdcan3_motor_temperature[i] =
        (uint8_t)g_fdcan3_motor[i].state.temperature;
  }

  if (ready) {
    UnitTestPackFdcan3MotorCurrents();
    FdcanTransmit(&hfdcan3, g_fdcan3_motor_tx_data, kUnitTestCanDataLength,
                  0x200U);
    UnitTestItemPass(&g_unit_test_report.fdcan);
  } else {
    UnitTestStopFdcan3Motors();
    if (!UnitTestRemoteIsOnline()) {
      UnitTestItemFail(&g_unit_test_report.remote, 1U);
    } else if ((uint32_t)(now - g_test_start_tick) > kUnitTestFdcanTimeoutMs) {
      UnitTestItemFail(&g_unit_test_report.fdcan, 2U);
    }
  }
}

static void UnitTestUpdateImu(void) {
  if (!UnitTestIsSelected(kUnitTestImu)) {
    return;
  }

  g_unit_test_report.imu.update_count++;
  Bmi088Status status = Bmi088Update(&g_bmi088);
  g_unit_test_report.imu_status = status;
  if (status != kBmi088NoError) {
    UnitTestItemFail(&g_unit_test_report.imu, (uint32_t)status);
    return;
  }

  g_unit_test_report.imu_update_count++;
  g_unit_test_report.imu_temperature = g_bmi088.temperature;
  g_unit_test_report.imu_accel_norm =
      UnitTestVectorNorm3(g_bmi088.corrected_accel);
  g_unit_test_report.imu_gyro_norm = UnitTestVectorNorm3(g_bmi088.corrected_gyro);

  bool accel_is_reasonable =
      g_unit_test_report.imu_accel_norm > 4.0f &&
      g_unit_test_report.imu_accel_norm < 16.0f;
  bool gyro_is_reasonable = g_unit_test_report.imu_gyro_norm < 8.0f;
  bool temp_is_reasonable = g_unit_test_report.imu_temperature > -40.0f &&
                            g_unit_test_report.imu_temperature < 100.0f;
  if (accel_is_reasonable && gyro_is_reasonable && temp_is_reasonable) {
    UnitTestItemPass(&g_unit_test_report.imu);
  } else {
    uint32_t error = 0U;
    error |= accel_is_reasonable ? 0U : (1U << 0);
    error |= gyro_is_reasonable ? 0U : (1U << 1);
    error |= temp_is_reasonable ? 0U : (1U << 2);
    UnitTestItemFail(&g_unit_test_report.imu, error);
  }
}

static void UnitTestUpdateUart(void) {
  if (!UnitTestIsSelected(kUnitTestUart)) {
    return;
  }

  uint32_t now = osKernelGetTickCount();
  g_unit_test_report.uart.update_count++;
  bool has_rx = false;
  for (uint8_t i = 0; i < 6U; i++) {
    if (g_unit_test_report.uart_rx_count[i] > 0U &&
        (uint32_t)(now - g_unit_test_report.uart_last_tick[i]) <=
            kUnitTestUartTimeoutMs) {
      has_rx = true;
      break;
    }
  }

  if (has_rx) {
    UnitTestItemPass(&g_unit_test_report.uart);
  } else {
    UnitTestItemFail(&g_unit_test_report.uart, 1U);
  }
}

void UnitTestLoop(void) {
  g_unit_test_report.uptime_ms =
      (uint32_t)(osKernelGetTickCount() - g_test_start_tick);
  UnitTestUpdateRemote();
  UnitTestUpdateFdcan();
  UnitTestUpdateFdcan3MotorSpin();
  UnitTestUpdateRemoteChassisDrive();
  UnitTestUpdateImu();
  UnitTestUpdateUart();
}
