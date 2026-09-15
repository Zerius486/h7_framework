#ifndef APP_UNIT_TEST_H
#define APP_UNIT_TEST_H

#include <stdint.h>

#include "dev_bmi088.h"

typedef enum {
  kUnitTestNone = 0U,
  kUnitTestRemote = (1U << 0),
  kUnitTestFdcan = (1U << 1),
  kUnitTestImu = (1U << 2),
  kUnitTestUart = (1U << 3),
  kUnitTestFdcan3MotorSpin = (1U << 4),
  kUnitTestRemoteChassisDrive = (1U << 5),
  kUnitTestAll = kUnitTestRemote | kUnitTestFdcan | kUnitTestImu |
                 kUnitTestUart | kUnitTestFdcan3MotorSpin |
                 kUnitTestRemoteChassisDrive,
} UnitTestMask;

typedef enum {
  kUnitTestStateDisabled = 0U,
  kUnitTestStateRunning = 1U,
  kUnitTestStatePass = 2U,
  kUnitTestStateFail = 3U,
} UnitTestState;

typedef struct {
  UnitTestState state;
  uint32_t update_count;
  uint32_t last_update_tick;
  uint32_t last_error;
} UnitTestItemReport;

typedef struct {
  uint32_t selected_mask;
  uint32_t uptime_ms;
  UnitTestItemReport remote;
  UnitTestItemReport fdcan;
  UnitTestItemReport imu;
  UnitTestItemReport uart;

  uint32_t remote_frame_count;
  uint8_t remote_online;
  uint8_t remote_sw1;
  uint8_t remote_sw2;
  int16_t remote_ch[5];
  float remote_drive_vx;
  float remote_drive_vy;

  uint32_t fdcan_rx_count[3];
  uint32_t fdcan_last_id[3];
  uint8_t fdcan_last_data[3][8];
  uint32_t fdcan3_motor_rx_count[4];
  float fdcan3_motor_target_omega;
  float fdcan3_motor_given_omega[4];
  float fdcan3_motor_omega[4];
  float fdcan3_motor_current[4];
  float fdcan3_motor_given_current[4];
  uint8_t fdcan3_motor_temperature[4];

  Bmi088Status imu_status;
  uint32_t imu_update_count;
  float imu_temperature;
  float imu_accel_norm;
  float imu_gyro_norm;

  uint32_t uart_rx_count[6];
  uint16_t uart_last_length[6];
  uint32_t uart_last_tick[6];
  uint32_t uart_error_code[6];
} UnitTestReport;

extern volatile UnitTestReport g_unit_test_report;

uint32_t UnitTestInit(uint32_t selected_mask);
void UnitTestLoop(void);

#endif  // APP_UNIT_TEST_H
