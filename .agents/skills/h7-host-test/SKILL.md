---
name: h7-host-test
description: >-
  Run fast host-side C11 white-box unit and integration tests for Skywalker
  Algorithm, Device, and Application modules using the built-in STM32H7 HAL
  and CMSIS-DSP stub micro-kernel. Use whenever adding, fixing, or verifying
  firmware modules on the host.
---

# H7 Host White-Box Test Skill

Compiles and runs host-side C11 white-box tests against `Skywalker/` source files with `-std=gnu11 -Wall -Wextra -Werror`, backed by a virtual STM32H7 HAL and CMSIS-DSP matrix micro-kernel.

## Provided Host Helpers (`stm32h7xx_hal.h` + `h7_host_stubs.c`)

- **Virtual Time (`HAL_GetTick()` / `HAL_Delay()`)**:
  - `HostResetEnv()`: Reset virtual tick (`0 ms`), IRQ lock counter, and captured UART TX buffer.
  - `HostSetTick(uint32_t tick_ms)`: Set virtual tick explicitly.
  - `HostAdvanceTick(uint32_t delta_ms)`: Advance virtual tick to test `PeripheralGuard` timeouts and time-based filters.
- **UART TX Capture (`UartTransmitTry` weak stub)**:
  - `HostGetLastUartTx(const uint8_t **data, uint16_t *len)`: Inspect the latest frame transmitted via `RefereeSend()` or UART drivers.
- **CMSIS-DSP Matrix (`arm_math.h`)**:
  - Implements `arm_mat_init_f32`, `arm_mat_add_f32`, `arm_mat_sub_f32`, `arm_mat_mult_f32`, `arm_mat_trans_f32`, and `arm_mat_inverse_f32` (up to 12x12) so `alg_kf`, `alg_imu_ekf`, and `ChassisSpeedKf` execute natively on the host.

## Workflow

1. Write a temporary test file (e.g., `/tmp/test_module.c`) including standard headers (`<assert.h>`, `<math.h>`, `<stdio.h>`) and the `Skywalker/` headers under test.
2. Execute the runner script from the repository root:
   ```bash
   .agents/skills/h7-host-test/scripts/run_host_test.sh /tmp/test_module.c
   ```
   Pass extra compiler flags or additional `.c` files after the test file path if needed (for example `-DREFEREE_MODE_RMUC=1`).
3. Verify exit code `0` and expected `[PASS]` output.
4. Delete the temporary `/tmp/test_module.c` file when done, and ensure `git status` remains clean of test artifacts.
