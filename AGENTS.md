# H7 Framework Engineering Rules

## Architecture & Layering

- `Skywalker/BSP/`: Hardware wrappers (`bsp_uart`, `bsp_spi`, `bsp_fdcan`, `bsp_iwdg`, `bsp_stream`, `bsp_profile`, `bsp_debug_log`).
- `Skywalker/Device/`: Peripheral drivers (`dev_dji_motor`, `dev_dm_motor`, `dev_lk_motor`, `dev_bmi088`, `dev_dji_dt7`) and unified lifecycle state machine (`dev_peripheral`). Every actuator or sensor driver MUST embed `PeripheralGuard` and force zero control/torque output when disabled, offline, or faulted.
- `Skywalker/Algorithm/`: Pure C11, zero global mutable state, caller-allocated structs, SI units (`rad`, `rad/s`, `m`, `m/s`, `A`, `N·m`, `s`).
- `Skywalker/Application/`: Robot configuration, FreeRTOS task logic, referee system (`app_referee`, RoboMaster 2027 V1.0.0 with `REFEREE_MODE=RMUL|RMUC`, default `RMUL`), vision protocol, and tuning telemetry.

## Memory & Cache Rules (STM32H723XG)

- Both `I-Cache` and `D-Cache` are enabled in `Core/Src/main.c` (`MPU_Config()`).
- `RAM_D2` (`0x30000000`, `32 KB`) is configured by MPU as Normal Non-Cacheable. All `DMA1`/`DMA2` TX/RX buffers MUST be placed in `.dma_buffer` and aligned to 32-byte Cache Lines (`__attribute__((section(".dma_buffer"), aligned(32)))`). `DMA1`/`DMA2` cannot access `DTCMRAM`.
- High-frequency 1 kHz state objects (e.g., `g_bmi088`) should be placed in `.dtcmram` (`0x20000000`, `128 KB`). Never allocate multi-KB structs (`MotorTuneObject`, `AutotuneObject`) on FreeRTOS task stacks.

## Numerical & FPU Rules (Cortex-M7 FPv5-D16)

- Never use bare `M_PI` in `float` expressions without a `(float)` cast or `float` constants to avoid implicit `double` promotion and strict ISO C11 compilation errors.
- All public algorithm entry points MUST guard against `NULL` pointers, `!isfinite()` inputs, and `dt <= 0.0f` so a single corrupted sample never poisons filter, integrator, or covariance state.

## Verification & Git Conventions

- After modifying `Skywalker/Algorithm/`, `Skywalker/Device/`, or `Skywalker/Application/`, run host white-box tests using `.agents/skills/h7-host-test/scripts/run_host_test.sh` (and delete temporary test `.c` files afterward), and verify target builds with `cmake --build --preset Debug && cmake --build --preset Release`.
- Commit messages must use Chinese Conventional Commits (`feat(scope): ...`, `fix(scope): ...`, `perf(scope): ...`) describing concrete technical changes without internal task/priority labels.
