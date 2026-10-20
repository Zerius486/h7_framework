# Framework 结构说明

H7 Framework 是完整 STM32H723 固件工程；Skywalker 是其中的 BSP、Device、Algorithm、Application 四层代码。

## 分层与依赖

| 层 | 职责 | 主要接口 |
| --- | --- | --- |
| BSP | HAL 外设封装、数据传输、流缓存与计时 | `bsp_uart.h`、`bsp_fdcan.h`、`bsp_spi.h`、`bsp_stream.h`、`bsp_profile.h` |
| Device | 设备反馈解析、状态与执行指令 | DJI/DM/LK 电机、DT7、BMI088 |
| Algorithm | 控制、估计与运动计算 | PID、DOB、滤波、运动学、轨迹、RLS 和整定 |
| Application | 设备配置、协议、任务与机器人策略 | `app_config.h`、`app_protocol.h`、`app_task.h` |

Device 使用 BSP 完成通信，可组合算法完成设备控制；通用算法不依赖具体设备。Application 组合驱动与算法，决定目标来源、机构坐标、运行状态和发送时序。

`Core/` 保留 CubeMX 管理的启动与外设代码；HAL/CMSIS 位于 `Drivers/`，FreeRTOS 位于 `Middlewares/`。新增板级接入写在 CubeMX 的 USER CODE 区域内。

## 初始化与执行

```text
main()
  → HAL、时钟、GPIO、DMA、CAN、UART、SPI、TIM 初始化
  → osKernelInitialize()
  → MX_FREERTOS_Init()
      → AppConfigInit()
      → 创建任务
  → osKernelStart()
      → 各任务 TaskInit()
      → 周期执行 TaskLoop()
```

`AppConfigInit()` 负责公共设备、通信回调及算法对象的初始化。各任务的 `TaskInit()` 在任务上下文中执行，返回周期；`app_task.c` 使用 `osDelayUntil()` 调度。当前 RTOS tick 为 1 ms。

每个任务按机构需要执行反馈读取、坐标转换、滤波、目标处理、控制和发送。跨任务共享数据的访问方式由应用确定。

## 配置边界

`app_config.h/.c` 是普通应用 C 代码，保存角色、来源、任务周期、外设映射和机械参数。公共设备对象可在这里声明并初始化，也可按机构拆分应用文件。

`app_robot.c` 实现机器人行为。`app_protocol.h/.c` 与它平级，定义协议字段、解析和发送；裁判系统协议位于 `app_referee.h/.c`。

默认应用没有配置实际电机闭环。角色枚举只表达配置意图，应用需实现对应的单板/双板分工和控制来源选择。

## 构建组织

根目录 `CMakeLists.txt` 收集 Skywalker 各层 `Src/` 下的 C 文件并编译，直接包含各层 `Inc/`。`cmake/stm32cubemx/` 提供 HAL、Core 与 RTOS 构建配置，`cmake/gcc-arm-none-eabi.cmake` 提供工具链设置。

Debug 和 Release 预设分别使用独立的 `build/Debug/`、`build/Release/`。增加源文件后重新运行 CMake 配置；更换业务实现时确保四组任务接口只保留一份强定义。

具体开发步骤见 [应用开发](manual-application.md)，整定接口见 [自动整定与辨识](autotune.md)。
