# H7 Framework

面向 RoboMaster 的 STM32H723 固件工程，基于 STM32 HAL、FreeRTOS 和 CMSIS-DSP。Skywalker 提供外设封装、设备驱动、控制算法与应用任务接口。

## 构建

安装 CMake 3.22+、Ninja 和 ARM GNU Toolchain，并将工具加入 `PATH`。在工程根目录执行：

```sh
cmake --preset Debug
cmake --build --preset Debug
```

发布构建使用 `Release` 预设。产物位于 `build/Debug/` 或 `build/Release/`，包括 `h7_framework.elf` 和链接映射文件。使用 STM32CubeProgrammer 或调试器烧录 ELF。外设、引脚和时钟由 `h7_framework.ioc` 管理。

## 工程结构

| 目录 | 职责 |
| --- | --- |
| `Core/` | 板级启动、时钟、外设和 FreeRTOS 配置 |
| `Drivers/` | STM32 HAL、CMSIS 与 CMSIS-DSP |
| `Middlewares/` | FreeRTOS |
| `Skywalker/BSP/` | UART、CAN、SPI、流缓存、DWT 计时等封装 |
| `Skywalker/Device/` | 电机、遥控器和 IMU 驱动 |
| `Skywalker/Algorithm/` | 控制、滤波、状态估计、运动学、辨识和整定 |
| `Skywalker/Application/` | 机器人配置、任务业务及通信协议 |
| `cmake/` | ARM 工具链和固件构建配置 |
| `docs/` | 应用开发和算法说明 |

## 配置与控制逻辑

使用者主要通过修改 `app_robot.c`、`app_config.c`、`app_protocol.h/.c`，实现针对特定机器人的控制逻辑。

| 文件 | 内容 |
| --- | --- |
| `Skywalker/Application/Src/app_config.c` | 板卡角色、控制来源、通信映射、机构参数和设备/算法初始化 |
| `Skywalker/Application/Inc/app_config.h` | 配置类型及应用对象的公共声明 |
| `Skywalker/Application/Src/app_robot.c` | 管理、云台、底盘和发射任务的控制逻辑 |
| `Skywalker/Application/Inc/app_protocol.h` | 自定义消息字段和通信接口 |
| `Skywalker/Application/Src/app_protocol.c` | 协议接收、编解码和发送 |

`AppConfigInit()` 在外设与 RTOS 内核初始化后、任务创建前调用一次。四组 `TaskInit()` 返回周期，单位为 ms；`TaskLoop()` 负责反馈处理、目标计算、闭环控制和指令发送。调度由 `app_task.c` 完成。

默认应用是业务骨架：四个任务周期均为 1 ms，没有初始化电机，任务循环为空；视觉串口映射为 UART10，机构参数需按实车填写。角色和控制来源配置由应用解释，不会自动启用某套控制逻辑。单板实现各机构任务；双板分别配置板卡角色、设备及板间消息。

应用层物理量采用 SI。PID 必须配置正的输出上限；速度环输出通常为电流 A，位置外环输出为角速度 rad/s。算法可以直接组合，调用方负责保持坐标、采样周期及输入输出含义一致。

## 算法

- PID、DOB、跟踪微分器、低通、中值与陷波滤波、Kalman 和 IMU EKF。
- 全向轮、麦轮和舵轮运动学；舵轮目标约束与三次余弦修正。
- 变化率限制、五次轨迹、轮速反算和电机模型功率缩放。
- 固定尺寸积分 RLS、单电机自动整定与慢速参数自适应。
- 三种底盘的落地响应测试与角速度外环整定。
- DWT 周期计数，用于测量板端实际耗时。

这些模块均通过 C 接口调用。整定的激励输出、任务切换和参数保存由应用实现。

## 文档

- [框架结构](docs/framework-guide.md)
- [应用开发](docs/manual-application.md)
- [自动整定与辨识](docs/autotune.md)
- [下位机算法路线与研究来源](docs/algorithm-roadmap.md)
