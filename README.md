# H7 Framework 工程

H7 Framework 是一套面向 RoboMaster 的 STM32H723 固件工程。工程包含 STM32 HAL、FreeRTOS 和 CMSIS-DSP；`Skywalker/` 中的代码负责外设封装、设备驱动、控制算法及机器人应用接口。

## 编译固件

需要 CMake 3.22 或更新版本、Ninja 和 `arm-none-eabi-gcc`，并确保这些程序在 `PATH` 中。Windows 可使用 STM32CubeCLT 提供的 ARM GNU Toolchain，Linux 可安装对应发行版或 ARM 官方工具链。

在工程根目录编译：

```sh
cmake --preset Debug
cmake --build --preset Debug
```

发布构建使用 `Release`：

```sh
cmake --preset Release
cmake --build --preset Release
```

固件和链接映射文件位于 `build/Debug/` 或 `build/Release/`。烧录 `h7_framework.elf` 可使用 STM32CubeProgrammer 或调试器。引脚、时钟及外设实例由根目录的 `h7_framework.ioc` 管理；修改 CubeMX 配置后，检查并保留 USER CODE 区域内的应用改动。

## 首次接入机器人

机器人相关代码集中在 `Skywalker/Application/`：

| 文件 | 用途 |
| --- | --- |
| `Src/app_config.c`、`Inc/app_config.h` | 板卡角色、控制来源、任务周期、串口/CAN映射、机构参数和设备初始化 |
| `Src/app_robot.c` | 管理、云台、底盘和发射任务的业务逻辑 |
| `Inc/app_protocol.h`、`Src/app_protocol.c` | 自定义通信消息、解析和发送 |

建议按以下顺序接入：

1. 在 `h7_framework.ioc` 配置硬件外设和引脚。
2. 在 `app_config.c` 填入本板使用的通信外设、板卡角色、任务周期和机构参数，并在 `AppConfigInit()` 初始化设备、算法及接收回调。
3. 在 `app_robot.c` 实现任务初始化和周期循环，完成反馈读取、坐标换算、控制及指令发送。
4. 需要自定义通信时，在 `app_protocol.h/.c` 定义字段和收发逻辑。
5. 编译、烧录后，按电机方向、坐标系、限幅和实际任务周期逐项核对。

默认应用尚未接入电机和控制闭环。四个任务的 `TaskLoop()` 为空，`TaskInit()` 使用配置中的 1 ms 周期；通信配置和机构参数是可修改的起始值，不代表已匹配某台机器人。单板机器人可在同一工程中实现各机构任务；双板分别配置云台板与底盘板，通过应用协议传递两板所需的数据。板卡角色和控制来源是配置项，具体选择与切换逻辑由应用实现。

应用层的物理量使用 SI 单位。控制链中要让目标、反馈和输出保持相同的坐标定义及采样周期。例如，电机速度环通常接收 rad/s、输出 A；位置外环可接收 rad、输出速度目标 rad/s。电机协议的原始计数和厂商单位由驱动层换算。

## Skywalker 目录

| 目录 | 内容 |
| --- | --- |
| `Skywalker/BSP/` | UART、FDCAN、SPI、流缓存、DMA及周期计时封装 |
| `Skywalker/Device/` | DJI、DM、LK 电机，DT7 遥控器和 BMI088 IMU 驱动 |
| `Skywalker/Algorithm/` | PID、DOB、运动学、轨迹、滤波、状态估计、系统辨识与整定 |
| `Skywalker/Application/` | 机器人配置、业务任务和通信协议 |

电机驱动包括 DJI M2006、M3508、GM6020，DM J4310/J8009P，以及 LK 电机。GM6020 的电流控制需要在 RoboMaster Assistant 中启用对应模式。DM J8009P 的模式、量程及接线示例见[应用开发说明](docs/manual-application.md#dm8009p-电机)。

## 控制与辨识算法

算法以普通 C 接口提供，应用按需要组合：

- PID、DOB、跟踪微分器，以及低通、中值、陷波、Kalman 和 IMU EKF。
- 全向轮、麦克纳姆轮和舵轮运动学；舵轮目标协调、零速保持和驱动轮速修正。
- 五次轨迹、变化率限制、底盘轮速估计、电机模型功率约束。
- 积分递推最小二乘，估计等效惯性、黏性阻尼、库仑摩擦和恒定负载。
- 单电机整定接口。`alg_motor_tune` 在板端辨识单电机，并按带宽、阻尼比、前馈比例测试候选参数；只有满足用户设置的性能范围且通过独立轨迹验证的候选，才进入输出的帕累托集合。

整定不是默认启动的功能。应用须提供一致的采样、反馈、限幅后的实际指令，并负责开始/取消测试及保存结果。整定需要电机实际运行；配置与接入步骤见[自动整定说明](docs/autotune.md)。

## 文档

- [框架结构](docs/framework-guide.md)：启动顺序、层间职责及配置边界。
- [应用开发](docs/manual-application.md)：外设、任务、电机和协议接入。
- [算法清单](docs/algorithms.md)：算法模块及组合关系。
- [辨识与自动整定](docs/autotune.md)：模型假设、调用方法、参数范围和结果解释。
