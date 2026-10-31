# H7 Framework

面向 RoboMaster 的 STM32H723 固件框架，包含 STM32 HAL、FreeRTOS、CMSIS-DSP，以及电机/传感器驱动、控制算法和应用接口。框架提供可组合的 C 模块；具体机器人配置和任务逻辑由应用完成。

## 当前范围

设备通信、PID/DOB、滤波、姿态估计、底盘运动学、轨迹和计时作为基础模块维护；机器人状态机、模式仲裁和控制权切换由应用定义。

辨识与自动整定按实验功能使用，分阶段辨识、SIFt-RLS、PI 性能优化及帕累托搜索尚需台架验证。

## 编译

需要 CMake 3.22 或更新版本、Ninja 和 `arm-none-eabi-gcc`。Windows 可使用 STM32CubeCLT 提供的 ARM GNU 工具链，Linux 可安装 ARM GNU 工具链并确保命令在 `PATH` 中。

```sh
cmake --preset Debug
cmake --build --preset Debug
```

如果迁移过 STM32CubeCLT 或更换过工具链位置，先删除对应的 `build/Debug/` 或 `build/Release/` 目录，再重新配置。CMake 会从当前 `PATH` 查找 `cmake`、`ninja` 和 `arm-none-eabi-gcc`；工程不保存本机工具链的绝对路径。

发布构建将 `Debug` 替换为 `Release`。生成的 ELF 和链接映射文件位于 `build/Debug/` 或 `build/Release/`。引脚、时钟和外设实例由 `h7_framework.ioc` 管理；重新生成 CubeMX 工程后，检查应用代码及 USER CODE 区域。

### 持续集成

GitHub Actions 工作流位于 [`.github/workflows/ci.yml`](.github/workflows/ci.yml)，推送、PR 或手动触发后执行 Debug/Release 构建和 Python 工具语法、命令入口检查。构建产物 ELF/MAP 在运行页面的 **Artifacts** 中下载，保留 7 天。构建环境为 `ubuntu-24.04`，ARM GNU 工具链由 Ubuntu 软件源安装，实际版本输出到日志。

提交并推送工作流后，在仓库 **Settings → Actions → General** 允许 Actions 运行，无需配置 Secret。首次成功后，可在 **Settings → Rules → Rulesets** 为 `main` 设置合并检查：`Firmware / Debug`、`Firmware / Release` 和 `Python tools`。这些检查覆盖编译和工具入口，电机响应与整定效果仍需实测。

## 应用接入

机器人专属代码位于 `Skywalker/Application/`：

| 文件 | 职责 |
| --- | --- |
| `app_config.h/.c` | 板卡角色、控制来源、任务周期、通信外设、机构参数和初始化 |
| `app_robot.c` | 管理、云台、底盘和发射任务的业务逻辑 |
| `app_protocol.h/.c` | 应用通信消息、解析和发送 |

当前 `AppConfigInit()` 只初始化调试记录器；四个任务循环为空，任务周期默认 1 ms。接入机器人时，在 `app_config.c` 填写本机配置，在 `AppConfigInit()` 初始化设备、算法与接收回调，再在 `app_robot.c` 实现任务初始化和周期逻辑。协议按机器人需要在 `app_protocol.h/.c` 实现。单板/双板分工及遥控、视觉、板间控制来源的切换由应用任务负责。

应用层接口使用 SI 单位，例如角度 rad、角速度 rad/s、电流 A、速度 m/s。设备驱动完成协议数值到 SI 单位的换算；机构方向、零位和轴侧转换由应用配置。目标、反馈及限幅须使用相同坐标，算法采样周期须与调用周期相符。

## 框架模块

| 目录 | 内容 |
| --- | --- |
| `Skywalker/BSP/` | UART、FDCAN、SPI、流缓存、DMA、周期计时和调试日志 |
| `Skywalker/Device/` | DJI、DM、LK 电机，DT7 遥控器和 BMI088 IMU 驱动 |
| `Skywalker/Algorithm/` | PID、DOB、运动学、轨迹、滤波、状态估计、辨识和整定 |
| `Skywalker/Application/` | 机器人配置、业务任务、应用协议及整定日志适配 |

支持 DJI M2006、M3508、GM6020，DM J4310/J8009P 和 LK 电机。GM6020 电流控制需在 RoboMaster Assistant 中启用对应模式；J8009P 的量程与接入说明见[应用开发](docs/manual-application.md#dm8009p-电机)。

算法按需组合，包括 PID/DOB、滤波和状态估计、全向/麦轮/舵轮运动学、轨迹与限幅、机械模型辨识及单电机性能整定。舵轮还提供零速保持和依据实际舵角的驱动速度修正。模块说明见[算法文档](docs/algorithms.md)。

## 调试记录和整定

串口调试记录器默认关闭。调试时在 `app_config.c` 将 `g_app_debug_log_config.enabled` 设为 `true`，并通过 USB-UART 将配置的 UART 接到 PC。当前示例使用 UART7、921600 baud；更换 UART 时，波特率须与 CubeMX 设置一致。

在电机控制周期中调用 `AppTuneLogAutotune()` 或 `AppTuneLogMotorTune()`，记录对应整定对象的波形和英文事件。结束后继续调用适配器，或在同一任务周期调用 `AppTuneLogProcess()`，直到缓存发送完成。PC 采集器保存原始串口流、CSV、事件日志和运行元数据。普通控制波形使用 `DebugLogObject` 接口；调用方法和离线分析命令见[整定工具说明](tools/tuning/README.md)。

固件整定需要应用提供反馈、电流发送、控制权切换和结果处理；框架不会自动启动电机或替应用配置 CAN。当前 `MotorTune` 搜索内验证用于候选筛选，最终性能仍需用未参与搜索的轨迹或负载确认。接口与限制见[自动整定文档](docs/autotune.md)。

## 文档

- [应用开发](docs/manual-application.md)：分层、初始化、设备接入、任务与通信。
- [算法说明](docs/algorithms.md)：算法用途、接口和组合方式。
- [自动整定与辨识](docs/autotune.md)：模型、调用流程、性能指标和边界。
- [整定数据采集与离线分析](tools/tuning/README.md)：串口记录器及 Python 工具。
