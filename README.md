# H7 Framework

这是一个基于 STM32H723、HAL、FreeRTOS 和 CMSIS-DSP 的 CMake 工程框架。
工程已经包含 STM32CubeMX 生成的底层代码，以及 `Skywalker` BSP、算法、设备和应用层代码。

## 初次配置

### 1. 修改工程名

打开根目录下的 `CMakeLists.txt`，修改项目名：

```cmake
set(CMAKE_PROJECT_NAME h7_framework)
```

例如改为：

```cmake
set(CMAKE_PROJECT_NAME team_a_robot)
```

这个名称会用于生成目标文件、ELF 文件和 map 文件。修改后建议删除旧的构建目录，再重新配置：

```powershell
Remove-Item -Recurse -Force build\Debug
```

如果工程使用 Release 配置，也需要删除 `build\Release`。

### 2. 应用文件

当前应用入口文件是：

```text
Skywalker/Application/Src/app_robot.c
```

这个文件不需要改名。队员直接在其中编写自己的应用逻辑即可。

文件提供四组任务接口：

```c
ManagerTaskInit();  ManagerTaskLoop();
GimbalTaskInit();   GimbalTaskLoop();
ChassisTaskInit();  ChassisTaskLoop();
ShootTaskInit();    ShootTaskLoop();
```

初始化函数返回任务周期，单位为毫秒；循环函数由 FreeRTOS 任务周期性调用。
任务调度实现位于 `Skywalker/Application/Src/app_task.c`，通常不需要修改。

如果需要保留多个应用版本，可以在工程外备份，或者在 `Skywalker/Application/Src` 中只保留当前正在使用的一个应用实现文件。多个文件同时定义同名任务接口会导致链接时重复定义。

## 编译链配置

工程使用以下编译工具：

- CMake 3.22 或更高版本
- Ninja
- ARM GNU Toolchain，提供 `arm-none-eabi-gcc`、`arm-none-eabi-g++`、`arm-none-eabi-objcopy` 和 `arm-none-eabi-size`
- Windows PowerShell（以下命令以 PowerShell 为例）

当前 `CMakePresets.json` 默认使用 STM32CubeCLT 1.22.0 的安装路径：

```text
C:/ST/STM32CubeCLT_1.22.0/GNU-tools-for-STM32/bin/
```

如果本机安装路径不同，请修改 `CMakePresets.json` 中的两个编译器路径：

```json
"CMAKE_C_COMPILER": "C:/ST/STM32CubeCLT_1.22.0/GNU-tools-for-STM32/bin/arm-none-eabi-gcc.exe",
"CMAKE_CXX_COMPILER": "C:/ST/STM32CubeCLT_1.22.0/GNU-tools-for-STM32/bin/arm-none-eabi-g++.exe"
```

同时确保 ARM GNU Toolchain 的 `bin` 目录已经加入 `PATH`，因为 `cmake/gcc-arm-none-eabi.cmake` 还会使用 `arm-none-eabi-objcopy`、`arm-none-eabi-size` 等工具。

可以在 PowerShell 中检查：

```powershell
arm-none-eabi-gcc --version
cmake --version
ninja --version
```

## 配置和构建

在工程根目录执行：

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

Release 构建：

```powershell
cmake --preset Release
cmake --build --preset Release
```

生成文件位于：

```text
build/Debug/
build/Release/
```

其中 ELF 文件名跟随 `CMakeLists.txt` 中的工程名。工程也会生成 `compile_commands.json`，可供 VS Code、clangd 等代码分析工具使用。

## 工程依赖

### 已经随仓库提供

以下依赖已经位于工程目录中，通常不需要额外下载：

- STM32H7xx HAL Driver
- CMSIS Core、Device 和 RTOS2 头文件
- CMSIS-DSP 源码
- FreeRTOS 内核和 CMSIS-RTOS V2 适配层
- STM32H723 的启动文件、链接脚本和 CubeMX 生成的底层代码
- Skywalker BSP、算法、设备和应用层代码

### 需要在电脑上安装

- CMake 3.22+
- Ninja
- ARM GNU Embedded Toolchain，或包含该工具链的 STM32CubeCLT

### 可选工具

- STM32CubeMX：只有在需要重新生成 `Core`、`cmake/stm32cubemx` 或外设配置时才需要。重新生成后应检查生成代码是否覆盖了手工修改。
- VS Code：不是构建必需品，但仓库提供了 `.vscode/c_cpp_properties.json`，默认读取 `build/Debug/compile_commands.json`。

## 硬件和芯片假设

当前工程针对 STM32H723xG，使用：

- Cortex-M7，硬件浮点 ABI
- `STM32H723xx` 编译宏
- `STM32H723XG_FLASH.ld` 链接脚本

如果更换芯片或 Flash/RAM 配置，不能只修改工程名，还需要同步修改 CubeMX 配置、设备宏、启动文件和链接脚本。

## 常见问题

### 找不到编译器

确认 `CMakePresets.json` 中的路径存在，并且下面的命令可以执行：

```powershell
arm-none-eabi-gcc --version
```

### 修改工程名后仍然使用旧文件

删除对应的 `build\Debug` 或 `build\Release` 目录，再重新执行 `cmake --preset ...`。CMake 会缓存旧的项目配置。

### 链接时出现任务函数重复定义

检查 `Skywalker/Application/Src` 中是否同时存在多个定义了 `ManagerTaskInit`、`GimbalTaskInit`、`ChassisTaskInit` 或 `ShootTaskInit` 的应用文件。只保留当前使用的应用实现。

### Ninja 无法写入构建目录

确认没有其他 CMake/Ninja 构建进程正在使用该目录，并检查目录权限。必要时关闭 IDE 的后台构建任务后，删除构建目录并重新配置。
