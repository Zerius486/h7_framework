# H7 Framework

这是一个基于 STM32H723、HAL、FreeRTOS 和 CMSIS-DSP 的 CMake 工程框架。
工程已经包含 STM32CubeMX 生成的底层代码，以及 `Skywalker` BSP、算法、设备和应用层代码。

## 编译链配置

工程使用以下编译工具：

- CMake 3.22 或更高版本
- Ninja
- ARM GNU Toolchain，提供 `arm-none-eabi-gcc`、`arm-none-eabi-g++`、`arm-none-eabi-objcopy` 和 `arm-none-eabi-size`

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

## 控制逻辑的具体实现

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

## 常见问题

### 找不到编译器/CMake/Ninja

请确认所有的这些编译链工具的路径都已经成功添加入PATH, 如果发现没有的话请自行学习环境变量的配置方法。

### 如何修改工程名

打开根目录下的 `CMakeLists.txt`，修改项目名：

```cmake
set(CMAKE_PROJECT_NAME h7_framework)
```

例如改为：

```cmake
set(CMAKE_PROJECT_NAME team_a_robot)
```

这个名称会用于生成目标文件、ELF 文件和 map 文件。修改后建议删除旧的构建目录，再重新配置。

如果工程使用 Release 配置，也需要删除 `build\Release`。

### 链接时出现任务函数重复定义

检查 `Skywalker/Application/Src` 中是否同时存在多个定义了 `ManagerTaskInit`、`GimbalTaskInit`、`ChassisTaskInit` 或 `ShootTaskInit` 的应用文件。只保留当前使用的应用实现。

### Ninja 无法写入构建目录

确认没有其他 CMake/Ninja 构建进程正在使用该目录，并检查目录权限。必要时关闭 IDE 的后台构建任务后，删除构建目录并重新配置。
