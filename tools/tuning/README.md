# 整定数据采集与离线分析

PC 工具用于保存 UART 调试流、分析时序记录及生成候选建议。电机测试和实时控制在 H7 上执行，固件不依赖 Python。

需要 Python 3.10 或更新版本及 NumPy。在工程根目录执行：

```sh
python -m pip install -r tools/tuning/requirements.txt
python tools/tuning/tune.py --help
```

| 命令 | 输入 | 输出 |
| --- | --- | --- |
| `collect.py` | UART 调试流、工况元数据 | 原始流、采样 CSV、事件日志、元数据 |
| `tune.py identify` | 连续定周期 CSV 或旧版 24 字节 BIN | 窗口/延迟与 LS/IV 辨识对照 |
| `tune.py frequency` | 按完整周期对齐的多正弦 CSV | 各激励频率的增益、相位及诊断量 |
| `tune.py propose` | 搜索配置、逐次候选 JSONL | 下一组建议参数 |
| `tune.py replay` | 搜索配置、实测候选 JSONL | 同一候选表上的搜索次序对照 |

## 串口记录

### 板端配置与接线

采集器依赖 `pyserial`，已包含在 `requirements.txt` 中。固件默认调试 UART 为 UART7（921600 baud）。需要记录时，在 `app_config.c` 将 `g_app_debug_log_config.enabled` 改为 `true`，并用 USB-UART 适配器连接板端 TX、GND 与 PC。UART 引脚和电平按板卡原理图确认；采集端口须与电脑实际枚举的串口一致。

`AppConfigInit()` 已初始化记录器。更换 UART 时，`.huart` 和 `.baudrate` 必须与 CubeMX 初始化一致；日志发送使用该 UART 的 DMA。通道默认关闭，不发送调试帧。开启后，记录器仅在运行记录期间发送数据。

```c
const DebugLogConfig g_app_debug_log_config = {
    .huart = &huart7,
    .enabled = true,
    .binary_enabled = true,
    .text_enabled = true,
    .sample_period = 1U,
    .baudrate = 921600U};
```

### 周期调用

自动整定日志适配器需要应用显式调用。每次整定更新及最终执行电流跟踪完成后，在同一个控制周期中调用对应函数：

```c
AppTuneLogAutotune(&autotune, HAL_GetTick(), position, omega,
                   actual_current, requested_current, executed_current);
AppTuneLogMotorTune(&motor_tune, HAL_GetTick(), position, omega,
                    actual_current);
```

仅调用当前使用的整定对象对应的函数。首次检测到对象活跃时自动开始记录，调用端需要继续传入运行值；完成、失败或取消状态也必须至少调用一次，以缓存结果事件并请求停止采样。适配器会处理缓存发送，结束后继续周期调用即可；若调用仅放在整定活跃分支中，应在同一任务每周期调用 `AppTuneLogProcess()`，并在退出活跃分支前记录一次结束状态。`requested_current` 是限幅前请求值，`executed_current` 是限幅、功率分配后实际发送的指令，不能互换。

普通控制波形可直接使用 `DebugLogObject` 的 `Start/SampleWrite/Info/Error/Stop` 接口，并在同一任务每周期调用 `DebugLogProcess()`。对象应静态分配，一个对象和对应 UART 的日志发送由同一任务管理。`DebugLogStop()` 停止接收新采样，周期处理继续发送缓存事件及结束摘要；最后一帧 DMA 完成后，`recording` 才清零。此时才能开始下一轮，不要在旧记录发送期间重新初始化对象。

### PC 采集

采集端启动如下。Windows 使用实际枚举的 `COM` 端口；Linux 将 `COM7` 替换为实际串口路径，例如 `/dev/ttyUSB0`：

```sh
python tools/tuning/collect.py COM7 --baudrate 921600 --output records/bench-20261027-001 --duration 120 --metadata tools/tuning/collect.example.json
```

先启动采集器，再开始板端试验。`--duration 0` 会持续采集到 `Ctrl-C`。输出目录必须不存在，避免覆盖已有实验；每次试验使用新的目录：

| 文件 | 内容 |
| --- | --- |
| `raw.bin` | 串口原始字节，便于重新解码 |
| `samples.csv` | 采样波形，选取连续数据段后用于辨识或其他分析 |
| `events.log` | ASCII 英文事件、丢同步和 CRC 错误 |
| `metadata.json` | 用户提供的工况字段、端口、波特率、帧数、坏 CRC 数、序号缺口、板端丢弃计数和 UTC 起止时间 |

一次采集可包含多轮板端运行。分析前按运行、阶段和连续区间选段，保留原始采集目录。

### 帧格式与丢弃统计

板端二进制帧由 `A5/version/type/length/sequence/payload/CRC16` 组成，当前版本为 1。44 字节采样负载按小端格式编码，含时间、采样序号、阶段、标志、限幅状态、参考、位置、速度、电流及辨识模型摘要。文本事件以 ASCII 封装在相同帧格式中，例如 `RUN_START`、`STAGE`、`RUN_COMPLETE` 和 `RUN_FAILED`，采集器将其写入 `events.log`。

`DebugLogSampleWrite()` 按 `sample_period` 抽样，通过 DMA 非阻塞发送；UART 忙或有待发送事件时丢弃波形帧，已选中的采样仍消耗序号。英文事件使用 8 槽静态缓存，发送失败时保留待下一周期处理；缓存满时淘汰最早的待发事件，为最新结果留出位置，文本超长时丢弃该文本，两者均计数。待发事件优先于波形，结束摘要单独生成，不占事件槽。

结束摘要为 `RUN_STOP DROPPED=n TEXT_DROPPED=m`。`DROPPED` 包含板端丢弃的波形和文本帧，`TEXT_DROPPED` 是其中文本数量。PC 的 `sequence_gaps` 反映接收帧之间的序号缺口，可能包含板端丢弃和传输缺失，不与板端计数相加。`firmware_dropped_frames`、`firmware_dropped_text_frames` 累加收到的结束摘要，`run_summaries` 记录摘要数量；没有收到摘要时前两个字段为 `null`，不能解释为零丢弃。多轮采集只统计已收到的摘要，文本关闭或采集提前结束时不具备完整板端计数。缺失波形无法恢复。采集器的 `--baudrate` 必须与固件 UART 波特率相同；它不会更改板端设置。

自动整定适配器记录运行中的波形、阶段切换、候选序号及成功、失败或取消事件。它不自动导出完整 `AutotuneConfig`/`MotorTuneConfig`、控制器配置或帕累托候选表；运行工况应填写到元数据 JSON，最终参数仍由应用读取整定对象并保存。普通控制波形可使用 `DebugLogSampleWrite()`，开始和停止时机由应用决定。

## 记录格式

CSV 至少包含：

```csv
time_ms,speed_rad_s,actual_current_a
0,0,0
2,0.01,0.03
```

数据须按固定周期连续采集，至少 100 点。速度、电流与实际控制器使用相同坐标。缺失片段先拆分，不插值跨越丢包。原始文件保留，输出不能覆盖输入。

`identify` 的 CSV 输入可从采集器生成的 `samples.csv` 选段得到。采集目录中的 `raw.bin` 是带帧头和 CRC 的原始串口流，不是 `identify` 支持的旧版 BIN，不能直接作为该命令输入。

旧版 BIN 为 24 字节小端定长记录：`uint32 time_ms, rx_count, rx_age_ms; float speed_rad_s, actual_current_a, executed_current_a`。`rx_count`、`rx_age_ms` 作为原始信息保留，分析器不据此自动校准时间。

可选字段：

| 字段 | 用途 |
| --- | --- |
| `reference` | 独立设计的外部参考，供工具变量及闭环频响使用；不能用控制器输出或执行电流代替 |
| `sample` | 多正弦周期内下标，从0到`period_ticks-1`；用于检查周期对齐 |
| `limited` | 本周期是否发生控制输出限幅或外部功率分配；频响分析排除整段受限周期 |
| `period`、`measuring` | 建议保存，便于人工核对激励阶段；命令通过显式周期长度和过渡周期数选段 |

这些字段需按试验类型填写。整定适配器的 `reference` 保存对应控制阶段的目标值，不会自动接入 `MultisineObject`；CSV 的 `sequence` 是采样序号，也不是多正弦的周期内 `sample`。多正弦记录需由应用保存外部参考及周期标记，并保证参考、电流和速度的采样时序配对。

## 窗口、延迟与工具变量对照

```sh
python tools/tuning/tune.py identify motor.csv --min-speed 0.16 --windows 0.01 0.02 0.04 --delays 0 2 --output comparison.json
```

默认用前65%数据拟合，后35%检验，可用`--split`修改。每种窗口和延迟单独报告约束最小二乘、工具变量两阶段最小二乘的参数、物理性、电流拟合度与速度自由预测误差。离线电流拟合度使用验证段积分电流去均值后的能量作分母，与板端`fitness`的分母不同。工具变量由外部参考及其延迟生成，用SVD去除共线工具，再对投影后的方程枚举阻尼、摩擦非负约束。没有`reference`或投影后秩不足时，会报告不可用。

速度预测每250ms用一个实测初值重新开始，片段内只输入电流；静止和过零另起片段。报告至少需要0.5s有效预测和4个片段，速度拟合度门槛为0.75。它评价滑动模型，不评价静摩擦起动过程。

离线最小二乘与板端 RLS 的遗忘权重、数据段检查不同，两者结果分别保存。IV 要求外部参考与相关测量噪声满足外生性条件；仅有名为 `reference` 的列并不足够。比较后选出的窗口和延迟，用另一次未参与选择的记录确认。

## 闭环多正弦频响

板端使用`alg_multisine`生成速度或位置参考，原有控制器继续闭环运行。保存相同采样时刻的实际电流、速度和外部参考，第一行对应第一个过渡周期的第0点：

```sh
python tools/tuning/tune.py frequency multisine.csv --period-ticks 1000 --harmonics 1 3 7 13 --settling-periods 2 --output response.json
```

周期长度取`MultisineObject.period_ticks`，不能用未量化的设定频率反算。脚本只取完整周期，至少两个测量周期。限幅周期须通过`limited`明确标记。

用外部参考$r$作工具变量，计算电流到速度的被控对象频响：

$$
\hat G(j\omega)=\frac{\langle\Omega(j\omega)R^*(j\omega)\rangle}
{\langle I(j\omega)R^*(j\omega)\rangle}
$$

输出幅值、相位、实虚部、输入输出相干度与逐周期比值的离散程度。后两项是诊断量，不能单独证明模型正确。重复相同相位主要用于观察重复性；非线性失真评估还需要不同相位或幅度的独立试验。这里不自动调整控制器。

## 贝叶斯候选建议

复制`search.example.json`，填写带宽、阻尼、前馈范围及性能约束。`reference`是两个目标的超体积参考点，必须比关心的可行前沿更差；示例值不是电机推荐值。`relative_noise`是归一化观测噪声，重复试验另计样本均值方差；`length_scale`控制归一化空间中的相关长度。

每次实测追加一行JSON到`trials.jsonl`。空文件也可用于第一次建议：

```json
{"run_id":"motor01-001","context":{"motor":"M2006","load":"bench-a"},"stage":"search","theta":[18.0,1.1,0.5],"metrics":{"rms_error":0.12,"overshoot":0.08,"settling_time":0.2,"idle_ripple":0.01,"rms_slew":20.0,"saturation":0.02,"settled":true}}
```

记录全部试验，包括不满足性能约束的候选。相同参数的重复试验逐行保留。`theta`顺序固定为$[\omega_n,\zeta,k_{\mathrm{ff}}]$，指标含义与`MotorTuneMetrics`一致。模型辨识失败、没有完整指标的试验保留在原始档案中，不作为数值训练样本。

```sh
python tools/tuning/tune.py propose search.json trials.jsonl --output next.json
```

初始点不足时做空间填充；随后用固定长度尺度 Matérn 5/2 高斯过程，联合采样已测点与候选的后验，估计满足约束时的含噪声超体积增益。目标各自归一化。实现为有限候选池中的单点近似，未采用完整 BoTorch qNEHVI 及其超参数推断；没有采样到增益时返回空间探索点。

应用按现有 PI 设计关系生成建议参数对应的控制器，在板端测试完整控制链，并将指标写入下一条试验记录。工具只生成建议，不写回板端；`MotorTune` 可独立使用。

同一次优化只接受相同`context`，不同电机、负载、姿态或控制链先拆分。最终确认用未参与候选选择的轨迹/负载，记录为`stage: "confirmation"`；此类数据不参与后续建议。板端`validation`数据参与搜索细化，导入时应归入`search`，不能当作完全留出的最终确认。

## 在实测候选表中回放

```sh
python tools/tuning/tune.py replay search.json trials.jsonl --budget 8 --output replay.json
```

在已完整测过的有限候选表内，比较记录顺序、固定随机顺序和贝叶斯选择顺序。只有选中一组参数后，才向该策略暴露其搜索指标。独立`confirmation`只用于报告，输出每步的确认超体积。缺少最终确认数据时不报告算法胜负；没有原始网格数据时，记录顺序也不能称为网格基线。一次固定随机次序只提供可复现对照，不能代表统计优势。

原始时序记录、逐次候选指标和工况元数据都应保留，之后可用于本地NN训练。回放不会为未测参数编造结果。

## 已有记录的对照

[2026-10-27 辨识报告](reports/identification-20261027.json)保留了四份历史记录的文件摘要、分析配置、当时板端算法的回放和离线窗口/延迟对照。原始文件位于测试工程，未随本仓库分发；报告用于保留该批数据的分析结果。

20ms窗口、无电流延迟、前65%拟合后的速度预测结果如下：

| 记录 | 离线惯性参数$M$ | 后段速度RMSE，rad/s | 速度拟合度 |
| --- | --- | --- | --- |
| M2006 18:59 | 0.004458 | 0.179 | 0.967 |
| M2006 19:21 | 0.003814 | 0.296 | 0.917 |
| GM6020 15:23 | 负值 | 拒绝预测 | — |
| GM6020 15:39 | 负值 | 拒绝预测 | — |

板端回放中，两份M2006通过原有模型检查；GM6020在2点电流延迟下能得到正惯量，但仍未通过数据段质量检查。离线固定前段拟合的GM6020参数也未得到可靠的后段预测，说明不能仅凭正惯量或电流拟合度判定成功。新分阶段激励、实际配对延迟和机构负载仍需重新实测。

这四份记录没有独立外部参考，也没有完整候选搜索表，因此报告没有给出IV、闭环频响或贝叶斯搜索的实机优劣结论。
