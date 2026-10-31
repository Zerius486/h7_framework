# 算法说明

通用算法位于 `Skywalker/Algorithm/Inc/` 和 `Src/`，通过 C 对象与函数组合。下表列出主要模块及用途；SIFt-RLS、PI 性能优化和帕累托整定按实验功能使用。

## 控制与反馈处理

| 模块 | 用途 | 接口或配置 |
| --- | --- | --- |
| `alg_pid` | 并联 PID，支持梯形积分、微分先行、参考差分前馈、条件积分、过零衰减、死区与限幅 | `PidInit/Calculate/Reset`；`PidSetGains` 改参，`PidTrackOutput` 跟踪外部限幅 |
| `alg_dob` | 一阶 Q 滤波扰动观测器，输出补偿电流 | `DobInit/Update/Reset`；输入实际电流与转子速度 |
| `alg_td` | 临界阻尼二阶跟踪微分器，输出平滑值与微分估计 | `r`、`dt` |
| `alg_td_nl` | 含 Fhan 分段函数的非线性跟踪微分器 | `r` 为跟踪加速度上限，滤波时间 $h\geq\Delta t$ |
| `alg_lpf` | 一阶低通与二历史输出的递推平滑 | `alpha` 或 `alpha_1/alpha_2` |
| `alg_median` | 固定 5 点中值滤波 | 抑制孤立尖峰 |
| `alg_notch` | 二阶陷波 | 中心频率 `f0`、品质因数 `Q`、采样率 `fs` |

PID 的 `output_max` 必须为正；`integral_max` 限制误差积分。当 $K_i>0$ 且积分输出预算为 $I_{\max}$ 时，设置为 $I_{\max}/K_i$。外部电流限幅或功率分配后，用 `PidTrackOutput()` 反馈“最终执行量减限幅前总请求”的差值。

DOB 使用名义惯性模型：

$$
\hat d(s)=Q(s)[K_t i(s)-Js\omega(s)],
\qquad
J=J_{\mathrm{motor}}+\frac{J_{\mathrm{load}}}{n^2}
$$

$n$ 对应 `reduction_ratio`。$J_{\mathrm{motor}}$ 为转子惯量，$J_{\mathrm{load}}$ 为负载侧惯量，合成后的 $J$ 与输入速度均取转子侧。按 $J\dot\omega=K_ti-d$ 定义阻力矩 $d$，将返回的补偿电流加到控制请求中，再做最终限幅。这里采用电调内部电流环快于机械动态的近似。

低通递推为：

$$
y[n]=\alpha x[n]+(1-\alpha)y[n-1]
$$

$$
y[n]=\alpha_1x[n]+\alpha_2y[n-2]+(1-\alpha_1-\alpha_2)y[n-1]
$$

第一式的 $\alpha$ 越小，响应越慢；第二式按系数直接递推，不自动由截止频率生成 Butterworth 系数。

## 状态估计与姿态

| 模块 | 用途 | 使用条件 |
| --- | --- | --- |
| `alg_kf` | 通用线性 Kalman 预测与观测更新 | 应用配置 F/B/H/Q/R/P，工作区由调用方提供 |
| `alg_imu_ekf` | 融合三轴陀螺与加速度 | 六状态：roll/pitch/yaw 和陀螺零偏 |
| `alg_quaternion` | 四元数乘法、归一化、欧拉角转换 | 姿态数学 |
| `alg_euler` | 四元数转欧拉角，姿态及角速度数据结构 | 姿态表达 |

IMU EKF 用角速度预测、重力观测修正姿态。当前没有绝对航向观测，yaw 会漂移；平移加速度会影响重力观测。BMI088 驱动已组合此 EKF、安装修正、静止标定、重力扣除和加热 PID。

## 底盘与运动计算

### 运动学

`alg_kinetics` 提供全向轮、X 型麦轮和舵轮运动学。输入车体速度 $(v_x,v_y,\omega)$ 与半轮距，输出四轮线速度；舵轮另输出舵角。公共轮序为左前、左后、右后、右前，$x$ 向前、$y$ 向左、逆时针为正。

接口为 `ChassisInit()`、`ChassisSetParams()`、`ChassisSetVelocity()`、`ChassisUpdateState()`。舵轮可用实际舵角选取较短转向路径，超过 $90^\circ$ 时翻转驱动方向，零速保留舵角。

### 轨迹、协调与功率

`alg_motion` 包含：

| 接口 | 功能 |
| --- | --- |
| `MotionRampUpdate` | 分别限制标量目标的上升、下降速度 |
| `MotionTrajectoryReset/Update` | 五次轨迹，目标变化时保持位置、速度、加速度连续；时长由应用指定 |
| `SteerSetpointReset/Update` | 按轮速加速度、舵向速度生成四舵轮协调目标，含选向滞回与静止预转向 |
| `SteerSpeedCorrect` | 按实际舵角误差修正驱动轮速 |
| `MotorPowerEstimate` | 累加各电机功率 $\max(k_i i^2+K_t\omega i,0)$ |
| `MotorPowerScale` | 二分求所有电流的统一缩放比例 |
| `ChassisEstimateVelocity` | 由四轮反馈最小二乘反算车体速度，可加入加权陀螺约束，输出残差 |

五次轨迹不求解额外的速度、加速度或 jerk 约束；协调器限制规划目标，轮胎附着需由实车评价。

三次余弦轮速修正：

$$
\begin{aligned}
\delta&=\operatorname{wrap}(\theta_{\mathrm{target}}-\theta_{\mathrm{actual}}),\\
\phi&=\min(a\lvert\delta\rvert,\pi/2),\\
v&=v_{\mathrm{target}}\max(\cos\phi,0)^3.
\end{aligned}
$$

$a\geq0$ 调节衰减，0 关闭修正，1 使用原角度误差。输出保留驱动方向。修正结果送入驱动速度环，不写回 `SteerSetpoint.wheels`，避免重复衰减。调用示例见[舵轮驱动速度修正](manual-application.md#舵轮驱动速度修正)。

## 辨识与整定

| 模块 | 功能 |
| --- | --- |
| `alg_sysid` | 四参数积分 RLS，估计等效惯性、黏性阻尼、库仑摩擦、恒定负载；可选 SIFt-RLS |
| `alg_autotune` | 主动辨识、模型生成速度 PI、可选位置与 DOB 验证；正常运行可慢速自适应 |
| `alg_pid_tune` | 借鉴 DiffTune+ 的敏感度优化，调整速度 PI 的两个增益 |
| `alg_motor_tune` | 机构特征测量、最多 27 组候选测试、性能约束与重复搜索内验证，导出非支配候选 |

`alg_multisine` 提供闭环多正弦参考，最多 8 个谐波；接口为 `MultisineInit/Start/Calculate/Cancel`。应用选择接入的控制器并记录实际周期与测量标记，频响在[离线工具](../tools/tuning/README.md)中计算。

应用提供反馈、最终执行指令、开始/取消和结果处理入口。当前流程需要有限行程边界及双向测试，未提供连续旋转、单向机构模式、结构化结果导出和工况参数函数。调试时可使用串口记录器保存整定波形和英文事件，说明见[整定工具文档](../tools/tuning/README.md)。模型、参数范围、指标定义及示例见[自动整定与辨识](autotune.md)。

## 数值与通信辅助

| 模块 | 功能 |
| --- | --- |
| `alg_conversion` | 转速、角度、弧度和编码器数值转换 |
| `alg_crc` | CRC8、CRC16 计算与校验 |

## 控制链组合

| 机构 | 调用顺序 |
| --- | --- |
| 云台、舵向、拨弹 | 轨迹/TD → 位置外环 → 速度内环 → 可选前馈与 DOB → 限幅发送 |
| 驱动轮、摩擦轮 | 速度 PI/PID → 可选补偿 → 限幅发送 |
| 全向、麦轮底盘 | 车体目标 → 运动学 → 各轮速度环 |
| 舵轮底盘 | 车体目标 → 协调目标 → 实际舵角轮速修正 → 驱动速度环；舵角目标送位置环 |

DJI 驱动已封装速度与位置串级控制、连续角度展开。GM6020 通过 $\theta+\omega\Delta t$ 选择等价目标角度分支。DOB、功率分配和整定由应用接入。

对象和矩阵工作区可静态分配，调用周期由应用保持。板端用时测量使用 `bsp_profile`，接入方法见[应用开发](manual-application.md#4-编译与测量)。

