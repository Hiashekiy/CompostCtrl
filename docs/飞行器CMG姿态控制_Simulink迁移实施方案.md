# 飞行器六自由度–气动舵面–CMG 姿态控制系统 Simulink 迁移实施方案

版本：实施基线 V1.0  
目标：将现有 Python 姿态控制仿真迁移为 MATLAB/Simulink 原生系统仿真，并保留现有统一姿态 MPC 的数学问题与 HPIPM 求解路线。

---

## 1. 迁移目标与基本原则

### 1.1 最终目标

建立一套可在 MATLAB/Simulink 中直接打开、运行、检查和修改的完整飞行器控制仿真系统。系统采用分层控制结构：

- **外环：** 飞行器位置/速度控制与制导，生成期望姿态及未来姿态参考序列；
- **内环：** 现有气动舵面–CMG 统一姿态 MPC，负责三轴姿态跟踪和 CMG 构型迁移；
- **优化求解：** 保留 HPIPM，不因迁移到 MATLAB 而改用 `quadprog`；
- **被控对象：** 使用 Simulink/Aerospace Blockset 提供的六自由度飞行器模块，以及 Simulink 中现有的气动、执行机构、信号处理和查表模块；
- **CMG：** 使用 Simulink 基础模块表达几何、角动量和力矩映射；在需要机械结构真实性时再使用 Simscape Multibody 建立独立机械模型。

本次迁移的重点不是把 Python 代码翻译成 MATLAB 代码，而是将系统模型真正落到 Simulink 已有的物理模块和信号模块上。

### 1.2 强制原则

1. 能用 Simulink/Aerospace Blockset 现成模块完成的部分，必须优先使用现成模块。
2. 六自由度动力学必须优先使用 Aerospace Blockset 的 6DOF 模块，不允许自行用 MATLAB Function 重写一套六自由度模块来替代。
3. 气动查表必须使用 Lookup Table 模块，不允许将查表和气动计算全部塞入一个 `func`。
4. 舵机动态必须优先使用 Transfer Fcn、State-Space、Rate Limiter、Saturation 等模块。
5. 姿态坐标变换、四元数处理优先使用 Aerospace Blockset/Simulink 提供的姿态和坐标变换模块。
6. CMG 的几何和矩阵运算允许由 Matrix Multiply、Trigonometric Function、Gain、Sum、Product 等基础模块组成；不得用一个函数隐藏整个 CMG 模型。
7. MPC 的矩阵组装与 HPIPM 接口属于算法计算，可由 MATLAB 代码实现，但必须独立成模块并提供输入输出、数学说明和单元测试。
8. 不允许把整个飞行器模型、气动模型、CMG 模型或舵机模型封装成一个 MATLAB Function Block、S-Function 或隐藏 MEX。
9. 允许少量 MATLAB Function Block 处理确实不适合用基础模块表达的局部运算，但必须逐个登记、说明必要性，且不能承载完整物理子系统。
10. 交付必须包含 `.slx`、模型模块清单、模型截图、参数文件、测试数据和验证报告。只有 `.m` 脚本不算完成。

### 1.3 本次研究边界

本项目研究重点仍然是姿态控制，不是位置与姿态联合 MPC。

- 外环生成姿态参考；
- 内环只优化姿态、角速度和 CMG 构型相关变量；
- 位置和速度由六自由度真值模型积分得到；
- 位置/速度反馈给外环，并用于更新高度、速度、动压和气动参数；
- 不把位置和速度误差强行加入现有姿态 MPC 的目标函数。

---

## 2. 迁移前代码与参数审计

开始搭建模型前，先对现有仓库进行审计，形成迁移基线，禁止直接边读代码边重写。

### 2.1 必须审计的代码

以仓库当前实际版本为准，至少检查：

- `nearspace/model6.py`、`nearspace/model6c.py`：现有刚体/CMG 真值动力学；
- `nearspace/cmg.py`：CMG 几何、角动量、雅可比及力矩符号；
- `nearspace/cmg_control.py`：现有 MPC 状态、控制量、预测模型、代价、约束、HPIPM 调用；
- `nearspace/trajectory3.py`：姿态参考和机动时间定义；
- `nearspace/torque_gap_*`：气动能力、舵面力矩、环境和执行器定义；
- 当前配置文件：质量、惯量、气动参数、CMG 参数、采样周期、预测时域、求解器参数；
- 现有 P0/P1 定义：确认理想力矩接口与真实舵面执行器的区别。

若文件路径或名称已经变化，应以当前仓库实际文件为准，并在审计报告中记录，不允许根据旧文档猜测。

### 2.2 形成统一参数字典

建立 `simulink/data/vehicle_params.m` 或 MAT 文件，集中存放：

- 飞行器质量、惯量矩阵、质心位置；
- 参考面积、参考长度、气动中心；
- 舵面效能、舵机时间常数、偏角和速率限制；
- 四台 CMG 的安装方向、金字塔角、转子角动量；
- 框架速率、加速度、驱动动态参数；
- 大气参数和飞行初始条件；
- 外环和内环采样周期；
- MPC 预测时域和 HPIPM 参数。

参数文件只保存数据，不应把整个模型计算逻辑写成脚本函数。

### 2.3 单位与坐标系审计

建立 `docs/simulink/coordinate_and_unit_conventions.md`，明确：

- 惯性坐标系采用 NED、ENU 还是其他约定；
- 机体系轴向和右手规则；
- 四元数排列、主动/被动旋转和乘法顺序；
- 角度使用 rad，角速度使用 rad/s；
- 位置使用 m，速度使用 m/s；
- 力使用 N，力矩使用 N·m；
- 角动量使用 N·m·s；
- 高度正方向、重力方向；
- 气动力与气动力矩在机体系/惯性系之间的变换约定。

所有模块必须遵守同一套约定。严禁靠在某个隐藏函数里临时改变符号来“对齐结果”。

---

## 3. 顶层 Simulink 模型结构

主模型命名为：

`NearSpace_CMG_FlightControl.slx`

建议顶层由以下 Subsystem 组成：

```text
NearSpace_CMG_FlightControl
│
├── Guidance_OuterLoop
│   ├── Position_Velocity_Controller
│   ├── FlightPath_Guidance
│   └── Attitude_Command_Generator
│
├── Reference_Manager
│   ├── Attitude_Reference
│   └── Future_Reference_Buffer
│
├── Environment
│   ├── Standard_Atmosphere
│   ├── Gravity_Model
│   └── Flight_Condition
│
├── Aerodynamic_Model
│   ├── Alpha_Beta
│   ├── Dynamic_Pressure
│   ├── Aero_Coefficient_Lookup
│   ├── Aerodynamic_Forces
│   └── Aerodynamic_Moments
│
├── Surface_Actuator
│   ├── Surface_Command
│   ├── Servo_Dynamics
│   ├── Position_Rate_Limits
│   └── Surface_Aero_Effect
│
├── CMG_Cluster
│   ├── Pyramid_Geometry
│   ├── Gimbal_Drive_Dynamics
│   ├── Rotor_Momentum
│   ├── Cluster_Momentum
│   └── CMG_Torque
│
├── Attitude_MPC
│   ├── State_And_Reference_Input
│   ├── LTV_Model_Update
│   ├── OCP_QP_Assembly
│   ├── HPIPM_Interface
│   └── Command_Output
│
├── Aircraft_6DOF
│   └── Aerospace_Blockset_6DOF
│
├── Sensor_Feedback
│
└── Logging_And_Metrics
```

顶层模型只负责系统连接和信号流，不要把复杂公式集中写在顶层。

---

## 4. 六自由度飞行器：必须使用 Aerospace Blockset 现成模块

### 4.1 指定模块

优先使用 Aerospace Blockset 中的：

**6DOF (Quaternion)** 模块。

若当前 MATLAB 版本中该模块名称或库路径有所变化，dsh 必须先在 Library Browser 中确认实际模块，记录完整库路径、模块名称、版本和端口定义，再实施。不得自行创建一个名为 `6DOF` 的 Subsystem 来冒充工具箱模块。

如果项目采用地球固定局部坐标系，必须明确选择适合的 6DOF 模块变体和坐标约定。若使用其他 6DOF 模块变体，需说明其坐标系、姿态参数化方式和输入输出含义。

### 4.2 模块承担的职责

该模块负责六自由度刚体运动方程的积分，包括：

- 三维位置；
- 三维速度；
- 姿态四元数；
- 三轴角速度。

模型应从 6DOF 模块端口读取位置、速度、姿态和角速度，并将总力、总力矩输入对应端口。

### 4.3 输入力和力矩

建立独立的力/力矩合成信号：

\[
\boldsymbol F_B
=
\boldsymbol F_{\mathrm{aero},B}
+
\boldsymbol F_{\mathrm{other},B}
\]

\[
\boldsymbol M_B
=
\boldsymbol M_{\mathrm{aero},B}
+
\boldsymbol M_{\mathrm{CMG},B}
+
\boldsymbol M_{\mathrm{dist},B}
\]

其中舵面产生的增量气动力和气动力矩已经包含在气动模型输出中，禁止再重复叠加一次舵面力矩。

**重要：** Aerospace Blockset 的 6DOF 刚体模块通常将飞行器作为整体刚体处理。若要把内部 CMG 转子角动量耦合严格纳入，需要确认该 6DOF 模块的力矩端口是否能够正确表达 CMG 内部角动量变化。不能仅把 `-A(gamma)*nu` 加到外力矩端口后，就宣称所有内部角动量耦合均已由模块自动处理。

因此，必须设置两种清晰的模型模式：

- **模式 1：等效外力矩模式。** 使用 6DOF 模块，将 CMG 依据现有模型推导得到的等效反作用力矩输入力矩端口。此模式适合与当前模型对照，但必须说明其建模假设。
- **模式 2：内部角动量耦合模式。** 如果需要完整表达 `Jω + H_CMG` 的总角动量关系，需验证 Aerospace Blockset 6DOF 模块能否通过正确的外力矩/附加项接口表达；若不能，才允许使用经过独立推导和验证的自定义刚体动力学子系统作为补充，并明确这部分是工具箱模块能力边界，而不是偷偷用函数替代 6DOF 模块。

优先尝试并验证模式 1 与现有模型的一致性；不得未经推导就将模式 1 称为完整的 CMG 内部耦合模型。

### 4.4 六自由度模块验收

- 查明并记录 6DOF (Quaternion) 的真实库路径；
- 提供模块参数对话框截图；
- 提供输入输出端口截图；
- 用无力、无力矩条件验证状态保持；
- 用恒定力验证平动加速度；
- 用恒定力矩验证角加速度；
- 验证四元数范数；
- 与独立解析算例或已有模型结果对比。

---

## 5. 外环：位置/速度控制与姿态参考生成

外环不是本文创新点，不需要再建立一个 MPC。先使用传统控制方式，目标是产生可执行的期望姿态。

### 5.1 外环输入

- 位置参考 `r_ref`；
- 速度参考 `v_ref`；
- 当前惯性系位置 `r`；
- 当前惯性系速度 `v`；
- 当前姿态；
- 飞行环境参数。

### 5.2 建议实现

位置/速度外环采用经典 PID/PD 或比例导航/航迹控制结构，具体形式依据飞行任务确定。不要在缺乏飞行器布局和任务定义时擅自假定固定翼、升力体或火箭的姿态指令关系。

模块优先使用：

- Sum：位置/速度误差；
- Gain：比例、微分或前馈增益；
- Discrete-Time Integrator：积分环节；
- Saturation：期望加速度或姿态指令限幅；
- Rate Limiter：姿态参考变化率限制；
- Coordinate Transformation：坐标系转换；
- Trigonometric Function：必要的几何关系。

将期望加速度/航迹方向转换为姿态指令的关系必须由飞行器气动和控制布局决定。若当前尚无完整制导律数据，第一阶段可使用已有姿态参考直接输入；外环自主生成参考作为第二阶段任务。

### 5.3 外环输出

- 期望姿态 `q_ref` 或等价姿态角；
- 期望角速度 `omega_ref`（若制导律提供）；
- 未来姿态参考序列 `q_ref[k:k+N]`。

未来参考序列应通过统一的 Reference Manager 提供给 MPC，不能让 MPC 每次只收到一个当前参考点，否则会失去现有提前构型迁移所依赖的未来参考信息。

---

## 6. 环境与大气模型：优先使用现成模块和查表模块

### 6.1 大气模型

如果已安装 Aerospace Blockset，先检查是否提供适用的 Atmosphere Model 模块及其具体模型选项。若有，优先使用现成模块，并记录采用的标准大气模型和适用高度范围。

若现有高空场景超出工具箱模块的适用范围：

- 不允许直接外推标准大气模块并声称可信；
- 应根据选定的大气模型离线生成高度—温度—密度—压力数据；
- 使用 1-D Lookup Table 或 n-D Lookup Table 进行插值；
- 数据文件和来源必须归档；
- 对表格范围外的输入设置明确保护和告警。

### 6.2 重力模型

根据仿真范围选择 Aerospace Blockset 现成重力模型；若仅使用局部平地近似，可使用 Constant 或简单高度相关模块。必须说明采用恒定重力还是高度相关重力。

### 6.3 飞行状态量

使用现成模块计算或输出：

- 高度；
- 速度模长；
- 动压；
- Mach 数；
- 迎角；
- 侧滑角；
- Knudsen 数（用于判断连续流/稀薄流模型适用性）。

---

## 7. 气动模型：查表模块为主，不允许一个函数包办

### 7.1 迎角与侧滑角

优先使用 Aerospace Blockset 的空气数据/气流角计算模块（若当前版本提供且适用）；否则使用 Coordinate Transformation、Trigonometric Function、Product、Divide 等模块形成清晰的信号图。

### 7.2 动压

使用 Product、Gain 和 Divide 等基础模块：

\[
\bar q=\frac12\rho V^2
\]

### 7.3 气动系数

气动系数由数据表驱动。使用：

- 1-D Lookup Table；
- 2-D Lookup Table；
- n-D Lookup Table。

输入维度根据真实数据决定，例如 Mach、迎角、侧滑角、舵偏角等。

不得把一整套气动系数计算和气动力矩计算写入一个 MATLAB Function Block。

### 7.4 力与力矩

使用 Gain、Product、Sum、Matrix Multiply 等模块实现气动力和气动力矩计算。自然气动力矩与舵面增量力矩分开输出并记录。

若当前只有线性气动导数，应明确标记为“参数化气动模型”，不得将其包装成高保真 CFD 气动模型。

---

## 8. 舵面执行机构

### 8.1 输入输出

输入：内环 MPC 给出的舵面期望控制量，物理含义必须与当前代码一致。

输出：

- 实际舵偏角；
- 舵偏角速度；
- 实际舵面增量气动力；
- 实际舵面增量气动力矩。

### 8.2 模块实现

优先使用：

- Transfer Fcn / Discrete Transfer Fcn：舵机一阶或二阶动态；
- Saturation：舵偏角上下限；
- Rate Limiter：舵偏角速度限制；
- Gain、Product、Lookup Table：舵偏角到气动力/力矩映射；
- Scope/To Workspace：记录实际执行量。

如果控制器输出的是三轴舵面期望力矩而不是单舵面偏角，必须显式建立控制分配子系统。分配算法可使用 MATLAB 代码，但不得将舵机动态和气动效能模型一起藏入函数。

---

## 9. CMG 集群模型

### 9.1 必须保留的物理量

四台金字塔 SGCMG：

- 四个框架角 `gamma`；
- 四个实际框架角速度 `nu`；
- 四个框架角加速度；
- 单台转子角动量；
- 集群总角动量；
- 集群几何映射矩阵 `A(gamma)`；
- CMG 输出力矩。

### 9.2 模块实现

| 计算 | 优先使用的模块 |
|---|---|
| `sin(gamma)`、`cos(gamma)` | Trigonometric Function |
| 几何向量各分量 | Gain、Product、Sum |
| 向量/矩阵乘法 | Matrix Multiply |
| 框架角速度动态 | Transfer Fcn、Rate Limiter、Saturation |
| 框架角积分 | Integrator / Discrete-Time Integrator |
| 角动量求和 | Sum |
| 输出力矩 | Matrix Multiply |
| 结果监测 | Scope、To Workspace |

核心关系：

\[
H_{\mathrm{CMG}}(\gamma)=h_0\sum_{i=1}^{4}s_i(\gamma_i)
\]

\[
A(\gamma)=\frac{\partial H_{\mathrm{CMG}}}{\partial\gamma}
\]

\[
\dot H_{\mathrm{CMG}}=A(\gamma)\nu
\]

\[
\tau_{\mathrm{CMG}}=-A(\gamma)\nu
\]

矩阵 `A(gamma)` 的具体符号、转子旋向和几何参数必须从现有代码审计后迁移，不能根据公式自行猜测。

### 9.3 执行机构动态

MPC 输出 `nu_cmd`，CMG 子系统必须经过框架驱动动态后得到实际 `nu`：

- 框架速率上限；
- 框架加速度上限；
- 驱动时间常数；
- 摩擦/阻尼（分阶段增加）；
- 框架角积分；
- 无限行程或机械行程配置。

当前采用无限行程时，应明确这是滑环 SGCMG 假设。不得在某个隐藏模块中重新加入 ±90° 限位。

### 9.4 Simscape Multibody 的使用边界

第一阶段不强制用 Simscape Multibody 重新建立整个四台 CMG 机械装配体。先用 Simulink 可检查的基础模块完成理论动力学迁移。

第二阶段若要验证机械惯量、轴承摩擦、电机扭矩和机械装配，再使用 Simscape Multibody 的 Revolute Joint、Rigid Transform、Inertia、Joint Actuator、Joint Sensor 等现成组件建立真实机械结构。

Simscape 机械模型与理论 CMG 模型应进行力矩和角动量交叉验证，不允许把三维动画作为动力学验证的替代品。

---

## 10. 姿态 MPC 与 HPIPM 迁移

### 10.1 保持研究问题不变

内环状态：

\[
x=
\begin{bmatrix}
q & \omega & \gamma
\end{bmatrix}^{T}
\]

控制量：

\[
u=
\begin{bmatrix}
\tau_a & \nu
\end{bmatrix}^{T}
\]

具体状态参数化和控制变量排列必须以当前 Python 实现为准。若代码使用姿态误差状态而不是直接使用四元数，应按现有实现迁移，不得为了书写方便擅自改变。

保留现有：

- 姿态跟踪目标；
- 控制增量/平滑代价；
- 动力学预测；
- 舵面能力约束；
- CMG 框架速率/加速度约束；
- 角动量相关约束；
- 预测时域和离散步长；
- 热启动和失败回退机制。

不重新加入已经删除的 `J_cfg`、三轴余缺调制或额外构型奖励项。

### 10.2 MATLAB 侧模块划分

`Attitude_MPC` 内部拆为：

1. `State_Reference_Assembler`：整理当前状态和未来姿态参考；
2. `LTV_Model_Updater`：更新时变预测模型；
3. `OCP_QP_Assembler`：组装 OCP-QP 的矩阵和约束；
4. `HPIPM_Solver_Interface`：调用 HPIPM；
5. `Warm_Start_Manager`：保存和更新热启动变量；
6. `Solver_Status_Manager`：检查求解状态、残差、超时和回退；
7. `Control_Output_Manager`：提取首个控制量并输出给执行机构。

这些模块可以由 MATLAB 代码完成算法运算，但不得承担飞行器、气动、舵机或 CMG 的真值动力学。

### 10.3 HPIPM 使用方式

继续使用 HPIPM。优先检查 HPIPM 官方仓库中 MATLAB MEX/Simulink 接口的当前可用版本和构建方式。

迁移前必须确认：

- HPIPM 是否能在当前 MATLAB 版本和操作系统编译；
- BLASFEO 依赖是否正确；
- MEX 接口支持的 OCP-QP 数据结构；
- Simulink 中调用 MEX 的方式；
- 是否需要 C-MEX S-Function 或 MATLAB System Block；
- 代码生成和实时运行是否受限。

不得假定“HPIPM 有 MATLAB 接口”就等于“HPIPM 可以直接放入 Simulink 并自动支持代码生成”。必须实际编译并做最小求解测试。

### 10.4 求解器一致性测试

选取现有 Python 版本的一组固定 QP 输入，分别在 Python 和 MATLAB HPIPM 接口中求解，比较：

- Hessian；
- gradient；
- 动态矩阵；
- 等式约束；
- 不等式约束；
- 最优控制序列；
- 目标函数值；
- primal/dual residual；
- 求解状态；
- 求解时间。

矩阵必须逐项一致或在明确的数值容差内一致。不能只比较最终姿态曲线。

---

## 11. 外环与内环的采样周期

采用多速率结构：

- 六自由度物理模型：使用适当的小步长连续积分或固定步长求解器；
- 内环姿态 MPC：先保持现有 `dt_ctrl`；
- 外环位置/速度控制：通常比姿态内环慢，具体周期依据飞行器时间尺度和稳定性分析确定；
- 参考管理器：在外环周期更新，并向内环提供未来参考序列；
- 执行机构：按其自身动态和采样需求更新。

所有周期必须集中配置，不能在不同模块中写死不同采样时间。

第一阶段采用固定步长离散仿真，便于复现和调试。确认无代数环、采样同步和数值问题后，再考虑变步长求解。

---

## 12. A/B 实验定义

所有对比都在同一个 Simulink 六自由度真值模型上运行。

### A 组：仅气动舵面

- 不安装/不启用 CMG；
- `M_CMG=0`；
- CMG 内部角动量不进入飞行器动力学；
- 舵面模型和外环完全正常。

### B 组：气动舵面 + CMG

- 启用四台 SGCMG；
- CMG 框架动态、角动量和力矩映射全部生效；
- 内环 MPC 联合优化舵面控制量与 CMG 框架角速度。

A/B 两组保持相同：

- 初始位置和速度；
- 初始姿态和角速度；
- 飞行环境和大气参数；
- 外环参数；
- 姿态参考；
- 外扰；
- 舵面模型；
- 仿真时长；
- 控制周期和预测时域。

不能将“CMG 框架锁定但保留转子角动量”当作无 CMG 基线。A 组必须真正移除 CMG 对飞行器的内部角动量贡献。

---

## 13. 分阶段实施任务

### 阶段 0：工具箱、版本与模块核查

先在 MATLAB 中检查：

- MATLAB 版本；
- Simulink；
- Aerospace Blockset；
- Simscape；
- Simscape Multibody；
- Optimization Toolbox（仅用于对照或辅助，不替换 HPIPM）；
- Instrument Control Toolbox（若通信需要）。

必须在 Library Browser 中实际找到并确认：

- 6DOF (Quaternion)；
- Atmosphere Model（若当前版本提供）；
- Coordinate Transformation / Direction Cosine Matrix / Quaternion 模块；
- Lookup Table；
- Transfer Fcn；
- Rate Limiter；
- Saturation；
- Matrix Multiply；
- Simscape Multibody 关节和惯量模块。

交付：`docs/simulink/00_toolbox_and_block_audit.md`。

审计表必须包括模块真实名称、库路径、版本、许可证、输入输出端口、是否计划使用。找不到的模块不能写成“已使用”。

### 阶段 1：六自由度模块最小验证

只建立一个最小 `.slx`：

- Aerospace Blockset 6DOF (Quaternion)；
- Constant/Step 输入力；
- Constant/Step 输入力矩；
- Scope；
- To Workspace。

完成恒力、恒力矩、自由响应和四元数范数测试。

交付：`tests/01_6dof_block_validation.slx` 和验证报告。

### 阶段 2：环境、气动与舵面

使用现成大气模块（若适用）、Lookup Table、Product、Gain、Sum、Transfer Fcn、Rate Limiter、Saturation 建立气动和舵面子系统。

先与现有 Python 模型在相同状态下逐点对照。

交付：`models/Aerodynamic_Surface.slx` 和验证报告。

### 阶段 3：CMG 原生模块模型

使用基础 Simulink 模块建立四台金字塔 CMG 的几何、角动量、雅可比、框架驱动和输出力矩。

先不接 MPC，给定框架角速度输入，验证：

- `H(gamma)`；
- `A(gamma)`；
- `Hdot=A*nu`；
- `tau=-A*nu`；
- 无限行程下跨越 360° 的连续性；
- 框架速率和加速度限制。

交付：`models/CMG_Cluster.slx` 和验证报告。

### 阶段 4：统一飞行器被控对象

将 6DOF、环境、气动、舵面和 CMG 接起来。明确内部角动量耦合的建模模式，并与现有 `model6c` 对照。

交付：`models/Plant_6DOF_CMG.slx`。

### 阶段 5：外环姿态参考生成

先实现直接姿态参考输入模式，再实现位置/速度外环生成姿态参考模式。

外环方法采用与飞行器类型和任务匹配的经典控制结构，不在本阶段引入另一个 MPC。

交付：`models/Guidance_OuterLoop.slx`。

### 阶段 6：MATLAB MPC + HPIPM

迁移 OCP-QP 组装和 HPIPM 接口。先做离线固定 QP 一致性测试，再接入 Simulink 闭环。

交付：`models/Attitude_MPC.slx`、HPIPM MEX 文件、QP 对照报告。

### 阶段 7：完整闭环与实验

运行：

- 现有八组 P0 姿态机动；
- P1 舵面执行器场景；
- 未来姿态参考 A/B/C 预测机制实验；
- 多初始条件和参数不确定性实验。

所有实验均进行仅舵面 A 与舵面+CMG B 对比。

交付完整 `.slx`、配置、CSV/MAT 数据、图表和报告。

---

## 14. 防止偷懒的强制审查规则

每次提交都必须附带以下内容：

### 14.1 Simulink 模型证据

- 顶层模型截图；
- 六自由度模块参数和端口截图；
- 气动子系统内部模块截图；
- 舵面执行机构内部模块截图；
- CMG 子系统内部模块截图；
- MPC/HPIPM 接口截图。

### 14.2 模块清单

使用 Simulink 模型查询功能导出全部 Block 的：

- Block Name；
- Block Type；
- Reference Block；
- Parent Subsystem；
- Mask 信息；
- 是否为 MATLAB Function / S-Function / Fcn / Interpreted MATLAB Fcn。

对所有自定义代码块逐个解释。若核心动力学被一个函数块包住，直接判定该阶段不合格。

### 14.3 模型结构检查

必须能在 GUI 中逐层打开和修改模型。不能只有一个顶层函数块。

### 14.4 数值验证

每个子系统都要有独立输入、输出和参考解。不能只展示最终姿态跟踪曲线。

### 14.5 版本管理

新建分支：

`experiment/simulink-native-flight-control`

不得覆盖既有 Python 仿真、历史结果和论文图表。每个阶段单独提交。

---

## 15. 最终目录结构

```text
simulink/
├── NearSpace_CMG_FlightControl.slx
├── models/
│   ├── Aircraft_6DOF.slx
│   ├── Environment.slx
│   ├── Aerodynamic_Model.slx
│   ├── Surface_Actuator.slx
│   ├── CMG_Cluster.slx
│   ├── Guidance_OuterLoop.slx
│   └── Attitude_MPC.slx
├── data/
│   ├── vehicle_params.mat
│   ├── atmosphere_table.mat
│   ├── aerodynamic_table.mat
│   └── reference_cases/
├── hpipm/
│   ├── mex/
│   ├── build/
│   └── README.md
├── tests/
│   ├── 01_6dof_block_validation.slx
│   ├── 02_aerodynamic_validation.slx
│   ├── 03_surface_actuator_validation.slx
│   ├── 04_cmg_mapping_validation.slx
│   └── 05_python_matlab_qp_equivalence/
└── results/
    ├── A_surface_only/
    └── B_surface_cmg/
```

文档：

```text
docs/simulink/
├── 00_toolbox_and_block_audit.md
├── 01_code_and_parameter_audit.md
├── 02_coordinate_and_unit_conventions.md
├── 03_simulink_architecture.md
├── 04_6dof_validation.md
├── 05_aerodynamic_surface_validation.md
├── 06_cmg_validation.md
├── 07_hpipm_matlab_migration.md
├── 08_closed_loop_validation.md
└── 09_ab_experiment_report.md
```

---

## 16. 第一阶段的具体执行指令

不要一次性实现全部系统。第一阶段只做工具箱核查和原生 6DOF 模块验证。

dsh 必须完成：

1. 输出 MATLAB 版本及所有相关工具箱许可证状态；
2. 在 Library Browser 中定位真实的 `6DOF (Quaternion)` 模块；
3. 记录模块完整库路径、参数、输入输出端口；
4. 创建 `tests/01_6dof_block_validation.slx`；
5. 模型中直接放置真实 Aerospace Blockset 6DOF 模块，不允许用 MATLAB Function 替代；
6. 使用 Step/Constant、Gain、Scope、To Workspace 完成恒力和恒力矩测试；
7. 给出理论解析结果与仿真结果对照；
8. 导出模块清单，证明模型中确实调用了 Aerospace Blockset 的原生 6DOF 模块；
9. 提交模型、测试数据、截图和报告。

**第一阶段验收通过后，才允许进入气动和舵面建模。**

---

## 17. 最终验收标准

只有同时满足以下条件，才认为迁移成功：

- 六自由度飞行器使用 Aerospace Blockset 现成模块，而不是自定义函数重写；
- 外环能够根据位置/速度或直接姿态参考生成期望姿态序列；
- 内环保留当前姿态 MPC 的优化问题；
- QP 求解继续使用 HPIPM；
- 气动系数由可追溯数据和 Lookup Table 提供；
- 舵面执行机构由可检查的动态、限幅和气动效能模块构成；
- CMG 几何和动力学可从 Simulink 模块连接中逐项追踪；
- A/B 对照中 A 组真正不含 CMG 内部角动量贡献；
- 各子系统有独立验证数据；
- 模型可以在 MATLAB/Simulink GUI 中打开、运行、检查和修改；
- 不存在一个或几个自定义函数包办全部物理模型的情况。

**核心要求：优先使用 Simulink/Aerospace Blockset 已有模块，尤其是六自由度模块。自定义 MATLAB 代码只用于现有工具箱没有直接提供的算法环节和局部计算，不得替代现成物理模块。**
