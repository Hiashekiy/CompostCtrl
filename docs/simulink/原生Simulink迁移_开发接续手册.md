# 飞行器气动舵面–CMG 原生 Simulink 开发接续手册

更新日期：2026-11-02。项目根目录：`D:\项目\飞轮`。开发分支：`experiment/simulink-native-flight-control`。
本手册按当前实际代码编写，面向后续接手开发，不是最初实施方案的完成声明。

## 目录

1. [当前版本到底完成了什么](#1-当前版本到底完成了什么)
2. [环境与第一次运行](#2-环境与第一次运行)
3. [系统结构与模块接口](#3-系统结构与模块接口)
4. [坐标、单位与动力学](#4-坐标单位与动力学)
5. [代码地图与调用顺序](#5-代码地图与调用顺序)
6. [参数和数据如何进入模型](#6-参数和数据如何进入模型)
7. [MPC 与 Simulink 如何联动](#7-mpc-与-simulink-如何联动)
8. [日常运行、读日志和复现实验](#8-日常运行读日志和复现实验)
9. [怎样修改现有模型](#9-怎样修改现有模型)
10. [怎样建立新工况](#10-怎样建立新工况)
11. [怎样修改算法、时域和采样周期](#11-怎样修改算法时域和采样周期)
12. [从源代码重建的顺序](#12-从源代码重建的顺序)
13. [验证结果与证据位置](#13-验证结果与证据位置)
14. [下一步开发任务](#14-下一步开发任务)
15. [故障排查](#15-故障排查)
16. [提交与交接约定](#16-提交与交接约定)

## 1. 当前版本到底完成了什么

### 1.1 已实现

- 真值刚体使用 Aerospace Blockset 的真实 6DOF 四元数模块，不由自定义函数替代。
- 原生 CMG 几何、框架驱动、角积分、气动力矩查表、舵机动态、限幅和控制分配。
- MATLAB 在线 RTI：每个控制周期重新预测、线性化、组装 OCP-QP 并调用 HPIPM；不是回放 Python 控制序列。
- 移位热启动、求解失败后的有界回退；A/B 两套真实状态维数。
- 公开升阻力替代模型、既有 MSIS2 配置的离线密度表、恒重力和实际平动反馈。
- 八组 P0 机动、16 项 A/B 仿真已完成窗口运行；默认 A/B 飞行主模型保存后重载运行检查通过。

### 1.2 不能视为已经完成的内容

**P0 闭环已与历史 Python 同量级，但精度一致性仍未验收通过。** 八组 A/B 的最大/RMS 与 Python 对照见 `09_ab_experiment_report.md`：
修复 1.4 的缺陷后 B 组从 3–40 度回落到与 Python 同一量级，A 组逐条吻合。剩余差异来自真值积分格式与扰动采样（见 `08_closed_loop_validation.md` 末节），
逐点轨迹等价、控制序列回放、步长收敛与敏感性分析均未完成。单步/固定 QP 对照通过、运行不报错、零回退，不能替代全闭环一致性证明。

还没有完成：ABC 预测机制完整实验、多初态/参数不确定性完整实验、自主位置速度制导、依据实际飞行反馈重建全部未来预测环境、GUI 参数对话框截图、实时部署和代码生成验收。

升阻力属于公开基准与现有参数组合的工程替代，不是当前 1200 kg 飞行器的实测/CFD 数据。侧向力系数假设为零，推力为零，代表无动力滑翔。不能将当前结果直接作为真实飞行器结论或替换旧论文图表。

这里的零侧向系数是风轴系假设；机体系Y方向仍可能出现由阻力投影产生的分量，不能据此将机体系Fy强行置零。

### 1.3 两套系统不要混用

| 系统 | 入口 | 控制器在哪里运行 |
|---|---|---|
| 本次原生迁移 | `simulink/native_startup.m`、`NearSpace_CMG_FlightControl*.slx` | MATLAB RTI + HPIPM MEX |
| 旧 S08 移植 | `simulink/README.md`、`s08_startup.m`、`open_case.m`、`run_case.m` | 历史方案含 Python/TCP 与封装植物 S-Function |

开发原生主链不需要启动旧 `ctrl_server.py`。旧结果保留供回归比较。

### 1.4 P0 B 组曾整段失效的缺陷（已修复，勿回退）

P0（`flight=false`）用 `configure_ideal_p0_drive` 把实测过的框架驱动换成理想速率执行。
`Simulink.SubSystem.deleteContents` 会同时删掉该子系统端口在父系统上的连线，重建后驱动块与
CMG 内部完全断开：`nu` 不到框架、`gamma` 恒为初值、`tau_CMG≡0`、`H≡H(0)=0`。
模型仍能编译、仿真、零回退，缺陷表现为“B 组比 A 组还差”。修复方式是删前记录端口连线、
删后清理悬空线段并重连，再加构建期端口断言；`simulink/check_cmg_actuation.m` 是专用回归门。
改动任何“替换子系统内容”的建模代码时，必须保留“删除会带走父级连线”这条经验。
完整证据与修复前后指标见 `08_closed_loop_validation.md`。

## 2. 环境与第一次运行

### 2.1 本机依赖

| 项目 | 当前路径/说明 |
|---|---|
| MATLAB | `D:\Program Files\MATLAB\R2024a\bin\matlab.exe`，版本 24.1.0.2537033 |
| 工具箱 | MATLAB、Simulink、Aerospace Toolbox、Aerospace Blockset |
| HPIPM | `D:\Tools\hpipm` |
| BLASFEO | `D:\Tools\blasfeo` |
| C 编译器 | 本机 `D:\w64devkit`；仅重编译时需要。当前版本有 MathWorks 非官方支持版本警告 |
| Python | 当前参考导出使用 Python 3.14，依赖 NumPy、SciPy、PyYAML、CasADi、项目大气后端及其依赖 |

已有 Windows x64 MEX 位于 `simulink/hpipm/mex/`。换平台、MATLAB ABI 或 C 库版本后应重新编译，不能直接复制二进制就假定兼容。
只运行已保存原生主模型时不需要常驻 Python；导出新配置、数据和预测 C 代码时才使用 Python。

### 2.2 先跑默认 B 组短时仿真

在 MATLAB 命令窗口执行：

```matlab
cd('D:\项目\飞轮');
addpath('simulink');
native_startup;
m = 'NearSpace_CMG_FlightControl';
load_system(fullfile('simulink', [m '.slx']));
in = Simulink.SimulationInput(m);
in = in.setModelParameter('StopTime', '2'); % 仅覆盖本次运行，不修改保存的时长
out = sim(in);
open_system(m);
```

A 组把模型名改成 `NearSpace_CMG_FlightControl_SurfaceOnly`。默认保存时长为 60 s；确认短时正常后可直接点击 Run 运行完整窗口。

不要直接把 A 模型另存为不含 `SurfaceOnly` 的名字：当前 S-Function 会依据这个子串选择 A/B 端口维数。未来应将它重构为显式配置参数。

### 2.3 从 PowerShell 启动批处理

```powershell
Set-Location -LiteralPath 'D:\项目\飞轮'
$env:MATLAB_HOME='D:\Program Files\MATLAB\R2024a'
.\simulink\mlab.cmd "maxNumCompThreads(1); addpath('simulink'); native_startup; check_native_rti"
```

`mlab.cmd` 把 MATLAB 临时目录、偏好目录重定向到项目中的 `.matlab_*`，并透传 MATLAB 退出码。`maxNumCompThreads(1)` 用于本次小矩阵批处理，不代表实时性能认证。

## 3. 系统结构与模块接口

### 3.1 完整工程飞行主链

```mermaid
flowchart LR
  Bank[模型内嵌未来参考与名义环境] --> MPC[Online_RTI_HPIPM]
  Feedback[姿态 角速度 框架角反馈] --> MPC
  MPC --> Split[总气动力矩与框架速率命令]
  Split --> Allocate[Surface_control_allocation]
  Allocate --> Surface[Aerodynamic_Surface]
  Split --> CMG[Native_Plant 内 CMG]
  Surface --> Rigid[原生 Aircraft_6DOF]
  CMG --> Rigid
  Rigid --> Env[实际位置 速度 DCM]
  Env --> Forces[大气 空气数据 公开升阻力 重力]
  Forces --> Rigid
  Rigid --> Feedback
  CMG --> Feedback
```

主模型根路径为 `simulink/`；下列独立组件位于 `simulink/models/`。

| 模型/模块 | 输入 | 输出/职责 |
|---|---|---|
| `Plant_6DOF_CMG.slx` | 总气动力矩3、扰动力矩3、机体系总力3、框架速率命令4 | Ve、Xe、Euler、DCM、Vb、omega、omega_dot、gamma |
| `Plant_6DOF_SurfaceOnly.slx` | 上表前三个输入，无框架输入 | 前七个输出；结构上不含 CMG |
| `CMG_Drive.slx` | nu_cmd，4维 | 实际 nu、实际加速度、unwrapped gamma |
| `CMG_Geometry.slx` | 独立几何验证的 Constant 输入 | H、A、tau；不是直接接入闭环的端口化组件 |
| `CMG_Cluster.slx` | nu_cmd，4维 | 依次为 nu、加速度、gamma、H、A、tau |
| `Surface_Actuator.slx` | delta_cmd=[de,da,dr] | 实际偏角、偏角速度 |
| `Aerodynamic_Model.slx` | `[qbar,V,alpha,beta,p,q,r,de,da,dr]` | 自然力矩、舵面增量、总力矩 |
| `Aerodynamic_Surface.slx` | delta_cmd3；`[qbar,V,alpha,beta,p,q,r]` | 实际偏角、偏角速度、自然力矩、舵面增量、总力矩 |
| `Surrogate_Aero_Forces.slx` | `[qbar,alpha,beta]` | 机体系气动力3；带迎角保护 |
| `Environment_And_Surrogate_Forces.slx` | NED位置3、机体系速度3、地面到机体 DCM3×3 | 含气动力和重力的总力3；`[qbar,V,alpha,beta]` |
| `HPIPM_Interface.slx` | 序列化固定维数 QP | 首控制增量、状态、四项残差；仅接口验证模型 |

`HPIPM_Interface.slx` 不等于在线 MPC 主模型。真正闭环使用 `native_mpc_sfun.m`，由它调用 `native_rti_step.m`。

### 3.2 P0 与工程飞行模式

`build_native_closedloop(duration, flight, enabled, case_name)`：

| 参数 | 含义 |
|---|---|
| duration | 本次仿真时长；不能超过未来参考覆盖窗口后仍声称参考在更新 |
| flight=false | P0 总力矩命令接口、零平动力；B 组直接执行框架速率命令，不加驱动滞后 |
| flight=true | 接公开气动力、重力、真实舵机和 CMG 驱动的工程飞行链路 |
| enabled=true/false | B 有 CMG / A 无 CMG |
| case_name | 可选工况后缀；省略时重建默认模型，可能覆盖现有同名 SLX |

P0 构建器会改写复制进去的 `CMG/Drive` 为理想速率执行加角积分。其“加速度诊断零输出”不是实际物理加速度，不可用于 P0 驱动加速度论证。工程飞行模式保留独立验证的一阶驱动与真实限幅。
该改写会调用 `deleteContents`，删前删后必须保住端口连线——这正是 1.4 节缺陷的由来，改这段代码后务必跑 `check_cmg_actuation`。

## 4. 坐标、单位与动力学

- 地面局部系 NED：北、东、下。机体系：前、右、下，右手系。
- q=[w,x,y,z]，Hamilton 标量在前；项目 Python 定义 `v_N=R(q)*v_B`，`qdot=0.5*q⊗[0,omega_B]`。
- Euler 顺序 roll、pitch、yaw，ZYX 参数化。角用 rad，角速度 rad/s，力 N，力矩 N·m，角动量 N·m·s，惯量 kg·m²。
- CMG `gamma` 不折返、不自动加 ±90° 限位；滑环无限行程假设。
- 6DOF 输出 DCM 在当前力反馈链中按地面到机体使用，将 `[0;0;m*g]` 转到机体系。

```text
H = h0 Σ(s0_i cos(gamma_i) + gxs0_i sin(gamma_i))
A_i = h0(-s0_i sin(gamma_i) + gxs0_i cos(gamma_i))
tau_CMG = -A*nu
M_6DOF = tau_a_total + tau_d - A*nu - omega×H
```

最后的 `-omega×H` 不可遗漏；工具箱刚体本身负责 `-omega×(J*omega)`，不能重复加入。
A 组同时去除 CMG 反作用力矩和内部角动量贡献；不能只锁死框架速率后仍保留 H。
工具箱入口 `aerolib6dof/6DoF (Quaternion)` 在本机转发后的真实链接是 `aerolibprivatev1p5/6DoF (Quaternion)`。

`tau_a_total` 已含自然、阻尼和舵面增量。分配器先减去自然项求舵面增量，实际气动模块最后只相加一次。不能把总力矩直接当偏角，也不能重复叠加自然力矩。

## 5. 代码地图与调用顺序

### 5.1 Python 原始数学与参考层

| 文件 | 职责 |
|---|---|
| `nearspace/config.py` | YAML 深度继承、覆盖、解析 |
| `nearspace/cmg.py` | 几何、H、A、力矩符号的原始实现 |
| `nearspace/model6.py`、`model6c.py` | 坐标/四元数函数、旧姿态和 P1 模型 |
| `nearspace/torque_gap_model.py` | P0 A/B 真值数学，A为7状态3控制，B为11状态7控制 |
| `nearspace/torque_gap_rti.py` | 现有 RTI 数学、CasADi 图、QP 组装与求解 |
| `nearspace/torque_gap_scenario.py` | 网格、场景、未来参考、气动力矩包络 |
| `nearspace/torque_gap_authority.py` | 舵面效能矩阵、自然项、原有有界分配 |
| `nearspace/torque_gap_sim.py` | 原执行端裁剪、回退、P0/P1 步进及日志时间定义 |
| `nearspace/trajectory3.py` | 现有参考机动与扰动 |

旧方案提到的 `cmg_control.py` 当前不存在。继续开发以这些实际文件为准。

### 5.2 原生运行时调用链

```text
Simulink native_mpc_sfun.Start
  → 读取当前模型 ModelWorkspace 的 template 和 bank
每 0.2 s Outputs
  → 选择当前及未来 N+1 点参考/包络
  → Euler 转为 q，拼当前优化状态 x
  → native_rti_step
      → 移位上次 U 或名义初值
      → native_F_A/B 预测
      → assemble_native_qp
          → native_A_A/B、native_B_A/B、native_E_A/B
      → solve_hpipm_snapshot → 官方 HPIPM MATLAB 类 → MEX → HPIPM/BLASFEO DLL
      → 线搜索、首命令、裁剪或回退
  → 输出绝对力矩命令和（B组）框架速率命令
```

`native_F_B` 中 B 表示有 CMG 分支；`native_A_B` 是该分支的状态雅可比，`native_B_A` 是无 CMG 分支的输入雅可比，别被函数名混淆。

### 5.3 主要 MATLAB 文件

| 文件 | 修改什么时读它 |
|---|---|
| `build_native_closedloop.m` | 顶层接线、模式、初始化、内嵌数据、日志、构建输出 |
| `build_native_plant.m` | 真实 6DOF、耦合项及 A/B 物理结构 |
| `build_native_actuators_aero.m` | 驱动、舵机、气动力矩各独立组件 |
| `build_native_cmg_geometry.m` | CMG 基础块几何与随机样本验证 |
| `build_native_integrations.m` | 将几何+驱动、气动+舵机构成组合模型 |
| `build_public_force_model.m` | 公开升阻力表、力坐标变换、迎角断言 |
| `build_surrogate_flight_forces.m` | 大气、空气数据、重力合成 |
| `hpipm/native_mpc_sfun.m` | Simulink调度、端口、时间、模型内嵌数据与控制器状态 |
| `hpipm/native_rti_step.m` | RTI外层、热启动、线搜索、回退、执行端裁剪 |
| `hpipm/assemble_native_qp.m` | Q/R/q/r/A/B/b 和边界的在线组装 |
| `hpipm/solve_hpipm_snapshot.m` | 官方HPIPM对象、矩阵写入、读回验证、求解、释放 |
| `hpipm/native_surface_allocation_sfun.m` | 控制周期分配与上一偏角状态 |
| `hpipm/bounded_control_allocation.m` | 3/4变量有界最小二乘活动集枚举 |
| `hpipm/hpipm_startup.m` | 官方类、MEX与DLL路径；已去重并缓存 |

## 6. 参数和数据如何进入模型

### 6.1 当前有四层，不是改一个 YAML 就全部生效

1. YAML/历史 `config_resolved.yaml`：原始和最终运行配置。
2. `simulink/data/`、`simulink/tests/05_python_matlab_qp_equivalence/`：Python 导出的 MAT/JSON。
3. `simulink/models/*.slx`：构建时写入的块参数/查表数据。
4. 顶层 `.slx`：复制子模型内容，且在 ModelWorkspace 内嵌控制模板和参考。

**子模型复制不是动态库链接。** 修改 `CMG_Cluster.slx` 不会自动修改已经保存的顶层模型；必须重新构建上层。仅修改 YAML 或 MAT 也不会自动替换顶层里已固化的物理增益。

### 6.2 关键数据文件

| 文件 | 生成器/用途 |
|---|---|
| `data/native_baseline/<case>/vehicle_params.mat/json` | `export_native_baseline.py`：实际参数构造器输出、几何、J |
| 同目录 `validation.json` | 继承链各源文件哈希、随机参考验证 |
| `data/vehicle_params.m` | 只负责加载；默认固定为 S08 S00 B，不是自动读取当前模型工况 |
| `data/native_actuators/reference.mat` | 驱动/舵机8秒参考及组合轨迹 |
| `data/native_environment.mat` | 原 MSIS2 配置的0–120km密度表、初始高度与NED速度 |
| `data/public_surrogate/force_polar.mat` | -40°–40°极曲线，表断点rad；公式系数中的角是deg |
| `tests/.../production_qp.mat` | 早期固定 QP 测试夹具；不等于当前在线模板 |
| `tests/.../production_online_qp.mat` | B在线模板、固定对照、物理与权重数据 |
| `tests/.../production_online_A_qp.mat` | A在线模板 |
| `data/online_plan_bank.mat` | 当前导出的参考/名义环境序列，用于下一次构建 |

`export_hpipm_qp.py` 不会更新旧 `production_qp.mat`。单独改导出脚本后，不能误以为 `check_hpipm_snapshot` 已经测试新夹具。

### 6.3 顶层 ModelWorkspace

```matlab
load_system('simulink/NearSpace_CMG_FlightControl.slx');
mw = get_param('NearSpace_CMG_FlightControl','ModelWorkspace');
s = getVariable(mw,'native_controller_template');
bank = getVariable(mw,'native_reference_bank');
```

- `native_controller_template`：nx、nu、N、dts、权重、QP结构、信赖域、J/CMG常量等。
- `native_reference_bank`：`time`、`x0`，以及每个控制时刻的未来 `q_ref/omega_ref/tau_d/tau_min/tau_max/tau_nom/gap`。
- `disturbance_schedule`：供 From Workspace 使用的扰动时序。

bank.time 是从0开始的仿真相对时间，原场景可能从90 s开始；Python已将绝对时间查询转换为 bank 内容。不要再在 MATLAB 中重复加90 s。
名义预测环境并非由实际位置速度每周期重新生成。

## 7. MPC 与 Simulink 如何联动

### 7.1 在线端口

| 分支 | S-Function输入 | 内部优化状态 | 输出 |
|---|---|---|---|
| A | `[Euler(3);omega(3)]`，6维 | `[q(4);omega(3)]`，7维 | 总气动力矩3 |
| B | `[Euler(3);omega(3);gamma(4)]`，10维 | `[q(4);omega(3);gamma(4)]`，11维 | `[tau_a(3);nu_cmd(4)]`，7维 |

这里是优化状态维数，不是整个六自由度物理状态维数。位置、速度由真值6DOF积分，没有加入当前 MPC 目标。
S-Function 用持久 Map 按块句柄保存热启动与上一命令；Start 重置、Outputs 按控制时刻计算、Terminate 写统计并清理。
当前声明 `HasNoSimState`，未实现这些持久状态的保存恢复。不要依赖 Fast Restart、运行中保存恢复或模型快照继承热启动；先按普通独立仿真使用。

### 7.2 在线 QP 数据尺寸

MATLAB按阶段放在最后一维：

```text
X: nx×(N+1)             U: nu×N
A: nx×nx×N             B: nx×nu×N
b: nx×N                Q: nx×nx×(N+1)
q: nx×(N+1)            R: nu×nu×N
r,lu,uu: nu×N          q_ref: 4×(N+1)
omega_ref: 3×(N+1)     tau_d: 3×(N+1)（组装使用前N列）
tr_J: n_tr×nx          tr_bound: n_tr
```

QP变量是围绕预测轨迹的增量，不是绝对 x/u；首状态增量固定为零。
HPIPM阶段索引从0开始，MATLAB数组从1开始。QP内 `b=0` 对应当前 rollout 增量形式，不能在未推导时换成绝对状态模型的偏置。

### 7.3 热启动和回退

- 热启动将上次 U 去掉第一列并复制末列，再用当前状态重新 rollout。
- 当前 RTI 最多12轮，线搜索步长 `[1,.5,.25,.125]`，步长阈值 `1e-4`。
- 保留原 Python “初始 rollout 先于初始命令 clip”的顺序；曾因调整顺序产生不同首命令。
- 失败时优先上次动作并按当前界限重查；冷启动失败则使用有界PD/最小二乘稳定器，清空热启动U。
- 实际框架命令还要裁剪到上一命令±`gddot*dt_ctrl`。

没有完成统一的墙钟超时策略；当前不应宣称实时超时保护已验收。
MATLAB移植覆盖当前验证配置，并非自动支持 Python RTI 的所有可选项；例如新增 sigma-aware、自适应迭代、gamma damping 或成本缩放时，必须同时移植并重新做矩阵检查。

### 7.4 舵面控制分配

分配块输入为 `[tau_total(3);tau_natural(3);qbar]`，输出 `[de,da,dr]`。

```text
min ||M_delta(qbar)*delta - (tau_total - tau_natural)||²
subject to 偏角限幅，及上一分配命令±delta_dot_max*0.2
```

当前保存的是上一分配命令，不能擅自当成实际舵机偏角。其参数由默认 `vehicle_params()` 加载；更换飞行器时要与前向舵机/气动参数一起更新。
算法枚举最多4个变量的活动集，不依赖 Optimization Toolbox，也不是用 quadprog 替换 HPIPM。

## 8. 日常运行、读日志和复现实验

### 8.1 查看输出

```matlab
out.who
t = out.states.Time;
state = squeeze(out.states.Data);
if size(state,1) ~= numel(t), state = state'; end
plot(t, rad2deg(state(:,1:3))); grid on;
legend('roll','pitch','yaw'); xlabel('相对时间/s'); ylabel('姿态角/deg');
```

`states` 是反馈输入拼接，不含真值四元数；不能用 Euler 重构四元数的单位范数证明积分四元数范数合格。后者应读取原生模块积分状态，参考 `build_native_stage1.m`。
`commands` 为绝对命令；工程模式另有 `flight_conditions=[qbar,V,alpha,beta]`。
`native_subsystem_log_XXXX` 是为避免重名使用的编号，编号不稳定；查 Block 清单/To Workspace 的实际块路径定位，不要跨版本硬编码编号。

### 8.2 结果位置

| 路径 | 内容 |
|---|---|
| `results/native_stage1/` | 刚体解析算例、原生积分状态、误差与块清单 |
| `results/native_plant/` | 独立角动量耦合检查、CSV、Python对照 |
| `results/native_actuators_aero/`、`native_integrations/` | 驱动、舵机、气动、组合验证 |
| `results/native_closedloop/p0_matrix_summary.csv` | 16项P0最大误差、RMS、回退次数 |
| 同目录 `p0_matrix_progress_pre_drivefix.json` | P0 B 组修复前的进度快照（证明哪些项已重跑） |
| 同目录 `tracking_report.json` | 默认工程飞行模式A/B跟踪指标 |
| 同目录 `*_runtime_60s.json` | 对应时长的求解回退统计；无时长后缀文件只代表最近一次运行 |
| `results/native_delivery/` | 主模型重载检查、完整块清单、导图、验收缺口 |
| `tests/05_python_matlab_qp_equivalence/` | 固定QP、组装、在线首命令对照 |
| `simulink/check_cmg_actuation.m` | P0 B 组“框架角必须真的移动”的回归门（修复前必然失败） |

当前 `simulink/results/...`，不要与项目根目录的历史 `results/...` 混淆。
短时测试可能覆盖同名短时日志；无后缀 runtime JSON 会被后续重载测试改写，应使用带时长版本作证据。

### 8.3 批量复现

```powershell
python simulink/py/run_native_p0_matrix.py
python simulink/py/finish_native_delivery.py
```

第二条会检查/补跑回归、恢复默认导出、刷新默认模型、做重载检查并重写汇总报告，不是只格式化文档。
只想按现有 `p0_matrix_summary.csv` 重写 `09_ab_experiment_report.md` 与 `finish_report.json` 时，加 `--report-only`，不必重跑仿真。
回归优先使用历史 `results/torque_gap_no_cmg_comparison_rti2/P0/<case>/config_resolved.yaml`；正式实验包含基础 YAML 未记录的 `state_tr_gamma=0.5`、`state_tr_omega=1.0` 等设置。
进度在 `p0_matrix_progress.json`。**修改算法、MEX或参数后不要盲目相信断点缓存**：当前跳过逻辑不包含全部源代码哈希。开发新版本应备份旧进度并重新运行相关项目；重建器或接线方式改动后，至少单跑 `check_cmg_actuation` 确认执行机构仍然生效。

## 9. 怎样修改现有模型

### 9.1 临时 GUI 调试

先另存副本，保留 A 名字中的 `SurfaceOnly`，关闭原模型后调试；避免把两套不同物理参数的模型同时用于同名预测 MEX。
直接改顶层复制进去的 Gain/Lookup 适合定位问题，但要记录变更。**运行构建脚本会重新创建同名模型，GUI修改可能丢失。**

### 9.2 永久改动的落点

| 要改什么 | 修改位置 | 必须重新生成/验证 |
|---|---|---|
| 初始姿态/角速度/框架角 | 场景 YAML 的 init/轨迹，导出bank | 重建顶层；原生初值来自bank.x0 |
| 质量/惯量/CMG动量与几何 | 原参数源、baseline导出、预测导出 | 物理组件、预测C/MEX、QP与耦合检查、上层模型 |
| 舵面导数/限幅/时间常数 | 参数源与默认加载器、相关生成器 | 分配、前向气动、舵机都要同步，重新组合/建主模型 |
| 升阻曲线 | `export_public_force_tables.py` 或替换数据源 | 公开力组件、环境组合、顶层及适用域断言 |
| 大气/初始速度高度 | `export_native_environment.py` 当前配置路径 | 密度MAT、环境组件、顶层 |
| 参考机动/时域/权重 | 场景配置、QP/plan导出 | 在线模板、bank、顶层；必要时MEX与尺寸检查 |
| MPC成本/约束 | Python RTI及MATLAB assembler/RTI | 逐矩阵、单次解、在线首命令、闭环验证 |
| 执行/回退/分配逻辑 | MATLAB运行时文件 | 单元故障注入、执行器边界、闭环 |

目前尚未把所有配置统一成一个任意case参数入口：`vehicle_params.m`、执行器参考导出、环境导出等仍有固定默认场景。换平台之前应先整理这些入口，不要只改一个 MAT 就开始批量实验。

### 9.3 特别注意共享预测 MEX

顶层内嵌的是数据和物理块，**没有内嵌算法代码或预测 MEX**。改变质量/J/h0/几何后，CasADi生成的预测代码可能包含旧常量；只改s.J或SLX惯量不够。
同名 `native_F_B` 等 MEX 会被所有相应分支模型调用；同一 MATLAB 会话不能据“模型内嵌数据”就认为可以混跑不同物理参数版本。
更改物理参数时，推荐分目录保存源C、MEX、模型与配置哈希，串行验证，并关闭/清除旧MEX调用后再重编译。

## 10. 怎样建立新工况

先做一个明确的新 YAML，保留原配置，不修改历史结果目录中的 resolved 文件。新模型名用短的有效 MATLAB 标识符。

例如新建 `configs/native_dev/my_case_with_cmg.yaml`（UTF-8）：

```yaml
base: ../torque_gap_s08/cases/A4_descent_replay_S00_with_cmg.yaml
torque_gap:
  start: 90.0
  end: 150.0
  cmg_present: true
  cmg_enabled: true
trajectory3:
  maneuver:
    axis: pitch
    start: 110.0
    duration: 20.0
    delta_deg: 20.0
    shape: hold
```

对应 `configs/native_dev/my_case_no_cmg.yaml`：

```yaml
base: my_case_with_cmg.yaml
torque_gap:
  cmg_present: false
  cmg_enabled: false
```

此例只改机动，不改变物理平台。下面命令中的占位路径应替换成上述实际路径，例如 `configs/native_dev/my_case_with_cmg.yaml`。

```powershell
# 在项目根目录。将路径替换为实际存在的新B组工况。
python simulink/py/export_hpipm_qp.py --config configs/my_case_with_cmg.yaml
python simulink/py/export_online_plan_bank.py --config configs/my_case_with_cmg.yaml
```

MATLAB：

```matlab
native_startup;
% 如果预测数学/物理常量有变化，应先重编译：
build_prediction_mex;
build_native_closedloop(2, false, true, 'MyCase');
% 确认短时结果后，再用完整参考窗口构建/运行。
```

A组必须有 `cmg_present=false`、`cmg_enabled=false` 的配置：

```powershell
python simulink/py/export_hpipm_qp.py --surface-only --config configs/my_case_no_cmg.yaml
python simulink/py/export_online_plan_bank.py --config configs/my_case_no_cmg.yaml
```

再运行 `build_prediction_mex; build_native_closedloop(2,false,false,'MyCase')`。
`--surface-only` 控制导出文件和函数名，配置文件控制实际物理维数，二者必须一致，不能只改旗标。
P0新参考/算法实验优先保持原物理参数。若同时换物理参数，请执行第9节完整同步链，尤其是分配器默认参数与共享MEX。

## 11. 怎样修改算法、时域和采样周期

### 11.1 修改成本与约束

先对固定 X_nom/U_nom 组装：

```matlab
native_startup;
s = load('simulink/tests/05_python_matlab_qp_equivalence/production_online_qp.mat');
qp = assemble_native_qp(s,s.X_nom,s.U_nom);
result = solve_hpipm_snapshot(qp);
```

随后分别对照 Q、q、R、r、A、B、b、lu、uu，不要只看首命令或姿态曲线。原生约束中已包含当前验证配置的状态信赖域；不要重新加入已移除的 J_cfg 构型奖励。
`hpipm_pack_qp.m` 的字段顺序是 `A,B,b,Q,q,R,r,lu,uu`，按 MATLAB 列主序展开。仅固定QP接口模型依赖此打包格式；在线主链直接传struct。

### 11.2 采样周期不是改一个 dt 字段

当前已验证：物理/执行器基础步长0.001 s，MPC和分配0.2 s，预测网格为非均匀dts。
改控制周期至少核对：

1. YAML `dt_ctrl` 与未来bank.time；
2. `native_mpc_sfun.m` 的 SampleTimes；
3. `native_surface_allocation_sfun.m` 的 SampleTimes和偏角变化界中的0.2；
4. `native_rti_step` 的命令加速度界与稳定器；
5. 参考时间索引和所有测试日志的时刻定义。

改基础步长还要改顶层构建器、P0理想角积分、执行器参考生成、各组合模型及对应验证。原生6DOF解析测试的专用步长应单独记录。

## 12. 从源代码重建的顺序

普通阅读/运行不需要全量重建。以下用于依赖丢失或确实修改了底层定义时；会覆盖生成物，应先提交/备份需要保留的模型和结果。

### 12.1 导出参考与数据（PowerShell）

```powershell
python simulink/py/export_native_baseline.py
python simulink/py/export_actuator_reference.py
python simulink/py/export_allocation_reference.py
python simulink/py/export_public_force_tables.py
python simulink/py/export_native_environment.py
python simulink/py/export_hpipm_qp.py
python simulink/py/export_hpipm_qp.py --surface-only
python simulink/py/export_online_plan_bank.py
```

### 12.2 构建与验证（MATLAB）

```matlab
addpath('simulink'); native_startup;
build_hpipm_mex;       % HPIPM/BLASFEO或ABI变化时才需要
build_prediction_mex;
build_native_stage1;
build_native_cmg_geometry;
build_native_actuators_aero;
build_native_integrations;
check_native_integrations;
build_native_plant;
build_public_force_model;
build_surrogate_flight_forces;
check_native_allocation;
check_native_rti;
finalize_native_delivery;
```

`check_native_assembly` 和 `check_hpipm_snapshot` 使用旧固定夹具 `production_qp.mat`。测试新配置时应明确更新其输入或写新测试，不要把旧夹具通过当新配置通过。
`export_native_plant_logs` 后再执行 `python simulink/py/check_native_plant.py` 可做独立耦合对照。

HPIPM编译要读取本机官方 MATLAB类和C源，以及两个 `.dll.a` 导入库。`-lhpipm` 在本机未识别该文件名，构建脚本已使用明确路径。不要修改官方外部安装目录来迁就本项目。

## 13. 验证结果与证据位置

| 层级 | 已取得证据 | 不代表什么 |
|---|---|---|
| 原生6DOF | 四组解析；最大状态误差约2.2e-13，积分q范数误差约2.3e-14 | 不代表气动模型真实 |
| CMG几何/驱动/舵机 | 67静态点、8001点动态、连续一阶解析解和约束检查 | 不代表机械装配已建模 |
| 耦合刚体 | A/B独立RK4角速度对照约6e-17 | 不代表全部闭环逐点相同 |
| QP/在线算法 | 固定数据、读回、组装、首命令、热启动和故障注入 | 不代表实时运行达标 |
| 舵面分配 | 100组BVLS相对目标差约4.4e-16 | 不代表分配后的姿态任务必然可达 |
| 闭环 | 八组P0×A/B窗口完成；修复1.4缺陷后A/B与Python同量级；公开替代模型下60s A/B完成；保存后重载通过 | **逐点轨迹等价与步长收敛仍未通过** |

详细验收表为 `docs/simulink/09_ab_experiment_report.md`；最终状态 JSON 为 `simulink/results/native_delivery/finish_report.json`。
`development_failed/` 存放开发阶段失效/错误设置的材料，不得混进正式统计。

## 14. 下一步开发任务

### 优先级1：把“同量级”推进到“逐点等价”

P0 B 组的致命缺陷（1.4 节）已定位修复，剩下的差异不再是数百个百分点的失效，而是积分与采样口径差异。建议顺序：

1. 从历史 `results/torque_gap_no_cmg_comparison_rti2/P0/<case>/timeseries.csv` 读取控制：`tauc_cmd_x/y/z`、`nuc1..4`；注意 q、omega 等日志状态属于 `t_next`，不是 t。
2. 给原生 P0 真值回放同一控制序列，逐步比较 q、omega、gamma，把残余差异归因到真值积分而不是控制器。
3. 对比扰动采样：旧Python在物理子步中点查询，当前主模型 From Workspace 使用零阶保持；需要时改为插值或加密采样。
4. 做 0.001→更小步长收敛，特别检查离散 gamma 与连续刚体耦合。
5. 逐控制周期对比预测 X/U、首QP、线搜索与首命令，确认剩余差异不来自求解器容差。
6. 再分析奇异附近的敏感性。旧项目有敏感性记录，但不能只用“混沌”作为差异的未经验证解释。

### 优先级2及以后

- 统一配置入口、消除默认case及采样时间散落；保存生成器、参数和二进制哈希。
- 显式A/B参数，替代模型名称推断；改善控制状态保存与完整状态日志。
- 实际反馈下的未来环境/能力预测更新，之后再接匹配布局的经典位置速度外环。
- ABC、多初态、参数不确定性；改变物理参数时同步真值和预测，或明确记录模型失配试验。
- 真实气动数据库、稀薄流/适用域处理；补GUI证据。
- 最后才讨论代码生成、C-MEX在线系统与硬件实时性能。

## 15. 故障排查

| 现象 | 首先检查 |
|---|---|
| 找不到MEX或S-Function | 先native_startup，再which；确认mexw64与平台匹配 |
| Aerospace许可test=1但不能建块 | 用license checkout实际签出；重启后检查真实块创建和sim |
| 长时运行DLL加载失败 | PATH是否重复增长；不要恢复旧的每QP重复追加路径写法 |
| 工作区read-only | Start内用getVariable，避免evalin |
| A组输入/输出尺寸错 | 模型名SurfaceOnly、nx/nu、模板与实际无CMG结构是否一致 |
| Demux把3维量拆成3标量 | A组当前使用Gain传递整条3维向量，不用“Outputs=3”假装一个3维出口 |
| 日志只有末尾1000点 | To Workspace MaxDataPoints=inf |
| 日志变量重名 | 使用唯一短名；超过63字符的长路径名会被截断 |
| GBK/Unicode异常中断队列 | UTF-8与errors=replace，完整控制台落盘，检查真正的MATLAB退出码 |
| 批处理报错却返回0 | 使用已修复退出码透传的mlab.cmd，并检查成功标记/产物 |
| 迎角或高度Assertion停止 | 查实际alpha与高度，不要直接关断言；当前alpha[-40,40]°、大气[0,120000]m |
| 换配置但输出没变 | 顶层是否仍用旧内嵌数据/复制块、预测MEX是否旧常量、进度缓存是否跳过 |
| B组比A组还差、框架角恒为初值 | P0理想驱动的端口连线是否还在（`check_cmg_actuation`）；`deleteContents`/删块会带走父级连线 |
| 模型能跑但某个执行机构毫无作用 | 先给该子系统单独喂常值输入看输出，再看父级端口是否存在连线，不要只看闭环曲线 |
| `.mat` 导出报 EINVAL/无效参数 | Windows 上刚结束的 MATLAB 会话仍占用文件；用 `run_with_retry` 或稍后重试 |
| GUI修改消失 | 同名构建器重新new_system覆盖；将永久变更回写生成器 |

不确定数据结构时先 `out.who`、`whos('-file',file)`、`size(...)`，不要靠转置试错后只比较终值。

## 16. 提交与交接约定

- 新工作使用独立分支，保留旧 Python、历史结果、论文与图表。
- 每项变更记录：源配置及继承、源代码/MEX版本、模式、步长、工况、指标和未通过项。
- 参数变更先验证子模块，再验证固定QP/在线命令，再跑短闭环，再跑完整窗口。
- 交接时带上本手册、SLX、生成器、MAT/C源/MEX及依赖路径、报告与日志；只有SLX或截图不够。
- 当前重要提交：`e3dc1d6`（原生刚体与耦合）、`2a94cfc`（在线RTI与分配）、`bddd34a`（工程主模型、回归与验收缺口）。

继续开发时，先执行第2节短时运行，再按第14节处理精度一致性；不要把“工程可运行交付”误读为“原实施方案全部科研验收通过”。
