# Simulink 移植规格书（S08 → Simulink MIL）

本文件说明本轮把 S08 实验的飞行器/执行机构/控制器闭环搬到 Simulink 的**实现方式、数据契约、
验收判据与复现方法**。面向"为什么这样搭"而不是教程。

- 环境：MATLAB **R2024a**（`D:\Program Files\MATLAB\R2024a`），Simulink 24.1，
  Aerospace Blockset（授权 `license('test')=1`，但本安装里块库无法 `load_system`，因此主链未使用），
  Optimization/MPC/Control/Stateflow 均已授权。
- **C 工具链：可用**。机器上有独立 MinGW-w64 发行版 `D:\w64devkit`（GCC **15.2.0**，
  x86_64-w64-mingw32，含 g++/gfortran/make）。`simulink/mlab.cmd` 会设置
  `MW_MINGW64_LOC=D:\w64devkit`，`mex` 随即用 `matlabroot/bin/win64/mexopts/mingw64.xml`
  选中它。**实测**：`mex simulink/mexopts/s08_hello.c` 编译成功并运行（`2*21 = 42`，
  报 `gcc 15.2.0`）。MATLAB 会提示"该 MinGW 版本不在官方支持列表"
  （官方只支持 MathWorks MinGW 附加组件的那一版），但对 C MEX 而言**功能正常**——
  属于"非官方支持但可用"，若后续换 MATLAB 版本需重新验证 ABI。
- 目标层级：**MIL（高保真数学仿真）**。不含 PIL/HIL，暂不含代码生成。

---

## 1. 架构

```
            ┌──────────────────────── Simulink 模型 s08_<tag>.slx（定步长 0.2 s 离散） ───────────────┐
            │                                                                                        │
 replay 模式 │  s08_replay_sfun ──u(k)──► s08_plant_sfun ──x(k)──► Outport                             │
            │  (读 CURRENT_u.csv)        (RK4 8 子步/控制周期)                                        │
            │                                                                                        │
 closedloop │  s08_ctrl_sfun ──u(k)──► s08_plant_sfun ──x(k)──┐                                        │
   模式     │       ▲                                        │                                        │
            └───────┼────────────────────────────────────────┘                                        │
                    │  JSON over TCP (127.0.0.1)
            ┌───────┴─────────────────────────────────────────┐
            │ simulink/py/ctrl_server.py                      │
            │   = Scenario + TorqueGapRTI + accept/fallback    │
            │     与 nearspace.torque_gap_sim.ClosedLoop.run() │
            │     逐行同源的控制器（只把被控对象换成外部量测）  │
            └─────────────────────────────────────────────────┘
```

**关键设计**：控制器**不重写**。服务器直接复用 `Scenario` / `TorqueGapRTI`（HPIPM RTI）与
`ClosedLoop` 的接受策略、回退与 warm start，所以 Simulink 端与 Python 端是"同一个控制器 + 同一套方程"，
任何差异只能来自接口或数值路径，便于归因。

## 2. 文件地图

| 路径 | 作用 |
|---|---|
| `simulink/mlab.cmd` | 启动器：把 `APPDATA / LOCALAPPDATA / TMP / TEMP / MATLAB_PREFDIR` 重定向到仓库内 `.matlab_*`，否则 MATLAB 因 prefdir 不可写而启动失败 |
| `simulink/check_env.m`、`probe2.m`、`probe3.m` | 环境探测（版本/授权/编译器/无界面 sim/块库/中文路径 I/O） |
| `simulink/py/export_for_simulink.py` | 从 Python 参考模型导出「工况快照 + 单步黄金数据」 |
| `simulink/blocks/s08_case_load.m` | 读工况快照 → 参数结构体 |
| `simulink/blocks/s08_dyn.m`、`s08_plant_step.m` | 与 `nearspace.torque_gap_model` 一一对应的导数/单控制周期推进 |
| `simulink/blocks/s08_quat_mul.m`、`s08_cross3.m` | 四元数乘法、按列叉积（与 `model6.quat_mul`、`cmg._cross3` 同式） |
| `simulink/blocks/s08_plant_sfun.m` | Level-2 MATLAB S-Function：植物（Dwork 状态、8 子步 RK4、CMG 区间平均力矩） |
| `simulink/blocks/s08_replay_sfun.m` | Level-2 S-Function：回放录制的控制序列（开环验证） |
| `simulink/blocks/s08_ctrl_sfun.m` | Level-2 S-Function：TCP 客户端，向 Python 控制器请求 `u(k)` |
| `simulink/check_plant_step.m` | **C1** 单步等价性测试 |
| `simulink/run_case.m` | 建模型 → 运行 → 与 Python 参考逐点对比 → 写 CSV/metrics/图 |
| `simulink/py/check_controller_replay.py` | **C2** 控制器等价性（把录制状态喂回服务器，比对录制指令） |
| `simulink/py/ctrl_server.py` | TCP 控制器服务器 |
| `simulink/py/run_closedloop.py` | 闭环驱动（起服务器 → 跑 MATLAB → 收结果） |
| `simulink/py/sensitivity_check.py` | 敏感性归因：给**参考模型自己**注入 1e-9 扰动，衡量闭环差异的本底 |

## 3. 数据契约

### 3.1 导出的工况快照 `<tag>_case.json`
由 `export_for_simulink.py` 生成，**全部来自 Python 参考模型**，Simulink 端不再重新推导任何物理量：

| 字段 | 用途 |
|---|---|
| `nx, nu, cmg_present, cmg_enabled, substeps, dt, n_steps, t_start, t_end` | 维度、节拍、窗口 |
| `J`、`cmg_geometry{h0,g_axis,s0,gxs0,gdot_max,gddot_max,tau_g}` | 刚体惯量 / 金字塔几何（与 `CMGCluster` 同式） |
| `x0` | 初始状态 `= ClosedLoop._init_plant()` |
| `tau_d[n_steps][substeps][3]` | **子步中点**的外力矩。MSIS2 大气与解析弹道只在 Python 侧采样一次，因此两侧的外扰时间历程完全一致 |
| `diag{q_ref, omega_ref, rho, V, V_ned, alpha_ref_deg, h_m, mach, knudsen}` | 对比与诊断用参考量（在 `t_next` 处取值） |
| `ref_run{u, x, att_err_deg}` | 参考运行的录制指令/状态/误差（回放输入 + 对比基线） |

### 3.2 运行期快照
`results/simulink_port/data/CURRENT_case.json`、`CURRENT_u.csv`、`CURRENT_ctrl.json`：
`run_case.m` 在建模**之前**写入，S-Function 的 `setup` 直接读文件。

> 之所以走文件而不是 workspace/mask 参数：本安装里 Level-2 块的 `setup` 在 `add_block` 时就执行，
> 而 `block.UserData` 在 `setup` 中不可写、mask `Parameters` 报 "no dialog parameters"、
> `evalin('base',...)` 与 `From Workspace` 的数据传递均失败（详见 §6 调试记录）。
> 文件是唯一稳定通道。

### 3.3 时间约定
与 Python 日志严格一致：`t` = 控制周期起点（建模/指令/求解），`t_next = t + dt` = 该行所报状态的时刻。
Simulink 模型日志首个样本是 `x(0)`，对比时剔除，使 300 个样本与 Python 的 `t_next` 一一对应。

## 4. 坐标与符号契约（与 Python 一致，不采用 Blockset 约定）

- 惯性系 NED（x 北、y 东、z 下）；机体系 x 前、y 右、z 下；
- 四元数 scalar-first Hamilton，`q` 把机体系转到 NED，`q̇ = ½ q ⊗ [0;ω]`；
- `ω = [p,q,r]`，`J = diag(Jx,Jy,Jz)`；
- 迎角 `α = atan2(w_b, u_b)`，`V_b = R(q)ᵀ V_ned`（等价于 `α = θ − γ`，见 S08 审计 §2）；
- 力矩单位 N·m、力矩臂 m、惯量 kg·m²、角动量 N·m·s；
- 姿态误差取**短弧**：`e = |2·vec(q_ref⁻¹ ⊗ q)|`（不是 `2·acos(q_w)`，两者在小误差时差 1e-3 度量级）。

## 5. 验收判据与结果

| 判据 | 定义 | 容差 | 结果 |
|---|---|---|---|
| **C1** 单步植物等价 | 8 个随机可达状态/输入，一步后状态与 `ClosedLoop._step_P0` 比 | 1e-12 | **PASS**（7 个工况，最大 2.2e-16） |
| **C3** 开环回放（植物 + 全时间历程） | 用录制的 `tau_a,ν` 驱动 Simulink 植物，300 步状态与 Python 逐点比 | 相对 1e-9 | **PASS**（7 个工况，最大 5.0e-9） |
| **C3′** 姿态误差曲线 | 同上，`att_err_deg` 逐点比 | 1e-6 deg | **PASS**（最大 9.8e-9 deg） |
| **C2** 控制器等价 | 把录制状态喂回服务器，比对录制指令 | 见 §7 | 见 §7（A 组 1.8e-05；B 组受非光滑 QP 影响） |
| **C4** 闭环等价 | 同一控制器 + 同一植物跑闭环，与 Python 参考比 | 以 §7 的敏感性本底为准 | 见 §7 |

C1/C3 的残差量级（1e-16 / 1e-9）分别对应"浮点结合律"与"参考 CSV 只有 10 位有效数字"，
说明移植在数值上等价，而不是"看起来像"。

## 6. 环境适配与调试记录（可复现的坑）

| 现象 | 原因 | 处理 |
|---|---|---|
| `Fatal Startup Error: failed to load settings errors_warnings plugin` | 沙箱不允许写 `%APPDATA%\MathWorks` | `mlab.cmd` 重定向 `APPDATA/LOCALAPPDATA/TMP/TEMP/MATLAB_PREFDIR` 到仓库内 `.matlab_*`（已加 `.gitignore`） |
| `没有找到匹配构造函数 matlab.system.SystemInterface` | 本安装下 `matlab.System` 的 name-value 构造器异常（最小示例也复现） | 放弃 System object，改用 Level-2 MATLAB S-Function（纯函数，无构造器） |
| S-Function 参数 `Parameters` 报 "no dialog parameters" | 同上，mask 参数机制异常 | 改为文件通道 `data/CURRENT_*.json` |
| `block.UserData` 在 `setup` 中不可写 | 文档语义：`UserData` 仅运行时可用 | 用函数 `persistent` 缓存参数 |
| `From Workspace` 无法展开数据 | 同上，workspace 传递不可靠 | 用 `s08_replay_sfun` 从 CSV 回放 |
| `bdroot(block.BlockHandle)` 返回 `'?'` | 句柄在 `setup` 阶段不可用 | 不再依赖模型名/句柄，改固定文件路径 |
| 中文路径 | `D:\项目\飞轮` | 已实测：文件 I/O、`dlmread`、S-Function 均正常；仅控制台回显为 GBK 乱码 |

### 6.1 C 工具链（w64devkit）

| 项 | 值 |
|---|---|
| 位置 | `D:\w64devkit`（独立 MinGW-w64 发行版，非 MathWorks 附加组件） |
| 版本 | GCC 15.2.0，target `x86_64-w64-mingw32`，含 g++ / gfortran / make |
| 接入方式 | `simulink/mlab.cmd` 设 `MW_MINGW64_LOC=D:\w64devkit`；`mingw64.xml` 通过 `<envVarExists name="MW_MINGW64_LOC"/>` 定位，无需改 MATLAB 安装目录 |
| 实测 | `mex simulink/mexopts/s08_hello.c` → 编译成功、运行输出 `gcc 15.2.0`、`2*21 = 42` |
| 提示 | MATLAB 警告"该 MinGW 版本不在官方支持列表"（官方仅支持附加组件版本）。功能正常，但属非官方支持组合，升级 MATLAB 后需复验 |

### 6.2 闭环通信开销：Nagle 算法

第一版闭环 300 步耗时 **87.6 s**（0.29 s/步），而进程内 Python 闭环只要 0.12 s/步。
原因是 TCP 默认开启 Nagle + 延迟 ACK，请求-应答式的小包往返被拖到 ~0.2 s/次。
两侧都设 `TCP_NODELAY` 后降到 **39.4 s**（0.13 s/步），与进程内闭环一致——
说明接口本身已不再是瓶颈，剩余时间几乎全是 RTI 求解。

## 7. 闭环结果与"混沌本底"归因

完整表格见 `results/simulink_port/report.md`，此处给结论与归因方法。

### 7.1 实测（P0 植物，300 步，90–150 s）

| 工况 | 回放 max rel Δstate | 闭环 max rel Δstate | 闭环 max Δ(姿态误差) | RMSE Python / Simulink |
|---|---|---|---|---|
| S00 / B | 1.3e-10 | 1.2e+00 | 0.682 deg | 0.4331 / 0.3825 |
| S00 / A | 6.3e-11 | 4.6e-07 | 3e-05 deg | 3.3258 / 3.3258 |
| S03 / B | 4.0e-10 | 2.0e+00 | 13.9 deg | 9.4900 / 5.6788 |
| S03 / A | 5.1e-11 | 4.8e-09 | 6e-08 deg | 8.2643 / 8.2643 |
| A1 α=20° / B | 2.6e-10 | 2.3e-10 | 5e-12 deg | 0.0052 / 0.0052 |
| A1 α=20° / A | 5.2e-11 | 1.0e-09 | 5e-10 deg | 0.7447 / 0.7447 |

### 7.2 归因方法：把"混沌本底"当成标尺

`simulink/py/sensitivity_check.py` 给**参考模型自己**的初始四元数注入 1e-9 rad 后再跑一遍闭环：

| 工况 | 自跑复现（无扰动）max Δu | 1e-9 扰动后 max Δu | 1e-9 扰动后 max Δ(姿态误差) | RMSE |
|---|---|---|---|---|
| S00 / A | 4.9e-09 | 5.9e-04 | 2.8e-05 deg | 3.3258 / 3.3258 |
| S00 / B | 5.0e-09 | 12.56 | 0.637 deg | 0.4331 / 0.3712 |
| S03 / B | 5.0e-09 | 15.51 | 7.347 deg | 9.4900 / 8.3485 |

- **自跑复现**到 5e-9 N·m / 5e-10 deg → 控制器与日志链路完全确定，不是随机性；
- 1e-9 的姿态扰动就足以让 S00/B 在 **t≈96.6 s**、S03/B 在 **t≈93.6 s** 起分离，并放大到
  0.64 / 7.35 deg —— 因为这两个窗口把 CMG 推到 σ_min ≈ 0.6/0.1 的近奇异区，QP 解对状态极度敏感；
- 移植闭环的差异（0.68 / 13.9 deg）与此**同量级**（1.1× / 1.9×），而 A 组（无 CMG，无放大机制）
  的差异只有 3e-05 deg，同样与本底 2.8e-05 deg 相当。

因此：**移植没有引入系统误差**；闭环差异是模型自身条件敏感性对"种子差"（CSV 记录精度 +
浮点顺序 ≈1e-9）的放大。这也意味着：任何声称"把 S08-B 的结果精确复现到 1e-6"的做法，
在不冻结求解器路径的前提下是不可实现的。

## 8. 复现方法

```bash
# 1) 导出数据（Python）
python simulink/py/export_for_simulink.py --all

# 2) C1 单步等价性（MATLAB）
D:\...\simulink\mlab.cmd "addpath('simulink'); check_plant_step"

# 3) C3 开环回放（MATLAB）
D:\...\simulink\mlab.cmd "addpath('simulink'); run_case('A4_descent_replay_S00_with_cmg_P0','replay')"

# 4) C2 控制器等价性（Python，无需 MATLAB）
python simulink/py/check_controller_replay.py

# 5) 闭环（Python 起服务器 + MATLAB 跑模型）
python simulink/py/run_closedloop.py --all

# 6) 敏感性本底归因
python simulink/py/sensitivity_check.py --tag A4_descent_replay_S00_with_cmg_P0
```

（`mlab.cmd` 需 `MATLAB_HOME` 指向 `D:\Program Files\MATLAB\R2024a`。）

## 9. 本轮边界

1. **只用 P0 植物**（11/7 状态虚拟力矩）。P1（18 状态、真实舵机与有界分配）尚未移植。
2. **控制器未生成 C 代码**：D2① 的"C + S-Function"在工具链上已经**不再被阻塞**
   （w64devkit/GCC 15.2 实测可用），剩下的工作是**移植本身**，不是环境：
   CasADi 对 RTI 用到的 `f/F/A/B/e` 函数生成 C，用 HPIPM 的 C 接口（本机 `.pydeps` 里已有
   `hpipm` 的 Windows DLL）替换 Python 侧封装，再把 `_iterate` / 线搜索 / 接受-回退逻辑
   用 C 重写一遍，最后包成 S-Function。预估 1–2 天，且必须用现有 C2/C3/C4 判据回归。
   当前用"Python 控制器 over TCP"保持**同一份控制器代码**，保真度等价但需两个进程。
3. 植物是解释执行的 M-S-Function：单次 60 s 仿真 0.3–3 s（回放）；
   闭环 300 步约 **39 s**（0.13 s/步，与进程内 Python 闭环的 0.12 s/步相当——见 §6.2）。
   不用于实时性结论。用 C MEX 重写植物预计可把回放降到亚秒级，但闭环时间由控制器支配。
4. Aerospace Blockset 块库在本安装不可加载，主链未使用；如需 Blockset 版本，需先解决块库加载。
