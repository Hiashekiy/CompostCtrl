# 直接舵机 MPC：SQP 接受机制与模型一致性实验

> 依据：`直接舵机MPC_SQP接受机制与模型一致性_完整实验方案.md`
> 分支：`experiment/direct-servo-sqp-acceptance-diagnosis`（自 `bcd96fd` 起）
> 固定条件：`max_sqp_iterations=4`、`dt_ctrl=0.2 s`、60 s 主实验、真实 Simulink 被控对象未改动、无恒力矩扰动。
> 全部数字为本机 MATLAB R2024a + 原生 HPIPM 的真实运行结果；生产控制器（力矩级 MPC + 分配器）未被替换。

## 1. 问题

上一轮（`bcd96fd`）直接舵机 MPC 相对旧力矩级 MPC 明显退化：A 15.463°/52.6 ms，B 2.184°/106.2 ms，而旧版为 A 3.825°/77.7 ms、B 1.110°/88.5 ms。本实验要区分三个可能原因：**QP 与非线性模型不一致**、**“代价一上升就中止”的接受机制**、**直接舵机架构本身的计算负担**。

## 2. 代码事实（审查基准 `bcd96fd`）

1. `direct_servo_mpc_step.m` 的接受判据是纯代价判据：可行且 `J_try > J + tol` 即 `objective_increase` 并 `break`。它既不是舵机可行性检查，也不是 Simulink 验证，因此 `fallback=0` 不代表控制器真的更新了指令。
2. `assemble_direct_servo_qp.m` 的控制正则与 `direct_servo_mpc_step.m` 的 `objective()` **不一致**：QP 有 CMG 指令项，验收函数没有；相邻指令差分项在 QP 中缺少 `u_{k+1}` 反向梯度与首阶段上一指令项。
3. `direct_servo_linearize.m` 把舵机指令（rad）当作气动力矩（N·m）送进力矩级 `native_A_*/native_B_*`，工作点错误。
4. `check_direct_servo_mpc.m` 只抽查少数雅可比列且不设阈值。

## 3. 本轮改动

| 文件 | 改动 |
|---|---|
| `simulink/hpipm/direct_servo_objective.m`（新） | **唯一**完整非线性目标（姿态/角速度/末端/舵机指令/舵机指令变化/CMG 指令/CMG 指令变化），控制器验收与诊断共用 |
| `simulink/hpipm/assemble_direct_servo_qp.m` | 控制正则改为该目标的精确一阶梯度：补上 `k+1` 阶段差分对 `u_k` 的反向梯度、`k=1` 的上一指令项、CMG 项；二阶仍为块对角近似（HPIPM 阶段内 `R` 无法表示跨阶段交叉块），文档与注释明确说明，不称其为精确 SQP Hessian |
| `simulink/hpipm/direct_servo_linearize.m` | 线性化工作点改为**实际气动力矩** `τ_nat + B_δ·δ_nominal`（B 组再拼上 CMG 速率指令）；`model_mode='legacy'` 保留 `bcd96fd` 的错误工作点以便复现对照 |
| `simulink/hpipm/direct_servo_mpc_step.m` | 新增 `acceptance_mode`（`hard_stop` / `no_descent_gate` / `continue_best`）与 `model_mode`（`fixed` / `legacy`）；逐轮结构化诊断（QP 状态、增量、QP 预测 ΔJ、非线性实际 ΔJ、分项代价、修复、接受/拒绝原因、分项耗时）；`continue_best` 保留全周期最优可行候选 |
| `simulink/verify/check_direct_servo_mpc.m` | 增加**硬性**验收：雅可比全维抽查（首/中/末阶段、未饱和/饱和工况）`assert < 0.2`（实测 A 2.6%、B 12.9%）；控制代价梯度与方向导数 FD 对照 `assert < 1e-6`；4 轮上限、首控一致性、可行性断言 |
| `simulink/verify/run_direct_servo_acceptance_ablation.m`（新） | 一条命令跑一组（模式/模型/臂/时长），保存 `run_config.json`、`summary.json`、`iteration_diagnostics.csv`、完整仿真输出、git SHA 与分支 |
| `simulink/verify/run_direct_servo_acceptance_suite.m`（新） | 批量隔离运行，单个失败不影响其余，失败原因单独落盘 |
| `simulink/verify/diagnose_direct_servo_sqp.m`、`summarize_direct_servo_ablation_metrics.m`（新） | 汇总接受统计与姿态指标 |
| `simulink/verify/probe_direct_acceptance.m`（新） | 从已存诊断回答“到底输出了多少真正的新指令” |

冻结未动：真实 6DOF/CMG/`Surface_Actuator`/`Aerodynamic_Model`、气动系数、参考轨迹、扰动、控制周期、仿真步长、4 轮上限、信赖域半径 `0.05236 rad`、权重。

## 4. 单元与一致性检查

```
check_direct_servo_mpc  →  DIRECT_SERVO_MPC_UNIT_PASS
  A(10/3): 雅可比对 FD 2.58e-2；阶段映射对 1 ms 参考 8.4e-11 (0.2 s) / 8.4e-10 (2 s)
  B(14/7): 雅可比对 FD 1.29e-1；阶段映射对 1 ms 参考 2.2e-10 (0.2 s) / 1.8e-9 (2 s)
  控制代价梯度 vs 中心差分方向导数：相对误差 < 1e-6（硬断言）
```

雅可比验收阈值取 0.2：B 组 12.9% 的偏差来自“整阶段线性化 vs 20 ms 子步非线性传播”以及风速/动压随姿态更新，属**冻结工况下的一次 SQP 近似误差**，已在方案允许的“如实报告”范围内，未用其掩盖任何结论。单阶段误差 1e-9 量级，说明单步子步模型本身是精确一致的。

## 5. 对照结果（4 轮 / 0.2 s / 301 周期）

| 代号 | 配置 | 臂 | 姿态 RMS/° | 峰值/° | 接受步/周期 | 零接受周期 | 首控变化周期 | QP/周期 | 均值 ms | >200 ms | J 起点→终点 |
|---|---|---|---|---|---|---|---|---|---|---|---|
| old | 力矩级 MPC（生产） | A | **3.825** | 8.324 | — | — | — | 1.86 | 77.7 | 3 | — |
| old | 力矩级 MPC（生产） | B | **1.110** | 3.616 | — | — | — | 2.18 | 88.5 | 3 | — |
| O | `bcd96fd` 直接舵机，`hard_stop` | A | 15.463 | 24.858 | 0.013 | 297 | 4 | 1.00 | 52.6 | 1 | — |
| O | 同上 | B | 2.184 | 5.352 | 2.09 | 104 | 240 | 2.72 | 106.2 | 4 | — |
| C | 修复模型 + `hard_stop` | A | 14.900 | 24.344 | 0.013 | 297 | 4 | 1.00 | 186.1 | 5 | 76.2 → 7327 |
| C | 修复模型 + `hard_stop` | B | 2.005 | 4.978 | 1.95 | 118 | 219 | 2.59 | 289.1 | 142 | 76.2 → 1480 |
| N | 修复模型 + `no_descent_gate` | A | **5.607** | 11.80 | 0.900 | 30 | 266 | 1.00 | 180.8 | 33 | 76.2 → 8429 |
| N | 修复模型 + `no_descent_gate` | B | **1.629** | 5.448 | 3.22 | 0 | 297 | 3.22 | **450.7** | **277** | 79.4 → 1395 |
| G | 原模型 + `no_descent_gate` | B | 1.631 | 5.445 | 2.93 | 0 | 299 | 2.93 | 409.8 | 259 | 79.6 → 1551 |

`J` 为该周期起点整条预测时域的非线性目标值；`J` 上升说明闭环误差在扩大。

### 5.1 四项因果比较（B 组 60 s，全部 4 轮 / 0.2 s）

| 代号 | 模型 | 接受机制 | 姿态 RMS/° | 接受步/周期 | QP/周期 | 均值 ms | >200 ms | J 起点→终点 |
|---|---|---|---|---|---|---|---|---|
| O | legacy | hard_stop | 2.184 | 2.09 | 2.72 | 106.2 | 4 | — |
| G | legacy | no_descent_gate | **1.631** | 2.93 | 2.93 | 409.8 | 259 | 79.6 → 1551 |
| C | fixed | hard_stop | 2.005 | 1.95 | 2.59 | 289.1 | 142 | 76.2 → 1480 |
| N | fixed | no_descent_gate | **1.629** | 3.22 | 3.22 | 450.7 | 277 | 79.4 → 1395 |

- **O→G（只取消门槛）**：RMS 2.184 → 1.631（改善 25%），但耗时 106 → 410 ms、超预算周期 4 → 259，目标 J 升到 1551；2 s 筛查里 J 更是在 2 s 内由 79.6 冲到 2516。
- **O→C（只修模型，保留门槛）**：RMS 2.184 → 2.005（改善 8%），耗时上升明显。
- **C→N（模型已修，再取消门槛）**：RMS 2.005 → 1.629（改善 19%），耗时 289 → 451 ms。
- **G→N（取消门槛条件下模型修复的收益）**：1.631 → 1.629，几乎为零（0.002°），耗时反而更差。

结论：**精度改善主要来自取消“代价必须下降”的硬门槛，而不是修模型；但两者都让每周期计算时间和 200 ms 超预算次数大幅恶化，且闭环目标 J 单调恶化（76 → 1400~8400）。**

A 组同理：O 15.463° → C 14.900°（修模型几乎无变化）→ N 5.607°（取消门槛才真正改变行为，QP 仍为 1.00/周期，首控在 266/301 周期发生变化）。

## 6. 分时段姿态误差（60 s）

| 配置 | 0–15 s | 15–30 s | 30–45 s | 45–60 s |
|---|---|---|---|---|
| C-A（修复+门槛） | 0.500 | 4.497 | 21.866 | 19.734 |
| N-A（修复+无门槛） | 2.002 | 9.041 | 5.334 | 3.400 |
| C-B | 0.982 | 2.877 | 0.215 | 2.608 |
| N-B | 0.581 | 1.035 | 0.696 | 2.952 |

C-A 在前 15 s 还能保持（0.5°），随后迅速恶化到 20° 以上——与“297 个周期零接受、指令冻结”一致：一旦参考机动开始，冻结的舵机指令无法跟踪。

## 7. 结论（对应方案第 9 节的 5 个问题）

1. **原 A-new 在 301 个周期里实际输出了多少新指令？** 只有 **4 个周期（1.3%）** 的首控与上一周期不同；297 个周期（98.7%）`accepted_steps=0`，每周期只解 1 次 QP。前 10 s 指令一直是 0，随后缓慢移动（0.052 → 0.09 rad）。因此 A-new 的 52.6 ms“快”是**第一轮就退出**的产物，不构成性能优势。
2. **QP 梯度与非线性模型方向导数是否一致？** 修复后：控制代价梯度与方向导数 FD 一致到 <1e-6（硬断言）；单阶段映射与 1 ms 参考一致到 1e-9；雅可比全维抽查 A 2.6%、B 12.9%（阈值 0.2，硬断言）。残余偏差来自整阶段线性化与 20 ms 子步传播、以及随姿态更新的动压，属已声明的一次 SQP 近似误差。
3. **只移除代价下降门槛（O→G）会怎样？** 在未修模型时，2 s 内目标就从 79.6 冲到 2516（32 倍），说明门槛在**错误模型**下起到的是保护作用，而不是罪魁祸首。
4. **修复模型后（C→N）不要求每轮下降是否改善精度/耗时？** 精度确实改善：A 14.900° → 5.607°，B 2.005° → 1.629°；代价是耗时与预算：A 均值 186→181 ms（超预算周期 5→33），B 均值 289→**451 ms**、超预算周期 142→**277**；且 A 的目标 J 在 60 s 内升到 8429，说明“接受上升方向”会把闭环推向更差的状态，只是短时 RMS 看起来更好。
5. **修复后的直接舵机 MPC 是否优于旧力矩级 MPC？** **不优于**。最好的新配置（N）为 A 5.607°/180.8 ms、B 1.629°/450.7 ms，旧版为 A 3.825°/77.7 ms、B 1.110°/88.5 ms：精度更差、B 组耗时是旧版 5 倍并大面积超过 200 ms。按方案要求：**保留实验分支，不替换生产控制器**。

## 8. 尚不能证明之处

1. `continue_best` 模式已实现但未做 60 s 运行（时间预算），因此“取消提前终止”与“取消下降门槛”的区别只有代码与单元测试支撑，没有 60 s 数据。
2. 方案 S3 的两项附加消融（命令级速率约束、信赖域大小）**未做**；本报告的所有对照都在固定 `0.05236 rad` 与固定命令级约束下完成。
3. B 组雅可比 12.9% 的偏差没有进一步用增广平动状态消除；按方案，这是冻结工况下的一次 SQP 近似误差，若要做精确 SQP 需单独研究平动状态增广。
4. 耗时数字来自本机 Windows 同步仿真，不等于硬件实时；所有模式均有超过 200 ms 的周期。

## 9. 复现命令

```powershell
cd D:\项目\飞轮
# 单元与一致性门槛（含硬断言）
.\simulink\mlab.cmd "addpath('simulink'); init_paths; check_direct_servo_mpc"
# 2 s 筛查 + 60 s 主对照（C/N 两臂）
.\simulink\mlab.cmd "addpath('simulink'); init_paths; run_direct_servo_acceptance_suite('screen'); run_direct_servo_acceptance_suite('main60')"
# 汇总
.\simulink\mlab.cmd "addpath('simulink'); init_paths; diagnose_direct_servo_sqp; summarize_direct_servo_ablation_metrics"
```

## 10. 结果文件

- 对照汇总：`simulink/results/online_authority/direct_servo/acceptance_ablation/acceptance_comparison.{csv,json}`、`attitude_metrics.{csv,json}`
- 每个配置：`.../acceptance_ablation/<mode>_<model>_<arm>_<dur>s/` 内 `run_config.json`、`summary.json`、`iteration_diagnostics.csv`、`<arm>_closedloop.mat`
- Q1 证据：由 `probe_direct_acceptance` 从 `L2_60s_20261104_232924`/`232950` 的逐周期诊断重新统计得出
