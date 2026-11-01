# S08 代码与模型审计

对应实验方案 `docs/S08_aero_degradation_experiment.md` §4（实验前代码审计）与 §14 阶段一。
本轮**没有**修改控制器、目标函数、权重、CMG 构型、M50 框架限制、舵面参数或仿真窗口；
唯一进入 `nearspace/` 的改动是一个纯观测的诊断模块和若干日志列（§11）。

- 分支：`experiment/torque-gap-modulated-ocp`
- 审计时 HEAD：`b6009957edb3c4bb3a0de6f05e04573b243e74e3`
- 本轮新增/改动代码：`nearspace/aero_diagnostics.py`（新）、`nearspace/torque_gap_sim.py`
  （仅日志列、`log_conventions`、`_log_row` 中的观测计算）、
  `scripts/{build_s08_matrix,check_s08_config,run_s08_matrix,analyze_s08_aero,summarize_s08_factorial,plot_s08_overview,replay_check_s08}.py`、
  `configs/torque_gap_s08/**`。

---

## 1. 状态方程：这是一个只有姿态自由度的模型

| 项 | 位置 | 内容 |
|---|---|---|
| 平动/弹道 | `nearspace/trajectory.py::SkipTrajectory`、`nearspace/trajectory3.py::Trajectory3D` | **解析给定**：`h(t)=h0+0.5·amp·(1−cos(2πt/T))`、`V(t)=max(V0−dVdt·t, 500)`、`γ(t)=atan2(ḣ,V)`。不是积分出来的，也不受气动力/舵面/CMG 影响 |
| 大气 | `nearspace/atmosphere_real.py`（`backend: msis2`），`atmosphere_factory.make_atmosphere` | `rho(h)`、`T(h)`、`a(h)`、`knudsen(h, cbar)`；MSIS2 表由 `config.yaml → environment.msis2` 决定 |
| 姿态方程（P0，B 组） | `nearspace/torque_gap_model.py::dynamics_tg` | `q̇=½ q⊗[0,ω]`，`Jω̇=τ_a − A(γ)ν + τ_d − ω×(Jω+H(γ))`，`γ̇=ν`（11 状态 / 7 输入） |
| 姿态方程（P0，A 组） | `torque_gap_model.py::dynamics_tg_nocmg` | `Jω̇=τ_a + τ_d − ω×(Jω)`（7 状态 / 3 输入，无 `γ`、无 `ν`、无 `H`、无 `A(γ)ν`） |
| 姿态方程（P1 真值） | `nearspace/model6c.py::dynamics6c` | 18 状态：`q, ω, δ_aero, γ, γ̇`；`δ̇=(δ_c−δ)/τ_s`、`γ̈=(ν_c−γ̇)/τ_g` |
| 扰动 | `trajectory3.Trajectory3D.disturbance` | `M_d0 + M_d,amp·sin(2πf t)`，`trajectory3.dist_scale_qinf: true` 时气动部分按 `q̄(t)/q̄(0)` 缩放；本实验 `dist_pulse_amp=[0,0,0]` |

**审计结论 1：模型里没有平动方程，也没有气动力（升力/阻力）。**
"弹道下降 → 动压增长"在本模型中是**外部给定**的飞行条件时间历程，不是动力学结果。
因此本报告里所有"动压增长"都指解析弹道的 `q̄(t)`，不是"因为姿态或能量变化而生成的动压"。

## 2. 迎角计算

| 项 | 位置 |
|---|---|
| 公式 | `nearspace/model6.py::flow_angles`：`V_b=R(q)ᵀ V_ned`，`α=atan2(w_b, max(u_b,1e-6))`，`β=asin(v_b/|V|)` |
| 使用者 | `model6.aero_coeffs`、`model6c.aero_moment6c`、`torque_gap_authority.build_boxes`（预测时用参考姿态）、`torque_gap_sim._log_row`（本轮的观测列） |
| 坐标系 | NED 惯性系 / 机体系 x 前、y 右、z 下；四元数 scalar-first Hamilton，`q` 把机体系转到 NED；角度代码内为 rad，CSV 内为 deg |

**符号核实（不能假定 α=θ−γ）**：用冻结配置在 S00/S03 弹道上逐点采样，把参考姿态
`q_ref(t)` 与 `V_ned(t)` 代入 `flow_angles`，与 `θ_ref(t)−γ(t)` 对比：

| t [s] | h [km] | V [m/s] | ρ [kg/m³] | q̄ [Pa] | α=atan2(w_b,u_b) | θ_ref | γ | θ_ref−γ |
|---|---|---|---|---|---|---|---|---|
| 0 | 30.0 | 4000.0 | 1.6729e−02 | 133832.1 | 2.000° | 2.000° | 0.000° | 2.000° |
| 30 | 40.3 | 3880.0 | 3.2774e−03 | 24669.7 | 2.000° | 11.480° | 9.480° | 2.000° |
| 60 | 65.0 | 3760.0 | 1.2478e−04 | 882.05 | 2.000° | 15.696° | 13.696° | 2.000° |
| 90 | 89.7 | 3640.0 | 2.7393e−06 | 18.147 | 2.000° | 12.093° | 10.093° | 2.000° |
| 120 | 100.0 | 3520.0 | 5.3412e−07 | 3.312 | 2.000° | 2.000° | 0.000° | 2.000° |
| 150 | 89.7 | 3400.0 | 2.7393e−06 | 15.826 | 2.000° | −8.789° | −10.789° | 2.000° |

结论：在当前符号约定下 `α = θ − γ` 成立（差值恒为 0），且参考迎角恒等于
`trajectory.alpha_trim_deg`（=2°）。`pitch_ref = γ + alpha_trim + manœuvre`
（`trajectory3.Trajectory3D.pitch_ref`），所以**改变参考迎角只能通过 `alpha_trim_deg`
或俯仰机动幅值**。实验设计据此进行（见 §12）。

`α` 依赖**实际姿态**，因此姿态跟踪误差本身会被记入 `alpha_deg`：A/B 两组的"实际迎角"
在退化后必然不同（例如 A1_alpha_p30 中 A 组实际迎角跌到 2.7°，B 组 20.4°）。

## 3. 动压计算

| 项 | 位置 | 定义 |
|---|---|---|
| q̄ | `nearspace/model6.py::qinf`、`torque_gap_authority.natural_moment_v`/`axis_hull`/`build_boxes`、`trajectory3.qinf` | `q̄ = ½ ρ |V_ned|²`，Pa |
| ρ、V 来源 | `trajectory3.rho/V`，即 `h(t)`、`V(t)` 经大气后端 | 与姿态无关 |
| 使用时刻 | 每个控制步在 `t` 处评估预测网格、在 `t_next` 处记录实测（§10） | |

`q̄` 是气动力矩、舵面能力盒、CMG 卸载门限（`mpc_cmg.q_unload_min`，本链未启用）和
气动扰动幅值的共同标度：`τ ∝ q̄`，`舵面半宽 ∝ q̄`。

## 4. 气动模型：线性、无升阻、无马赫/雷诺/稀薄依赖

来源：`nearspace/model6.py::aero_coeffs`，系数来自 `config.yaml → vehicle3`。

```
Cl = Cl_β β + Cl_p p b/(2V) + Cl_r r b/(2V) + Cl_δa δa + Cl_δr δr      (滚转力矩系数)
Cm = Cm0 + Cm_α α + Cm_q q c̄/(2V) + Cm_δe δe                          (俯仰力矩系数)
Cn = Cn_β β + Cn_p p b/(2V) + Cn_r r b/(2V) + Cn_δa δa + Cn_δr δr      (偏航力矩系数)
τ_aero = q̄ S [ b·Cl , c̄·Cm , b·Cn ]
```

| 系数 | 值 | 对迎角的依赖 | 对马赫数/雷诺数/稀薄的依赖 |
|---|---|---|---|
| `Cm_α` | −0.35 1/rad | 线性 | 无 |
| `Cm_q` | −12.0 1/rad | 常数 | 无 |
| `Cm_δe` | 0.45 1/rad | 常数（**控制效能不随迎角变化**） | 无 |
| `Cm0` | 0.0 | — | — |
| `Cl_β/p/r/δa/δr`、`Cn_·` | 见 `config.yaml` | 常数 | 无 |

**审计结论 2：**

1. **没有 CL、CD、升力、阻力**。代码中的 `Cl` 是**滚转力矩系数**，不是升力系数；模型
   只有力矩方程，没有力方程。方案 §11.2 中的 `CL`、`CD`、`lift_N`、`drag_N` 在本模型中
   **不存在**，日志中写成空单元格并标注"未建模"，不用估计值冒充。
2. `Cm` 对 `α` 严格线性，没有失速、没有 `α` 上限、没有 `Cm_α` 随 `α` 的非线性变化。
   扫描到 40° 仍会是线性的，这一点在结论中必须与"真实飞行器不失速"区分开。
3. 气动系数的**控制效能**（`Cm_δe`）与迎角无关：本模型不可能出现"大迎角导致舵效下降"，
   只可能出现"所需配平力矩随 α 增大而增大，而可用舵面力矩只随 q̄ 增大"。
4. **马赫数、雷诺数、Knudsen 数完全不进入气动系数**。当前飞行条件为 M≈12.6、
   Kn≈0.002–0.018（见 §7 表），属于连续介质假设已经可疑的稀薄过渡区，但模型对此**没有任何修正**；
   这些量只在日志中被记录（`mach`、`knudsen` 列）。
5. 模型没有任何声明的适用范围（`config.yaml` 的注释只说明系数是 "example" 数据）。

## 5. 舵面模型

| 植物 | 位置 | 内容 |
|---|---|---|
| P0（虚拟力矩） | `nearspace/torque_gap_authority.py` | 控制器/植物直接给**总气动力矩** `τ_a`，物理上被裁剪到能力盒 `[τ_nat−h, τ_nat+h]`。没有舵偏角状态、没有舵机动态、没有速率限制 |
| P1（18 状态真值） | `torque_gap_authority.allocate_deflection` + `model6c.plant_step6c` | `τ_a,des − τ_nat` 经**有界最小二乘**分配到 `δ=[δe,δa,δr]`，界为 `max(−δmax, δ_prev−δ̇max·dt)`；一阶舵机 `δ̇=(δ_c−δ)/τ_s`，`τ_s=0.05 s`；`δmax=[25,20,20]°`，`δ̇max=[60,60,60]°/s` |
| 饱和判据 | `torque_gap_sim._summary` | 位置饱和 `|δ|≥0.999δmax`；实测速率饱和 `|Δδ|/dt ≥ 0.999δ̇max` |

P1 中"指令力矩不可达"会体现为分配残差 `alloc_res_*`（已记录）。

## 6. 气动能力盒（预测用的舵面权限）

| 项 | 内容 |
|---|---|
| 中心 | `τ_nat(q̄, α_ref, β_ref, ω_ref)`——**参考状态下零舵偏的气动力矩**（`build_boxes`） |
| 半宽 | `scale · |M_δ| δmax`，`M_δ = q̄ S diag(b, c̄, b) × (Cl_δ, Cm_δ, Cn_δ)`（`surface_matrix`、`axis_hull`） |
| 标定 | `authority.calibrate: true`：沿场景的 9 个 q̄ 采样点，按"盒内随机目标被有界最小二乘可达的比例 ≥ 99%"标定一个统一缩放系数，结果写入 `meta.json`；本实验链实测 **0.7738** |
| 含义 | 盒只是"舵面可实现集合的轴对齐内接盒"。它**不含迎角依赖**，只随 q̄ 变化 |
| 本实验条件下 | Q0=18.147 Pa 时舵面可达半宽：滚转 9.68、俯仰 **47.89**、偏航 7.61 N·m；预测用的盒半宽 = 0.7738 × 该值（俯仰 37.1 N·m）。作为对照，同一条件下的 CMG 俯仰轴能力 `Σ|A_y|·νmax = 57.74 N·m` |

## 7. CMG 模型

| 项 | 位置 | 值/公式 |
|---|---|---|
| 几何 | `nearspace/cmg.py::CMGCluster` | 4 台金字塔，`β=54.7356°`，`g_i=[sinβcosθ_i, sinβ sinθ_i, cosβ]`，`s_i(γ)=cosγ s_i0+sinγ (g_i×s_i0)` |
| 角动量 | `cluster.momentum` | `H(γ)=Σ h0 s_i(γ)`，`h0=50 N·m·s`（6000 rpm 稳速） |
| 雅可比/力矩 | `cluster.jacobian`、`cluster.torque` | `A=h0[g_i×s_i]`（3×4），`M=−A(γ)ν` |
| 限制 | `CMGParams`（M50 链） | `|ν|≤1.0 rad/s`，`|ν̇|≤3.0 rad/s²`，速率回路 `τ_g=0.03 s` |
| 奇异 | `cluster.sigma_min` | home `σ_min=40.82 N·m·s`，奇异下限 `0.30×=12.25` |
| 陀螺耦合 | `torque_gap_model.dynamics_tg`、`model6c.dynamics6c` | `−ω×(Jω+H(γ))` |
| 预测侧 | `torque_gap_ocp`（CasADi，与 `cluster_constants()` 同一闭式几何） | B 组 NLP 407 变量 / 407 约束 |

`mpc_cmg.M_cmg_box=61` 与 `H_lim_frac/H_planner_max` 属于旧 `mpc_cmg` 块，**本链未使用**（
本链走 `torque_gap.*`）。

## 8. 控制器

| 项 | 内容 |
|---|---|
| 后端 | `nearspace/torque_gap_rti.py`（HPIPM RTI，本轮全部运行使用）或 `torque_gap_ocp.py`（CasADi+IPOPT） |
| 结构 | 22 个预测步 / 23 个节点（6×0.2 s + 4×0.5 s + 4×1.0 s + 8×2.0 s），预测时域 23.2 s；B 组 407 变量 / 407 约束，A 组 7 状态 / 3 输入 |
| 代价 | `e_scale=1°`、`omega_scale=0.02`、`w_att=w_rate=1`、`q_terminal=10`、`r_nu=0.05`、`r_dnu=0.1`、`r_dtau=1e-4`（`configs/torque_gap/base.yaml`，本轮未改） |
| 约束 | 舵面力矩盒 `τ_min≤τ_a≤τ_max`（每节点、随时间变化）、四元数归一、CMG 的 `|ν|≤νmax` 与框架速率增量约束（B 组） |
| 回退 | `torque_gap_sim.ClosedLoop._fallback`：先用上一步可行解并重新投影到当前盒/框架限制，否则用 PD 镇定 + （B 组）最小二乘框架速率填缺口；每次回退都被记录 |
| 接受策略（本轮固定） | `solver.tol=1e-6`、`max_iter=600`、`accept_feasible: false`、`warm_shift_on_fallback: false`（与历史基线实验一致；不收敛/违反约束的步**不使用**） |
| 决策时刻 | `t` 时刻求解与指令；状态、参考、误差统计一律在 `t_next=t+dt_ctrl`（`_log_row`，方案 §9.1 对齐） |

## 9. CMG 开关的真实影响（方案 §3、§10 的硬要求）

配置层：`torque_gap.cmg_present`（物理上有没有 CMG）与 `torque_gap.cmg_enabled`
（装了但用不用）。`cmg_present: false` 与 `cmg_enabled: true` 同时出现会直接 `ValueError`。

`results/torque_gap_s08/config_audit.md` 的自动核查结果：

- A 组：`nx=7`、`nu=3`、`Scenario.cluster is None = True`、`cmg_enabled=False`；
  预测问题是 7 状态 / 3 输入（`X[7,N+1]`、`U[3,N]`），
  `plan()` 的名义需求 `H_nom≡0`，日志中所有 CMG 列为空单元格。
- B 组：`nx=11`、`nu=7`、`cmg_present=True`。
- **冻结框架 ≠ 无 CMG**：取 `γ=[0.3,0.1,−0.2,0.4] rad`、`|H|=30.76 N·m·s`、同一 `ω` 时，
  B 组把 `ν` 置零后的角加速度与 A 组相差 `4.58e-05 rad/s²`（陀螺项 `−ω×H`）。
  差值非零即证明 A 组不是"框架锁定"的伪基线。相关回归测试见
  `tests/test_torque_gap_no_cmg.py::test_5`。

## 10. 日志：本轮新增的观测量

`nearspace/torque_gap_sim.py` 的 `COMMON_COLUMNS` 新增（**全部是观测，不进入任何方程**）：

| 列 | 含义 |
|---|---|
| `alpha_deg`、`alpha_ref_deg`、`beta_deg` | 实际迎角/侧滑角（`flow_angles`，实际姿态 vs 实际相对风）与参考迎角 `θ_ref−γ` |
| `mach`、`knudsen`、`qbar_pa`、`V_rel` | 由大气后端与相对风得到（诊断用，不进入气动系数） |
| `CL`、`CD`、`lift_N`、`drag_N` | **空单元格**：模型无气动力，明确标注未建模 |
| `Cm_nat`、`Cm`、`Cm_alpha_deriv`、`Cm_delta_e`、`delta_e_rad` | 零舵偏的 `Cm`、含舵偏的 `Cm`（仅 P1 有舵偏状态）、以及两个常值系数（记录其"恒定"这一事实） |
| `tau_nat_x/y/z` | 零舵偏气动力矩 |
| `half_x/y/z` | 舵面可达力矩半宽 `|M_δ|δmax` |
| `tau_req_x/y/z` | `τ_a,cmd − τ_nat`：控制器要求舵面提供的控制力矩 |
| `surf_util_x/y/z`、`surf_util_max`、`surf_margin_y` | 需求利用率 `|τ_req|/half`（1.0=需求填满可用能力）与绝对余量 |

约定写入 `meta.json → experiment.log_conventions`，包含"CL/CD 未建模"和"P0 无舵偏状态"两条。

## 11. 配置解析

- `configs/torque_gap_s08/base.yaml` → `../torque_gap_no_cmg_comparison/base.yaml`
  → `../torque_gap_m50/cases/c1_pitch30.yaml` → `torque_gap_unlimited` → `torque_gap` →
  `config.yaml`。每条用例文件都通过同一条链解析，运行后写 `config_resolved.yaml` 与
  `meta.json.config`（**解析后的完整配置**，不是局部覆盖文件）。
- `scripts/check_s08_config.py` 逐键比对 A/B 两臂的解析配置：16 个工况的差异**只有**
  `torque_gap.cmg_present` 与 `torque_gap.cmg_enabled`；质量、惯量、气动系数、扰动、
  舵面参数、预测网格、求解器、权重、时间窗口全部相同。
- `scripts/build_s08_matrix.py --check` 保证 48 个生成文件与生成器一致（无手改漂移）。

## 12. 由审计决定的实验构造方式（方案 §5–§7）

审计直接决定了 A1–A3 的可实施形式：

1. **迎角只能通过参考俯仰角构造**（`alpha_trim_deg`），且模型没有力方程，所以"迎角工况"
   等价于"一个恒定的参考俯仰角 + 恒定飞行条件"。用例使用**平飞定常条件**
   （`trajectory.amp=0`、`dVdt=0`、无机动）构造：`γ≡0`、`ρ`、`V`、`M`、`Kn` 全窗口恒定，
   于是 `α_ref≡alpha_trim`、`q̄` 全场恒定——这正好满足方案要求的"固定动压"/"固定迎角"。
2. **动压只能通过 ρ 或 V 改变**。A2/A3 保持原窗口的空速 `V=3640 m/s`（M≈12.6 基本不变），
   用**密度（高度）**把 `q̄` 调到 `{0.5,1,2,4}×Q0`（h=94.1/89.7/85.4/81.1 km）。这与原始弹道
   "下降 → 密度增大 → 动压增大"的机制一致，同时把马赫数的变化压到最小（12.64→12.72）。
3. **参考水平 Q0 取原始 S00/S03 评价窗口起点**（t=90 s，h=89.7 km）：`Q0=18.147 Pa`，
   `V=3640 m/s`，`M=12.64`，`Kn=0.009`。A1–A3 与 A4 因此在同一参考状态下可比。
4. **外扰必须按物理规律折算**（本轮实现中发现并修正的一个陷阱）：
   `trajectory3.dist_scale_qinf: true` 用 `q̄(t)/q̄(0)` 缩放气动外扰，而 `q̄(0)` 取的是
   **该工况自身弹道** t=0 的值。对原始弹道 q̄(0)=133832 Pa（30 km），所以 S00/S03 窗口内
   气动外扰几乎为零（只剩 `Md0=[0.2,0.5,0.3] N·m`）；而"平飞定常条件"工况的 q̄(0) 就是
   自己的 q̄，缩放因子恒为 1，会把 `Md_amp=[4,20,6] N·m` 全额加进来——同一 q̄ 下外扰被放大
   约 1000 倍，受控实验将无法与 A4 对照。因此每个受控工况在 `vehicle3.Md_amp` 上预乘
   `q̄_case/133832`，使等效幅值等于"原始规律在该 q̄ 下的值"（≈ `Md0`），
   `Md0` 不做缩放。折算系数与结果幅值写进 `design.json` 与工况文件注释。
5. **受控工况叠加同一条 +8° 俯仰凸起参考**（110–130 s，`shape: bump`，两端归零）。
   理由：纯保持工况下参考恒定、外扰只有 Md0，控制器可以把误差压到 1e-3 度量级，
   扫描退化为"可行/崩溃"的阶跃，测不到加载能力。凸起对所有工况完全相同，
   因此工况间差异仍只有 α / q̄，同时每个工况都能测到舵面力矩、CMG 负载与跟踪误差。
   迎角考核因此取机动窗之外的时刻（`check_s08_config.py`），机动中点的参考迎角
   （= 设定值 + 8°）单独记录。
4. **A5（非线性气动对照）本轮不执行**：审计确认当前模型确实是线性小迎角近似（`Cm` 对 `α`
   线性、`Cm_δe` 与 `α` 无关），满足方案的执行前提；但仓库里没有任何可追溯到该构型、
   马赫数、高度或雷诺数范围的公开/风洞/CFD 数据，而方案 §9 明确禁止"人为添加升力下降曲线来制造失速"。
   在没有数据支撑时，任何 N1 模型都只能算敏感性构造，不能回答"是否真实失速"，
   因此本轮以"缺少气动数据"作为 A5 的交付结论，而不是编一个 N1 去跑。
## 13. 原始 S00/S03 复现（阶段一验收）

见 `results/torque_gap_s08/replay_check.md`（由 `scripts/replay_check_s08.py` 生成）：
A4 两个工况与历史 `results/torque_gap_no_cmg_comparison_rti2/P0` 的同名运行逐项对比
（全程/机动窗/末段 RMSE、最大误差、回退比例、步数、求解耗时）。

## 14. 审计对结论的直接限制（写进最终报告的"事实"部分）

1. 本模型没有升力/阻力，也没有气动力的平动反馈：**不能用它讨论"失速"、"升力损失"或"下降加速"**。
2. `Cm` 对 `α` 线性、`Cm_δe` 与 `α` 无关：本模型里**不存在"大迎角舵效下降"**这一机制；
   可能出现的是"配平需求随 `α` 增大而增大，可用舵面力矩只随 `q̄` 增大"。
3. 气动系数与马赫数/雷诺数/Knudsen 数无关；本实验的飞行条件（M≈12.6，Kn=0.002–0.018）
   已经进入稀薄过渡区，而模型没有稀薄修正——所有定论都只在这个模型的适用性内成立。
4. 因此"迎角效应"在本模型中只通过**配平需求**和**参考姿态**体现，不通过气动系数非线性体现；
   A1 的结果必须按这个语义解读（见 `docs/torque_gap/S08_results_analysis.md`）。
