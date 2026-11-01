# 无 CMG / 舵面+CMG 对照 —— 代码审查与实现记录

对应实验指导书 `docs/CMG多场景姿态机动对照实验指导书_无CMG对照版.md` 的
§3（代码审查与实现要求）、§9（日志与实现问题）、§11 阶段 0 与 §15 交付清单的代码部分。
实验结论见 `docs/torque_gap/no_cmg_vs_cmg_scenario_experiment.md`。

## 1. 基线与分支

| 项 | 值 |
|---|---|
| 分支 | `experiment/torque-gap-modulated-ocp` |
| 基线 HEAD（本轮改动前） | `99cda4c3ede0638409afe243b600b5a1a83927cf` |
| 工作树（改动前） | 干净，仅新增本实验指导书一份未跟踪文档 |
| 本轮改动 | 见 §3，全部位于 `nearspace/`、`scripts/`、`tests/`、`configs/torque_gap_no_cmg_comparison/`、`results/torque_gap_no_cmg_comparison/`、`figures/torque_gap_no_cmg_comparison/`、`docs/torque_gap/` |

历史结果目录（`results/torque_gap*`、`figures/torque_gap*`）一律未被覆盖：
本实验的全部输出写入新的 `*_no_cmg_comparison` 目录。

## 2. 飞行器惯量是否包含 CMG 硬件惯量（指导书 §2.3 审计）

配置来源：`config.yaml → vehicle3`（`Jx=1200`、`Jy=8500`、`Jz=9000` kg·m²）。
CMG 参数：`h0 = 50 N·m·s/台`、`rotor_rpm = 6000`、`n = 4`、`mass_per_unit = 30 kg`。

1. **`J` 不是由含 CMG 的质量/惯量分解生成的。** 仓库中 `J` 是一张通用飞行器惯量表，
   没有任何质量块、安装位置或转动惯量分解可以回溯到 CMG 组件；`VehicleParams` 也没有
   任何 CMG 质量项。因此现行 `J` 的语义是**不含 CMG 组件的飞行器基准惯量**，
   按指导书 §2.3 第 1 条，A/B 两组直接使用该 `J`。
2. **转子自转惯量**可从数据算出：`Ω = 6000 rpm = 628.32 rad/s`，
   `Jr = h0/Ω = 0.0796 kg·m²` 每台。四台按自转轴与垂直轴（`Jr` 与 `Jr/2`）在零框架角
   构型下合成 `ΔJ = diag(0.2387, 0.2387, 0.1592) kg·m²`，即
   `0.0199% / 0.0028% / 0.0018%` 的 `Jx/Jy/Jz`，低于本轮任一效应量级。
   该量在两臂中都被保留（A 组不因“拆掉转子”而改惯量），并在报告中如实说明。
3. **CMG 结构质量（4 × 30 kg）不在 `J` 中，也无法从现有数据剥离或加上**：
   没有安装半径/几何，无法计算其转动惯量贡献。因此按指导书 §2.3 第 3 条，
   本轮 `J` 明确标记为
   **“匹配惯量的算法对照假设”（matched-inertia algorithmic comparison）**，
   不得宣称是“完整硬件移除后的质量/惯量级工程对比”。

结论：A/B 两组使用同一 `J_b`、同一气动系数、同一舵面限幅与同一外部扰动；
差异只有 CMG 是否存在及其控制能力。`scripts/audit_no_cmg_configs.py` 每次运行前
都会逐键核对这一点（见 `results/torque_gap_no_cmg_comparison/config_audit.md`）。

## 3. 实现内容

### 3.1 新增 `cmg_present`，与 `cmg_enabled` 解耦

`nearspace/torque_gap_scenario.py::Scenario.__init__`

| 配置 | 含义 | 模型 |
|---|---|---|
| `cmg_present: false` | 物理上没有 CMG | 7 状态 / 3 输入；`cmg_enabled` 强制为 false |
| `cmg_present: true, cmg_enabled: false` | 装了 CMG 但锁定框架（仅诊断用，**不是本轮 A 组**） | 11 状态 / 7 输入，`ν ≡ 0` |
| `cmg_present: true, cmg_enabled: true` | 本轮 B 组 | 11 状态 / 7 输入 |

校验：`cmg_present: false` 且 `cmg_enabled: true` 直接 `ValueError`
（`tests/test_torque_gap_no_cmg.py::test_8`）。A 组下 `Scenario.cluster is None`，
CMG 几何对象根本不被创建；`plan()` 的名义需求 `H_nom ≡ 0`（不是“冻结当前框架角”）。

### 3.2 P0 植物：真正的无 CMG 动力学

`nearspace/torque_gap_model.py`

```
NX_TG = 11 (q, ω, γ)      NU_TG = 7 (τa, ν)          # cmg_present: true
NX_TG_NOCMG = 7 (q, ω)    NU_TG_NOCMG = 3 (τa)        # cmg_present: false
```

新增 `dynamics_tg_nocmg / rk4_step_tg_nocmg / plant_step_tg_nocmg`：

```
J ω̇ = τa + τd − ω × (J ω)
```

没有 `γ`、没有 `ν`、没有 `H(γ)` 陀螺耦合、没有 `A(γ)ν` 力矩项。
`test_5` 用一个非零角动量的 CMG 构型证明：A 组结果**不等于**“B 组把 ν 置零”
（后者仍带 `ω × H(γ)`），因此 A 组不是冻结框架的伪无 CMG 模型。

### 3.3 OCP：按物理维数建模

`nearspace/torque_gap_ocp.py::TorqueGapOCP._build` 按 `cmg_present` 条件构建：

| | A 组 | B 组 |
|---|---|---|
| 决策变量 | `X[7, N+1]`、`U[3, N]` | `X[11, N+1]`、`U[7, N]` |
| 动力学 | `q̇ = ½ q⊗[0,ω]`，`Jω̇ = τa + τd − ω×Jω` | 现有 11 状态模型（含 `A(γ)ν` 与 `H(γ)`） |
| 参数 | 无 `nuprev` | 含 `nuprev` |
| 约束 | 初始状态 7、打靶 7×N、四元数 (N+1) | 追加 4×N 框架速率增量约束 |
| 上界 | 只有舵面力矩盒 | 追加 γ 行程（本链为无界）与 `|ν| ≤ γ̇max` |
| 22 节点规模 | 227 变量 / 184 约束 | 407 变量 / 364 约束 |

`test_2` 逐项断言 A 组 NLP 中不含 `γ`、`ν`、框架几何函数（`_A_of`/`_H_of` 不存在）、
参数向量不含 `nuprev`、状态块没有任何框架行程界。**没有**用“把 ν 权重调大”或
“ν 界设为零”来伪造无 CMG（指导书 §14.1–§14.3）。

### 3.4 P1 真实舵面植物

`nearspace/torque_gap_sim.py::ClosedLoop._step_P1`

A/B 两组走**同一段**代码：`allocate_deflection`（有界最小二乘舵偏分配）、
`delta_max` 限幅、`delta_dot_max` 速率限幅、`tau_s` 一阶伺服、同一个 `plant_step6c`。
B 组额外把 `ν` 送进框架回路；A 组构造 `VehicleCMGParams(cmg_on=False)`，
其 18 状态真值中 `dx[10:18] ≡ 0`、`Hc = Mc = 0`（`test_9b` 断言），
即框架状态被结构性冻结且不携带任何角动量/力矩。

### 3.5 日志时间对齐（指导书 §9.1）

原先 `_log_row()` 把积分后的状态与**周期起点**的参考配对。现改为：

* `t`：控制周期起点，命令、求解耗时、力矩上下限、fallback 都属该时刻；
* `t_next = t + dt_ctrl`：该行所报状态的时刻；参考姿态、参考角速度、
  姿态/角速度误差、时间戳统一取 `t_next`；
* 窗口统计（`man0`/`man1`/`pre`/`post`/`all`）与机动窗口都以 `t_next` 判定。

`tests/test_torque_gap_no_cmg.py::test_log_time_alignment` 逐行验证
`row.qref == q_ref(t_next)`、`att_err == |e_R(q_ref(t_next), q)|`，且该值与
`q_ref(t)` 的结果可区分。

### 3.6 CMG 力矩统计口径（指导书 §9.2）

* `cmg_x/y/z`：**控制区间平均**的已实现 CMG 力矩。P0 在积分器子步内按中点法则对
  `−A(γ)ν` 积分再除以 `dt_ctrl`；P1 用与真值模型同结构的 RK4 子步积分框架回路
  （`γ̇ = γ̇`、`γ̈ = (ν_c − γ̇)/τ_g`，该回路与其余状态解耦）后做同样的子步平均。
  该辅助积分与真值模型的一致性被逐次记录为 `p1_gimbal_mismatch_max`
  （实测 0.0）。
* `cmg_end_x/y/z`：**周期末瞬时值** `−A(γ)ν`。
* 两种口径分别成列，从不混用；`meta.json` 的 `experiment.log_conventions` 里写明。
* A 组：所有 CMG 列在 CSV 中为**空单元格**（“不适用”），不写零值曲线。

### 3.7 逐机动窗口统计（指导书 §9.3）

`ClosedLoop._summary()` 输出 `windows` 与 `stats`：每个 maneuver 一个 `man{i}` 窗口
（`t0`、`t1`、轴向、幅值），另加 `combined`（并集）、`pre`（机动前 12 s）、
`post`、`all`。两次机动之间的保持阶段不会混入 `man0`/`man1` 的 RMSE。
`test_per_manoeuvre_windows_do_not_include_the_hold_phase` 对 S07 断言两个窗口分别
为 110–130 s 与 150–170 s。

### 3.8 其余指标

姿态：全程/窗口 RMSE、窗口最大误差、收敛时间、三轴误差分量、角速度 RMSE、
四元数短弧为主、欧拉角分量仅作辅助（`e_roll_deg`/`e_pitch_deg`/`e_yaw_deg`）。
舵面：实际力矩 RMS/峰值、贴边比例、P1 舵偏 RMS/峰值、**实测舵偏角速率饱和比例**
（`|Δδ|/dt` 对 `δ̇max`）、位置饱和比例、期望与实际力矩差。
CMG（仅 B）：`γ`、`ν`、`H`、`sigma_min`、迁移起始时刻与提前量、机动前累计框架行程、
框架速率饱和比例、区间平均与实际 CMG 力矩、沿任务方向的瞬时力矩能力。
求解：成功率、fallback 次数/比例、均值/P95/最大耗时、迭代数、最大约束违反、
是否满足 0.2 s 控制周期（`realtime_met`）。失败与 fallback 样本全部保留。

## 4. 测试

| 文件 | 覆盖 |
|---|---|
| `tests/test_torque_gap_no_cmg.py` | 指导书 §3.4 的 10 项要求 + 日志时间对齐 + CMG 力矩口径 + 逐机动窗口 |
| `tests/test_torque_gap_model.py` | 原有 B 组模型/OCP 回归（未改动，全部通过） |
| `tests/test_torque_gap_unlimited.py` | 原有无限框架臂回归（仅适配 `_step_P0/_step_P1` 新的 5/7 元返回值） |
| `tests/test_atmosphere_real.py` | 大气后端回归（未改动，全部通过） |

B 组 NLP 规模仍为 407 变量 / 364 约束，与改动前一致，说明 B 组模型没有回退。

## 5. 配置与脚本

| 文件 | 作用 |
|---|---|
| `configs/torque_gap_no_cmg_comparison/base.yaml` | 共同物理，继承 M50 C1 链（`configs/torque_gap_m50/cases/c1_pitch30.yaml`） |
| `configs/torque_gap_no_cmg_comparison/scenarios/*.yaml` | 每个场景唯一定义（窗口 + maneuver） |
| `configs/torque_gap_no_cmg_comparison/{cases,sequential}/*_{no_cmg,with_cmg}.yaml` | 由场景文件派生的 A/B 配置 |
| `scripts/build_no_cmg_comparison.py` | 生成上述 A/B 配置（`--check` 检测漂移） |
| `scripts/audit_no_cmg_configs.py` | A/B 逐键差异审计 → `config_audit.md/json` |
| `scripts/audit_no_cmg_references.py` | 参考姿态采样、方向/符号/叠加语义核验与出图 |
| `scripts/run_no_cmg_matrix.py` | 运行矩阵（`smoke`/`P0`/`P0seq`/`P1`/`seed`/`all`） |
| `scripts/summarize_no_cmg_comparison.py` | `summary.csv` / `summary.md` |
| `scripts/plot_no_cmg_comparison.py` | 逐场景 A/B 对照总图（9 联：姿态°/误差°/误差分量°/力矩分解/能力包络/CMG 力矩/框架角°/角动量/求解耗时），以及跨场景汇总图 |
