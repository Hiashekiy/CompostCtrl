# B0 提前构型迁移与角动量预置机制验证实验报告

> 任务书：`docs/CMG提前构型迁移与角动量预置机制验证实验方案.md`
> 分支：`experiment/torque-gap-modulated-ocp`；本报告只写实际运行过的内容。

## 0. 结论摘要

| 问题 | 答案 |
|---|---|
| 23.2 s 时域是否让优化器提前看到 110 s 的主机动？ | **是**。A 组在 t = 90 s 时窗口覆盖到 113.2 s |
| 主机动未进入 5 s 窗口时是否仍会提前迁移？ | **不会**。B 组迁移起点 **108.6 s**（A 组 92.8 s），机动前构型最大偏离仅 14.6°（A 组 177.1°） |
| 删除未来主机动后，机动前的 $H_y$ 是否仍被预置？ | **不会**。C 组 $|H|$ 峰值仅 **6.6 N·m·s**，$H_y(110\text{ s}) = -0.001$（A 组 **+155.7**） |
| 舵面是否在准备阶段抵消 CMG 迁移的反作用力矩？ | **是**。A 组准备段 $\overline{\lvert\tau_{a,y}\rvert}=8.70$ N·m 与 $\overline{\tau_{cmg,y}}=-7.69$ N·m **反向**，86% 的样本反号 |
| 提前迁移主要由未来姿态参考、基础飞行参考还是动力学耦合驱动？ | **主要由预测窗口内的未来姿态参考驱动**；基础飞行参考+动力学耦合只造成 <1° 的微小构型偏移（C 组） |

按任务书 §6 的四种情形判读：**observed = 情形 2**（迁移被推迟到主机动进入预测窗口之后），
情形 1/3/4 均未被观察到。

## 1. 代码检查结论（阶段 0）

- 当前 B0 目标函数**不含** $J_{\text{cfg}}$：调制机制已在提交 `30573a6` 中删除
  （见 `docs/torque_gap/modulation_removed_report.md`）。因此本轮不存在“调制项被误开”的可能。
- 目标函数只含：姿态误差、角速度跟踪误差、终端姿态、$\nu$ 及 $\Delta\nu$ 正则、$\Delta\tau_a$ 正则。
- 动力学为 11 状态耦合模型，$\dot\gamma=\nu$，$\tau_{\text{cmg}}=-A(\gamma)\nu$。
- 本轮**未修改** `nearspace/torque_gap_ocp.py` 的目标函数与动力学，未加任何角动量奖励、
  构型参考或奇异值惩罚。
- 基线提交：`30573a6`（工作树在实验前为干净状态）。

## 2. 三组配置与差异审计（阶段 1–2）

配置根：`configs/torque_gap_prediction_mechanism/`
（`base.yaml` 继承冻结的 M50 C1 用例：P0、B0、无限行程、$\dot\gamma_{\max}=1.0$ rad/s、
$\ddot\gamma_{\max}=3.0$ rad/s²、$h_0=50$ N·m·s）。

| 组 | 用例文件 | 改动 |
|---|---|---|
| A | `cases/A_full_horizon.yaml` | 无（= 冻结基线） |
| B | `cases/B_short_horizon.yaml` | `torque_gap.horizon: [{n: 25, dt: 0.2}]` → 时域 5.0 s |
| C | `cases/C_no_future_maneuver.yaml` | `trajectory3.maneuver.delta_deg: 0.0` |

| 组 | 节点数 | 时域 [s] | 首步 $dt$ [s] | 机动 | 机动首次进入窗口 [s] |
|---|---|---|---|---|---|
| A | 22 | 23.2 | 0.20 | 30°/20 s @110 s | 90.0（运行起点即可见） |
| B | 25 | 5.0 | 0.20 | 30°/20 s @110 s | **105.0** |
| C | 22 | 23.2 | 0.20 | 无 | — |

逐叶子配置审计（`results/torque_gap_prediction_mechanism/config_diff.md`，由
`scripts/check_prediction_mechanism_configs.py` 生成）：

- **A vs B** 仅差 `torque_gap.horizon`；
- **A vs C** 仅差 `trajectory3.maneuver.delta_deg`；
- **B vs C** 仅差上述两项。
- 飞行器、CMG、气动盒、扰动、权重、求解器、`dt_ctrl = 0.2 s` 完全一致。

B 组用**均匀 25 × 0.2 s** 网格（而非保留粗节点），因此 B 的近端采样比 A 更密，
“时域变短”不是靠“采样变稀”换来的，机制对比不受采样密度干扰。

## 3. 原始运行数据

| 组 | 目录 |
|---|---|
| A | `results/torque_gap_prediction_mechanism/A_full_horizon/` |
| B | `results/torque_gap_prediction_mechanism/B_short_horizon/` |
| C | `results/torque_gap_prediction_mechanism/C_no_future_maneuver/` |

每组含 `timeseries.csv`（300 × 0.2 s）、`metrics.json`、`meta.json`、`config_resolved.yaml`。
汇总：`results/torque_gap_prediction_mechanism/summary.csv` / `summary.md`。
图：`figures/torque_gap_prediction_mechanism/{A,B,C}_overview.png`、
`mechanism_comparison.png`（均含 `.pdf`）。

## 4. 迁移起始时间

| | A | B | C |
|---|---|---|---|
| 迁移起点（判据 $\max_i\lvert\gamma_i-\gamma_{i,0}\rvert>10°$） | **92.8 s** | **108.6 s** | 未迁移 |
| 迁移起点（运行日志 `migration_onset_t`） | 90.6 s | 108.0 s | — |
| 相对机动的提前量 | **17.2 s** | **1.4 s** | — |
| 机动前构型最大偏离 [deg] | 177.1 | 14.6 | 0.008 |
| 机动前框架行程 [rad] | 3.81 | 0.56 | 0.000 |

B 的迁移起点 108.6 s 与“机动首次进入 5 s 窗口”的 105.0 s 之间相差 3.6 s，
正是框架从零位转到可用构型所需的**受限转场时间**（$\dot\gamma_{\max}=1.0$ rad/s，
$|{\gamma}|$ 需转过约 3.6 rad/4 ≈ 0.9 rad）——即优化器一看到机动就立刻开始迁移，
没有额外延迟。

## 5. 机动开始前角动量（t = 110 s）

| | A | B | C |
|---|---|---|---|
| $H_x$ [N·m·s] | −28.25 | −4.72 | −0.001 |
| $H_y$ [N·m·s] | **+155.71** | **−0.69** | **−0.001** |
| $H_z$ [N·m·s] | 3.10 | 3.86 | −0.013 |
| $\lvert H\rvert$ [N·m·s] | 158.28 | 6.14 | 0.013 |
| $H_y$ 归一化位置（0 = 下界，1 = 上界） | **0.99** | 0.50 | 0.50 |
| 全程 $H_y$ 区间 [N·m·s] | [−155.2, +155.9] | [−157.7, +10.9] | [−0.0, +6.3] |
| $\lvert H\rvert$ 峰值 [N·m·s] | 162.5 | 157.7 | **6.6** |

- A 把 $H_y$ **顶到可达上界的 99%**（$\pm157.7$ N·m·s），这是预置，不是机动中被动产生。
- B 在 t = 110 s 时 $H_y$ 仍在 0 附近（归一化 0.50 = 区间中点），预置**没有发生**；
  它的 $H_y$ 是在机动开始后才被推到下界 −157.7 的（见 `mechanism_comparison.png` 面板 1）。
- C 没有任何角动量积累（$|H|$ 峰值 6.6 N·m·s，来自基础飞行姿态的小幅跟踪）。

> 遵守任务书 §9.7：以上是**角动量储备**，不是瞬时输出力矩包络。A 组 $H_y$ 增大
> **不**意味着某时刻能输出的力矩更大——见 `docs/torque_gap/C1_control_overview_analysis.md` §5.4：
> 把 $H_y$ 推到上界反而使瞬时俯仰力矩包络下降到 31.9 N·m。

## 6. 姿态跟踪与执行器分工

### 6.1 跟踪（注意：C 组无机动，其 RMSE 不与 A/B 的机动 RMSE 直接比较）

| | A | B | C |
|---|---|---|---|
| 全程 RMSE [deg] | 0.685 | 2.761 | 0.070 |
| 机动窗口 RMSE [deg] | **0.309** | **4.713** | — |
| 机动窗口最大误差 [deg] | 0.644 | 7.985 | — |
| 窗口后最大误差 [deg] | 4.596 | 3.189 | — |
| 机动窗口角速率 RMSE [rad/s] | 0.0016 | 0.0151 | — |

**丢掉预测的代价**：机动窗口 RMSE 从 0.309° 恶化到 4.713°（15.2×），峰值误差 8.0°。

### 6.2 执行器分工（准备段 90–110 s / 机动段 110–130 s）

| | A | B | C |
|---|---|---|---|
| 准备段 $\overline{\lvert\tau_{a,y}\rvert}$ [N·m] | 8.70 | 2.02 | 1.38 |
| 准备段 $\overline{\tau_{cmg,y}}$ [N·m] | **−7.69** | −0.07 | 0.00 |
| 准备段合成为力矩 [N·m] | −0.35 | −1.27 | −1.38 |
| 准备段舵面/CMG 反号比例 | **0.86** | 0.12 | 0.23 |
| 准备段 $\nu$ RMS [rad/s] | 0.147 | 0.061 | 0.000 |
| 准备段舵面贴边比例 | 0.32 | 0.00 | 0.00 |
| 机动段 $\overline{\tau_{cmg,y}}$ [N·m] | 1.42 | 7.37 | −0.30 |
| 机动段 $\nu$ RMS [rad/s] | 0.344 | 0.286 | 0.018 |
| 机动段 $\nu$ 饱和比例 | 0.000 | **0.052** | 0.000 |

A 组准备阶段：CMG 用 −7.69 N·m 的俯仰力矩改变自身构型，舵面用 +8.70 N·m 抵住反作用，
净力矩 −0.35 N·m，姿态误差保持在 0.2° 以内（§6.1 的全程/准备段数据）。
**舵面确实承担了抵消 CMG 迁移反作用力矩的角色**（86% 样本反号）。

B 组因为不迁移，准备段几乎没有分工；但代价是机动段必须**在机动窗口内**完成同样的
转场：$\nu$ 饱和比例 5.2%（A 组 0%），$\nu$ 直接顶到 ±1.0 rad/s 上限，
机动段 CMG 平均输出力矩 7.37 N·m（A 组 1.42 N·m）——转场与控制抢同一份力矩预算。

## 7. 求解器可靠性

| | A | B | C |
|---|---|---|---|
| 回退比例 | 0.050 | 0.000 | 0.000 |
| `Solve_Succeeded` 占比 | 0.943 | 1.000 | 1.000 |
| 单步耗时 均值 / P95 / 最大 [s] | 0.76 / 2.95 / 5.92 | 0.02 / 0.04 / 0.14 | 0.05 / 0.10 / 0.36 |
| 平均迭代次数 | 107.2 | 4.5 | 12.9 |

- A 组有 15/300 步回退（91.4 s、94.6 s 各 1 步，其余 13 步在 140.8–149.8 s），
  与历史 M50 运行的回退率同量级（6%），**失败样本全部保留**。
- B 组 NLP 规模更小（26/25 节点 vs 23/22 节点但时域短）且更容易，单步 0.02 s、零回退。
  **这不构成对 B 的“性能优势”**：它的任务更简单（看不见未来）。
- 三组的失败样本全部保留在 `timeseries.csv` 的 `status` 列中。

## 8. 对四种结果情形的判读（任务书 §6）

| 情形 | 预期判据 | 本轮结果 | 判定 |
|---|---|---|---|
| 1. 依赖未来参考 | B 仍像 A 一样提前迁移 | B 迁移起点 108.6 s、前置 1.4 s | **否** |
| 2. 迁移被推迟到机动进入窗口之后 | B 的迁移起点 ≈ 105 s + 转场时间 | 108.6 s ≈ 105 + 3.6 | **是（观察到的情形）** |
| 3. 角动量预置不依赖未来参考 | C 的 $H_y(110)$ 仍显著为正 | C 的 $H_y(110)=-0.001$、$|H|$ 峰值 6.6 | **否** |
| 4. 迁移主要来自动力学耦合 | C 的构型偏差与 A 同量级 | C 的机动前构型偏差 0.008° | **否** |

因此：**提前构型迁移与角动量预置，需要优化器在预测窗口内“看到”未来的主姿态机动；
两者都由预测窗口内的未来姿态参考驱动，而不是由基础飞行参考或耦合动力学自发产生。**

机理解释（与数据一致）：未来参考要求 110 s 起产生 +y 方向的持续力矩。由于
$dH_y/dt=-\tau_{\text{cmg},y}$，交付 +y 力矩必须消耗 $H_y$，因此优化器在 110 s 之前
把 $H_y$ 预置到上界（A 组归一化位置 0.99），用的是**原本空闲的时间与力矩预算**；
如果看不到未来，这笔“本金”只能在机动窗口内临时筹集，于是框架速率饱和、跟踪恶化（B 组）。

## 9. 当前实验仍不能证明的结论

1. **不证明某个具体代价项负责预置**。只证明“需要未来参考”；未做终端项/跟踪项/正则项
   逐项开关的分解实验（那会改变目标函数，本轮禁止）。
2. **不证明最优性**。预置到 99% 上界是当前权重下的解，不是“最优预置策略”；
   也没有验证更少预置是否更好。
3. **单次运行、单一种子**，无 Monte-Carlo；迁移起点对 10° 判据敏感（日志判据 90.6 s vs
   10° 判据 92.8 s），报告了两个口径。
4. **P0 虚拟力矩植物**：$\tau_a$ 是理想气动力矩，未涉及舵偏角速率与伺服滞后；
   P1 机理性结论需另跑。
5. **B 组的差表现不等于“短时域一定差”**：它还同时失去了终端约束的作用距离；
   本实验只用 B 来检验“是否有预测”，不作为控制器设计建议。
6. **C 组的极小 RMSE 不能与 A/B 的机动 RMSE 比较**（任务书 §9.6）。
7. 无限框架行程模型（滑环假设）；有限行程下的预置能力会受行程限制约束，本轮未覆盖。

## 10. 复现

```powershell
# 阶段 2：配置审计
python scripts/check_prediction_mechanism_configs.py

# 阶段 3：三组运行（各 ~1–4 min）
python scripts/run_torque_gap.py --case A_full_horizon --plant P0 `
  --config-root configs/torque_gap_prediction_mechanism/cases `
  --out results/torque_gap_prediction_mechanism/A_full_horizon
python scripts/run_torque_gap.py --case B_short_horizon --plant P0 `
  --config-root configs/torque_gap_prediction_mechanism/cases `
  --out results/torque_gap_prediction_mechanism/B_short_horizon
python scripts/run_torque_gap.py --case C_no_future_maneuver --plant P0 `
  --config-root configs/torque_gap_prediction_mechanism/cases `
  --out results/torque_gap_prediction_mechanism/C_no_future_maneuver

# 阶段 4：指标、图
python scripts/analyze_prediction_mechanism.py
```
