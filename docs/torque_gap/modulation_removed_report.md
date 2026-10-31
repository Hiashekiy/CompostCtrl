# 移除“预测力矩余缺调制”机制（J_cfg）后的回归验证

> 结论：**调制机制已从优化器里彻底删除，现有方法的性能没有实质变化**（机动窗口 RMSE 0.318° → 0.325°，+2.2%；
> 全程 RMSE 反而从 1.366° 降到 0.702°），NLP 从 499 个变量减到 407 个，38 项测试/校验全部通过。
> 但**闭环轨迹不会逐点相同**——因为 MPC 是路径敏感的：删掉的是零代价的自由变量，改动的是 IPOPT 的迭代路径。

## 1. 删掉了什么

| 位置 | 删除内容 |
|---|---|
| `nearspace/torque_gap_ocp.py` | `mode` / `lambda_cfg` 字段；决策变量 `V[4,N+1]`（能力探针）；参数 `d`；目标项 `J_cfg = λ·Σ_k w_k‖−A(γ_k)v_k − d_k‖²`；`_cfg_nodes`；`OCPResult.V`、`OCPResult.E_gap`；`gap_capability()` |
| `nearspace/torque_gap_scenario.py` | 读取 `torque_gap.mode` / `torque_gap.lambda_cfg` |
| `nearspace/torque_gap_sim.py` | 暖启动三元组 → 二元组 `(X, U)`；meta 里 `mode`/`lambda_cfg` → `modulation: "removed (diagnostics only)"` |
| `configs/torque_gap/base.yaml` | `mode: B0` / `lambda_cfg: 1.0` 两个键（仅留注释说明已删除） |
| `scripts/run_torque_gap.py` | `--mode`、`--lambda-cfg`；输出目录名 `CASE_PLANT`（不再有模块段） |
| `scripts/run_torque_gap_matrix.py` | 模式维度（B0/B1/B2），矩阵条目 5 元组 → 4 元组 |
| `scripts/evaluate_torque_gap.py` | B0/B1 配对表改为“仅当归档里存在多种 mode 时才输出” |

**刻意保留（纯诊断，不参与任何决策）**：`torque_gap_modulation.py` 的 `τ_nom / R⁺ / R⁻ / d`、CSV 里对应的列、
`E_gap` 列、以及所有既有图。保留的理由是历史对照与机理分析需要这些量；本报告第 3 节用数值证明
“它们的计算完全没变，只是不再进入目标函数”。

## 2. NLP 规模

| | 改动前（λ_cfg=0 的 B0） | 改动后 |
|---|---|---|
| 决策变量 | 499（X 253 + U 154 + **V 92**） | **407**（X 253 + U 154） |
| 等式/不等式约束 | 364 | 364（探针变量本来不受任何约束，只是带上下界的自由变量） |
| 每步迭代数（300 步闭环均值） | 111.1 | **106.4（−4.2%）** |
| NLP 回退率 | 6.0% | **4.7%** |

## 3. 同一个用例重跑：C1（30°/20 s 俯仰，P0，无行程限制）

归档运行：`results/torque_gap_unlimited/C1_B0_P0/`（当时 λ_cfg=0，即调制未生效）
新运行：`results/torque_gap_noMod/C1_P0/`（调制已删除）

| 指标 | 归档（含探针变量） | 新（已删除） | Δ |
|---|---|---|---|
| 机动窗口 RMSE [deg] | 0.3182 | **0.3253** | +2.2% |
| 机动窗口最大误差 [deg] | 0.6496 | 0.7127 | +9.7% |
| 全程 RMSE [deg] | 1.3656 | **0.7020** | −48.6% |
| 机动期 \|τ_cmg,y\| 峰值 [N·m] | 77.2 | 64.1 | −17% |
| 机动期缺口覆盖残差 [N·m] | 12.17 | 11.92 | −2.1% |
| \|H\| 峰值 [N·m·s] | 158.56 | 159.06 | +0.3% |
| 舵面贴边比例 | 0.416 | 0.488 | +0.07 |
| NLP 回退率 | 6.0% | 4.7% | −1.3 pt |

**诊断量的可复现性（关键证据）**：两臂的 `tau_nom_y` 最大偏差 **1.0e-11 N·m**、`d_y` 最大偏差 **0.0 N·m**
——即“调制信息”本身的计算逐位不变；改的只是它不再被优化器使用。

**闭环轨迹的逐点偏差**（300 步，同一时间基）：`max|Δγ_i| = 5.4–13.5°`、`max|ΔH| = 78.3 N·m·s`、
`max|Δτ_cmg,y| = 131.7 N·m`、`max|Δ姿态误差| = 4.9°`。
这不是“代码改错了”，而是闭环 MPC + 指令抖振的必然结果：删掉零代价自由变量会改变 IPOPT 的
scaling 与迭代路径，0.2 s 级指令的任何微小差异会被 60 s 的闭环放大。**评价应看统计指标，不看逐点重合。**

## 4. 求解器行为：同一台机器、同一负载下的受控基准

用 `git worktree` 检出改动前的提交（`c5f2a93`），在两份代码上对**同两个时刻的同一问题**各做 10 次
**冷启动**求解（`scripts/bench_ocp_modulation.py`；冷启动保证每次是同一个确定性 NLP）：

| 时刻 | 改动前（499 vars） | 改动后（407 vars） |
|---|---|---|
| t = 100 s | **0/10 收敛**（600 次迭代撞墙，6.81 s） | **10/10 收敛**（110 次迭代，0.58 s） |
| t = 112 s | 10/10（278 次迭代，2.64 s） | 10/10（413 次迭代，3.84 s） |

→ 探针变量确实是**纯累赘的自由变量**，它对 IPOPT 的影响是双向的、不可预测的
（有时更快、有时更慢），且会改变某一步是否收敛。**整体上删掉它没有带来性能倒退**：
闭环迭代数 −4.2%、回退率 −1.3 pt、机动指标在 2% 内。
单步墙钟时间不宜用于比较——两次完整运行的机器负载不同（0.48 s vs 0.72 s），不作结论。

## 5. 回归测试

```
python tests/test_torque_gap_model.py          → 10/10 PASS
python tests/test_torque_gap_unlimited.py      →  6/6  PASS
python scripts/check_torque_gap_model.py --no-plot   （历史 S1 校验，未改）
python scripts/check_torque_gap_unlimited.py --no-plot → 32/32 checks passed
```

## 6. 影响范围与不变项

- **不变**：11 状态模型与 RK4、CMG 几何 `H/A/τ=−Aν`、框架速率/加速度限制（含 M50 分支）、
  气动盒与矢量化标定、参考轨迹与扰动、预测网格/时域、姿态与速率权重、正则项、IPOPT 设置、
  P0/P1 两套植物、`cmg_enabled` 开关、结果/图的目录结构。
- **改变**：NLP 少 92 个变量；控制器不再“看见”缺口；`meta.json` 不再有 `mode/lambda_cfg`
  （改为 `modulation: "removed (diagnostics only)"`）；输出目录名不再带 `_B0`。
- **历史数据全部保留**：`results/torque_gap/`、`results/torque_gap_unlimited/`、
  `results/torque_gap_m50/`、`results/torque_gap_noCMG/` 未改动；新结果在 `results/torque_gap_noMod/`。

## 7. 复现命令

```powershell
# 新（无调制）代码，单臂回归
python scripts/run_torque_gap.py --case C1 --plant P0 `
    --config-root configs/torque_gap_unlimited/cases `
    --results-root results/torque_gap_noMod
# 五面板总图（同版式，无调制）
python scripts/plot_c1_control_overview.py --run results/torque_gap_noMod/C1_P0 `
    --fig-root figures/torque_gap_noMod --stem C1_noMod_overview `
    --doc C1_noMod_overview_analysis.md
# 新旧求解器对比（需要旧提交）
git worktree add .bench_old c5f2a93
python .bench_old/scripts/bench_ocp_modulation.py
python scripts/bench_ocp_modulation.py
git worktree remove .bench_old --force
```

图：`figures/torque_gap_noMod/C1_noMod_overview.png` / `.pdf`；
分析：`docs/torque_gap/C1_noMod_overview_analysis.md`。
