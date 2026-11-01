# S08 → Simulink 移植报告（MIL）

由 `simulink/py/make_report.py` 从运行产物自动生成；实现细节见 `docs/torque_gap/simulink_port_spec.md`。

口径：Simulink 侧为定步长 0.2 s 离散模型（植物内部 8 个 RK4 子步，与 Python 同构），所有对比都以 Python 参考运行的 `timeseries.csv` 为基线，时间戳语义同为 `t_next`。

## C1 单步植物等价性（MATLAB ↔ Python，tolerance 1e-12）

| 工况 | 状态维数 | 测试数 | max abs Δx | max rel Δx | 结果 |
|---|---|---|---|---|---|
| A1_alpha_p20_no_cmg_P0 | 7 | 8 | 1.110e-16 | 1.110e-16 | PASS |
| A1_alpha_p20_with_cmg_P0 | 11 | 8 | 2.220e-16 | 2.220e-16 | PASS |
| A2_qbar_p0p5_no_cmg_P0 | 7 | 8 | 1.110e-16 | 1.110e-16 | PASS |
| A4_descent_replay_S00_no_cmg_P0 | 7 | 8 | 1.110e-16 | 1.110e-16 | PASS |
| A4_descent_replay_S00_with_cmg_P0 | 11 | 8 | 2.220e-16 | 2.220e-16 | PASS |
| A4_descent_replay_S03_no_cmg_P0 | 7 | 8 | 1.110e-16 | 1.110e-16 | PASS |
| A4_descent_replay_S03_with_cmg_P0 | 11 | 8 | 2.220e-16 | 2.220e-16 | PASS |

**判定：全部通过**（残差 1e-16 量级 = 浮点结合律，说明方程逐条同源）。

## C3 开环回放：植物 + 300 步时间历程

用 Python 录制的 `tau_a, ν` 驱动 Simulink 植物，逐点比较 300 个控制步的状态与姿态误差。

| 工况 | 步数 | max rel Δstate | max Δ(姿态误差) [deg] | RMSE Python | RMSE Simulink | 结果 |
|---|---|---|---|---|---|---|
| A4_descent_replay_S00_with_cmg_P0 | 300 | 1.33e-10 | 9.63e-09 | 0.433104 | 0.433104 | PASS |
| A4_descent_replay_S00_no_cmg_P0 | 300 | 6.31e-11 | 2.08e-09 | 3.325804 | 3.325804 | PASS |
| A4_descent_replay_S03_with_cmg_P0 | 300 | 3.99e-10 | 9.82e-09 | 9.489976 | 9.489976 | PASS |
| A4_descent_replay_S03_no_cmg_P0 | 300 | 5.05e-11 | 5.01e-09 | 8.264280 | 8.264280 | PASS |
| A1_alpha_p20_with_cmg_P0 | 300 | 2.60e-10 | 2.19e-09 | 0.005216 | 0.005216 | PASS |
| A1_alpha_p20_no_cmg_P0 | 300 | 5.24e-11 | 6.57e-10 | 0.744690 | 0.744690 | PASS |
| A2_qbar_p0p5_no_cmg_P0 | 300 | 4.99e-11 | 8.99e-10 | 0.558827 | 0.558827 | PASS |

最坏相对状态偏差 3.99e-10（容差 1e-9），最坏姿态误差偏差 9.82e-09 deg（容差 1e-6）。

## C2 控制器等价性（把录制状态喂回服务器，比对录制指令）

| 工况 | 步数 | max abs Δu [N·m] | 中位/分布 | 结果 |
|---|---|---|---|---|
| A4_descent_replay_S00_with_cmg_P0 | 300 | 1.556e+01 | — | 差异见 §混沌本底 |
| A4_descent_replay_S00_no_cmg_P0 | 300 | 1.830e-05 | — | 差异见 §混沌本底 |

说明：A 组（无 CMG）的差异只有 1.8e-05 N·m，属于非光滑 QP 的末位敏感；B 组在第 ~36 步之后分离——该时段 CMG 被推到 σ_min≈0.6 的近奇异区，参考模型自身对该处的 1e-9 扰动同样地分离（见下节）。

## 混沌本底：参考模型自身对 1e-9 扰动的响应

| 工况 | 自跑复现 max Δu | 首次分离 (步/t) | 扰动后 max Δu | 扰动后 max Δerr | RMSE 参考/扰动 |
|---|---|---|---|---|---|
| A4_descent_replay_S00_no_cmg_P0 | 4.92e-09 | -1 / nan s | 0.00 | 0.000 deg | 3.3258 / 3.3258 |
| A4_descent_replay_S00_with_cmg_P0 | 4.98e-09 | 33 / 96.6 s | 12.56 | 0.637 deg | 0.4331 / 0.3712 |
| A4_descent_replay_S03_with_cmg_P0 | 4.99e-09 | 18 / 93.6 s | 15.51 | 7.347 deg | 9.4900 / 8.3485 |

结论：闭环差异的量级与该模型自身的敏感性本底相同——移植没有引入额外误差，差异来自模型在近奇异处的混沌放大。

## C4 闭环：Simulink 植物 + Python 控制器（TCP）

| 工况 | 步数 | 仿真墙钟 [s] | max rel Δstate | max Δ(姿态误差) [deg] | RMSE Python | RMSE Simulink | 最大误差 Python/Simulink |
|---|---|---|---|---|---|---|---|
| A4_descent_replay_S00_with_cmg_P0 | 300 | 87.6 | 1.21e+00 | 0.682 | 0.4331 | 0.3825 | 3.77 / 3.09 |
| A4_descent_replay_S00_no_cmg_P0 | 300 | 49.4 | 4.64e-07 | 0.000 | 3.3258 | 3.3258 | 6.22 / 6.22 |
| A4_descent_replay_S03_with_cmg_P0 | 300 | 93.4 | 1.98e+00 | 13.902 | 9.4900 | 5.6788 | 30.99 / 21.69 |
| A4_descent_replay_S03_no_cmg_P0 | 300 | 58.9 | 4.84e-09 | 0.000 | 8.2643 | 8.2643 | 15.97 / 15.97 |
| A1_alpha_p20_with_cmg_P0 | 300 | 70.1 | 2.28e-10 | 0.000 | 0.0052 | 0.0052 | 0.02 / 0.02 |
| A1_alpha_p20_no_cmg_P0 | 300 | 33.2 | 1.03e-09 | 0.000 | 0.7447 | 0.7447 | 1.59 / 1.59 |

### C4 判读：与模型自身敏感性本底对比

| 工况 | 闭环 max Δ(姿态误差) | 参考模型 1e-9 扰动的 max Δ(姿态误差) | 比值 | 判读 |
|---|---|---|---|---|
| A4_descent_replay_S00_with_cmg_P0 | 0.682 deg | 0.637 deg | 1.07 | 与本底同量级 → 混沌放大，非移植误差 |
| A4_descent_replay_S00_no_cmg_P0 | 3.36e-05 deg | 2.75e-05 deg | 1.22 | 与本底同量级 → 混沌放大，非移植误差 |
| A4_descent_replay_S03_with_cmg_P0 | 13.9 deg | 7.35 deg | 1.89 | 与本底同量级 → 混沌放大，非移植误差 |
| A4_descent_replay_S03_no_cmg_P0 | 6.3e-08 deg | （未测） | — | 数值上等价（1e-3 deg 以下） |
| A1_alpha_p20_with_cmg_P0 | 5.08e-12 deg | （未测） | — | 数值上等价（1e-3 deg 以下） |
| A1_alpha_p20_no_cmg_P0 | 4.99e-10 deg | （未测） | — | 数值上等价（1e-3 deg 以下） |

说明：本底 = 给**参考模型自己**注入 1e-9 rad 初始姿态扰动后，其闭环结果相对参考运行的最大姿态误差差。闭环移植的种子差（CSV 记录精度 + 浮点顺序）正是这个量级，因此「移植差 ≈ 本底」即说明差异来自模型自身的条件敏感性。


图：`figures\simulink_port\summary_replay.png`

图：`figures\simulink_port\summary_closedloop.png`
