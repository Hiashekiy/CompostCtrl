# E3 QP 失效因果诊断

自动生成：`scripts/e3_qp_forensics.py`。原始 B 组数据只读，未修改。

## 1 失效时序（按控制步，不按四舍五入的整秒）

| 事件 | 时刻 [s] | 步号 | 备注 |
|---|---:|---:|---|
| 首次 QP 回退 | 107.8 | 539 | 该步命令由 t=107.6 s 的状态算出（回退标志滞后一步） |
| σ_min 首次低于下限 | 108.2 | 541 | σ_min=8.921（下限 12.247） |
| |H| 首次超过预算 | 109.0 | 545 | |H|=61.56（预算 60） |

同一步粒度的先后关系：回退先于 σ 跌破下限 = **True**，相差 2 步。决策时刻的状态：|H|=30.35 N·m·s、σ_min=34.727，两者都还在限制之内 —— 也就是说第一次回退**不是**由当时的超预算或奇异状态直接触发的。

## 2 重放一致性

| 记录 | 最大绝对差 |
|---|---:|
| omega | 4.912e-12 |
| H_cmg | 4.997e-09 |
| M_cmg | 4.967e-09 |
| gimbal_deg | 3.933e-08 |
| gdot | 4.939e-10 |
| delta_deg | 4.934e-09 |
| err_norm_deg | 5.000e-09 |
| fallback | 0.000e+00 |

重放与前 626 步记录存在差异（见上表）。

## 3 冻结 QP 的独立可行性检查

每个冻结时刻都分别在原求解器上重放、并用独立 LP（最小总松弛）检查可行性。动力学行（lb=ub）默认作为硬等式，不允许松弛；另有一行 `lp_relax_equality` 把动力学行也放进松弛集，用来量化模型本身与其余约束的差距。

| 时刻 | 原求解器状态(warm) | 违反量 | 独立 LP 最小相对违反 t* | 可行? | 主要松弛来源 |
|---|---|---:|---:|---|---|
| 107.6 s | solved | 1.491e-03 | 1.740e-16 | 是 | momentum_box=7.11e-15, delta_rate=4.16e-17, state_delta=0 |
| 107.8 s | primal infeasible | 3.233e+11 | 3.287e-01 | **否** | momentum_box=281, no_unload_thin_air=155, slack_nonneg=4.6 |
| 109.0 s | primal infeasible | 3.233e+11 | 1.534e+00 | **否** | momentum_box=1.07e+03, no_unload_thin_air=723, cmg_torque_box=23 |

### 3.1 逐组最小必要松弛

| 时刻 | 约束组 | 行数 | 需要松弛的行数 | 总松弛 | 最大松弛 |
|---|---|---:|---:|---:|---:|
| 107.6 s | momentum_box | 42 | 0 | 7.105e-15 | 7.105e-15 |
| 107.6 s | delta_rate | 84 | 0 | 4.163e-17 | 2.082e-17 |
| 107.6 s | state_delta | 42 | 0 | 0 | 0 |
| 107.6 s | state_gimbal_rate | 56 | 0 | 0 | 0 |
| 107.6 s | singularity | 14 | 0 | 0 | 0 |
| 107.6 s | gimbal_rate_cmd | 56 | 0 | 0 | 0 |
| 107.6 s | torque_envelope | 42 | 0 | 0 | 0 |
| 107.6 s | cmg_torque_box | 42 | 0 | 0 | 0 |
| 107.6 s | momentum_radial | 14 | 0 | 0 | 0 |
| 107.6 s | no_unload_thin_air | 14 | 0 | 0 | 0 |
| 107.6 s | delta_box | 42 | 0 | 0 | 0 |
| 107.6 s | slack_nonneg | 14 | 0 | 0 | 0 |
| 107.8 s | momentum_box | 42 | 32 | 281 | 13.42 |
| 107.8 s | no_unload_thin_air | 14 | 14 | 155 | 11.39 |
| 107.8 s | slack_nonneg | 14 | 14 | 4.602 | 0.3287 |
| 107.8 s | cmg_torque_box | 42 | 9 | 2.959 | 0.3287 |
| 107.8 s | delta_rate | 84 | 3 | 0.9228 | 0.3287 |
| 107.8 s | gimbal_rate_cmd | 56 | 4 | 0.02189 | 0.005571 |
| 107.8 s | state_gimbal_rate | 56 | 1 | 0.002005 | 0.002005 |
| 107.8 s | state_delta | 42 | 0 | 0 | 0 |
| 107.8 s | singularity | 14 | 0 | 0 | 0 |
| 107.8 s | torque_envelope | 42 | 0 | 0 | 0 |
| 107.8 s | momentum_radial | 14 | 0 | 0 | 0 |
| 107.8 s | delta_box | 42 | 0 | 0 | 0 |
| 109.0 s | momentum_box | 42 | 34 | 1074 | 62.62 |
| 109.0 s | no_unload_thin_air | 14 | 14 | 723.2 | 53.14 |
| 109.0 s | cmg_torque_box | 42 | 15 | 23.01 | 1.534 |
| 109.0 s | slack_nonneg | 14 | 14 | 21.47 | 1.534 |
| 109.0 s | delta_rate | 84 | 1 | 0.3921 | 0.3921 |
| 109.0 s | gimbal_rate_cmd | 56 | 6 | 0.1507 | 0.026 |
| 109.0 s | state_gimbal_rate | 56 | 2 | 0.04786 | 0.02409 |
| 109.0 s | state_delta | 42 | 0 | 0 | 0 |
| 109.0 s | singularity | 14 | 0 | 0 | 0 |
| 109.0 s | torque_envelope | 42 | 0 | 0 | 0 |
| 109.0 s | momentum_radial | 14 | 0 | 0 | 0 |
| 109.0 s | delta_box | 42 | 0 | 0 | 0 |

## 4 反事实放松（仅诊断，不是性能结果）

| 版本 | 独立 LP 最小相对违反 t* | 可行? | 原求解器状态 | 违反量 |
|---|---:|---|---|---:|
| 107.6 s / Q0 原始 | 1.740e-16 | 是 | solved | 1.491e-03 |
| 107.6 s / Q1_relax_momentum_budget | 2.082e-17 | 是 | solved | 4.110e-08 |
| 107.6 s / Q2_relax_singularity | 1.740e-16 | 是 | solved | 7.776e-04 |
| 107.6 s / Q1_momentum_box_only | 2.082e-17 | 是 | solved | 3.618e-06 |
| 107.6 s / Q1_no_unload_thin_air_only | 2.082e-17 | 是 | maximum iterations reached | 5.430e-04 |
| 107.6 s / Q1_momentum_radial_only | 1.740e-16 | 是 | maximum iterations reached | 1.232e-05 |
| 107.6 s / Q3_relax_rate_and_servo_limits | 1.740e-16 | 是 | maximum iterations reached | 1.413e-05 |
| 107.8 s / Q0 原始 | 3.287e-01 | **否** | primal infeasible | 3.233e+11 |
| 107.8 s / Q1_relax_momentum_budget | 2.082e-17 | 是 | solved | 2.532e-08 |
| 107.8 s / Q2_relax_singularity | 3.287e-01 | **否** | primal infeasible | 3.233e+11 |
| 107.8 s / Q1_momentum_box_only | 2.082e-17 | 是 | solved | 2.804e-08 |
| 107.8 s / Q1_no_unload_thin_air_only | 2.082e-17 | 是 | maximum iterations reached | 8.738e-03 |
| 107.8 s / Q1_momentum_radial_only | 3.287e-01 | **否** | primal infeasible | 3.233e+11 |
| 107.8 s / Q3_relax_rate_and_servo_limits | 3.285e-01 | **否** | primal infeasible | 3.233e+11 |
| 109.0 s / Q0 原始 | 1.534e+00 | **否** | primal infeasible | 3.233e+11 |
| 109.0 s / Q1_relax_momentum_budget | 2.082e-17 | 是 | maximum iterations reached | 3.676e-06 |
| 109.0 s / Q2_relax_singularity | 1.534e+00 | **否** | primal infeasible | 3.233e+11 |
| 109.0 s / Q1_momentum_box_only | 2.082e-17 | 是 | solved | 1.546e-07 |
| 109.0 s / Q1_no_unload_thin_air_only | 1.083e+00 | **否** | primal infeasible | 3.233e+11 |
| 109.0 s / Q1_momentum_radial_only | 1.534e+00 | **否** | primal infeasible | 3.233e+11 |
| 109.0 s / Q3_relax_rate_and_servo_limits | 1.533e+00 | **否** | primal infeasible | 3.233e+11 |

## 5 数值设置的影响（约束不变）

| 时刻 | 设置 | 状态 | 迭代 | 违反量 |
|---|---|---|---:|---:|
| 107.6 s | internal_scaling | maximum iterations reached | 20000 | 6.723e-04 |
| 107.6 s | max_iter_200k | solved | 17100 | 1.491e-03 |
| 107.6 s | eps_1e8 | maximum iterations reached | 20000 | 9.747e-08 |
| 107.6 s | cold_start_no_warm | maximum iterations reached | 20000 | 2.273e-07 |
| 107.6 s | 基线（warm, 项目设置） | solved | 17100 | 1.491e-03 |
| 107.8 s | internal_scaling | primal infeasible | 450 | 3.233e+11 |
| 107.8 s | max_iter_200k | primal infeasible | 175 | 3.233e+11 |
| 107.8 s | eps_1e8 | primal infeasible | 175 | 3.233e+11 |
| 107.8 s | cold_start_no_warm | primal infeasible | 175 | 3.233e+11 |
| 107.8 s | 基线（warm, 项目设置） | primal infeasible | 175 | 3.233e+11 |
| 109.0 s | internal_scaling | primal infeasible | 7050 | 3.233e+11 |
| 109.0 s | max_iter_200k | primal infeasible | 225 | 3.233e+11 |
| 109.0 s | eps_1e8 | primal infeasible | 225 | 3.233e+11 |
| 109.0 s | cold_start_no_warm | primal infeasible | 200 | 3.233e+11 |
| 109.0 s | 基线（warm, 项目设置） | primal infeasible | 225 | 3.233e+11 |

## 6 结论（由上面四节的数据决定）

* 冻结的 3 个控制步中，独立 LP 判定原始 QP 可行的是 **1 个**：107.6 s→可行、107.8 s→不可行、109.0 s→不可行
* 原求解器在这些步上是否给出可用解（违反量 < 1e-3）：107.6 s→否、107.8 s→否、109.0 s→否
* 需要松弛最多的约束组：momentum_box（3 步）
* 反事实放松的结果见第 4 节：若只有放松某类约束才恢复可行，该约束就是该预测问题上的关键限制；若原始问题本就可行而求解器失败，则应归因于数值求解。

