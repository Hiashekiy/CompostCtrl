# 在线 MPC 五轮 SQP 求解逻辑重构报告

> 实施依据：`Codex_Simulink_MPC五轮SQP求解逻辑重构实施方案.md`
> 分支：`experiment/simulink-mpc-five-sqp`（自 `experiment/simulink-online-torque-authority` 的 `34d2808` 起）
> 环境：本机 MATLAB R2024a + Simulink/Aerospace Blockset + 原生 HPIPM（无 Python 参与闭环）
> 全部数字来自本机真实运行日志，未做任何预估或放大。

## 1. 结论摘要

1. 在线求解读取路径已改为**每周期最多 5 轮、每轮恰好 1 次 HPIPM QP、全步（无 alpha 线搜索）**；默认 5，可配 4，其他值直接报错。
2. 60 s A/B 全部 301 个控制周期成功，**0 次安全回退**，每周期 QP 次数最大值 5（4 轮配置为 4），无任何 `for alpha` 残留。
3. 逐周期计算耗时从 **1.588 s / 2.403 s（均值）降到 0.0749 s / 0.0869 s**，最坏从 9.47 s / 6.97 s 降到 0.575 s / 0.260 s（且这两个最坏值都是首拍或前几拍）。
4. 姿态跟踪：A 臂与基线持平（3.8127° vs 3.8232° RMS）；B 臂 5 轮配置变差（1.5338° vs 0.7826°），而 **4 轮配置为 0.6815°（优于基线）**。该差异在本仿真中是完全可重复的确定性结果（见 §5.4），但它反映的是闭环轨迹分叉，而不是轮次越多越准。
5. 认证/约束口径完全保留：八顶点力矩残差 ≤3.55e-15 N·m、舵机顶点残差 ≤6.0e-15 rad、俯仰独立比例仍为 1、滚转–偏航联合认证与 CMG 限幅均有效。
6. 主要耗时项不是 QP（占比 7.9% / 8.6%），而是 `online_authority_rollout` 的在线预测与认证（90.9% / 90.3%）。下一阶段优化应针对该函数，不要动认证精度。

## 2. 基线冻结

- 修改前 HEAD：`34d2808`（`experiment/simulink-online-torque-authority`），`git status` 仅有 `simulink/results/native_closedloop/*_runtime*.json` 两处运行统计残留，已单独提交为 `3421aef`，未丢弃或回退用户数据。
- 基线 60 s A/B（同一物理模型与控制器参数，旧 12 轮 + 4 alpha 求解）：`simulink/results/online_authority/L2_60s_20261104_173125`（A）、`L2_60s_20261104_173201`（B）。
- 旧模型副本仍在 `simulink/results/online_authority/frozen_models/`，未被覆盖。

## 3. 代码改动

| 文件 | 动作 |
|---|---|
| `simulink/hpipm/native_online_rti_step.m` | 重写控制流：≤5 轮、每轮一次 QP、全步更新、accepted/proposal 分离、诊断与早停原因 |
| `simulink/hpipm/online_shift_controls.m` | 新增：按绝对时间重采样热启动（`s.dts` 非均匀） |
| `simulink/hpipm/online_verify_first_command.m` | 新增：输出首控前的二次校验（盒、分配器可达性、CMG 锥、与 `authority` 一致性） |
| `simulink/hpipm/solve_hpipm_snapshot.m` | 增加 `setup_time_s` / `fill_time_s` / `readback_time_s` 分项计时（数值口径不变） |
| `simulink/hpipm/native_mpc_sfun.m` | 记录并打印迭代配置与早停原因；`max_sqp_iterations` 非法值报错 |
| `simulink/modelgen/build_closedloop.m` | 新增第 7 个参数 `sqp_iterations`（默认 5，只允许 4/5），写入模板与日志标签 |
| `simulink/verify/check_online_five_sqp.m` | 新增：冷/热启动、4/5 轮、非法配置、时间对齐、第 1/3/5 轮故障注入、首控门 |
| `simulink/verify/report_five_sqp_performance.m` | 新增：逐周期耗时 mean/median/P95/max、QP 次数、分项占比、早停分布 |
| `simulink/verify/report_five_sqp_comparison.m` | 新增：指定历史目录的指标对照（不改写历史结果目录） |

**未改动**：`assemble_native_qp.m`、`online_authority_rollout.m`、`online_certified_torque_box.m`、`online_certify_servo_vertices.m`、`online_surface_reachability.m`、`online_translation_prediction.m`、`verify_authority_candidate.m`、离线分支 `native_rti_step.m`（仍保留旧 12 轮 + alpha 算法）、目标函数与全部权重、状态/输入维数、`native_F/A/B/E_*` MEX、6DOF/CMG/舵机真值模型、PID/舵面内环、全部 Python 代码。

## 4. 新求解流程（对应方案 §3）

- **冷启动**：`U=0` → `online_authority_rollout(...,true)` 做一次因果修复，再用 `verify_authority_candidate` 独立认证；不通过直接走在线安全回退，不进入 HPIPM。
- **热启动**：`online_shift_controls` 把上周期已接受控制按 `[t_j,t_{j+1})` 分段常值、以 `0.2+Σdts(1:j-1)` 的绝对时刻重采样（越过旧末端则保持末段）。实测该网格下 **22 个阶段里有 15 个与“左移一格”不同**，所以旧写法在非均匀段确实是错的。旧状态轨迹一律不复用，必须从新实测状态重新传播。
- **每轮**：在当前已接受 `(X,U,a)` 处组装并求解**一个** QP（`r.U` 是增量），`U_try=U+r.U` 全步；只做一次在线预测，必要时对**同一个未被改写的全步输入**做一次因果修复 + 重新认证。
- **接受/停止**：`cost_try ≤ cost_now + objective_tol`（`objective_tol=1e-9·max(1,|J|)`，仅容纳浮点舍入）才接受；目标恶化即拒绝并结束本周期；`max|r.U|<1e-4`、相对改进 <1e-6 或达到轮次上限提前结束。停止原因写入 `info.early_stop_reason`。
- **失败与首控**：任一轮 QP 失败/NaN → 用本周期最后一个**完整认证**的 `(U,X,a)`；输出前 `online_verify_first_command` 复核首控与实测位置、实测舵偏、上一分配指令、CMG 锥的一致性，失败则在线安全回退（不允许把未验证序列留作下周期热启动）。
- **故障注入**：`state.inject_failure_at=[...]` 可在指定轮次注入 QP 失败（第 1 轮失败 → 输出本轮已验证名义首控，不进入 fallback；第 3/5 轮失败 → 保留此前已接受序列）。旧的 `force_failure` 第 4 参数语义与既有测试行为完全保留。

## 5. 验证结果

### 5.1 单元/集成检查（全部通过）

```
check_online_five_sqp            ONLINE_FIVE_SQP_TEST_PASS
check_online_authority_integration  ONLINE_INTEGRATION_PASS A / B
check_online_authority_static    ONLINE_AUTHORITY_STAGE_A_PASS samples=120 moment_error=1.42109e-13 servo_error=1.94289e-15
check_online_flight_prediction   ONLINE_FLIGHT_REPLAY_PASS
check_pitch_authority_scaling    PITCH_SCALE_PASS alpha=32 scales=[0.753086419752333 1 0.753086419752333]
```

`check_online_five_sqp` 断言覆盖：默认 5 轮、4 轮配置、非法值 `0/3/6/12` 均报错、`qp_solved≤5`、`accepted_steps≤qp_solved`、`iteration_authority` 数量=已接受步数+1、热启动逐阶段时间对齐、第 1/3/5 轮注入失败后输出仍为已验证首控或安全回退、首控门拒绝被篡改的控制。
结果文件：`simulink/results/online_authority/five_sqp/{A,B}_five_sqp.json`、`A/B_injected_round1.mat`、`A/B_cold_injected_round3/5.mat`。

### 5.2 60 s A/B 对照（同一物理模型与控制器参数）

| 运行 | 配置 | 姿态 RMS/° | 峰值/° | 三轴力矩 RMS/N·m | 回退 | 迭代总数 | 修复次数 | 均值/最坏控制耗时 |
|---|---|---|---|---|---|---|---|---|
| `L2_60s_20261104_173125` A | 旧 12 轮 + alpha | 3.8232 | 8.0854 | 0.168 / 0.720 / 0.110 | 0 | 1828 | 7131 | 1.588 / 9.467 s |
| `L2_60s_20261104_173201` B | 旧 12 轮 + alpha | 0.7826 | 2.6669 | 0.135 / 0.535 / 0.137 | 0 | 3120 | 11532 | 2.403 / 6.971 s |
| `L2_60s_20261104_201906` A | 5 轮全步 | 3.8127 | 8.2806 | 0.086 / 1.039 / 0.062 | 0 | 557 | 556 | 0.0749 / 0.575 s |
| `L2_60s_20261104_201940` B | 5 轮全步 | 1.5338 | 4.9040 | 0.143 / 0.670 / 0.155 | 0 | 667 | 667 | 0.0869 / 0.260 s |
| `L2_60s_20261104_204309` B | 5 轮全步（复算） | 1.5338 | 4.9040 | — | 0 | 667 | 667 | 0.0879 / 0.641 s |
| `L2_60s_20261104_203114` A | 4 轮全步 | 3.8250 | 8.3244 | — | 0 | 561 | 560 | 0.0757 / 0.660 s |
| `L2_60s_20261104_203147` B | 4 轮全步 | 0.6815 | 2.4490 | — | 0 | 651 | 651 | 0.0853 / 0.234 s |

2 s 快速回归（接线/控制输出/CMG 运动）：`L2_2s_20261104_201657` A 姿态 RMS 9.59e-5°，`L2_2s_20261104_201708` B 4.99e-4°，0 回退。

### 5.3 逐周期计算耗时（301 周期/臂，`report_five_sqp_performance`）

| 运行 | 均值 | 中位数 | P95 | 最大 | >0.2 s 周期 | QP/周期 均值·最大 | 已接受步/周期 |
|---|---|---|---|---|---|---|---|
| A 5 轮 | 0.0749 s | 0.0698 s | 0.1453 s | 0.5749 s | 3 | 1.85 / 5 | 1.17 |
| B 5 轮 | 0.0869 s | 0.0783 s | 0.1499 s | 0.2599 s | 4 | 2.22 / 5 | 1.22 |
| A 4 轮 | 0.0757 s | 0.0709 s | 0.1252 s | 0.6599 s | 4 | 1.86 / 4 | 1.18 |
| B 4 轮 | 0.0853 s | 0.0767 s | 0.1424 s | 0.2342 s | 4 | 2.16 / 4 | 1.19 |

分项（5 轮 A/B）：QP 组装+求解均值 5.9/7.5 ms（其中 HPIPM 内部 `time_ext` 0.76/1.24 ms），在线预测+认证均值 68.1/78.5 ms → **占周期总耗时 90.9% / 90.3%**。

超预算周期明细：A 在 t=0（0.575 s，含 MEX/HPIPM 首次冷启动）、t=0.2（0.233 s）、t=2.2 s（0.202 s）三拍 >0.2 s，此后最坏 0.183 s；B 在 t=0（0.260 s）、t=1.0、1.8、5.4 s（≤0.216 s）四拍 >0.2 s。

早停分布（301 周期）：A `objective_increase` 206 / `objective_stagnation` 93 / `max_iterations` 2；B `objective_increase` 300 / `max_iterations` 1。每周期 1.16–1.22 次已接受更新，说明全步更新几乎总在第二轮被拒绝。

### 5.4 为什么 4 轮比 5 轮在 B 臂更好

B 臂 5 轮的两次独立运行（`201940` 与 `204309`）控制统计、姿态 RMS/峰值**逐位相同**，说明仿真完全确定性：差别不是噪声，而是“第 5 轮多用或少用一次更新”后闭环轨迹分叉的结果（667 vs 651 次迭代、300 vs 292 次拒绝）。因此这组数据只能说明两条轨迹的优劣，不能推导“轮次越多越准”或反之。方案要求的默认值仍为 5，`sqp_iterations=4` 作为已验证可用的配置对照。

### 5.5 认证与约束（5 轮 A/B，301 周期全量）

- 力矩盒八顶点残差 ≤ 3.55e-15 N·m；舵机顶点反求残差 ≤ 6.00e-15 rad；分配残差 ≤ 3.55e-15 N·m。
- 俯仰通道独立缩放仍为 1，滚转/偏航取 `0.7531` 联合内接盒（`check_pitch_authority_scaling`）。
- B 臂 CMG 全程运动：γ 峰值 3.29/1.82/3.58/2.02 rad，ν 峰值≈1.0 rad/s；A 臂无 CMG 项，与旧版一致。
- 未来环境量预测偏差（闭环重规划导致，非回放误差）：A 高度 0.0401 m、动压 2.06e-5 Pa、速度 1.24e-3 m/s、迎角 0.0814 rad；B 0.0152 m、3.55e-6 Pa、7.07e-4 m/s、0.0391 rad。

## 6. 已知限制（如实记录，未做任何隐瞒或补偿）

1. **B 臂 5 轮配置姿态精度变差**（1.5338° 对基线 0.7826°），A 臂持平；4 轮配置 0.6815° 优于基线。方案已预告“无线搜索可能停在较差局部解”，这里给出的是真实数据，没有恢复 alpha、没有放宽约束、没有改权重。
2. **全步几乎总被拒绝**：B 臂 300/301 周期以 `objective_increase` 结束；A 臂 206 次。原因是用 `online_authority_rollout` 的因果修复把越界全步拉回盒内后，代价反而上升。若后续要改善，方案允许的下一步是“固定信赖域上界 + 重新验证”，而不是恢复四步长搜索。
3. **首拍超预算**：t=0 的 0.575 s / 0.260 s 含 MEX、HPIPM 对象首次创建的冷启动成本；Windows 仿真墙钟耗时也不等于硬件实时运行时间，任何“满足 200 ms 硬实时”的结论都需在目标机上另测。
4. 耗时主项是 `online_authority_rollout`（≈91%），本次按要求**没有**在重构中改动其数值算法（45 次二分、平动预测等留待后续单独优化）。
5. 每周期 `online_authority_rollout` 调用次数为 2–5 次（初始认证 1 次 + 每轮全步 1 次 + 必要时修复 1 次），比旧版（最多 12×(1+4)=60 次）大幅下降，但认证本身从未被跳过或降精度。

## 7. 重跑命令

```matlab
addpath('simulink'); init_paths;
check_online_five_sqp;
check_online_authority_integration;
check_online_authority_static;
check_online_flight_prediction;
check_pitch_authority_scaling;
% 5 轮（默认）
build_closedloop(2,true,false); build_closedloop(2,true,true);
build_closedloop(60,true,false); build_closedloop(60,true,true);
% 4 轮配置对照（写到 simulink/cases，不覆盖生产模型）
build_closedloop(60,true,false,'SQP4','online_predicted',true,4);
build_closedloop(60,true,true ,'SQP4','online_predicted',true,4);
report_five_sqp_performance;                       % 逐周期耗时/分项/早停
report_five_sqp_comparison({'L2_60s_20261104_173125','L2_60s_20261104_173201', ...
 'L2_60s_20261104_201906','L2_60s_20261104_201940', ...
 'L2_60s_20261104_203114','L2_60s_20261104_203147','L2_60s_20261104_204309'});
```

注：本机必须用 `simulink\mlab.cmd` 启动 MATLAB（把 `APPDATA`/`TEMP`/`MATLAB_PREFDIR` 重定向进仓库），直接调用 `matlab.exe` 会因沙箱无法写用户配置而卡死。

## 8. 结果文件

- 耗时与早停：`simulink/results/online_authority/five_sqp/five_sqp_performance.{csv,json}`
- 指标对照：`simulink/results/online_authority/five_sqp/five_sqp_comparison.{csv,json}`
- 单元检查：`.../five_sqp/{A,B}_five_sqp.{json,mat}`、`.../{A,B}_injected_round1.mat`、`.../{A,B}_cold_injected_round3/5.mat`
- 原始 60 s 运行：`.../L2_60s_20261104_201906`（A 5 轮）、`201940`（B 5 轮）、`203114`（A 4 轮）、`203147`（B 4 轮）、`204309`（B 5 轮复算）
