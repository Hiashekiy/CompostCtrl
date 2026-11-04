# 直接舵机指令 MPC 对照实验（S0 + S1 交付）

> 依据：`Simulink_MPC_直接舵机指令优化_对照实验方案 (1).md`
> 分支：`experiment/simulink-direct-servo-mpc`（自 `experiment/simulink-mpc-five-sqp` 的 `33f36f7` 起）
> 环境：本机 MATLAB R2024a + Simulink/Aerospace Blockset + 原生 HPIPM MEX；全部数据为本机真实运行。
> **本轮按用户要求只使用 4 轮 SQP 配置**（`max_sqp_iterations=4`，0.2 s 控制周期），不做 5 轮实验。
> 范围：本次交付 **S0（冻结基线 + 分项计时 + 真值舵机核对）与 S1（直接舵机预测/气动力矩模型独立验证）**；
> 方案第 6 节规定“第一次交付只实现 S0 + S1 并提交测试与报告”，S2/S3（14/10 维新 MPC 与 A/B 对照）尚未开始。

## 0. 结论摘要

1. 现有在线 MPC（力矩级 + 在线力矩盒/舵机认证 + 下层分配）在 4 轮配置下重新测量：A 60 s 姿态 RMS 3.8250°、B 1.1098°，**0 次安全回退**，逐周期均值 77.7 / 88.5 ms，P95 132 / 144 ms，最大值 285 / 240 ms。
2. 新写的 1 ms 离散舵机预测器 `servo_predict` 与 **真实 Simulink `Surface_Actuator` 模块**逐拍一致：8 个场景（保持、微步、速率饱和、反向、指令饱和、接近极限、2 s 长保持、0.5 s 中段）姿态最大误差 ≤ 5.0e-16 rad，速率最大误差 ≤ 1.0e-14 rad/s。
3. 气动力矩模型 `online_aero_moment` 与 **真实 `Aerodynamic_Model` 模块**一致：总力矩误差 ≤ 6.0e-11 N·m，`B_delta(qbar)=qbar·diag(S·[b cbar b])·C` 逐位相等（误差 0），自然力矩对机体角速度严格线性。
4. 用 60 s 实跑数据回放：按 1 ms 逐拍预测的舵偏与真值 **RMS 2.9e-17 rad / 峰值 3.6e-16 rad**；力矩历程 RMS 3.0e-2 N·m / 峰值 0.236 N·m（差异只来自控制拍内冻结的动压与角速度，属建模近似）。
5. 同一回放给出方案第 3.3 节禁止做法的量化代价：**若用阶段末端舵偏代表整个阶段，力矩 RMS 0.360 N·m、峰值 5.28 N·m**，即比逐拍历程差 12 倍（RMS）与 22 倍（峰值）。这直接支撑新架构必须建舵机动态、不能用末端值。
6. S1 门槛全部通过，可以进入 S2；新 MPC 尚未集成，生产控制器未被替换。

## 1. S0：冻结与测量

### 1.1 冻结信息

- 接手时工作区存在**并非本次任务产生的**未提交改动（0.1/0.2 s 控制周期参数化、参考窗口重采样、默认 4 轮、DT01/DT02 测量脚本与报告、对应模型）。按方案第 1 节要求原样提交为 `33f36f7 保留工作区已有 DT01/DT02 实验改动`，并在本地 `tmp/preserved_dt01_dt02/` 另存补丁与全部新文件副本；未执行任何回退或覆盖。
- 新分支：`experiment/simulink-direct-servo-mpc`，起点 `33f36f7`；旧控制器（力矩级 + 分配器）、旧 11/7 维 MEX、旧模型全部保留可运行。
- 真值舵机参数（直接读自模型，非文档推断）：`tau_s = 0.05 s`、`delta_max = [0.4363 0.3491 0.3491] rad`、`delta_dot_max = 1.0472 rad/s`、离散时钟 `0.001 s`、模块求解器 `FixedStepDiscrete`、模型固定步长 `0.001 s`。

### 1.2 真值舵机模型核对（方案 S0.3）

`simulink/models/Surface_Actuator.slx` 的实际结构（由 `check_direct_servo_model` 直接读块参数断言，模型改动会立刻报错）：

| 环节 | 模块 | 实测参数 | 与 1 ms 递推式的对应 |
|---|---|---|---|
| 指令限幅 | `Command_limit` | ±`delta_max` | `cmd = clip(delta_cmd, ±delta_max)` |
| 误差放大 | `Inverse_time_constant` | `1/tau_s = 20` | `(cmd − delta)/tau_s` |
| 速率限幅 | `Derivative_limit` | ±`delta_dot_max` | `clip(·, ±delta_dot_max)` |
| 欧拉步 | `Euler_increment` | `0.001` | `delta + 0.001·rate` |
| 状态限幅 | `State_limit` | ±`delta_max` | `clip(..., ±delta_max)` |
| 状态保持 | `Actual_state`（UnitDelay） | `SampleTime = 0.001`，初值可设 | 伺服时钟独立于 MPC 阶段长度 |

结论：真值递推为 `delta[j+1] = clip(delta[j] + 0.001·clip((clip(delta_cmd) − delta[j])/tau_s, ±delta_dot_max))`，与方案 3.2 的式子**逐步一致**；限幅顺序是“先限指令、再限速率、最后限状态”，误差用的是**限幅后的指令**。命令在整段内是零阶保持，舵角在阶段内部持续运动 —— 这正是不能用阶段末端值代替整段的原因。三个舵机共享同一组参数，初值在真值模型中为 0，预测器允许以实测舵偏为初值。

### 1.3 逐周期分项耗时（同一台机器，4 轮，0.2 s，301 周期）

| 运行 | 配置 | 均值 | P95 | 最大 | >200 ms 周期 | 稳态 >200 ms | QP/周期 | QP 求解 | QP 组装+求解 |
|---|---|---|---|---|---|---|---|---|---|
| `L2_60s_20261104_215019` | A（无 CMG） | 77.7 ms | 131.9 ms | 284.6 ms | 3 | 1 | 1.864 | 0.71 ms | 5.36 ms |
| `L2_60s_20261104_215054` | B（含 CMG） | 88.5 ms | 143.7 ms | 239.6 ms | 3 | 2 | 2.183 | 1.34 ms | 7.63 ms |

单次 `online_authority_rollout` 内部各阶段（22 段合计，单位 ms）：

| 阶段 | A | B | 说明 |
|---|---|---|---|
| `air` 飞行工况（大气/动压/迎角） | 0.56 | 0.56 | 查表 + 插值 |
| `aero` 自然力矩与 `B_delta` | 0.23 | 0.23 | 仅矩阵运算 |
| `reachability` 舵机端点（2 次） | 0.60 | 0.59 | 闭式解，很便宜 |
| `box` 内接力矩盒 | 2.10 | 2.07 | |
| `certificate` 八顶点舵机反求（45 次二分×8） | 8.58 | 8.58 | **在线预测内的最大开销** |
| `allocation` 分配到舵偏 | 0.16 | 0.16 | |
| `propagation` 姿态/CMG 传播 | 0.71 | 0.73 | `native_F_B/A` MEX |
| `translation` 平动与未来 `qbar` 预测 | 8.21 | 8.37 | **与认证同量级，方案要求保留** |

其它结论：新加计时为纯附加统计，**不改变数值结果** —— 与未加计时的同配置运行（`212212`/`212322`）逐项相同（姿态 RMS 3.8250/1.1098°、迭代 561/657、拒绝 207/289）。`212212`/`212322` 的均值 166/274 ms 是当时机器另有负载所致，同机复测为 77.7/88.5 ms，报告以复测值为准。

## 2. S1：直接舵机与气动力矩模型独立验证

### 2.1 新增文件

| 文件 | 作用 |
|---|---|
| `simulink/hpipm/servo_predict.m` | 与真值逐块对应的 1 ms 离散舵机递推，输出 `delta(t)`、`delta_dot(t)` 与（可选）逐毫秒气动力矩历程 |
| `simulink/verify/check_direct_servo_model.m` | S1 门槛：真值结构断言、8 个舵机场景对照、气动块/解析式对照、60 s 实跑 1 ms 回放 |
| `simulink/verify/report_direct_servo_s0.m` | S0 分项耗时统计（只读历史诊断，不重写） |
| `simulink/hpipm/online_authority_rollout.m` | 仅新增 `authority.phase_time_s` 分项计时，算术未改 |

### 2.2 舵机预测 vs 真实模块（8 个场景）

| 场景 | 阶段长度 | 舵偏最大误差 (rad) | 速率最大误差 (rad/s) | 端点误差 (rad) | 与 `online_surface_reachability` 端点差 |
|---|---|---|---|---|---|
| 保持不动 | 0.2 s | 0 | 0 | 0 | 0 |
| 小步阶跃 | 0.2 s | 8.7e-19 | 2.8e-17 | 0 | 5.2e-18 |
| 速率饱和 | 0.2 s | 2.4e-16 | 4.8e-15 | 0 | 4.2e-17 |
| 反向运动 | 0.2 s | 3.8e-16 | 7.6e-15 | 0 | 1.0e-17 |
| 指令饱和（3×限幅） | 0.2 s | 2.5e-16 | 2.4e-15 | 0 | 5.8e-16 |
| 接近正向极限反向拉回 | 1 s | 5.0e-16 | 1.0e-14 | 0 | 1.1e-16 |
| 长保持 | 2 s | 3.5e-17 | 2.4e-15 | 0 | 3.5e-16 |
| 中段保持 | 0.5 s | 6.9e-18 | 1.1e-16 | 0 | 6.9e-18 |

误差量级为双精度舍入，说明预测器与真值模块是**同一个递推**，而非近似。既有闭式端点模型 `online_surface_reachability` 与逐拍递推的端点一致（≤5.8e-16），可以继续用于端点可达性判断，但不能把端点当作整段力矩。

### 2.3 气动力矩与 `B_delta`

在 5 个工况点（`qbar` 1e3…6e4 Pa、`V` 600…1500 m/s、`alpha` −0.12…0.12 rad、含 `beta`、非零角速度与舵偏）对照真实 `Aerodynamic_Model`（10 维输入 `[qbar,V,alpha,beta,p,q,r,de,da,dr]`）三个输出口：

- 总气动力矩：最大误差 **6.0e-11 N·m**（真值块的浮点求和顺序差异）。
- 自然力矩分量、舵面分量分别与 `online_aero_moment` 的 `natural`、`B*delta` 相等（同量级误差）。
- `B_delta(qbar) = qbar·diag(S·[b cbar b])·C`：与 `online_aero_moment` 的 `B` **逐元素完全相等（误差 0.0）**。
- 自然力矩对机体角速度严格线性（`natural(2ω) − natural(0) = 2(natural(ω) − natural(0))`，残差 <1e-9）：不存在“冻结角速度”的隐患。

### 2.4 60 s 实跑回放（1 ms 分辨率，301 个控制周期）

数据来源 `L2_60s_20261104_212322`（4 轮、0.2 s、含 CMG）。每个控制周期用**实测舵偏**作为初值、用本期实际下发的分配指令作为保持指令，逐毫秒推进，与真值逐点比较：

| 指标 | 数值 |
|---|---|
| 舵偏轨迹 RMS / 峰值 | **2.9e-17 / 3.6e-16 rad** |
| 力矩历程 RMS / 峰值（逐拍更新动压与角速度） | **3.0e-2 / 0.236 N·m** |
| 力矩 RMS / 峰值（**若用阶段末端舵偏代表整段**） | 0.360 / 5.28 N·m |

结论：
1. 舵机部分是**精确**的（舍入级），可直接用于新的 14/10 维预测模型；
2. 力矩历程误差 3e-2 N·m 来自“控制拍内动压/角速度冻结”，属于可接受的建模近似，且随阶段长度增长；
3. 用末端舵偏代替整段会使力矩误差放大到 0.36 N·m（峰值 5.3 N·m），比逐拍版本差 12–22 倍。方案第 3.3 节的禁令在数值上成立，新 MPC 必须按 1 ms 子步传播舵偏与气动力矩。

## 3. 与方案门槛的对照

| 方案 S1 要求 | 状态 |
|---|---|
| 独立 `servo_predict`（1 ms 高保真参考版） | 完成，`simulink/hpipm/servo_predict.m` |
| 多实测舵偏/目标指令/饱和/反向/接近极限/不同保持时长对照真值与旧端点模型 | 完成，8 场景，误差 ≤5.0e-16 rad |
| `B_delta(qbar)` 与 `online_aero_moment` 一致、`tau_nat` 的角速度依赖 | 完成，B 误差 0.0、线性残差 <1e-9 |
| 同一段执行指令回放，比较每阶段舵偏与力矩时间轨迹 | 完成，301 周期 1 ms 回放 |
| 未达容差前不进入 MPC 集成 | 已满足，S2 尚未开始 |

## 4. 限制与风险（如实记录）

1. **只是 S0+S1**：还没有直接舵机指令 MPC，也没有 A-new/B-new 的 60 s 结果；方案要求的四组对照（A-old/B-old/A-new/B-new）目前只有 old 两组。
2. 力矩回放的 3e-2 N·m 误差是**建模近似**（拍内冻结动压/角速度），不是实现误差；S2 需要决定是否在阶段内也更新平动/动压，代价是 `translation` 阶段已占 8.2 ms/次。
3. 逐周期最大值 285/240 ms 仍超过 200 ms（稳态超预算 1–2 个周期/301），当前控制器**不满足 200 ms 硬实时**；新架构是否更快必须用同一测量口径比较，不能只看均值。
4. 认证（八顶点 45 次二分，8.58 ms）与平动预测（8.21 ms）是当前在线预测的两个主要开销项；按方案第 5 节要求，本次**没有**为了提速而降低认证精度或删掉平动预测。
5. 回放使用的是 4 轮配置的运行数据；按要求本轮不做 5 轮对照。

## 5. 下一步（S2，待确认后开始）

1. 新建 14 维（B）/10 维（A）离散预测动力学与雅可比，逐点用有限差分核对（方案明确禁止复用旧 11/7 维 MEX）。
2. 新 QP 组装 + solver wrapper + 诊断；热启动按绝对时间重采样**舵机指令与 CMG 序列**。
3. 新 controller mode 与独立模型：旁路 `Surface_control_allocation`，MPC 的 `delta_cmd` 直接进真实舵机，CMG 指令走原线路。
4. 首控安全门改为直接舵机状态/指令一致性检查（不再依赖 `authority.tau_min/max`）。
5. 2 s 集成 → 60 s A/B（A-old / B-old / A-new / B-new，全部 4 轮）→ 按方案第 4 节 6 项指标与第 4.7 节四类结论如实判定。

## 6. 重跑命令

```matlab
% 必须用仓库自带启动器（沙箱需重定向 APPDATA/TEMP）
.\simulink\mlab.cmd "addpath('simulink'); init_paths; check_direct_servo_model; report_direct_servo_s0"
% 需要 60 s 基线时（4 轮、0.2 s）：
.\simulink\mlab.cmd "addpath('simulink'); init_paths; build_closedloop(60,true,false,'','online_predicted',true,4,.2); build_closedloop(60,true,true,'','online_predicted',true,4,.2)"
.\simulink\mlab.cmd "addpath('simulink'); init_paths; report_direct_servo_s0"
```

## 7. 结果文件

- S1 验证：`simulink/results/online_authority/direct_servo/direct_servo_model_check.{json,mat}`
- S0 分项耗时：`.../direct_servo/s0_phase_timing.{csv,json}`
- S0 原始 60 s 运行：`.../L2_60s_20261104_215019`（A）、`.../L2_60s_20261104_215054`（B）
- 指标复算：`.../five_sqp/five_sqp_comparison.csv`（含上述两条与 `212212`/`212322` 的并列对照）

## 8. S2/S3：直接舵机指令 MPC 与四组 60 s 对照（已完成）

### 8.1 新增实现

| 文件 | 作用 |
|---|---|
| `simulink/hpipm/direct_servo_state_at.m` | 阶段内任意时刻的 1 ms 舵机闭式状态（与真值递推逐位一致） |
| `simulink/hpipm/direct_servo_stage.m` | 14/10 维单阶段离散映射：20 ms 子步、子步中点舵偏算气动力矩、姿态/CMG 复用 `native_F_*` |
| `simulink/hpipm/direct_servo_propagate.m` | 非线性预测：逐阶段更新动压/迎角、检查舵机指令速率与 CMG 锥、必要时因果修复 |
| `simulink/hpipm/direct_servo_linearize.m` | 解析离散雅可比：姿态块用 `native_A_*/native_B_*`，舵机块与 `B_delta` 链式组合成 14/10 维 |
| `simulink/hpipm/assemble_direct_servo_qp.m` | 新 QP：跟踪代价不变，舵机指令与指令增量按 `delta_max` 归一化正则 |
| `simulink/hpipm/direct_servo_verify.m` | 新安全门：以“真正执行的量”（舵机指令）与其物理包络替代旧力矩盒证书 |
| `simulink/hpipm/direct_servo_mpc_step.m` | 4 轮 SQP、每轮 1 次 HPIPM、一次全步、固定信赖域、首控门与安全回退 |
| `simulink/hpipm/direct_servo_mpc_sfun.m` | 新 S-function：输出直接是 `[delta_cmd; nu_cmd]` |
| `simulink/modelgen/build_closedloop.m` | 新增第 9 个参数 `controller_mode`；`direct_servo` 时旁路 `Surface_control_allocation`，MPC 直接驱动舵机 |
| `simulink/verify/check_direct_servo_mpc.m` | S2 单元门：闭式舵机、阶段映射对 1 ms 参考、雅可比对有限差分、单次 MPC 调用 |

关键数值（`check_direct_servo_mpc`）：

| 检查 | A（10 维/3 维） | B（14 维/7 维） |
|---|---|---|
| 阶段内闭式舵机对 1 ms 递推 | <1e-14 rad | <1e-14 rad |
| 单阶段映射对 1 ms 参考（0.2 s / 2 s） | 8.4e-11 / 8.4e-10 | 2.2e-10 / 1.8e-9 |
| 雅可比对有限差分（最坏相对误差） | 2.6% | 12.9% |

### 8.2 四组 60 s 对照（全部 4 轮 SQP、0.2 s、同一物理模型与参考）

| 组别 | 运行目录 | 姿态 RMS/° | 峰值/° | 回退 | 逐周期均值 | P95 | 最大 | >200 ms 周期 | QP/周期 |
|---|---|---|---|---|---|---|---|---|---|
| A-old（力矩+分配器） | `L2_60s_20261104_215019` | 3.825 | 8.324 | 0 | 77.7 ms | 131.9 ms | 284.6 ms | 3（稳态 1） | 1.86 |
| B-old（力矩+分配器） | `L2_60s_20261104_215054` | 1.110 | 3.616 | 0 | 88.5 ms | 143.7 ms | 239.6 ms | 3（稳态 2） | 2.18 |
| A-new（直接舵机） | `L2_60s_20261104_232924` | **15.463** | 24.858 | 0 | 52.6 ms | 71.4 ms | 442.2 ms | 1 | 1.00 |
| B-new（直接舵机） | `L2_60s_20261104_232950` | **2.184** | 5.352 | 0 | 106.2 ms | 175.6 ms | 291.6 ms | 4 | 2.72 |

结论（按方案 4.7 的分类，不粉饰）：

- **A 组：更快但更差**。每周期少用一半 QP、均值 52.6 ms 对 77.7 ms，但姿态 RMS 从 3.825° 恶化到 15.463°。
- **B 组：更慢且更差**。均值 106.2 ms 对 88.5 ms，姿态 RMS 从 1.110° 恶化到 2.184°。
- 两组均 0 次安全回退、无 NaN/求解崩溃，首控均通过新的直接舵机安全门。

### 8.3 为什么新架构在本轮没有胜出（可复现的诊断）

1. **A 组几乎不接受任何更新**：301 个周期里 `qp_solved` 均值 1.0、`accepted_steps` 为 0，每个周期都在第 1 轮以 `objective_increase` 结束。原因是全步方向虽然被 QP 认为能把代价降到 0（`qp_objective=-76.17`），但非线性预测的代价几乎不变（A 组 `delta_cost` 对 1/32 步长只有 +1.8e-8），即方向对实际影响可忽略；末端代价占主导，舵机在这种量级下无法体现差别。
2. **B 组前 15 s 全部拒绝、之后每周期接受 4 步**：`t<15 s` 时每周期 1 次 QP、0 接受；`t>=15 s` 后多数周期用满 4 轮（`stop=max_iterations`）。这是真实的收敛进展，但也说明冷启动（保持实测舵偏指令）离可行下降方向太远。
3. **QP 过度乐观**：雅可比对有限差分的偏差（B 组 12.9%）在 23.2 s 长时域上累积，QP 预测的改善远大于实际；旧力矩级控制器有同样现象（`objective_increase` 206/300），但它的控制量是力矩，单步幅度小且直接作用于角加速度，所以仍能接受约 1.2 步/周期。
4. **本轮已用的补丁**：对舵机指令加了**固定信赖域** `trust_rad = 0.25·min(delta_dot_max)·0.2 ≈ 0.052 rad`（因为线性舵机块的指数解只在速率限幅未饱和时成立）。这是确定性信赖域，不是 alpha 候选搜索；它让 B 组的更新被接受，但没有扭转精度结论。

### 8.4 未做的事与风险

- 没有实现“舵机时序与真实更新语义完全一致”的进一步细化（阶段内多次 200 ms 指令更新被参数化为单一常值，属保守近似）。
- 没有为了提升新组指标而调权重、改真值、放宽限幅；控制正则权重取旧 `r_dtau`，其影响未做敏感性扫描（方案允许单独报告，本轮未做）。
- A/B-new 的诊断文件名受 S-function 早期版本的文件名缺陷影响（写入 `*_direct_authority.mat` 时前缀异常），数据本身完整；已修正命名，后续运行会写出规范文件名。
- 按方案要求，新架构若更慢或更差则**保留实验分支、不替换生产控制器**；生产模型仍是 `native_mpc_sfun` + 力矩级 MPC。

### 8.5 复现命令

```powershell
# 4 轮、0.2 s、力矩级旧控制器
.\simulink\mlab.cmd "addpath('simulink'); init_paths; build_closedloop(60,true,false,'','online_predicted',true,4,.2); build_closedloop(60,true,true,'','online_predicted',true,4,.2)"
# 4 轮、0.2 s、直接舵机新控制器（新模型变体，不覆盖生产模型）
.\simulink\mlab.cmd "addpath('simulink'); init_paths; build_closedloop(60,true,false,'DIR','online_predicted',true,4,.2,'direct_servo'); build_closedloop(60,true,true,'DIR','online_predicted',true,4,.2,'direct_servo')"
# 指标与耗时
.\simulink\mlab.cmd "addpath('simulink'); init_paths; report_five_sqp_comparison({'L2_60s_20261104_215019','L2_60s_20261104_215054','L2_60s_20261104_232924','L2_60s_20261104_232950'}); report_direct_servo_s0"
```