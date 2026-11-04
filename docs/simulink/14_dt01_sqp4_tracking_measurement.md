# 0.1 秒控制周期、最多 4 轮 SQP 闭环测量

日期：2026-11-04。分支：`experiment/simulink-mpc-five-sqp`，基于 `d236358` 的工作区修改，未提交或推送。

## 配置与测量范围

- MATLAB R2024a、Simulink/Aerospace Blockset、原生 HPIPM。本机真实串行 A/B 各 60 秒，正式结果每臂 601 拍，无 Python 参与闭环。
- MPC 控制周期为 0.1 秒，最多 4 轮 SQP，保持已有全步更新和早停规则；显式 5 轮实验配置仍可使用。
- 6DOF 与舵机物理积分步长保持 0.001 秒。这里的 0.1 秒是控制更新周期，不是把连续物理模型的积分粗化到 0.1 秒。
- 控制器、舵面分配器和上一拍分配指令 UnitDelay 同步为 0.1 秒。分配器的命令变化限幅也按实际控制周期计算。
- 22 段预测网格只把首段从 0.2 秒改为 0.1 秒，其他阶段不变；预测总时长从 23.2 秒变为 23.1 秒。参考窗口同时按新控制时刻和预测节点重采样，四元数重新归一化。
- 原始物理扰动时间表保持逐值一致，仍按原始数据时间表保持输入。预测使用的新参考窗口独立重采样。认证精度、代价权重、气动参数、舵机及 CMG 真值模型均未调整。
- 力矩比较为同一仿真时刻的 MPC 三轴气动力矩指令与气动模块实际总力矩（自然力矩＋舵面力矩），不含 CMG 力矩。未做延迟对齐。
- 每轴相对误差 = 100 × RMS(指令−实际)/RMS(指令)；综合相对误差 = 100 × ||三轴误差||F/||三轴指令||F。姿态误差为参考和实际四元数之间的最小旋转角，参考插值口径与历史报告一致。
- 完整 MPC 时间为 `native_online_rti_step` 入口到出口的 tic/toc 墙钟时间，包含预测、认证、装配、HPIPM 和首控门；HPIPM 内部时间单列为每次 QP 的 time_ext。统计包含冷启动，不剔除超时样本。

## 60 秒闭环对照

| 控制周期 | 配置 | 力矩相对误差：滚转/俯仰/偏航 | 综合相对误差 | 姿态 RMS/峰值（度） |
|---|---|---|---|---|
| 0.2 s | A | 14.750% / 11.520% / 8.686% | 11.521% | 3.8250 / 8.3244 |
| 0.2 s | B | 12.609% / 11.744% / 10.352% | 11.695% | 0.6815 / 2.4490 |
| 0.1 s | A | 10.831% / 6.462% / 7.537% | 6.522% | 3.9005 / 8.4369 |
| 0.1 s | B | 9.887% / 6.776% / 5.336% | 6.801% | 1.2047 / 3.7301 |

A 为无 CMG；B 为含 CMG。0.2 秒对照也使用最多 4 轮 SQP。新配置降低了力矩跟踪误差，但姿态 RMS 在 A/B 上都变差；不能由力矩误差下降推断姿态精度同步改善。

| 控制周期 | 配置 | 完整 MPC 均值/中位数/P95/最大（ms） | 超过本周期预算 | HPIPM 内部均值/QP（ms） |
|---|---|---|---|---|
| 0.2 s | A | 75.67 / 70.94 / 125.19 / 659.88 | 4/301（1.33%） | 0.375 |
| 0.2 s | B | 85.34 / 76.72 / 142.43 / 234.20 | 4/301（1.33%） | 0.559 |
| 0.1 s | A | 171.65 / 178.79 / 335.98 / 924.29 | 366/601（60.90%） | 0.668 |
| 0.1 s | B | 197.00 / 158.70 / 409.53 / 529.68 | 461/601（76.71%） | 1.061 |

所有四组均无回退，QP/拍最大值为 4。正式新结果的力矩顶点认证残差不超过 3.56e-15 N·m，舵机顶点残差不超过 3.00e-15 rad，俯仰轴缩放始终为 1。

本次 0.1 秒配置未满足每拍 100 ms 预算，而且超时不只发生在首拍。墙钟计时受到本机运行状态影响，历史与本次不是受控硬件实时基准，不能将耗时差异全部归因于控制周期。同步 Simulink 仿真在求解时暂停仿真时间，因此这里的跟踪精度也不包含真实部署时错过截止时刻带来的延迟。

## 实际文件与调用链

- 生产模型：`simulink/models/NearSpace_CMG_FlightControl_SurfaceOnly.slx`（A）、`simulink/models/NearSpace_CMG_FlightControl.slx`（B），已保存 60 秒 / 0.1 秒 / 4 轮配置。
- 入口：`simulink/verify/run_dt01_sqp4.m` → `simulink/modelgen/build_closedloop.m`。
- 求解：`native_mpc_sfun.m` → `native_rti_step.m` → `native_online_rti_step.m` → `assemble_native_qp.m` / `solve_hpipm_snapshot.m`。
- 周期与分配：`native_mpc_sfun.m`、`native_surface_allocation_sfun.m`、模型中的 `Previous_Allocation_Command`。
- 参考重采样：`simulink/hpipm/online_resample_reference_bank.m`。
- 指标统计：`simulink/verify/report_dt01_sqp4.m`。已有五轮统计脚本的周期检查和超时预算也改为读取实际周期。
- 保存配置核验：`check_dt01_configuration.m` 编译后检查控制器和分配器真实采样周期，检查原始扰动时间表、延迟、四元数、预测首段与迭代上限。编译生命周期日志写到独立 configuration 目录，避免覆盖原始闭环日志。

## 原始数据

- A / 0.2 s：`simulink/results/online_authority/L2_60s_20261104_203114/`。该目录包含闭环结果 MAT 与 `*_authority.mat` 逐拍计时、QP 和认证数据。
- B / 0.2 s：`simulink/results/online_authority/L2_60s_20261104_203147/`。该目录包含闭环结果 MAT 与 `*_authority.mat` 逐拍计时、QP 和认证数据。
- A / 0.1 s：`simulink/results/online_authority/L2_60s_20261104_211309/`。该目录包含闭环结果 MAT 与 `*_authority.mat` 逐拍计时、QP 和认证数据。
- B / 0.1 s：`simulink/results/online_authority/L2_60s_20261104_211514/`。该目录包含闭环结果 MAT 与 `*_authority.mat` 逐拍计时、QP 和认证数据。

- 汇总：`simulink/results/online_authority/dt01_sqp4/comparison.json`、`comparison.csv`。
- 指标明细：同目录 `*_metrics.mat`，包含逐时刻力矩误差、姿态误差与逐拍耗时。
- 正式结果采用 `211309`（A）和 `211514`（B）。`210932` / `211026` 是重采样物理扰动的中间实验，未纳入正式结论；2 秒短跑也只用于接线与启动验证。

## 复现

```matlab
cd('D:\项目\飞轮');
addpath('simulink'); init_paths;
run_dt01_sqp4;
check_dt01_configuration;
check_online_five_sqp('dt01_sqp4/regression');
```

若只需重算正式四组指标，不重新仿真：

```matlab
report_dt01_sqp4({'L2_60s_20261104_203114','L2_60s_20261104_203147', ...
 'L2_60s_20261104_211309','L2_60s_20261104_211514'});
```

本机启动方式：设置 `MATLAB_HOME=D:\Program Files\MATLAB\R2024a` 后使用 `simulink\mlab.cmd`。

## 验证结果

- A/B 2 秒启动验证通过。
- 正式 A/B 60 秒均完成，各 601 拍，0 回退，最多 4 次 QP/拍。
- `check_dt01_configuration`：两份已保存生产模型均通过，编译采样周期为 0.1 秒，物理扰动时间表与原始数据逐值一致。
- `check_online_five_sqp('dt01_sqp4/regression')`：默认 4 轮、显式 5 轮、非法配置与故障注入检查均通过。
- 核验配置后再次读取正式原始日志，A/B 的 601 拍诊断数据均完整保留。
- `git diff --check` 通过。

## 剩余实现限制

本次只调整更新周期与迭代配置、对齐参考并实测。分配器仍以上一拍指令为变化范围中心；姿态预测仍使用理想阶段力矩，舵机动态与该姿态预测之间的一致性问题仍存在。不能把本次误差下降解释为这些问题已修复。
