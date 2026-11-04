# 独立俯仰轴被统一标定缩放的问题修复

本轮用户明确要求修复 `calibrate_scale`；因此修改 Python 标定/组盒源代码，并同步修复正式 MATLAB 在线盒。旧 MAT、旧 Python 运行结果和冻结基线未重新生成。

## 根因与修改

俯仰行只使用升降舵，而滚转/偏航共享副翼和方向舵。统一缩放的目的在于处理滚转/偏航的耦合，但把同一因子乘到俯仰半宽，会无必要地裁掉升降舵能力。

- `nearspace/torque_gap_authority.py` 新增 `authority_axis_scales`，当前构型返回 `[scale,1,scale]`。`realizable_fraction` 与 `build_boxes` 使用同一组分轴缩放；`calibrate_scale` 保留原标量返回接口，但其含义是耦合轴比例。独立轴保持 1。自然力矩中心不缩放。
- `nearspace/torque_gap_scenario.py` 将三个实际比例写入标定元数据，避免只记录一个 0.774 引起误解。
- `simulink/hpipm/online_certified_torque_box.m` 根据哪些输出轴共享舵面自动划分耦合组，逐组求内接比例。独立俯仰保留完整区间；满秩、秩亏和全耦合情况仍分别做八顶点认证。
- `online_authority_rollout.m` 记录每节点三个实际比例。兼容字段 `scale` 现在只是各轴比例的最小值，不能解释为仍统一缩放三个轴。
- `export_online_node_diagnostics.m` 导出 `scale_roll,scale_pitch,scale_yaw`。

这修复了上一版 MATLAB 内接盒同样存在的问题：它没有调用旧 Python `calibrate_scale`，但其统一约 0.753 的缩放也误伤了俯仰。

## 临界配平示例

给定 `hi_full=+0.55`、`hi_scaled=-1.55` 和 `s=0.774`，可推得舵面半宽为 9.292035 N·m、自然力矩中心为 -8.742035 N·m。恢复俯仰比例为 1 后，上界恢复为 +0.55 N·m。测试将这组数作为明确的代数回归，不冒充完整真值工作点回放。

另外使用当前车辆 MAT、α=32°、零角速度、V=3640 m/s，选择动压使完整俯仰上界为 +0.55 N·m：Q=46.8939 Pa，旧统一 0.774 会给出 -27.4175 N·m，新版本恢复 +0.55 N·m。与用户所述 -1.55 的差异来自未给全的工作点参数，不能混称为相同飞行状态。两个测试均保留严格的顶点可达性检查。

## Python 冷启动的相关回归

恢复完整俯仰区间后，已有 B 组 IPOPT 回归暴露初始状态/控制不一致：先以未裁剪前馈力矩 rollout X，再裁剪 U 传给求解器。旧统一缩放下该测试通过，完整俯仰下出现 `Maximum_Iterations_Exceeded`。现在先按当前节点边界裁剪初始 U，再生成 X；原回归恢复通过。未改变求解器参数、目标函数或控制维度，并新增逐阶段 shooting 一致性测试。

## 为什么之前仍有约 7–18% 的相对误差

之前百分比是 `RMS(cmd-actual)/RMS(cmd)`。滚转指令 RMS 只有约 1 N·m，因此 0.15–0.20 N·m 的绝对误差就会表现为 14–18%。缩小能力盒会使控制器低估可用力矩，但不意味着实际输出直接乘上该因子。

同一时刻误差还包含真实舵机的一阶、速率限制和工作点变化。上一版 60 s 日志在固定窗口下进行 0–200 ms 对齐诊断，最佳延迟为：

| 组 | 滚转 / ms | 俯仰 / ms | 偏航 / ms |
|---|---:|---:|---:|
| A | 74 | 60 | 63 |
| B | 56 | 59 | 59 |

该延迟与 50 ms 舵机时间常数相近。俯仰归一化 RMS 在对齐诊断中 A 从 6.956% 下降到 4.028%，B 从 9.268% 下降到 5.282%，支持动态滞后是残差来源之一。**这些对齐数值不是修复后的实际闭环误差，也不能替代同时刻指标。** 数据在 `pitch_scale_repair/previous_lag_diagnostic.mat`。

## 验证入口与证据

- Python：`python -m unittest discover -s tests -p test_pitch_authority_scale.py -v`。
- Python 原有 A/B 回归及新增冷启动检查：为 `.pydeps` 配置 PYTHONPATH 后运行 `python -u tests/test_torque_gap_no_cmg.py`。
- MATLAB：`check_pitch_authority_scaling; check_online_authority_static; check_online_authority_integration`。
- 闭环：分别调用 `build_closedloop(2,true,enabled)` 和 `build_closedloop(60,true,enabled)`，A/B 新结果单独保存。
- 最终同时刻相对误差、峰值、俯仰比例范围及动态对齐诊断由 `report_pitch_scale_repair` 输出至 `simulink/results/online_authority/pitch_scale_repair/closedloop_report.json`。

上一轮的 18 组表格仍是提交 `468c398` 时的历史结果；本轮新结果单独报告，不把旧表格冒充修复后的闭环验证。

## 本轮最终结果

2 项独立 Python 回归、18 项原有/扩展 A/B 回归全部通过；MATLAB 静态、单次 MPC、2 s 和 60 s A/B、保存重开及 CMG 回归均通过。60 s 两组回退均为 0，所有记录节点的俯仰比例与 1 相差不超过 1e-10，八顶点与舵机证书继续通过。

下表为同一时刻误差 RMS 除以该轴指令 RMS，单位 %：

| 组 | 滚转修复前→后 | 俯仰修复前→后 | 偏航修复前→后 | 三轴综合修复前→后 |
|---|---:|---:|---:|---:|
| A | 14.50→17.37 | 6.96→8.51 | 7.98→11.88 | 7.17→8.73 |
| B | 18.49→11.84 | 9.27→9.10 | 8.60→7.74 | 9.67→9.10 |

姿态 RMS：A 从 4.0073° 降到 3.8232°，B 从 1.1781° 降到 0.7826°；峰值分别从 8.3105°/4.1443° 降到 8.0854°/2.6669°。

A 的俯仰指令 RMS 从 7.9303 增至 8.4551 N·m，误差 RMS 从 0.5516 增至 0.7196 N·m，因此其比例确实上升。B 的俯仰误差 RMS 从 0.5104 增至 0.5348 N·m，但指令 RMS 也从 5.5080 增至 5.8806 N·m，因此比例小幅下降。不能把误差百分比单独当作能力盒物理正确性的判据。

修复后仍可观察到约 51–78 ms 的动态对齐时滞。原姿态预测 MEX 依旧采用理想阶段力矩，真实舵机仍有一阶、速率及角度限制；本次修改恢复正确的可行域，没有将该动态模型近似消除。对齐后的诊断误差没有用于上表。

当前原始结果：A `L2_60s_20261104_173125`，B `L2_60s_20261104_173201`。详细数据见 `pitch_scale_repair/closedloop_report.json` 和两份 `*_metrics.mat`。旧数据保持历史含义，不覆盖旧对照表。计算仍不满足 200 ms 硬实时预算。
