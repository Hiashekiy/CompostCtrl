# CMG 原生模块验证：几何部分已通过

2026-11-02。本次按用户后续授权，在 Aerospace 许可恢复前，继续独立的基础 Simulink 子系统迁移。

后续更新：驱动动态与几何连接已实现为 `simulink/models/CMG_Cluster.slx`，8001 点运行及 Python 组合对照通过；详见 `05_aerodynamic_surface_validation.md`。下文“尚未完成”列表记录前一轮仅几何阶段的状态，已由该后续验证覆盖框架积分、实际速率/加速度限幅与驱动时间常数；摩擦、-omega×H 和原生 6DOF 连接仍待实现。

## 已交付

- 原生模型：`simulink/models/CMG_Geometry.slx`。
- 可重建入口：`simulink/modelgen/build_cmg_geometry.m`。
- 参数、参考解：`simulink/data/native_baseline/A4_descent_replay_S00_with_cmg/`。
- 误差表、模型图、Block 清单：`simulink/results/native_cmg_geometry/`。

模型直接由 Constant、Trigonometric Function、Gain、Sum、Product、Mux、Reshape、To Workspace 组成。
没有 MATLAB Function、Fcn、S-Function 或隐藏 MEX。四台 CMG 的 sin/cos、H_i、A_i、A_i*nu_i 逐台可见。
增益向量直接从配置导出的真实几何常量写入模型，因此保存的模型不依赖 base workspace 才能打开。

这是**代数几何验证模型**，Constant 为可修改的测试输入，输出为 To Workspace；尚不是可直接接入完整闭环的端口化 CMG_Cluster 模型。

## 实际数值测试

MATLAB R2024a、基础 Simulink，固定步长离散求解器，StopTime=0 验证静态映射。
共 67 个输入配置：home、64 组 gamma ∈ [-4π,4π] 随机配置、2π 两侧相邻配置；nu 在实际 ±1 rad/s 范围内采样。
逐样本修改模型测试输入，调用 `sim` 读取输出，与 Python `CMGCluster` 对照。

| 输出 | 最大绝对误差 |
|---|---:|
| H，N·m·s | 2.8422e-14 |
| A，N·m·s/rad | 1.4211e-14 |
| tau，N·m | 1.4211e-14 |

全部低于阈值 1e-10。详细逐样本误差见 `errors.csv`。

Python 参考数据生成前另做独立性质检查：

- A 与 H 的中心有限差分导数最大差 4.2146e-8，阈值 1e-6；
- gamma 整体加 2π 的 H 周期误差 1.8504e-13，阈值 1e-10；
- 等效 CMG 力矩代数式与 P0 原动力学角加速度差 1.5266e-16，阈值 1e-12。

这些检查证明几何代数部分与参考实现一致；不证明驱动动态、机械模型或 Aerospace 闭环一致。

## 尚未完成

框架角积分、框架速率/加速度限制、驱动时间常数、无限行程动态穿越、摩擦，以及 -omega×H 与原生刚体的实际连接均待实施。
未交付完整 `CMG_Cluster.slx`，未将部分成果命名为整套 CMG 已完成。
模型 PNG 为 Simulink 程序导出的真实模型图，不是 GUI 参数对话框截图。

## 复现

项目根目录运行 `python simulink/py/export_native_baseline.py`。
MATLAB 中 `addpath('simulink'); build_cmg_geometry` 重建模型并重新执行全部静态对照。
