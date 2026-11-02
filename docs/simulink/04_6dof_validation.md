# 原生 6DOF 最小验证：实施记录

状态：**原生模块数值验证已通过**。许可证恢复后已实际运行四组解析算例。

入口：`simulink/build_native_stage1.m`。不得将已有 `results/simulink_port` 的 S-Function 移植结果作为本阶段通过证据。

## 设计的解析算例

全部初始姿态为零、角速度为零，质量 10 kg，惯量 diag([2 3 4]) kg·m²，不加入重力或其他外载荷。验证时间 2 s。

| 算例 | 输入与初值 | 理论值 |
|---|---|---|
| 静止 | 力、力矩、初速全零 | 位置、速度、角速度和姿态保持零 |
| 匀速 | 初速 [3,-2,1] m/s；无力、无力矩 | v=v0；x=t v0 |
| 恒力 | 机体系力 [20,0,0] N；无转动 | ax=2 m/s²；vx=2t；x=t² |
| 恒力矩 | 机体系力矩 [0,0,4] N·m；无平动力 | ωz=t rad/s；yaw=t²/2 rad |

脚本以固定步长 ode4、步长 0.001 s 执行。该真实库模块内部含连续积分器，因此这一步是固定步长连续动力学验证；控制器采用独立的 0.2 s 离散采样时间。

每个算例检查整个轨迹，而非只比较终值；位置、速度、角速度、欧拉角误差及内部积分四元数范数误差均要求小于 1e-7（各自 SI 单位；范数无量纲）。

## 实际结果与剩余证据

实际模型为 `simulink/tests/native_6dof_validation.slx`，使用有效 MATLAB 模型标识符。数据、误差表、Block 清单、参数与真实模型导出图在 `simulink/results/native_stage1/`。

四组全轨迹最大误差均低于 1e-7：静止全零；匀速位置误差 2.1894e-13 m；恒力速度误差 2.1894e-13 m/s；恒力矩角速度误差 1.0947e-13 rad/s；积分四元数范数误差 2.2982e-14。
库入口经 R2024a 转发后实际 ReferenceBlock 为 `aerolibprivatev1p5/6DoF (Quaternion)`，已记录于 block_parameters.json。没有自建函数替代刚体动力学。

验收还需要 Library Browser、参数对话框、端口的真实 GUI 截图；程序化导出的模型图不能替代参数对话框截图。

## 复现

在项目根目录 PowerShell 执行：

```powershell
$env:MATLAB_HOME='D:\Program Files\MATLAB\R2024a'
.\simulink\mlab.cmd "addpath('simulink'); native_stage1_probe"
.\simulink\mlab.cmd "addpath('simulink'); build_native_stage1"
```

也可在已授权的 MATLAB 桌面切换到项目根目录后执行两个入口函数。
HPIPM 根目录已确认是 `D:\Tools\hpipm`；本阶段未调用、编译或验证 HPIPM。
