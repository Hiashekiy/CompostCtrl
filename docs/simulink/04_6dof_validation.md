# 原生 6DOF 最小验证：实施记录

状态：**脚本已编写，模型创建被许可证阻塞，数值测试未执行。**

入口：`simulink/build_native_stage1.m`。不得将已有 `results/simulink_port` 的 S-Function 移植结果作为本阶段通过证据。

## 设计的解析算例

全部初始姿态为零、角速度为零，质量 10 kg，惯量 diag([2 3 4]) kg·m²，不加入重力或其他外载荷。验证时间 2 s。

| 算例 | 输入与初值 | 理论值 |
|---|---|---|
| 静止 | 力、力矩、初速全零 | 位置、速度、角速度和姿态保持零 |
| 匀速 | 初速 [3,-2,1] m/s；无力、无力矩 | v=v0；x=t v0 |
| 恒力 | 机体系力 [20,0,0] N；无转动 | ax=2 m/s²；vx=2t；x=t² |
| 恒力矩 | 机体系力矩 [0,0,4] N·m；无平动力 | ωz=t rad/s；yaw=t²/2 rad |

脚本计划以固定步长 ode4、步长 0.001 s 执行。该真实库模块内部含连续积分器，因此这一步是**固定步长连续动力学验证**；不是已经完成的全离散闭环迁移。后续控制器采用独立离散采样时间。

每个算例检查整个轨迹，而非只比较终值；位置、速度、角速度、欧拉角误差及内部积分四元数范数误差均要求小于 1e-7（各自 SI 单位；范数无量纲）。

## 计划输出与当前缺口

运行成功后才输出 `simulink/tests/01_6dof_block_validation.slx`，以及 `simulink/results/native_stage1/` 内的 CSV、MAT、误差表、Block 清单和模型图。
目前在首次复制真实库模块时失败，以上验收产物尚不存在；脚本后续部分也尚未经 MATLAB 运行验证，许可证恢复后需要继续调试。

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
