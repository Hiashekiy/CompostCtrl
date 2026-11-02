# 驱动、气动与舵面迁移验证

2026-11-02。Aerospace 许可仍未恢复，本阶段使用基础 Simulink；没有替代或伪造原生 6DOF。

## 模型与接口

| 模型 | 输入 | 输出 |
|---|---|---|
| `CMG_Drive.slx` | nu_cmd，4维 rad/s | 实际 nu、实际加速度、unwrapped gamma |
| `CMG_Cluster.slx` | nu_cmd，4维 rad/s | 实际 nu、加速度、gamma、H、A、反作用力矩 |
| `Surface_Actuator.slx` | delta_cmd=[de,da,dr]，rad | 实际偏角与偏角速度 |
| `Aerodynamic_Model.slx` | [qbar,V,alpha,beta,p,q,r,de,da,dr] | 自然力矩、舵面增量力矩、总力矩 |
| `Aerodynamic_Surface.slx` | delta_cmd；[qbar,V,alpha,beta,p,q,r] | 实际偏角/速度，及三种气动力矩 |

模型保存在 `simulink/models/`。构建入口为 `build_actuators_aero.m` 和 `build_integrations.m`。
所有物理模型仅含可检查的基础模块；对应 inventory CSV 已导出，程序断言无 Fcn、MATLAB Function、S-Function。
气动输入要求 qbar≥0、V≥0、alpha/beta 为物理有效流角；当前没有在模型内部提供完整空气数据估计、密度大气模型、三轴气动力或位置速度外环。

## 驱动离散实现

采样步长 dt=0.001 s；控制周期参数仍为现有 0.2 s，驱动测试使用独立外部输入。

CMG：

```text
cmd = clip(nu_cmd, -nu_max, nu_max)
acc = clip((cmd - nu)/tau_g, -acc_max, acc_max)
nu_next = clip(nu + dt*acc, -nu_max, nu_max)
gamma_next = gamma + dt*nu
```

框架角不折返、不加入 ±90° 限位。参数 nu_max=1 rad/s、acc_max=3 rad/s²、tau_g=0.03 s。
舵机采用同类离散状态更新，tau_s=0.05 s，偏角上限 [25,20,20]°，速率上限 [60,60,60]°/s。
参数来自冻结场景构造器，不由模块内部猜测。

本实现显式限制**实际**加速度与速率。原 `model6c` 一阶微分方程及 P1 执行策略未在同一位置强制全部实际导数约束，因此启用限幅时不能声称与旧 P1 全轨迹等价。
后续闭环应分别报告“保留旧执行策略”与“新物理限幅策略”的影响，不能把差异都归因于 Simulink 移植误差。

## 气动实现

线性导数参数化模型：alpha 与 beta 的静态系数采用 1-D Lookup Table；速率阻尼、舵面导数和参考几何通过 Gain、Mux、Product、Sum 明确构成。
查表断点 [-pi,0,pi]，表值为现有线性导数乘断点，保留原线性模型；不是新增 CFD/实测数据。
V 用 Saturation 保护到至少 1 m/s，保留原计算中的低速分母保护。
自然项、舵面增量项独立输出，总力矩只相加一次。

舵面总力矩指令到偏角的控制分配尚未接入。本阶段输入明确为偏角命令，不能直接把 MPC 的 tau_a 接到 delta_cmd。

## 实际验证

Python 参考生成：`simulink/py/export_actuator_reference.py`；验证随机种子和静态样本来源见此前 baseline 导出器。

1. 8 s、8001 点驱动输入：超限正负命令、换向、未饱和小幅命令，以及初始框架角跨越 2π。比较完整状态与导数轨迹；CMG 和舵面均通过。
2. 96 个独立气动状态与 Python 原气动力矩对照，最大误差 **2.32831e-10 N·m**。
3. 组合 CMG 与 Python：nu 误差 1.1102e-16 rad/s；gamma 5.5911e-13 rad；H 2.2783e-11 N·m·s；反作用力矩 **2.7910e-11 N·m**。
4. 组合气动舵面与 Python：偏角误差 8.1185e-16 rad；总力矩 **5.4570e-12 N·m**。
5. 独立连续一阶解析解：未饱和 0.01 阶跃下，CMG 与舵机离散轨迹最大误差分别为 **6.2179e-5**、**3.7098e-5**；均低于固定 dt Euler 离散容差 7e-5。

驱动参考有限幅递推与模型采用同一数学定义；连续解析解检查提供额外独立验证。没有仅以镜像递推作为全部物理正确性证据。

数值报告：`simulink/results/native_integrations/verification.json`；完整 MAT 日志、真实模型导出 PNG 与 Block 清单保存在相关 results 目录。PNG 并非 GUI 参数对话框截图。

## 复现

```powershell
python simulink/py/export_actuator_reference.py
$env:MATLAB_HOME='D:\Program Files\MATLAB\R2024a'
.\simulink\mlab.cmd "addpath('simulink'); build_actuators_aero; build_integrations; check_integrations"
```

测试中的跨周初始角仅作用于验证运行；保存的生产模型初始框架角为零，实际任务初态后续需统一接入参数字典。
