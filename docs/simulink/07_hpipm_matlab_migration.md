# HPIPM MATLAB / Simulink 接口迁移

2026-11-02。本阶段实际编译、实际求解，没有用 quadprog 替代 HPIPM。

## 本机构建

- MATLAB R2024a，Windows x64，已选择 MinGW64 C 编译器。
- HPIPM：`D:/Tools/hpipm`；BLASFEO：`D:/Tools/blasfeo`。
- 构建输出 `simulink/hpipm/mex/`：18 个官方 OCP-QP MEX 包装器，以及本地只读验证工具 `hpipm_qp_read.mexw64`。
- Windows 导入库需显式指定 `libhpipm.dll.a` 与 `libblasfeo.dll.a`，MEX 的 `-lhpipm` 默认搜索没有识别此文件名。
- 编译器版本不在 MathWorks 支持列表中，编译有警告；实际构建及求解已通过。不能据此声明所有编译器版本、实时目标或代码生成环境兼容。

构建脚本不修改 `D:/Tools`，所有产物写入本项目。运行时仍需以上两个安装目录中的 DLL 及官方 MATLAB 类文件。

## 真实 QP 一致性

输入并非玩具二次函数：`export_hpipm_qp.py` 从现有 `TorqueGapRTI.from_config` 及 S00 B 初始规划，导出实际 RK4 线性化、Gauss-Newton Hessian、梯度、力矩/框架速率边界、状态信赖域。
N=22，nx=11，nu=7，非均匀预测步长完整保存。
初始状态增量固定为零；终端包含状态代价与状态约束，没有虚构终端控制变量。

| 检查 | 实测 |
|---|---:|
| MATLAB 求解状态 | 0 |
| Python 求解状态 | 0 |
| 写入 HPIPM 后逐阶段矩阵/向量读回最大差 | 0 |
| 状态增量序列差 | 0 |
| 控制增量序列差 | 0 |
| 目标函数值差 | 5.68434e-14 |
| 最大 stationarity residual | 2.98406e-7 |
| 等式 residual | 5.55112e-17 |
| 不等式 residual | 5.13057e-12 |
| complementarity residual | 1.67492e-7 |
| 本次 MATLAB 原生求解调用时间 | 约 0.001025 s |

读回检查覆盖 A、B、b、Q、q、R、r、lu、uu；采用 typed C getters，因为当前本机官方通用 getter 仅支持 A 和部分状态界限。状态约束结构由同一快照逐项设置，状态边界与信赖域矩阵尚未扩展到全部 native readback。
上述调用时间不包括 MATLAB 对象创建、组装、验证和 Simulink 调度，不能用于宣称控制周期或实时性能达标。

数据与报告：`simulink/tests/05_python_matlab_qp_equivalence/`。

## Simulink 调用

`simulink/models/HPIPM_Interface.slx`：显式 Packed_OCP_QP 输入，Level-2 MATLAB S-Function 算法块调用 MATLAB/MEX，输出首个控制增量、求解状态和四项残差。
所有阶段数值通过输入端口传递，维度及状态信赖域模板来自 production_qp.mat。
序列化字段顺序为 A、B、b、Q、q、R、r、lu、uu，均 MATLAB 列主序；各字段尺寸见 MAT。
在 0 与 0.2 s 两个采样时刻调用，首控制增量与 Python 差 **0**，状态均成功。

S-Function 仅承载允许的算法接口，不含任何飞行器、CMG、舵机或气动物理模型。此解释执行通路**尚不支持已验证的代码生成或实时部署**。
目前每次求解创建/释放官方 HPIPM 对象，不保留热启动。QP 数据流的输入维度固定，不能在运行中切换 A/B 的状态维数。

此模型是 QP 求解接口，不是完整 `Attitude_MPC.slx`。在线 LTV/QP 组装、热启动、超时/失败回退和绝对控制命令提取仍需迁移。
失败时会输出状态，本阶段尚未配置安全的控制回退链，不能直接接入真实执行器。

## 复现

```powershell
python simulink/py/export_hpipm_qp.py
$env:MATLAB_HOME='D:\Program Files\MATLAB\R2024a'
.\simulink\mlab.cmd "addpath('simulink/hpipm'); build_hpipm_mex; check_hpipm_snapshot; build_hpipm_interface"
```

MATLAB 内先 `addpath('simulink/hpipm'); hpipm_startup`，再加载需要使用的接口模型。
