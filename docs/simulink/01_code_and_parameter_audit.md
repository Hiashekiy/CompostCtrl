# Python 迁移基线审计（首批）

2026-11-02：用户允许暂缓许可证恢复，继续不依赖 Aerospace 的迁移。原生 6DOF 第一阶段仍未通过；后续工作不代表完整闭环验收。

## 真实源码定位

| 方案所指对象 | 当前代码 | 迁移结论 |
|---|---|---|
| `cmg_control.py` | 文件不存在；当前为 `torque_gap_rti.py`、`torque_gap_ocp.py` | 迁移以实际 RTI OCP-QP 为准 |
| 刚体真值 | `model6.py`、`model6c.py` | 都是姿态动力学，不是位置速度完整六自由度；不能直接复制后声称完成平动迁移 |
| P0 真值 | `torque_gap_model.py` | B：11 状态、7 控制；A：7 状态、3 控制 |
| CMG 几何 | `cmg.py` | 采用实际 g_axis、s0、gxs0，禁止换用其他金字塔符号 |
| 舵面能力和分配 | `torque_gap_authority.py` | 自然项与舵面增量项分开；分配变量顺序必须保持 |
| 场景、参考和时域 | `torque_gap_scenario.py`、`trajectory3.py` | 参考保留未来 N+1 点，预测网格可能非均匀 |
| P0/P1 执行策略 | `torque_gap_sim.py` | P0 理想总气动力矩，P1 经真实舵面分配和执行机构 |
| HPIPM 加载 | `hpipm_bindings.py` | Windows DLL + Python wrapper；不是 MATLAB MEX |

## 冻结配置

首个基线为 `configs/torque_gap_s08/cases/A4_descent_replay_S00_with_cmg.yaml`。
导出器使用现有 `read_yaml` 解析九级继承链，归档每一级源文件 SHA256、合并配置和实际参数构造器输出。
不要从顶层 `config.yaml` 单独抄参数：实验覆盖后 CMG 框架速率为 **1 rad/s**、加速度为 **3 rad/s²**，不同于顶层的 1.5 和 10。

实际参数：质量 1200 kg，J=diag([1200,8500,9000]) kg·m²，参考面积 4.2 m²，翼展 2.6 m，弦长 3.2 m；4 台 CMG，各 50 N·m·s，速率环时间常数 0.03 s，控制周期 0.2 s，gamma 无机械行程限位。
参数属于现有研究模型，不能据此称为实测飞行器参数。

`vehicle_params.mat` 和 JSON 由 Python 生成；`simulink/data/vehicle_params.m` 只加载数据。完整合并配置保存在 JSON，未混入动力学代码。

## 控制变量和约束

P0 B：x=[q(4),omega(3),gamma(4)]；u=[tau_a(3),nu(4)]。
P0 A：x=[q(4),omega(3)]；u=[tau_a(3)]，不创建 CMG 状态。
P1 `model6c`：x=[q(4),omega(3),delta(3),gamma(4),nu_actual(4)]；u=[delta_cmd(3),nu_cmd(4)]。

tau_a 是**总气动力矩**，已包含自然、阻尼和舵面作用，不能再额外叠加自然力矩。P1 分配应先扣除自然项，得到所需舵面增量，再求偏角。
`surface_matrix` 列顺序为 [升降舵,副翼,方向舵]，行顺序为 [滚转,俯仰,偏航]；模块开头说明中的示意矩阵不能替代实际实现。

RTI 优化的是状态/控制增量，Gauss-Newton Hessian，保留 Q、R、q、r、A、B、b 及逐阶段约束；初始状态增量为零。
非线性 OCP 与 RTI 的加速度约束实现不同，RTI 文件明确说明执行器还要执行相邻命令硬裁剪。不能假定现有 QP 已完整表达所有跨节点约束。
`J_cfg` 已移除，诊断层余缺量不能重新成为目标函数。热启动、求解失败回退还需按所选场景逐项迁移，尚未完成矩阵级等价测试。

## 缺失数据和后续工作

- 当前气动代码主要提供力矩及线性导数，没有足够的三轴气动力数据支撑可信的完整平动控制；不得凭空补出升阻力系数。
- 初始位置、速度、重力、大气范围及任务外环仍需基于真实任务确认；首轮先使用已有姿态参考。
- 现有舵机与 CMG 驱动实现不等于已经完整施加所有物理加速度约束，需要显式核查执行器裁剪策略。
- 完整 QP 固定快照、MATLAB 组装、HPIPM MEX、未来参考导出和 A/B 原生闭环尚未完成。
