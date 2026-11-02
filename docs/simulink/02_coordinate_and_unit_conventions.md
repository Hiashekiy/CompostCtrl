# 迁移坐标、单位与 CMG 等效力矩

来源：`model6.py` 四元数函数、`model6c.py:dynamics6c`、`torque_gap_model.py:dynamics_tg`、`cmg.py`。本文件记录源码约定；Aerospace 原生模块与其四元数方向的数值对照仍待许可恢复。

- 地面局部坐标 NED：北、东、下；机体系：前、右、下，均为右手系。
- 四元数标量在前 [w,x,y,z]，Hamilton 乘法。源码 q 将机体系向量转为 NED：v_N=R(q)v_B；逆变换使用 R(q) 的转置。
- qdot=0.5*q⊗[0,omega_B]；欧拉角采用 ZYX，函数参数顺序 roll,pitch,yaw。
- 误差四元数为 conj(q_ref)⊗q，取标量部分非负的短弧；代价误差向量是 2*vector(q_error)，不是全局精确旋转向量。
- 角、角速度、角加速度为 rad、rad/s、rad/s²；位置、速度、质量、惯量为 m、m/s、kg、kg·m²；力、力矩、角动量为 N、N·m、N·m·s。
- 局部地面坐标高度应采用 -z 加参考高度。完整平动与重力尚未实现，不宣称现有姿态模型已经包含它们。
- 现有气动力矩在机体系；惯性系速度先转入机体系后计算迎角和侧滑。保留源码对低速、反向速度分量的保护，不能无意中换成另一个 atan2 定义。

## 接入工具箱刚体模块的模式 1

原模型：J*omegadot = tau_a + tau_d - A*nu - omega×(J*omega + H)。

工具箱刚体形式：J*omegadot = M_port - omega×(J*omega)。

因此应使用：

**M_port = tau_a + tau_d - A*nu - omega×H**。

其中 -A*nu 是框架运动反作用项，-omega×H 是旋转机体系中的转子角动量耦合修正；不是将转子并入工具箱惯量矩阵。
这是固定转子角动量、固定等效惯量假设下的等效力矩表达，不是完整 CMG 机械系统认证。

源码随机状态代数对照由 `export_native_baseline.py` 执行；尚未与 Aerospace 6DOF 做动态对照。
A 组必须同时移除 A*nu 和 omega×H，并移除 CMG 状态与控制量。只把 nu 设为零会保留陀螺耦合，不是无 CMG 基线。

## CMG 原生几何搭建

每台 i 使用已导出的 s0_i、gxs0_i：

H_i = h0*(s0_i*cos(gamma_i) + gxs0_i*sin(gamma_i))。

A_i = h0*(-s0_i*sin(gamma_i) + gxs0_i*cos(gamma_i))。

H=sum(H_i)，A 按列组装，Hdot=A*nu，tau=-A*nu。框架角保持 unwrapped，跨越 2π 时几何连续。
