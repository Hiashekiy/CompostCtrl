# 第三章 基于预测优化的气动舵面–CMG 协同姿态控制方法

## 3.1 问题描述与总体框架

第二章建立了高层大气飞行条件下的飞行器姿态动力学模型，并描述了气动舵面与四台单框架控制力矩陀螺（Single-Gimbal Control Moment Gyroscope，SGCMG）的作用机理。气动舵面的控制能力随飞行高度、速度及大气状态变化；CMG 则通过改变框架角度调整集群角动量方向，并利用框架运动产生控制力矩。因此，在气动控制能力随飞行环境变化的过程中，仅依据当前姿态误差进行控制分配，难以充分利用未来姿态参考和执行机构之间的动态耦合关系。

本文采用统一的模型预测控制（Model Predictive Control，MPC）框架，将飞行器姿态、角速度和 CMG 框架角作为预测状态，将气动控制力矩与 CMG 框架角速度作为联合优化输入。在每个控制周期内，优化器根据当前测量状态、未来姿态参考、外部扰动力矩以及预测气动控制力矩边界，求解整个预测时域内的状态和控制序列。实际执行时仅施加当前时刻的控制输入，随后根据更新后的状态重新构造并求解优化问题。

与先独立计算姿态控制力矩、再通过控制分配器将其分配给舵面和 CMG 的串级方式不同，本文将两类执行机构的输入直接纳入同一个有限时域优化问题。CMG 框架运动对机体产生的力矩及其角动量耦合均包含在预测动力学中，气动控制力矩则受到随飞行状态变化的可实现边界约束。由此，优化器能够在满足姿态跟踪和执行机构约束的前提下，联合确定舵面力矩与 CMG 框架运动。

需要指出，本文不额外设置 CMG 构型参考、角动量预置奖励或瞬时力矩能力引导项。CMG 构型是系统状态的一部分，其变化由框架角速度输入和运动学关系决定；不同构型对后续姿态控制的影响，则通过角动量雅可比矩阵及完整刚体动力学体现。若预测优化过程中出现机动前的构型迁移，该行为应理解为未来姿态跟踪需求、耦合动力学、气动控制边界和控制正则项共同作用下的优化结果，而不是由独立的构型引导目标直接指定。

本文方法的主要计算过程如下：

1. 读取当前姿态四元数、机体角速度和 CMG 框架角，构造预测问题的初始状态。
2. 根据给定弹道和预测时间网格，生成未来姿态参考、角速度参考、外部扰动力矩及各预测节点的气动控制力矩边界。
3. 建立包含气动舵面和 CMG 的联合姿态预测模型，并构造姿态跟踪与控制正则化目标。
4. 在动力学、气动控制力矩、框架角速度及框架角加速度等约束下，求解有限时域非线性优化问题。
5. 执行最优控制序列的首个控制输入，更新系统状态，并进入下一控制周期。

## 3.2 舵面–CMG 联合预测模型

### 3.2.1 状态变量与控制变量

考虑由飞行器本体和四台 SGCMG 构成的姿态控制系统。为在预测过程中同时描述飞行器姿态运动和 CMG 构型变化，定义系统状态为

$$
\boldsymbol x_k=
\begin{bmatrix}
\boldsymbol q_k^{\mathsf T}&
\boldsymbol\omega_k^{\mathsf T}&
\boldsymbol\gamma_k^{\mathsf T}
\end{bmatrix}^{\mathsf T}
\in\mathbb R^{11},
$$

其中，$\boldsymbol q_k\in\mathbb R^4$ 为飞行器姿态四元数，$\boldsymbol\omega_k\in\mathbb R^3$ 为机体角速度，$\boldsymbol\gamma_k\in\mathbb R^4$ 为四台 CMG 的框架角。

定义联合控制输入为

$$
\boldsymbol u_k=
\begin{bmatrix}
\boldsymbol\tau_{a,k}^{\mathsf T}&
\boldsymbol\nu_k^{\mathsf T}
\end{bmatrix}^{\mathsf T}
\in\mathbb R^7,
$$

其中，$\boldsymbol\tau_{a,k}\in\mathbb R^3$ 为气动舵面提供的三轴等效控制力矩，$\boldsymbol\nu_k\in\mathbb R^4$ 为四台 CMG 的框架角速度。

这里将气动控制力矩直接作为优化变量，而不将舵面偏角和舵机内部状态纳入主优化问题。这样可以在保留气动控制能力边界的同时，避免将舵面分配和伺服动态引入非线性规划变量。对于采用真实舵面执行模型的验证工况，优化得到的等效气动力矩还需要经过舵面分配与执行机构模型，才能得到实际施加于飞行器的力矩。

### 3.2.2 姿态与 CMG 耦合动力学

根据第二章建立的刚体姿态动力学，系统姿态运动学方程为

$$
\dot{\boldsymbol q}
=
\frac{1}{2}
\boldsymbol q\otimes
\begin{bmatrix}
0\\
\boldsymbol\omega
\end{bmatrix},
$$

其中，$\otimes$ 表示四元数乘法。

四台 SGCMG 的集群角动量记为 $\boldsymbol H(\boldsymbol\gamma)$，角动量雅可比矩阵定义为

$$
\boldsymbol A(\boldsymbol\gamma)
=
\frac{\partial\boldsymbol H(\boldsymbol\gamma)}
{\partial\boldsymbol\gamma}
\in\mathbb R^{3\times4}.
$$

由于框架角速度满足 $\dot{\boldsymbol\gamma}=\boldsymbol\nu$，根据角动量变化关系，CMG 对机体产生的控制力矩为

$$
\boldsymbol\tau_{\mathrm{CMG}}
=
-\boldsymbol A(\boldsymbol\gamma)\boldsymbol\nu.
$$

因此，考虑气动舵面控制力矩、外部扰动力矩及 CMG 集群角动量耦合后，飞行器刚体转动动力学为

$$
\boxed{
\boldsymbol J\dot{\boldsymbol\omega}
=
\boldsymbol\tau_a
-\boldsymbol A(\boldsymbol\gamma)\boldsymbol\nu
+\boldsymbol\tau_d
-\boldsymbol\omega\times
\left(
\boldsymbol J\boldsymbol\omega+
\boldsymbol H(\boldsymbol\gamma)
\right)
}
$$

其中，$\boldsymbol J$ 为飞行器本体转动惯量矩阵，$\boldsymbol\tau_d$ 为外部扰动力矩。结合框架运动学方程

$$
\dot{\boldsymbol\gamma}=\boldsymbol\nu,
$$

即可得到完整的连续时间预测模型：

$$
\boxed{
\begin{aligned}
\dot{\boldsymbol q}
&=
\frac{1}{2}\boldsymbol q\otimes
\begin{bmatrix}0\\\boldsymbol\omega\end{bmatrix},\\
\dot{\boldsymbol\omega}
&=
\boldsymbol J^{-1}
\left[
\boldsymbol\tau_a
-\boldsymbol A(\boldsymbol\gamma)\boldsymbol\nu
+\boldsymbol\tau_d
-\boldsymbol\omega\times
\left(\boldsymbol J\boldsymbol\omega+
\boldsymbol H(\boldsymbol\gamma)\right)
\right],\\
\dot{\boldsymbol\gamma}
&=\boldsymbol\nu.
\end{aligned}
}
$$

该模型体现了舵面与 CMG 在姿态控制中的耦合关系。一方面，气动舵面通过 $\boldsymbol\tau_a$ 直接作用于飞行器本体；另一方面，CMG 的控制力矩不仅取决于框架角速度，还取决于当前框架构型对应的雅可比矩阵。框架运动改变 $\boldsymbol\gamma$，进而改变后续时刻的 $\boldsymbol A(\boldsymbol\gamma)$ 和 $\boldsymbol H(\boldsymbol\gamma)$，使当前控制输入对未来姿态运动产生持续影响。

### 3.2.3 时变气动控制力矩边界

气动舵面的可用控制能力随飞行状态变化。为使预测模型反映不同高度和速度下的控制权限，在每个预测节点根据对应飞行状态计算三轴气动控制力矩上下界：

$$
\underline{\boldsymbol\tau}_{a,k}
\leq
\boldsymbol\tau_{a,k}
\leq
\overline{\boldsymbol\tau}_{a,k}.
$$

其中，$\underline{\boldsymbol\tau}_{a,k}$ 和 $\overline{\boldsymbol\tau}_{a,k}$ 分别为第 $k$ 个预测节点各轴气动控制力矩的下界和上界。将其写成分量形式，即

$$
\begin{aligned}
\tau_{a,x,k}^{\min}
&\leq \tau_{a,x,k}\leq\tau_{a,x,k}^{\max},\\
\tau_{a,y,k}^{\min}
&\leq \tau_{a,y,k}\leq\tau_{a,y,k}^{\max},\\
\tau_{a,z,k}^{\min}
&\leq \tau_{a,z,k}\leq\tau_{a,z,k}^{\max}.
\end{aligned}
$$

上述边界由预测飞行状态和气动控制能力模型确定，因此不同预测节点的控制力矩范围可以不同，也不要求关于零点对称。该约束将气动舵面的时变控制权限直接纳入联合优化：当某一轴的气动控制能力下降时，优化器能够在预测动力学允许的范围内调整舵面力矩和 CMG 框架运动。

需要说明的是，当前主优化采用逐轴气动力矩区间描述舵面能力，其本质是三轴力矩盒近似。该表示不自动保证三个轴的边界极值能够由真实舵面同时实现。因此，采用真实舵面分配与伺服模型的交叉验证时，还需检查优化输出的等效力矩是否能够由实际舵面实现。

## 3.3 有限时域联合优化问题

### 3.3.1 预测序列与优化变量

在当前控制时刻，设预测区间包含 $N$ 个控制区间和 $N+1$ 个状态节点。优化器求解状态序列

$$
\boldsymbol X=
\{\boldsymbol x_0,\boldsymbol x_1,\ldots,\boldsymbol x_N\},
$$

以及气动控制力矩和 CMG 框架角速度序列

$$
\boldsymbol U=
\left\{
\boldsymbol\tau_{a,0},\ldots,\boldsymbol\tau_{a,N-1},
\boldsymbol\nu_0,\ldots,\boldsymbol\nu_{N-1}
\right\}.
$$

因此，有限时域优化变量可表示为

$$
\mathcal Z=
\left\{
\boldsymbol X,\boldsymbol U
\right\}.
$$

初始状态由当前测量值固定：

$$
\boldsymbol x_0=\boldsymbol x(t).
$$

预测时域内的参考姿态、参考角速度、外部扰动力矩和气动控制力矩边界均由当前时刻对应的预测数据给定。由于控制周期和预测节点间隔可以不同，以下使用 $\Delta t_k$ 表示第 $k$ 个预测区间的实际时间长度。

### 3.3.2 姿态与角速度跟踪代价

本文以姿态跟踪为主要控制目标。为避免直接使用欧拉角差描述三维姿态误差，采用参考四元数与预测四元数构造相对姿态误差。定义

$$
\boldsymbol q_{e,k}
=
(\boldsymbol q_k^{\mathrm{ref}})^{-1}
\otimes\boldsymbol q_k,
$$

并取其向量部分构造三维姿态误差：

$$
\boxed{
\boldsymbol e_{R,k}
=
2\,\operatorname{vec}(\boldsymbol q_{e,k})
}
$$

其中，$\operatorname{vec}(\cdot)$ 表示提取四元数的三个虚部。角速度误差定义为

$$
\boldsymbol e_{\omega,k}
=
\boldsymbol\omega_k-\boldsymbol\omega_k^{\mathrm{ref}}.
$$

据此，预测区间内的姿态和角速度跟踪代价为

$$
\boxed{
\begin{aligned}
J_{\mathrm{att}}
=&
\sum_{k=0}^{N-1}
\frac{\Delta t_k}{T}
\left(
w_{\mathrm{att}}
\frac{\|\boldsymbol e_{R,k}\|_2^2}{e_s^2}
+
w_{\mathrm{rate}}
\frac{\|\boldsymbol e_{\omega,k}\|_2^2}{\omega_s^2}
\right)\\
&+
q_{\mathrm{terminal}}
\frac{\|\boldsymbol e_{R,N}\|_2^2}{e_s^2},
\end{aligned}
}
$$

其中，$T=\sum_{k=0}^{N-1}\Delta t_k$ 为预测时域总长度；$e_s$ 和 $\omega_s$ 分别为姿态误差与角速度误差的归一化尺度；$w_{\mathrm{att}}$、$w_{\mathrm{rate}}$ 和 $q_{\mathrm{terminal}}$ 为对应权重。

该代价同时考虑预测区间内的姿态和角速度跟踪误差，并通过终端姿态项约束预测末端的姿态偏差。阶段代价乘以 $\Delta t_k/T$，使不同时间长度的预测区间按照其实际持续时间参与累计，避免仅因预测网格节点数量变化而改变阶段代价的整体尺度。终端项只包含姿态误差，不额外引入终端角速度代价。

### 3.3.3 控制输入正则化

在满足姿态跟踪要求的基础上，还需要避免不必要的框架运动和气动力矩指令突变。为此，对 CMG 框架角速度、相邻节点框架角速度变化以及相邻节点气动力矩变化进行正则化。

定义控制正则项为

$$
\boxed{
\begin{aligned}
J_{\mathrm{reg}}
=
\sum_{k=0}^{N-1}\frac{\Delta t_k}{T}
\Bigg[
&
r_\nu
\frac{\|\boldsymbol\nu_k\|_2^2}{\nu_s^2}
+
r_{\Delta\nu}
\frac{\|\boldsymbol\nu_k-\boldsymbol\nu_{k-1}\|_2^2}{\nu_s^2}\\
&+
r_{\Delta\tau}
\frac{\|\boldsymbol\tau_{a,k}-\boldsymbol\tau_{a,k-1}\|_2^2}{d_s^2}
\Bigg],
\end{aligned}
}
$$

其中，$\nu_s$ 和 $d_s$ 分别为框架角速度与气动力矩变化的归一化尺度；$r_\nu$、$r_{\Delta\nu}$ 和 $r_{\Delta\tau}$ 分别为框架角速度、框架角速度变化和气动力矩变化的权重。

当 $k=0$ 时，式中的上一时刻控制输入采用上一控制周期实际执行的输入，即

$$
\boldsymbol\nu_{-1}=\boldsymbol\nu_{\mathrm{prev}},
\qquad
\boldsymbol\tau_{a,-1}=\boldsymbol\tau_{a,\mathrm{prev}}.
$$

该项对框架角速度本身进行惩罚，同时抑制框架角速度和气动力矩指令的快速变化。需要注意，当前实现没有单独设置 CMG 输出力矩幅值代价，也没有设置气动力矩幅值惩罚项；气动力矩的可行范围由前述时变边界约束保证。

### 3.3.4 完整目标函数

综合姿态跟踪和控制输入正则化，当前联合优化问题的目标函数为

$$
\boxed{
J=J_{\mathrm{att}}+J_{\mathrm{reg}}.
}
$$

其中，$J_{\mathrm{att}}$ 负责约束预测时域内的姿态与角速度跟踪性能，$J_{\mathrm{reg}}$ 用于限制框架运动和控制输入变化。两部分共同决定预测时域内的状态和控制序列。

这里需要明确，当前目标函数中没有独立的 CMG 构型优化项，也没有角动量储备目标、最小奇异值目标或基于未来力矩缺口的能力匹配代价。CMG 构型并非通过额外的参考轨迹或奖励项进行指定，而是作为状态变量参与动力学预测，并通过其对 CMG 力矩和角动量耦合的影响间接作用于姿态跟踪代价。

### 3.3.5 动力学与执行机构约束

优化状态必须满足初始状态约束和离散动力学约束：

$$
\boldsymbol x_0=\boldsymbol x(t),
$$

$$
\boxed{
\boldsymbol x_{k+1}
=
F_{\Delta t_k}
\left(
\boldsymbol x_k,\boldsymbol u_k,\boldsymbol\tau_{d,k}
\right),
\quad k=0,\ldots,N-1,
}
$$

其中，$F_{\Delta t_k}(\cdot)$ 表示对第二节连续时间耦合动力学在区间 $\Delta t_k$ 内进行数值积分得到的状态转移映射。当前实现采用四阶 Runge–Kutta（RK4）方法进行积分，并在积分步后对姿态四元数进行归一化处理。

气动控制力矩满足逐预测节点的时变边界：

$$
\underline{\boldsymbol\tau}_{a,k}
\leq
\boldsymbol\tau_{a,k}
\leq
\overline{\boldsymbol\tau}_{a,k}.
$$

CMG 框架角速度满足

$$
-\dot{\boldsymbol\gamma}_{\max}
\leq
\boldsymbol\nu_k
\leq
\dot{\boldsymbol\gamma}_{\max},
$$

其中，$\dot{\boldsymbol\gamma}_{\max}$ 为框架角速度上限。考虑框架角加速度限制，相邻预测节点的框架角速度变化满足

$$
-\ddot{\boldsymbol\gamma}_{\max}\Delta t_k
\leq
\boldsymbol\nu_k-\boldsymbol\nu_{k-1}
\leq
\ddot{\boldsymbol\gamma}_{\max}\Delta t_k.
$$

当 $k=0$ 时，$\boldsymbol\nu_{-1}$ 取上一控制周期实际执行的框架角速度。框架角由运动学关系递推：

$$
\boldsymbol\gamma_{k+1}
=
\boldsymbol\gamma_k+\int_{t_k}^{t_{k+1}}\boldsymbol\nu(\tau)\,d\tau,
$$

其具体数值更新包含在 RK4 状态转移过程中。若实验配置启用有限框架角行程，还需施加

$$
\boldsymbol\gamma_{\min}
\leq
\boldsymbol\gamma_k
\leq
\boldsymbol\gamma_{\max}.
$$

对于采用展开角且不限制框架行程的配置，该边界不启用，不能将其描述为所有实验均实施的约束。

四元数需满足单位范数条件：

$$
\|\boldsymbol q_k\|_2^2=1.
$$

当前求解器默认在各预测节点施加该等式约束，同时 RK4 积分后的四元数也进行归一化。若配置改变四元数约束节点范围，应以实际求解配置为准。

此外，代码保留了可选的气动力矩变化率约束：

$$
-\dot{\boldsymbol\tau}_{a,\max}\Delta t_k
\leq
\boldsymbol\tau_{a,k}-\boldsymbol\tau_{a,k-1}
\leq
\dot{\boldsymbol\tau}_{a,\max}\Delta t_k.
$$

该约束仅在配置提供相应力矩变化率上限时启用，不应与始终存在的气动力矩幅值边界混为一谈。

综上，当前有限时域联合优化问题可写为

$$
\boxed{
\begin{aligned}
\min_{\boldsymbol X,\boldsymbol U}\quad&
J_{\mathrm{att}}+J_{\mathrm{reg}}\\
\mathrm{s.t.}\quad&
\boldsymbol x_0=\boldsymbol x(t),\\
&
\boldsymbol x_{k+1}
=
F_{\Delta t_k}
(\boldsymbol x_k,\boldsymbol u_k,\boldsymbol\tau_{d,k}),\\
&
\underline{\boldsymbol\tau}_{a,k}
\leq\boldsymbol\tau_{a,k}
\leq\overline{\boldsymbol\tau}_{a,k},\\
&
-\dot{\boldsymbol\gamma}_{\max}
\leq\boldsymbol\nu_k
\leq\dot{\boldsymbol\gamma}_{\max},\\
&
-\ddot{\boldsymbol\gamma}_{\max}\Delta t_k
\leq
\boldsymbol\nu_k-\boldsymbol\nu_{k-1}
\leq
\ddot{\boldsymbol\gamma}_{\max}\Delta t_k,\\
&
\|\boldsymbol q_k\|_2^2=1,\\
&
\boldsymbol\gamma_{\min}\leq\boldsymbol\gamma_k
\leq\boldsymbol\gamma_{\max}
\quad\text{（仅在启用有限行程时）}.
\end{aligned}
}
$$

其中，气动力矩变化率约束在相应配置启用时一并加入。该问题以当前测量状态作为初始条件，在完整耦合动力学约束下同时求解气动控制力矩和 CMG 框架角速度。

## 3.4 非线性规划求解与滚动闭环控制

### 3.4.1 多重射击离散与非线性规划构建

将预测时域离散后，本文采用多重射击（Multiple Shooting）方式构建非线性规划问题。与单纯通过控制序列连续向前积分得到全部预测状态不同，多重射击将各预测节点的状态均作为优化变量，并通过离散动力学等式约束连接相邻节点。

对于每个预测区间，使用 RK4 方法计算状态转移：

$$
\boldsymbol x_{k+1}^{\mathrm{pred}}
=
F_{\Delta t_k}
(\boldsymbol x_k,\boldsymbol u_k,\boldsymbol\tau_{d,k}).
$$

随后施加节点间的动力学缺陷约束：

$$
\boldsymbol x_{k+1}
-
\boldsymbol x_{k+1}^{\mathrm{pred}}
=
\boldsymbol 0.
$$

这种处理使预测状态和控制输入能够在同一个非线性规划问题中联合调整，同时将系统动力学、执行机构边界及控制平滑要求统一表示为优化约束和目标函数。

当前实现使用 CasADi 构建非线性规划表达式，并调用 IPOPT 求解器进行数值优化。求解变量包括全部预测状态以及各预测区间的气动力矩和框架角速度。对于有限行程模式，框架角上下界直接施加于对应状态变量；对于无限行程模式，则不设置该位置边界。

### 3.4.2 初值设置与热启动

滚动优化过程中，相邻控制周期的初始状态通常较为接近，因此可以利用上一周期的优化结果构造当前问题的初始猜测。设上一周期的预测状态和控制序列分别为 $\boldsymbol X^{*}$ 和 $\boldsymbol U^{*}$，在下一控制周期将序列向前平移一个预测节点，形成新的初始猜测。

由于当前实测状态可能与上一周期的预测状态存在偏差，热启动后仍需将当前测量状态重新固定为初始节点：

$$
\boldsymbol x_0=\boldsymbol x(t).
$$

其余预测节点和控制输入则由平移后的解提供初值。若没有可用的历史解，则根据当前测量状态和初始控制猜测进行冷启动。当前实现中，名义需求力矩可参与冷启动控制初值构造，但它不属于主优化目标函数，也不构成独立的构型引导项。

热启动的目的在于利用相邻控制周期优化问题的连续性，减少重复求解的计算负担。若某一周期求解失败，当前实现还可以根据配置保留并平移上一可用解，供后续周期继续使用。

### 3.4.3 首段控制执行与状态更新

在当前控制时刻求得完整控制序列后，仅执行第一个控制区间的控制输入：

$$
\boxed{
\boldsymbol u(t)=\boldsymbol u_0^{*}
=
\begin{bmatrix}
(\boldsymbol\tau_{a,0}^{*})^{\mathsf T}&
(\boldsymbol\nu_0^{*})^{\mathsf T}
\end{bmatrix}^{\mathsf T}.
}
$$

在当前控制周期内，系统根据实际执行输入和外部扰动进行状态积分，得到下一时刻的姿态、角速度和 CMG 框架角。随后重新读取状态、更新预测参考和气动控制力矩边界，再次构造并求解有限时域优化问题。

因此，整个闭环过程可以表示为

$$
\text{状态测量}
\rightarrow
\text{未来参考与边界更新}
\rightarrow
\text{联合优化}
\rightarrow
\text{首段控制执行}
\rightarrow
\text{状态更新}.
$$

通过不断重复上述过程，优化器能够利用当前预测窗口内的未来姿态参考，同时根据每个控制周期获得的新状态修正后续控制序列。预测时域长度决定了优化器能够提前看到多远的未来参考，但并不意味着预测窗口之外的任务需求能够直接影响当前优化结果。

### 3.4.4 求解结果验收与后备控制

非线性规划求解器可能出现未收敛、约束违反或计算时间超过设定上限等情况。为避免仅根据求解器返回的状态字符串判断解是否可执行，当前实现同时检查约束最大违反量、目标函数数值有效性以及求解耗时。

当求解结果满足可行性容差、目标函数为有限值且求解时间未超过设定上限时，可以接受该解并执行首段控制。即使求解器没有返回严格意义上的正常收敛状态，只要结果满足上述可行性和时间要求，也可以按配置接受该可行解。

当优化结果不满足验收条件时，系统进入后备控制流程。若存在上一周期可用控制输入，则优先对其进行当前气动边界、框架角速度和框架角加速度限制检查；若没有可用历史输入，则采用姿态稳定控制，并在 CMG 存在时根据相应的有界最小二乘计算构造备用框架角速度。

需要区分正常 OCP 与后备控制的计算路径：当前正常优化问题不使用预测力矩缺口作为目标函数或约束；部分后备控制分支可以使用诊断计算得到的当前缺口辅助构造备用框架角速度。这一处理仅在优化结果无法使用时触发，不应被描述为正常 MPC 优化器中的构型调制机制。

实验记录应保留每个控制周期的求解状态、约束最大违反量、求解时间、是否接受可行解、是否触发后备控制及后备控制类型。求解失败和后备控制样本均属于闭环控制性能的一部分，不应从实验统计中剔除。

## 3.5 预测优化下的 CMG 构型演化机理

### 3.5.1 未来姿态参考对当前优化的影响

当前联合优化问题在每个控制周期内同时考虑整个预测时域的姿态和角速度跟踪误差。设预测时域内给定的参考序列为

$$
\left\{
\boldsymbol q_k^{\mathrm{ref}},
\boldsymbol\omega_k^{\mathrm{ref}}
\right\}_{k=0}^{N},
$$

则优化器需要选择当前及未来的气动力矩和框架角速度，使预测状态尽可能接近该参考序列。

由于 CMG 框架角属于预测状态，当前框架角速度不仅影响当前时刻的 CMG 输出力矩，还会通过

$$
\boldsymbol\gamma_{k+1}
=
\boldsymbol\gamma_k+
\int_{t_k}^{t_{k+1}}\boldsymbol\nu(\tau)\,d\tau
$$

改变后续预测节点的构型。构型变化进一步改变 $\boldsymbol A(\boldsymbol\gamma)$ 和 $\boldsymbol H(\boldsymbol\gamma)$，从而影响未来节点的姿态动力学。

因此，即使未来姿态机动尚未发生，只要该机动已经进入当前预测窗口，其参考信息就会参与当前优化问题。优化器可能通过调整当前框架角速度，使后续预测节点处于另一种 CMG 构型，以改善整个预测时域内的姿态跟踪结果。这种由未来参考影响当前控制输入的现象，是有限时域预测优化的前瞻性来源。

### 3.5.2 构型迁移与舵面力矩的协同作用

CMG 框架迁移并非不影响飞行器本体的内部构型调整。根据刚体动力学，框架角速度会通过

$$
-\boldsymbol A(\boldsymbol\gamma)\boldsymbol\nu
$$

对机体产生反作用力矩。因此，在调整 CMG 构型的同时，优化器必须考虑该力矩对当前及后续姿态运动的影响。

由于气动力矩 $\boldsymbol\tau_a$ 与框架角速度 $\boldsymbol\nu$ 是同一优化问题中的控制变量，优化器可以在气动控制力矩边界允许的范围内协调两者，使预测姿态尽可能满足参考要求。对于某些任务，舵面可能在框架迁移阶段提供与 CMG 反作用力矩相抵消的力矩；但这种分工并非由预设的舵面优先规则或专门的迁移补偿项决定，而是由姿态跟踪目标、动力学关系、控制输入正则化和执行机构约束共同形成。

因此，当前方法不规定必须提前迁移，也不保证每个任务都会产生相同的构型变化。是否迁移、何时迁移以及迁移幅度，取决于预测时域内姿态参考与当前系统状态之间的关系，以及相应控制输入是否满足动力学和执行机构约束。

### 3.5.3 角动量变化与瞬时力矩能力的区别

CMG 构型变化会改变集群角动量

$$
\boldsymbol H=\boldsymbol H(\boldsymbol\gamma),
$$

而 CMG 对机体的瞬时控制力矩为

$$
\boldsymbol\tau_{\mathrm{CMG}}
=
-\boldsymbol A(\boldsymbol\gamma)\boldsymbol\nu.
$$

二者具有不同的物理含义。$\boldsymbol H$ 描述转子角动量在机体系中的合成结果，其单位为 $\mathrm{N\cdot m\cdot s}$；$\boldsymbol\tau_{\mathrm{CMG}}$ 描述框架运动产生的瞬时控制力矩，单位为 $\mathrm{N\cdot m}$。角动量在某一方向上的增大，并不意味着该方向的瞬时力矩输出能力必然增大。

同时，瞬时力矩能力还与当前构型下的雅可比矩阵、框架角速度限制及框架运动约束有关。因此，分析构型迁移时，应分别考察框架角变化、角动量变化、实际 CMG 输出力矩和姿态跟踪结果，不能仅凭某一角动量分量的变化推断瞬时控制能力提高。

### 3.5.4 预测时域对构型迁移的作用边界

未来姿态参考只有在进入当前预测时域后，才能直接影响当前 OCP 的目标函数。若未来机动尚未进入预测窗口，优化器无法通过当前这次有限时域优化直接利用该机动参考作出针对性调整；随着控制时刻向前推进，预测窗口逐渐接近未来机动，相关参考信息才会进入优化问题。

因此，预测时域长度会影响未来参考对当前控制决策的作用时间范围。较长的预测时域能够更早包含未来姿态机动，但也会增加优化变量数量和求解负担；较短的预测时域则可能推迟未来机动对当前框架运动的影响。预测时域的选择需要结合机动持续时间、预测模型精度和求解器计算能力确定。

需要强调，预测时域能够提供未来参考信息，并不等价于优化器一定会提前迁移构型。构型迁移还受到姿态跟踪权重、框架运动正则项、气动力矩可行边界和系统动力学的共同影响。是否存在稳定、可重复的提前迁移行为，应通过改变预测窗口内可见的未来参考并进行闭环对照实验验证，而不能仅凭单次运行的构型曲线作出因果判断。

## 3.6 本章小结

本章建立了基于模型预测控制的气动舵面–CMG 联合姿态控制方法。首先，将飞行器姿态四元数、机体角速度和 CMG 框架角定义为预测状态，将三轴气动控制力矩和四台 CMG 框架角速度定义为联合优化输入，并建立包含 CMG 力矩和角动量耦合的姿态预测模型。随后，根据预测飞行状态设置时变气动控制力矩边界，并构造由姿态跟踪、角速度跟踪、终端姿态误差和控制输入正则化组成的有限时域优化目标。在此基础上，将离散动力学、气动控制力矩边界、框架角速度及框架角加速度等条件纳入非线性规划，通过多重射击和滚动优化实现闭环控制。

与在姿态控制器之后单独进行执行机构分配的方式不同，本文在统一优化问题中同时求解气动舵面力矩和 CMG 框架角速度，使 CMG 构型变化对未来姿态运动的影响能够直接参与预测优化。由于未来姿态参考在预测窗口内共同参与目标函数，优化器可能在机动发生前调整 CMG 构型；但该行为并非由显式构型引导或角动量预置目标强制产生，其出现条件和实际控制收益需要通过后续对照实验加以验证。
