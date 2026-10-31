# 基于预测力矩余缺调制的气动舵面–CMG协同姿态与构型优化方法

## 1. 系统动力学与控制变量

考虑配备四台单框架控制力矩陀螺（4-SGCMG）和气动舵面的三轴姿态系统。**保留飞行器姿态与 CMG 构型之间的动力学耦合，但不把舵面偏角和舵机运动纳入优化。** 直接以舵面合成的三轴控制力矩作为虚拟控制输入。

系统状态与控制定义为

$$
\boldsymbol x=\begin{bmatrix}\boldsymbol q^T&\boldsymbol\omega^T&\boldsymbol\gamma^T\end{bmatrix}^T,\qquad
\boldsymbol u=\begin{bmatrix}\boldsymbol\tau_a^T&\boldsymbol\nu^T\end{bmatrix}^T,
$$

其中，$\boldsymbol q\in\mathbb S^3$ 为机体姿态四元数，$\boldsymbol\omega\in\mathbb R^3$ 为机体角速度，$\boldsymbol\gamma\in\mathbb R^4$ 为 CMG 框架角；$\boldsymbol\tau_a\in\mathbb R^3$ 为待优化的气动舵面合成控制力矩，$\boldsymbol\nu=\dot{\boldsymbol\gamma}\in\mathbb R^4$ 为框架角速度。

令第 $i$ 台 CMG 的恒定转子角动量模长为 $h_i$，其在机体系下的单位角动量方向为 $\boldsymbol s_i(\gamma_i)$，框架转轴方向为 $\boldsymbol g_i$，则

$$
\boldsymbol H(\boldsymbol\gamma)=\sum_{i=1}^{4}h_i\boldsymbol s_i(\gamma_i),\qquad
\boldsymbol A(\boldsymbol\gamma)=\begin{bmatrix}h_1(\boldsymbol g_1\times\boldsymbol s_1)&\cdots&h_4(\boldsymbol g_4\times\boldsymbol s_4)\end{bmatrix}.
$$

CMG 由框架运动产生的力矩为 $\boldsymbol\tau_{\rm CMG}=-\boldsymbol A(\boldsymbol\gamma)\boldsymbol\nu$。采用恒定转子自旋、等效刚体惯量 $\boldsymbol J$ 的简化模型，飞行器动力学写为

$$
\boxed{\begin{aligned}
\dot{\boldsymbol q}&=\tfrac12\boldsymbol\Omega(\boldsymbol\omega)\boldsymbol q,\\
\boldsymbol J\dot{\boldsymbol\omega}&=\boldsymbol\tau_a-\boldsymbol A(\boldsymbol\gamma)\boldsymbol\nu+\boldsymbol\tau_d
-\boldsymbol\omega\times[\boldsymbol J\boldsymbol\omega+\boldsymbol H(\boldsymbol\gamma)],\\
\dot{\boldsymbol\gamma}&=\boldsymbol\nu.
\end{aligned}}
$$

其中 $\boldsymbol\tau_d$ 表示其他已知或估计的外力矩。由数值积分得到离散状态转移 $\boldsymbol x_{k+1}=F_{\Delta t}(\boldsymbol x_k,\boldsymbol u_k)$。四元数在积分时保持单位范数。

## 2. 预测气动力矩边界与三轴余缺调制

### 2.1 时变气动力矩约束

根据已知或预测的高度、速度、动压等飞行状态，预先计算预测时域内各时刻的三轴气动力矩上下限：

$$
\boxed{\underline{\boldsymbol\tau}_{a,k}\leq\boldsymbol\tau_{a,k}\leq\overline{\boldsymbol\tau}_{a,k}.}
$$

上下限可正负不对称。主优化器仅使用这些力矩边界，不显式优化舵偏角。初版采用三轴独立的区间近似；若真实舵面存在明显轴间耦合，则用保守的同时可实现力矩范围计算上下限。

### 2.2 需求力矩与调制信息

根据未来参考姿态、参考角速度和角加速度，利用飞行器标称动力学估计每个预测时刻的名义需求控制力矩 $\bar{\boldsymbol\tau}_{d,k}$。该量只用于提取余缺信息，**不预先规定实际气动力矩或 CMG 力矩分配**。

为同时保留力矩大小与正负方向，分别定义各轴相对需求的正向和负向余量：

$$
\boxed{\begin{aligned}
\boldsymbol R_k^+&=\overline{\boldsymbol\tau}_{a,k}-\bar{\boldsymbol\tau}_{d,k},\\
\boldsymbol R_k^-&=\bar{\boldsymbol\tau}_{d,k}-\underline{\boldsymbol\tau}_{a,k}.
\end{aligned}}
$$

$R_{j,k}^+$ 表示该轴气动力矩还能向正方向增加多少，$R_{j,k}^-$ 表示还能向负方向减少多少；**余量为负**表示对应方向已经超出气动可实现区间。

进一步把超出边界的部分提取为带有实际力矩方向的三轴缺口向量：

$$
\boxed{\boldsymbol d_k=\bar{\boldsymbol\tau}_{d,k}-\operatorname{clip}\!\left(\bar{\boldsymbol\tau}_{d,k},\underline{\boldsymbol\tau}_{a,k},\overline{\boldsymbol\tau}_{a,k}\right).}
$$

例如俯仰方向 $d_{y,k}=-60\,{\rm N\cdot m}$，表示未来需要 CMG 额外提供负向俯仰力矩 $60\,{\rm N\cdot m}$；即使 $\boldsymbol d_k=0$，也仍可能存在不同方向的气动富余。由此形成预测调制序列

$$
\boxed{\mathcal M_{0:N}=\left\{\boldsymbol R_k^+,\boldsymbol R_k^-,\boldsymbol d_k\right\}_{k=0}^{N}.}
$$

其中，**富余描述当前和未来准备阶段可用的气动补偿空间；缺口描述未来需要 CMG 承担的力矩大小、方向及出现时刻**。不额外引入转动惯量或舵面效率权重：转动惯量已经体现在需求力矩估计中。

## 3. 余缺调制下的联合轨迹优化

在预测区间 $k=0,\ldots,N-1$，联合优化三轴气动力矩序列 $\{\boldsymbol\tau_{a,k}\}$ 和四维框架速度序列 $\{\boldsymbol\nu_k\}$，由系统动力学同时得到姿态预测轨迹与 CMG 构型轨迹 $\{\boldsymbol\gamma_k\}$。

### 3.1 姿态跟踪主目标

给定参考姿态 $\boldsymbol q_k^{\rm ref}$ 和参考角速度 $\boldsymbol\omega_k^{\rm ref}$，利用姿态误差向量 $\boldsymbol e_{R,k}\in\mathbb R^3$，定义

$$
\boxed{J_{\rm att}=\sum_{k=0}^{N-1}\left(\|\boldsymbol e_{R,k}\|_{Q_R}^2+\|\boldsymbol\omega_k-\boldsymbol\omega_k^{\rm ref}\|_{Q_\omega}^2\right)+\|\boldsymbol e_{R,N}\|_{Q_f}^2.}
$$

姿态跟踪是优化器的首要目标；构型准备只在不明显破坏姿态跟踪的情况下发挥作用。

### 3.2 未来缺口引导的构型准备

仅靠姿态跟踪目标，优化器没有动力在机动发生前主动改变 CMG 构型。为此，将未来力矩缺口转化为**任务相关构型能力代价**。

对于预测时刻 $k$ 的候选构型 $\boldsymbol\gamma_k$，定义其满足缺口 $\boldsymbol d_k$ 的最小力矩误差：

$$
\boxed{E_k(\boldsymbol\gamma_k)=\min_{\boldsymbol v\in\mathcal V_k}\left\|-\boldsymbol A(\boldsymbol\gamma_k)\boldsymbol v-\boldsymbol d_k\right\|^2.}
$$

这里 $\mathcal V_k$ 是遵守 CMG 框架速度限制的候选速度集合；$\boldsymbol v$ **仅用于评价该构型的未来力矩能力**，并非实际施加的控制变量。缺口越容易由该构型产生，$E_k$ 越小。以此建立

$$
\boxed{J_{\rm cfg}=\sum_{k=1}^{N}w_k E_k(\boldsymbol\gamma_k).}
$$

$w_k$ 可统一取 $1$，不需要额外设计复杂调制函数；当 $\boldsymbol d_k=0$ 时，该项自然为零。当预测出现未来缺口时，优化器便有动力提前迁移到更适合该需求的构型。

如果已经存在一条外部生成的构型参考轨迹 $\boldsymbol\gamma_k^{\rm ref}$，也可用较轻量的软引导替代上述能力代价：

$$
J_{\rm cfg}^{\rm ref}=\sum_{k=0}^{N}\|\boldsymbol\gamma_k-\boldsymbol\gamma_k^{\rm ref}\|_{Q_\gamma}^2.
$$

**初版建议优先使用 $J_{\rm cfg}$ 直接引导，不必先构建第二套构型参考规划器。** 构型参考可作为后续加速或改进的可选方案，且不能成为硬约束。

### 3.3 气动富余参与构型迁移

CMG 主动改变框架角时产生力矩 $-\boldsymbol A(\boldsymbol\gamma_k)\boldsymbol\nu_k$。如果当前主要由气动舵面维持姿态，为抵消该迁移力矩，舵面需相应改变输出。三轴气动力矩上下限会自动限制可补偿的迁移动作。

例如在名义总力矩需求附近，若准备阶段希望由气动舵面补偿 CMG 迁移，则其所需力矩近似为

$$
\boldsymbol\tau_{a,k}\approx\bar{\boldsymbol\tau}_{d,k}+\boldsymbol A(\boldsymbol\gamma_k)\boldsymbol\nu_k.
$$

因此 $\boldsymbol R_k^+,\boldsymbol R_k^-$ 可以直接用来理解各方向能够承受多大的迁移补偿；**联合优化中仍以 $\underline{\boldsymbol\tau}_{a,k}\le\boldsymbol\tau_{a,k}\le\overline{\boldsymbol\tau}_{a,k}$ 为实际约束**，无需把相同余量重复写成另一个硬约束。姿态动力学会自动处理实际舵面力矩与 CMG 迁移反作用力矩的协同。

### 3.4 总体目标与约束

为了避免不必要的框架高速运动和气动力矩突变，加入简单正则项

$$
J_{\rm reg}=\sum_{k=0}^{N-1}\left(\|\boldsymbol\nu_k\|^2_{R_\nu}+\|\boldsymbol\nu_k-\boldsymbol\nu_{k-1}\|^2_{R_{\Delta\nu}}+\|\boldsymbol\tau_{a,k}-\boldsymbol\tau_{a,k-1}\|^2_{R_{\Delta a}}\right).
$$

最终问题为

$$
\boxed{\begin{aligned}
\min_{\{\boldsymbol x_k,\boldsymbol\tau_{a,k},\boldsymbol\nu_k\}}\quad&J_{\rm att}+\lambda_c J_{\rm cfg}+J_{\rm reg}\\
\text{s.t.}\quad&\boldsymbol x_{k+1}=F_{\Delta t}(\boldsymbol x_k,\boldsymbol\tau_{a,k},\boldsymbol\nu_k),\\
&\underline{\boldsymbol\tau}_{a,k}\le\boldsymbol\tau_{a,k}\le\overline{\boldsymbol\tau}_{a,k},\\
&\boldsymbol\gamma_{\min}\le\boldsymbol\gamma_k\le\boldsymbol\gamma_{\max},\\
&-\boldsymbol\nu_{\max}\le\boldsymbol\nu_k\le\boldsymbol\nu_{\max},\\
&-\dot{\boldsymbol\nu}_{\max}\Delta t\le\boldsymbol\nu_k-\boldsymbol\nu_{k-1}\le\dot{\boldsymbol\nu}_{\max}\Delta t.
\end{aligned}}
$$

初始状态等于当前测量值，$\boldsymbol\nu_{-1}$ 和 $\boldsymbol\tau_{a,-1}$ 使用上一控制周期的已执行值。若需要体现舵机响应速度，可增加等效气动力矩变化率限制。

**这个优化问题的两个输出是 $\boldsymbol\tau_a^*(t)$ 与 $\boldsymbol\gamma^*(t)$（及其角速度 $\boldsymbol\nu^*(t)$）。** 前者是舵面应承担的三轴力矩，而不是舵偏角；后者是 CMG 的连续构型轨迹。

## 4. 数值求解与执行流程

采用多重射击或直接配点离散化上述动力学，由 SQP 或 IPOPT 等非线性优化求解器求解。算法在每个更新周期执行：

1. **更新预测：** 读取当前姿态、角速度与框架角，获取未来参考姿态及飞行状态。
2. **计算余缺：** 估计名义需求力矩，计算时变气动力矩上下限和三轴 $\boldsymbol R_k^+,\boldsymbol R_k^-,\boldsymbol d_k$。
3. **联合优化：** 在完整姿态–CMG耦合动力学下，以姿态跟踪为主、未来缺口构型能力为辅，求出未来气动力矩与框架角速度序列。
4. **闭环执行：** 执行当前第一段气动力矩指令和框架角速度指令，更新实测状态，并以此前优化结果暖启动下一次求解。

预测时域应包含足够的准备时间和至少一个未来机动窗口；否则，优化器看不到提前迁移构型的收益。$E_k$ 是瞬时能力指标，无法单独保证机动期间的持续输出；完整动力学优化和闭环仿真用于检验连续构型变化后的实际表现。

方法首先只需要比较**不使用 $J_{\rm cfg}$ 的联合姿态优化**与**引入预测缺口引导 $J_{\rm cfg}$ 的联合姿态优化**。两者使用相同的动力学、气动力矩范围和执行机构限制，比较姿态跟踪、未来机动性能、构型提前迁移及舵面力矩轨迹，以确定余缺调制是否确实带来增益。
