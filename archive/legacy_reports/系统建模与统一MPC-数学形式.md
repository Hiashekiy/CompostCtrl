# 近空间稀薄大气飞行器复合姿态控制
## 系统建模与统一 MPC 优化问题（论文数学底稿 v1）

> **用途**：作为论文 *Problem Formulation / System Modeling* 与 *Unified MPC Controller* 两节的数学底稿；同时补齐审稿意见 2 要求的“详细建模方法 + 气动系数/插值/马赫-高度依赖 + 不确定性 + MPC 细节”。
> **阅读约定**：公式与英文段落可直接进论文；中文为落地实现说明与审稿答复要点。所有 `[FILL]` 必须用飞行器实际数据替换，不能留空——审稿人明确抱怨“没有给出任何细节、惯性矩阵数值也未提供”。
> **核心结构变化**：取消“指令力矩 $\to$ 分配 $\to$ 舵/飞轮”的链式分配层，MPC 决策变量直接是 $\delta_c$ 与 $M_{rw,c}$；角动量管理不再靠模式切换，而靠**预测时域内的控制裕度目标 + 角动量代价**。

---

## 0. 相对原方案必须修正的 6 处（否则论文会站不住）

| # | 原方案 | 问题 | 修正 |
|---|---|---|---|
| 1 | $\rho=\rho_0 e^{-h/H}$，$\rho_0=1.225$，$H=7500$ m 覆盖 20–100 km | 单一指数律只在 0–20 km 成立；20–100 km 的等效标尺高度在 5.5–8.4 km 间变化，且 80 km 以上稀薄效应、太阳活动导致密度可差 2–5 倍 | §2 分层指数 + Knudsen 修正 + 不确定性带 |
| 2 | $p_k=[\rho,V,Ma,\alpha]$，把 $\alpha$ 当外生参数 | 俯仰通道中 $\alpha=\theta-\gamma(t)$ 依赖姿态状态；若当外生，$M_a$ 对 $\theta$ 无依赖，闭环稳定性与“姿态跟踪”语义都变了 | §2.5、§4：$\alpha=\theta-\gamma(t)$ 写进模型 |
| 3 | 60–240 N·m 飞轮力矩需求 | 飞轮力矩受**功率**约束 $M=P/\Omega$；5000 rpm 下 60–240 N·m 需 31–126 kW，工程上不成立 | §5.3、§10.3：给出力矩–转速包线与功率核算，重定 RW 定位 |
| 4 | 裕度里出现 $M_{req}$、$M_{rw,max}(h)$ 但未定义 | 无法计算、无法复现 | §7.4：显式定义 $M_{req}$、$M_{rw,avail}(h_w)$、$\Gamma^\pm$ |
| 5 | $N=40$、50 Hz $\Rightarrow$ 0.8 s 预测 | 密度 e 折时间尺度约 30 s，0.8 s 根本“预见”不到跨高度气动能力下降；“主动预偏置”的物理主张不成立 | §8：非均匀长时域（≈65 s）或双时间尺度 |
| 6 | 统一状态空间 | 未说明可解性 | §6.3：输入仿射且 $B$ 恒定 $\Rightarrow$ 在线 QP/SQP 可行；$h_w$ 只经**约束**影响闭环，必须靠代价/约束管理 |

---

## 1. 符号、坐标系与假设

### 1.1 主要符号

| 符号 | 含义 | 单位 |
|---|---|---|
| $\theta,\ q$ | 俯仰角、俯仰角速度 | rad, rad/s |
| $\gamma,\ \alpha$ | 航迹倾角、攻角 $\alpha=\theta-\gamma$ | rad |
| $I_y$ | 俯仰转动惯量 | kg·m² |
| $M_a,M_{rw},M_d$ | 气动、飞轮反作用、扰动力矩 | N·m |
| $\delta,\ \delta_c$ | 舵面实际偏角、指令偏角 | rad |
| $M_{rw,c}$ | 飞轮力矩指令 | N·m |
| $h_w,\ \Omega_w,\ J_w$ | 飞轮角动量、转速、转子惯量 | N·m·s, rad/s, kg·m² |
| $\rho,V,a,Ma,q_\infty$ | 密度、速度、声速、马赫数、动压 | kg/m³, m/s, m/s, –, Pa |
| $S,\ \bar c$ | 参考面积、平均气动弦长 | m², m |
| $C_{m0},C_{m\alpha},C_{mq},C_{m\delta}$ | 俯仰力矩系数及其导数 | –, 1/rad, –, 1/rad |
| $Q$ | 气动系数→力矩的换算因子 $Q=\tfrac12\rho V^2S\bar c$ | N·m |
| $\Gamma^\pm,\ \eta$ | 控制裕度、最坏情况裕度变量 | N·m |
| $N,\ \Delta t_k$ | 预测步数、第 $k$ 步时长 | –, s |

### 1.2 正方向约定（必须写进论文，否则符号全是坑）

* $\theta>0$ 抬头，$q>0$ 抬头，$\gamma>0$ 爬升，$\alpha=\theta-\gamma$。
* $\delta>0$ 定义为**产生抬头正力矩**的偏转方向（后缘下偏）；$C_{m\delta}>0$。
* $M_{rw}$ 定义为**飞轮作用于机体的反作用力矩**，$M_{rw}>0$ 抬头。
* 飞轮角动量 $h_w=J_w\Omega_w$，$\Omega_w>0$ 为设计正转方向。

### 1.3 假设

* **A1** 俯仰通道解耦（论文单通道；三轴扩展见 §9）。
* **A2** 短周期近似：忽略气动力对质心平动的反馈耦合，$V(t),h(t),\gamma(t)$ 由 skip 弹道给定（“轨迹跟踪 + 姿态稳定”问题）。
* **A3** $I_y$ 常数（忽略燃料消耗/弹性；不确定性在 §7.6 处理）。
* **A4** 舵机一阶、飞轮力矩环一阶；采样零阶保持。
* **A5** 气动系数为 $(Ma,\alpha,Kn)$ 的查表函数，标称值确定、含乘性不确定度。
* **A6** 飞轮摩擦、电机损耗与时滞折算为 $\tau_w$ 与集总扰动 $M_d$。
* **A7** 状态可测/可估：$\theta,q,\delta,M_{rw}$ 直接量测，$h_w$ 由转速 $\Omega_w$ 换算。
* **A8** 无故障标称；故障/饱和保护在 §11 作为独立算例。

---

## 2. 稀薄大气环境模型

### 2.1 分层指数大气（20–100 km）

$$
\rho(h)=\rho_i\exp\!\left[-\frac{h-h_i}{H_i}\right],\qquad h\in[h_i,h_{i+1}],\qquad
H_i=\frac{h_{i+1}-h_i}{\ln(\rho_i/\rho_{i+1})}.
$$

由 US Standard 1976 拟合的分层参数（可直接用于仿真与论文表格）：

| 层 (km) | 下界参考密度 $\rho_i$ (kg/m³) | 标尺高度 $H_i$ (km) |
|---|---|---|
| 20–30 | $8.80\times10^{-2}$ | 6.30 |
| 30–40 | $1.80\times10^{-2}$ | 6.64 |
| 40–50 | $3.99\times10^{-3}$ | 7.37 |
| 50–60 | $1.027\times10^{-3}$ | 8.35 |
| 60–70 | $3.10\times10^{-4}$ | 7.57 |
| 70–80 | $8.28\times10^{-5}$ | 6.67 |
| 80–90 | $1.85\times10^{-5}$ | 5.92 |
| 90–100 | $3.42\times10^{-6}$ | 5.53 |

> 论文里务必写一句：单指数 $H=7.5$ km 外推到 100 km 会产生数量级误差，因此采用分层指数；高保真算例建议用 NRLMSISE-00 交叉校验。

### 2.2 声速、马赫数与动压

$$
a(h)=\sqrt{\gamma_{air}R_{air}T(h)},\qquad Ma=\frac{V}{a},\qquad
q_\infty=\tfrac12\rho V^2 .
$$

温度分层取值（US76，供 $a(h)$ 与稀薄参数使用）：20 km: 216.7 K，30: 226.5，40: 250.4，50: 270.7，60: 247.0，70: 219.6，80: 198.6，90: 186.9，100: 195.1 K。

### 2.3 稀薄效应（论文卖点之一）

分子平均自由程与 Knudsen 数：

$$
\lambda=\frac{k_BT}{\sqrt2\,\pi d^2p},\qquad Kn=\frac{\lambda}{\bar c},\qquad p=\rho R_{air}T,
$$

取 $d=3.7\times10^{-10}$ m、$\bar c=3.2$ m 时 20→100 km 的 $Kn$ 约 $2.8\times10^{-7}\to4.4\times10^{-2}$：连续流在 20–60 km 完全成立，80 km 以上进入滑移/过渡区。气动系数应写成

$$
C_m=C_m^{cont}(\alpha,Ma)\;\bigl[1-\sigma(Kn)\bigr]+C_m^{FM}(\alpha,Ma)\;\sigma(Kn),
$$

$\sigma$ 为桥函数（Schaaf–Chambre、CLL 或 DSMC 表插值），$\sigma\to1$ 为自由分子流。**不要**在 80–100 km 继续用连续流线性气动系数。

### 2.4 密度不确定性

$$
\rho(h,t)=\rho_{nom}(h)\bigl[1+\Delta_\rho(t)\bigr],\qquad |\Delta_\rho|\le
\begin{cases}
0.2,& h<50\ \text{km}\\
0.5,& 50\le h<80\ \text{km}\\
1.0\ (\text{太阳活动极端时 }2\text{–}5),& h\ge80\ \text{km}
\end{cases}
$$

工程上可用加速度计测得的气动加速度在线估计 $\hat\rho$（EKF），再送入 MPC 参数 $p_k$；这既提高精度，也是一个可写入论文的小贡献点。

---

## 3. 俯仰通道刚体动力学（含角动量守恒推导）

状态 $x_v=[\theta,q]^T$：

$$
\dot\theta=q,\qquad I_y\dot q=M_a+M_{rw}+M_d .
$$

**符号自洽性推导**（建议放进论文，堵住“建模笼统”的批评）：系统总俯仰角动量为 $H_{tot}=I_yq+h_w$，外力矩只有气动与扰动：

$$
\frac{d}{dt}\bigl(I_yq+h_w\bigr)=M_a+M_d
\;\Longrightarrow\;
\dot h_w=-\bigl(M_a+M_d-I_y\dot q\bigr)=-M_{rw}.
$$

即

$$
\boxed{\ \dot h_w=-M_{rw}\ }
$$

$M_{rw}<0$（低头反作用力矩）$\Rightarrow \dot h_w>0$：飞轮**储能（转速上升）**；$M_{rw}>0$ 则**释放角动量**。这与原方案的物理图一致，但现在是严格推导出来的，且与 $I_y\dot q$ 方程严格互补。

扰动力矩模型（标称 + 有界不确定）：

$$
M_d(t)=M_{d0}+A_d\sin(\omega_d t+\varphi)+w(t),\qquad |w(t)|\le \bar w .
$$

---

## 4. 气动操纵面与舵机动态

### 4.1 气动力矩

$$
M_a=q_\infty S\bar c\, C_m
=\underbrace{\tfrac12\rho V^2S\bar c}_{Q}\Bigl[C_{m0}(Ma)+C_{m\alpha}\alpha+C_{mq}\frac{q\bar c}{2V}+C_{m\delta}\delta\Bigr],
$$

$$
\alpha=\theta-\gamma(t).
$$

* $C_{m\alpha}<0$（静稳定）、$C_{mq}<0$（俯仰阻尼）、$C_{m\delta}>0$。
* $C_{m0},C_{m\alpha},C_{m\delta}$ 是 $(Ma,\alpha,Kn)$ 的二维/三维查表（双线性或三次样条），论文必须给出表格或数据来源；$C_{mq}$ 用 $q\bar c/2V$ 无量纲化。
* 高超声速下舵效可能因激波/边界层干扰下降甚至反效，由 $C_{m\delta}(Ma,\alpha)$ 表体现；结构/热气动弹性极限给 $M_a$ 加一个饱和上界 $\bar M_a$。

### 4.2 舵机一阶模型与约束

$$
\dot\delta=\frac{1}{\tau_s}\bigl(\delta_c-\delta\bigr),
\qquad
\delta\in[-\delta_{\max},\delta_{\max}],\qquad |\dot\delta|\le\dot\delta_{\max}.
$$

由于 $\delta_c$ 是决策变量，速率约束等价于对指令的状态相关线性约束：

$$
\delta-\tau_s\dot\delta_{\max}\ \le\ \delta_c\ \le\ \delta+\tau_s\dot\delta_{\max},
\qquad
\delta_c\in[-\delta_{\max},\delta_{\max}].
$$

（可选高阶模型：$\ddot\delta=\omega_n^2(\delta_c-\delta)-2\zeta\omega_n\dot\delta$，把 $\dot\delta$ 增广进状态；若舵机带宽 $\ge10$ Hz 且 MPC 步长 $\le20$ ms，一阶即可。）

---

## 5. 反作用轮模型

### 5.1 角动量与力矩环

$$
h_w=J_w\Omega_w,\qquad
\dot M_{rw}=\frac{1}{\tau_w}\bigl(M_{rw,c}-M_{rw}\bigr),\qquad
\dot h_w=-M_{rw}.
$$

$(M_{rw},h_w)$ 构成一个由 $M_{rw,c}$ 驱动的**双积分链**，这是 MPC 能同时管姿态与角动量的结构基础。

### 5.2 角动量约束

$$
|h_w|\le h_{\max}=J_w\Omega_{\max}\quad\Longleftrightarrow\quad |\Omega_w|\le\Omega_{\max}.
$$

### 5.3 力矩–转速包线（**关键修正**）

四象限驱动的飞轮同时受**峰值电流（堵转力矩）**与**峰值功率**限制：

$$
\boxed{\ |M_{rw}|\le M_{rw,avail}(h_w)=\min\!\left(M_{stall},\ \frac{P_{\max}J_w}{|h_w|+\varepsilon}\right)\ }
$$

等价地 $|M_{rw}|\cdot|h_w|\le P_{\max}J_w$：**存得越多，能出的力矩越小**。这正是“角动量管理 = 力矩权限管理”的物理根据，也是统一 MPC 用裕度目标取代模式切换的正当性来源。

数值示例（$J_w=0.35$ kg·m²，$\Omega_{\max}=5000$ rpm $\Rightarrow h_{\max}=183.3$ N·m·s，$M_{stall}=60$ N·m）：

| 转速 | $P_{\max}=1$ kW | $P_{\max}=3$ kW | $P_{\max}=10$ kW |
|---|---|---|---|
| 1000 rpm | 9.5 N·m | 28.6 N·m | 60 N·m |
| 3000 rpm | 3.2 N·m | 9.5 N·m | 31.8 N·m |
| 5000 rpm | 1.9 N·m | 5.7 N·m | 19.1 N·m |

满充时间（功率受限，$t=(h_{\max}^2-h_0^2)/(2P_{\max}J_w)$）：1 kW → 48.0 s；3 kW → 16.0 s；10 kW → 4.8 s。
**结论：$h_w$ 与 $M_{rw}$ 必须联合预测数十秒，否则飞轮要么提前饱和、要么在需要时无力矩。**

### 5.4 功率与热

$$
P_{elec}=M_{rw}\Omega_w+P_{loss},\qquad |P_{elec}|\le P_{\max},
$$

再生制动能量需电阻耗散或回充；热约束写成占空比约束即可（可选，作为论文加分项）。

---

## 6. 统一状态空间模型

### 6.1 状态、输入、参数

$$
\boxed{\ x=[\theta,\ q,\ \delta,\ M_{rw},\ h_w]^T\in\mathbb R^5,\qquad
u=[\delta_c,\ M_{rw,c}]^T\in\mathbb R^2\ }
$$

$$
p(t)=[\rho(t),\ V(t),\ \gamma(t)]^T
\quad(\text{由弹道与大气模型给出}).
$$

### 6.2 连续模型

$$
\dot x=f(x,u,p)=
\begin{bmatrix}
q\\[2pt]
\dfrac{1}{I_y}\Bigl[Q\,C_m(\theta,q,\delta;p)+M_{rw}+M_d(t)\Bigr]\\[6pt]
\dfrac{1}{\tau_s}(\delta_c-\delta)\\[6pt]
\dfrac{1}{\tau_w}(M_{rw,c}-M_{rw})\\[6pt]
-M_{rw}
\end{bmatrix},
$$

$$
Q=\tfrac12\rho V^2S\bar c,\qquad
C_m=C_{m0}+C_{m\alpha}(\theta-\gamma)+C_{mq}\frac{q\bar c}{2V}+C_{m\delta}\delta .
$$

**输入仿射结构（论文要点）**：

$$
f(x,u,p)=f_0(x,p)+B\,u,\qquad
B=\begin{bmatrix}
0 & 0\\
0 & 1/I_y\\
1/\tau_s & 0\\
0 & 1/\tau_w\\
0 & 0
\end{bmatrix}=\text{const}.
$$

### 6.3 雅可比与可解性

$$
A(x,p)=\frac{\partial f}{\partial x}=
\begin{bmatrix}
0 & 1 & 0 & 0 & 0\\[3pt]
\dfrac{QC_{m\alpha}}{I_y} & \dfrac{Q\bar c\,C_{mq}}{2VI_y} & \dfrac{QC_{m\delta}}{I_y} & \dfrac{1}{I_y} & 0\\[6pt]
0 & 0 & -\dfrac{1}{\tau_s} & 0 & 0\\[6pt]
0 & 0 & 0 & -\dfrac{1}{\tau_w} & 0\\[6pt]
0 & 0 & 0 & -1 & 0
\end{bmatrix}.
$$

* $(A,B)$ 可控（$\delta$ 通道驱动 $\theta,q$；$M_{rw,c}$ 通道驱动 $M_{rw},h_w$，并经 $1/I_y$ 驱动 $q$），可控性矩阵满秩 5。
* $h_w$ **不出现在** $\theta,q$ 方程中，只通过约束 $M_{rw,avail}(h_w)$、$|h_w|\le h_{\max}$ 影响闭环 $\Rightarrow$ 角动量管理只能靠**目标函数 + 约束**，不能靠反馈镇定。这是统一 MPC 必须显式写出裕度/角动量代价的原因。
* 若 $h_w$ 不可测而由 $\Omega_w$ 估计，$(A,C)$ 对 $h_w$ 不可观（纯积分环节），需转速量测或观测器。

### 6.4 离散化（非均匀步长）

RK4 或零阶保持矩阵指数：

$$
x_{k+1}=A_{d,k}x_k+B_{d,k}u_k+d_k,\qquad
A_{d,k}=e^{A(x_k,p_k)\Delta t_k},\quad
B_{d,k}=\int_0^{\Delta t_k}\!e^{A s}B\,ds,
$$

$d_k$ 含 $M_d$ 与线性化仿射项。仿真用 RK4@1 kHz，控制器用上式（或直接非线性多步打靶）。

---

## 7. 统一 MPC 优化问题

### 7.1 非均匀预测时域

$$
\mathcal T=\{0,\Delta t_1,\dots\}:\quad
\underbrace{0.05\,\text{s}\times20}_{0\text{–}1\ \text{s}}
\;+\;
\underbrace{0.5\,\text{s}\times20}_{1\text{–}11\ \text{s}}
\;+\;
\underbrace{2\,\text{s}\times27}_{11\text{–}65\ \text{s}},
\qquad N=67 .
$$

决策变量 $U=\{u_0,\dots,u_{N-1}\}$；取首项 $u_0=(\delta_c^\star,M_{rw,c}^\star)$ 送入执行机构。前段分辨舵机/电机动态，后段覆盖一次密度 e 折（约 30 s），从而**在同一优化问题内**完成“预测气动能力下降 $\to$ 提前预偏置”。

### 7.2 代价函数

$$
\begin{aligned}
J(U)=&\sum_{k=0}^{N-1}\Bigl[\|\theta_k-\theta_{ref,k}\|^2_{Q_\theta}
+\|q_k-q_{ref,k}\|^2_{Q_q}
+R_\delta\delta_{c,k}^2+R_wM_{rw,c,k}^2\Bigr]\Delta t_k\\
&+\sum_{k=1}^{N-1}\Bigl[S_\delta(\delta_{c,k}-\delta_{c,k-1})^2
+S_w(M_{rw,c,k}-M_{rw,c,k-1})^2\Bigr]\\
&+\sum_{k=0}^{N-1}W_h\bigl(h_{w,k+1}-h_{ref,k}\bigr)^2
+\|x_N-x_{ref,N}\|^2_P
-\lambda_\eta\,\eta .
\end{aligned}
$$

* $-\lambda_\eta\eta$：最大化最坏情况控制裕度（§7.4），是“主动角动量管理”的数学载体。
* $W_h$ 项是角动量管理的软手段：$h_{ref,k}$ 由 §8 的长时域/慢层给出；也可换成障碍函数 $W_h\sum (h_{w,k}/h_{\max})^{2p}$ 防止饱和。
* $P$ 由当前工作点线性化系统的 DARE 给出（$h_w$ 需给小权重以保证可检测性/数值良态）。

### 7.3 约束

$$
\begin{aligned}
&\text{状态：} && \delta\in[-\delta_{\max},\delta_{\max}],\quad |h_w|\le h_{\max},\quad |q|\le q_{\max},\quad \theta\in[\theta_{\min},\theta_{\max}]\\
&\text{舵指令：} && \delta-\tau_s\dot\delta_{\max}\le\delta_c\le\delta+\tau_s\dot\delta_{\max},\quad \delta_c\in[-\delta_{\max},\delta_{\max}]\\
&\text{飞轮指令：} && |M_{rw,c}|\le M_{cmd,\max},\quad |M_{rw,c,k}-M_{rw,c,k-1}|\le \Delta M_{\max}\\
&\text{力矩包线：} && |M_{rw,k}|\le M_{rw,avail}(h_{w,k})\\
&\text{裕度：} && \Gamma_k^\pm+s_k^\pm\ \ge\ \eta,\qquad s_k^\pm\ge0,\qquad \eta\le\eta_{\max}
\end{aligned}
$$

松弛 $s_k^\pm$ 以 $\rho_s\|s\|^2$ 惩罚，保证极端不确定下 QP 始终可行（安全回退）。

### 7.4 所需力矩与控制裕度（显式定义）

参考模型：

$$
\dot q_{ref}=k_\theta(\theta_{ref}-\theta)+k_q(\dot\theta_{ref}-q)+\ddot\theta_{ref},
\qquad
M_{req}=I_y\dot q_{ref}-\hat M_d .
$$

单侧最大能力：

$$
M_{a,\max,k}=Q_k\Bigl[C_{m0}+C_{m\alpha}\alpha_k+C_{mq}\frac{q_k\bar c}{2V}+C_{m\delta}\delta_{\max}\Bigr],
\qquad
M_{a,\min,k}=(\delta_{\max}\!\to\!-\delta_{\max}),
$$

$$
\Gamma_k^{+}=M_{a,\max,k}+M_{rw,avail}(h_{w,k})-M_{req,k},
\qquad
\Gamma_k^{-}=M_{req,k}-\bigl[M_{a,\min,k}-M_{rw,avail}(h_{w,k})\bigr].
$$

$\Gamma^+$ 为还可额外提供的抬头力矩储备，$\Gamma^-$ 为低头储备；$\eta=\min_k\min(\Gamma_k^+,\Gamma_k^-)$ 即全时域最坏情况裕度。最大化 $\eta$ 会自动产生：

1. 密度下降前把 $h_w$ 预偏到合适方向（提前储能）；
2. 接近 $h_{\max}$ 时让出飞轮力矩权限（因为 $M_{rw,avail}\!\downarrow$）；
3. 用舵面承担大机动、飞轮补足带宽与扰动抑制。

### 7.5 终端与递归可行性

终端代价 $P$ + 终端集 $\mathbb X_f=\{x:\ \|x-x_{ref,N}\|_P^2\le c\}$；非线性多步打靶时用参考轨迹终端约束或软终端。若采用鲁棒 MPC，则用约束收紧 $\mathbb X\ominus\mathbb S$、$\mathbb U\ominus K\mathbb S$，$\mathbb S$ 为误差动力学的最小鲁棒正不变集。

### 7.6 不确定性处理（回应审稿意见）

* 参数不确定：$\rho$ 乘性（§2.4）、$C_m$ 乘性 $\pm10\%\sim20\%$、$I_y$ $\pm10\%$、$M_d$ 有界。
* 方案 a：管式 MPC（tube MPC），误差动力学 $e_{k+1}=(A_k+B_kK_k)e_k+w_k$，不动点求 $\mathbb S$，约束收紧。
* 方案 b：场景/集合 MPC，$M$ 个 $(\rho,C_m)$ 场景并行预测，取最坏约束；$M=3\text{–}5$ 在 $N=67$ 时仍可实时。
* 方案 c：LPV 增益调度 MPC + 在线 $\hat\rho$ 估计（加速度计）。
* 建议论文至少给出方案 b 的蒙特卡洛结果，这是审稿人最认的“不确定性处理”。

### 7.7 在线求解

因 $B$ 恒定、输入仿射，采用**逐次线性化 + QP（RTI/SQP 1–2 次迭代）**：

1. 用上一拍解作为参考轨迹，沿轨迹线性化得 LTV 模型 $\{A_{d,k},B_{d,k},d_k\}$；
2. 形成稀疏 QP：变量 $2N=134$，约束约 $8N$；
3. OSQP/qpOASES 求解，热启动；必要时 IPOPT/acados 做全非线性 RTI。

应报告：单步求解时间（最坏/均值）、控制周期、代码与参数（可复现性）。

### 7.8 实现说明（v1 代码）与一处重要修正

* 实际代码采用非均匀时域 $[5\times0.25,\ 6\times0.5,\ 8\times4]$ s（≈36 s），QP 由 SQP + 行缩放 OSQP 求解，单步 6–13 ms。
* **硬裕度约束存在可行性问题**：若要求 $\eta_k\ge0$ 且 $\eta_k\le\Gamma_k$，当 $\Gamma_k<0$（权限不足）时无解。因此 v1 把 $\eta$ 作为**预测时域内的诊断指标**输出，角动量管理由二次惩罚 $w_h h_w^2$ + 力矩包线约束承担。
* **重要修正**：在功率受限包线 $|M_{rw}|\le P_{\max}J_w/|h_w|$ 下，瞬时权限在 $h_w=0$ 处最大。因此"最大化瞬时 margin"**不会**自动产生储能式预偏置，反而鼓励把 $h_w$ 保持在零附近。预偏置的真正意义是**为未来单向力矩需求预留动量空间（冲量）**，需要 $h_{ref}$ 调度（§8 方案 B）或带松弛的软裕度约束才能显式表达。论文中应据此修正"提前预偏置"的论证方式。

---

## 8. 为什么必须拉长/分层时域（预偏置的定量论证）

| 现象 | 时间尺度 |
|---|---|
| 舵机 $\tau_s$ / 电机 $\tau_w$ | 0.02–0.05 s |
| $N=40,\ \Delta t=0.02$ s 的时域 | 0.8 s |
| $N=67$ 非均匀时域 | 65 s |
| 密度 e 折（$\dot h\approx V\sin\gamma\approx200$ m/s，$H\approx6.3$ km） | ≈30 s |
| 飞轮满充（3 kW，0→183 N·m·s） | ≈16 s |
| 飞轮满放（同功率） | ≈16 s |

**0.8 s 时域比密度变化快 40 倍，不可能“预见”气动能力下降。** 因此：

* **方案 A（推荐，单 MPC）**：§7.1 非均匀长时域，前后段分别解析执行机构与密度演化；
* **方案 B（双时间尺度，仍无分配层）**：慢层（$\Delta t=1$ s，$N_s=60$，60–120 s 时域）以“全程最小裕度最大”为指标解出 $h_{ref}(t)$ 与 $\theta_{ref}(t)$；快层 MPC 跟踪并直接输出 $\delta_c,M_{rw,c}$。慢层只给**角动量参考**，不分配力矩，与原论文的“分配层”有本质区别。
* **预偏置规则**可由 A/B 自然导出：在 $\rho$ 下降前令 $M_{rw}<0$ 使 $h_w\uparrow$（储能），进入稀薄段后 $M_{rw}>0$ 释放——正是原方案第 11 节那张图的严格版本。

---

## 9. 三轴扩展（回应审稿意见 4）

状态 $x=[q_{bn},\ \omega,\ \delta,\ \tau_{rw},\ h_w]$，$q_{bn}$ 为姿态四元数，$\delta=[\delta_e,\delta_a,\delta_r]^T$，飞轮组轴矩阵 $A_w$：

$$
J\dot\omega=-\omega\times(J\omega+h_w)-A_w\tau_{rw}+M_{aero}+M_d,
\qquad
\dot h_w=-A_w^T\tau_{rw}\ \ (\text{逐轮}),
$$

$$
\dot q_{bn}=\tfrac12 q_{bn}\otimes[0,\omega^T]^T .
$$

MPC 直接输出 $[\delta_c,\tau_{rw,c}]$，无需伪逆分配；但需处理：

* 陀螺耦合 $\omega\times h_w$（高转速下不可忽略）；
* 飞轮组奇异/零空间内的角动量重分配；
* 横航向扰动对俯仰通道的影响：至少给出耦合项量级分析 + 三轴线性化 RGA/耦合范数，或直接给三轴仿真。

**单通道论文的写法建议**：明确“本文聚焦俯仰通道以隔离机理”，并补一段“多通道适用性分析 + 三轴数值算例”，否则按审稿意见 4 会被继续质疑。

---

## 10. 参数表与控制能力核算

### 10.1 飞行器与气动（$[FILL]$，必须给出数值）

| 参数 | 含义 | 建议值/来源 |
|---|---|---|
| $m,I_x,I_y,I_z,I_{xz}$ | 质量与惯量张量 | 由 CAD/质量特性给出，论文必须列表 |
| $S,\bar c$ | 参考面积/弦长 | $[FILL]$ |
| $C_{m0},C_{m\alpha},C_{mq},C_{m\delta}$ | 力矩系数 | CFD/风洞表，注明 $Ma,\alpha$ 范围与插值方法 |
| $\delta_{\max},\dot\delta_{\max},\tau_s$ | 舵面 | 例：25°，60°/s，0.05 s |
| $J_w,\Omega_{\max},M_{stall},P_{\max},\tau_w$ | 飞轮 | 例：0.35 kg·m²，5000 rpm，60 N·m，3 kW，0.03 s |
| $M_{d0},\bar w$ | 扰动 | 例：20 N·m + 15 N·m 周期项，$\bar w=10$ N·m |

### 10.2 气动权限随高度衰减（$V=4000$ m/s，$\delta=25°$，$C_{m\delta}=0.45$，$S=4.2$ m²，$\bar c=3.2$ m，$I_y=8.5\times10^3$ kg·m²）

| $h$ (km) | $\rho$ (kg/m³) | $q_\infty$ (Pa) | $Ma$ | $M_{a,\max}$ (N·m) | 满舵角加速度 (°/s²) |
|---|---|---|---|---|---|
| 20 | $8.80\times10^{-2}$ | $7.04\times10^5$ | 13.6 | $1.86\times10^6$ | 12523 |
| 30 | $1.80\times10^{-2}$ | $1.44\times10^5$ | 13.3 | $3.80\times10^5$ | 2562 |
| 40 | $3.99\times10^{-3}$ | $3.19\times10^4$ | 12.6 | $8.42\times10^4$ | 568 |
| 50 | $1.027\times10^{-3}$ | $8.22\times10^3$ | 12.1 | $2.17\times10^4$ | 146 |
| 60 | $3.10\times10^{-4}$ | $2.48\times10^3$ | 12.7 | $6.55\times10^3$ | 44.1 |
| 70 | $8.28\times10^{-5}$ | $6.62\times10^2$ | 13.5 | $1.75\times10^3$ | 11.8 |
| 80 | $1.85\times10^{-5}$ | $1.48\times10^2$ | 14.2 | $3.91\times10^2$ | 2.63 |
| 90 | $3.42\times10^{-6}$ | $2.74\times10^1$ | 14.6 | $7.22\times10^1$ | 0.49 |
| 100 | $5.60\times10^{-7}$ | $4.48$ | 14.3 | $1.18\times10^1$ | 0.08 |

这张表就是论文“最关键的一张图”的数据版：气动权限跨 5 个数量级衰减，而飞轮力矩受功率限制在 $10^1$ N·m 量级——**协同 + 提前管理角动量**是唯一可行路线。

### 10.3 飞轮力矩需求可行性（回应审稿意见 2.2）

产生力矩所需功率 $P=M\Omega$：

| $M_{rw}$ | 1000 rpm | 3000 rpm | 5000 rpm |
|---|---|---|---|
| 20 N·m | 2.1 kW | 6.3 kW | 10.5 kW |
| 60 N·m | 6.3 kW | 18.8 kW | 31.4 kW |
| 120 N·m | 12.6 kW | 37.7 kW | 62.8 kW |
| 240 N·m | 25.1 kW | 75.4 kW | 125.7 kW |

**结论（建议写进论文与回复信）**：
1. 60–240 N·m 属于 CMG 量级，不是常规 RW；在 3000–5000 rpm 下需 19–126 kW，对临近空间飞行器不现实。
2. 若坚持 RW，应把力矩指标降到 $M_{stall}\approx20\text{–}60$ N·m、$P_{\max}\approx1\text{–}5$ kW，并明确“大机动由舵面承担、飞轮提供带宽与扰动抑制”的分工；
3. 或改用 CMG 并重做对比——这同时能作为论文新的工程贡献点。
4. 无论如何，必须给出 $M_{stall},P_{\max},\Omega_{\max}$ 与力矩–转速包线，否则审稿意见 2.2 无法回答。

---

## 11. 对比实验设计（可直接作为论文第 5 节）

| 算例 | 控制器 | 执行机构 | 目的 |
|---|---|---|---|
| C0 | 理想执行机构（$\delta=\delta_c,M_{rw}=M_{rw,c}$） | 无动态 | 性能上界 |
| C1 | 仅气动舵（$M_{rw}\equiv0$） | 含舵机动态 | 复现“稀薄大气失效” |
| C2 | 传统分配：MPC/PD 出总力矩 $\to$ 链式分配 | 含执行机构 | 作为被比较基线（原论文方法） |
| C3 | **统一 MPC（本文）** | 含执行机构 + 角动量管理 + 裕度目标 | 主结果 |
| C4 | 统一 MPC + 鲁棒/场景 MPC | 含 $\rho,C_m$ 不确定 | 鲁棒性 |
| C5 | 统一 MPC + 执行机构故障/饱和 | 故障注入 | 安全回退 |

指标：$RMSE_\theta$、$\max|\theta-\theta_{ref}|$、$\int\delta^2dt$、$\int M_{rw}^2dt$、$\max|h_w|$、饱和时间 $T_{sat}$、最坏裕度 $\eta^\star$、单步求解时间。
消融：去掉 $-\lambda_\eta\eta$、去掉 $W_h$ 项、缩短/加长时域、改变 $\Delta t$ 分布。
蒙特卡洛：$\rho\pm50\%$（80 km 以上 $\pm100\%$）、$C_m\pm20\%$、$I_y\pm10\%$、$M_d$、舵效下降，100 次。

**HIL 说明（回应审稿意见 3）**：现有“飞轮–喷气”台架只能验证**飞轮力矩环与角动量演化**，不能代表气动通道。论文应：(i) 把该实验明确重命名为“反作用轮执行机构级验证”；(ii) 气动通道用数值仿真/气动数据库验证；(iii) 或补充舵机/气动舵面负载台架。不要在结论里声称“整机 HIL 验证了复合控制”。

---

## 12. 与文献的贡献界定（回应审稿意见 2.1）

| 内容 | 归属 |
|---|---|
| MPC 姿态控制、RW 去饱和、混合执行机构分配 | 已有文献，明确引用 |
| 执行机构动态 + 角动量状态纳入统一预测模型，MPC 直接输出 $\delta_c,M_{rw,c}$（无分配层） | 本文方法 |
| 控制裕度 $\Gamma^\pm$ 作为显式优化目标（含 $M_{rw,avail}(h_w)$ 功率–力矩耦合） | 本文贡献 |
| 非均匀/双时间尺度时域，跨密度 e 折的主动角动量预偏置 | 本文贡献 |
| 稀薄大气 $Kn$ 修正 + 气动/密度不确定性下的鲁棒 MPC | 本文贡献 |
| 与链式分配基线的定量对比与消融 | 本文验证 |

---

## 13. 可直接粘贴的英文段落

**System model (Section 2/3).** The vehicle is modelled in the pitch plane with state $x=[\theta,q,\delta,M_{rw},h_w]^T$ and control input $u=[\delta_c,M_{rw,c}]^T$. The aerodynamic moment is $M_a=\tfrac12\rho V^2S\bar c\,C_m$ with $C_m=C_{m0}(Ma)+C_{m\alpha}\alpha+C_{mq}q\bar c/(2V)+C_{m\delta}\delta$ and $\alpha=\theta-\gamma(t)$. A piecewise-exponential atmosphere fitted to the US Standard 1976 is used between 20 and 100 km, with a Knudsen-number bridging correction above 80 km to account for rarefaction. The control surface is a first-order servo with position and rate limits, and the reaction wheel is modelled by $\dot M_{rw}=(M_{rw,c}-M_{rw})/\tau_w$ and $\dot h_w=-M_{rw}$; its usable torque obeys $|M_{rw}|\le\min(M_{stall},P_{\max}J_w/|h_w|)$, so that stored angular momentum directly limits the achievable torque.

**Unified MPC (Section 4).** Rather than allocating a demanded torque, the proposed controller optimises the actuator commands directly over a non-uniform horizon that resolves the servo/motor dynamics in its first second and spans one density e-folding time thereafter. The cost combines attitude tracking, control effort and slew, a momentum-management term, and the maximisation of the worst-case control-authority margin $\eta=\min_k\min(\Gamma_k^+,\Gamma_k^-)$, where $\Gamma^\pm$ compare the total available aerodynamic and wheel torque against the torque required by a reference model. This formulation yields proactive momentum pre-biasing before the vehicle enters rarefied air, keeps the wheel away from saturation, and degrades gracefully under density and aerodynamic-coefficient uncertainty.

---

## 14. 真实大气后端（已实现，回应审稿意见"气动系数/大气依赖"）

代码：nearspace/atmosphere_real.py（+ atmosphere_factory.py），config.yaml 用 environment.backend 切换：

| backend | 模型 | 说明 |
|---|---|---|
| exp | 8 段指数拟合 | legacy，保证旧结果可逐位复现 |
| us76 | U.S. Standard Atmosphere 1976 完整解析 | 0–86 km 七层温度梯度（位势高度）+ 86–1000 km 上层解析 + 静力积分 |
| msis2 | NRLMSIS-2.0（pymsis 0.13） | 含 F10.7/Ap/经纬度/日期；预计算 0–120 km 查表并缓存到 results/*.csv |

20–100 km 密度对比（F10.7=150, Ap=4）：

| h (km) | exp 拟合 | US76 | NRLMSIS-2.0 | msis2/exp |
|---|---|---|---|---|
| 20 | 8.976e-2 | 8.891e-2 | 8.649e-2 | −3.6% |
| 50 | 9.774e-4 | 1.027e-3 | 8.772e-4 | −10.3% |
| 86 | 6.958e-6 | 6.958e-6 | 4.987e-6 | **−28.3%** |
| 100 | 5.610e-7 | 5.656e-7 | 5.341e-7 | −4.8% |

要点：

1. US76 后端在 86 km 以下相对公认值：密度误差 ≤0.006%、温度 ≤0.041%、100 km 密度 +0.93%（上层解析近似的正常量级）。
2. MSIS2 查表插值相对直接调用 pymsis 误差 4e-5；12000 点查询 21.6 ms；CSV 缓存命中 0.72 ms。
3. 旧的 8 段指数拟合在 70–90 km **明显偏厚**（86 km 高估 28%），会高估该高度段的气动权限——论文中应改用 US76/MSIS2。
4. **太阳活动敏感性（重要修正）**：NRLMSIS-2.0 在 100 km 处 F10.7 70→250 仅 1.05×（下部热层仍充分混合）；
   2.38× 出现在 200 km，300 km 约 7×，400 km 约 16×。若论文声称"100 km 太阳活动使密度变化 2 倍"必须按实际高度重算。
5. 上限提醒：默认表到 120 km，超过则截断；更高工况需 z_max_km=250 以上。

## 15. 三轴（6-DOF）实现与 QP 经验

模型（nearspace/model6.py）：状态 x = [q_bn(4), omega(3), delta(3), tau(3), h_w(3)]（16 维），
输入 u = [delta_c(3), tau_c(3)]（6 维），p = [rho, V_ned(3), M_d(3)]：

    q_dot = 0.5 q ⊗ [0, omega]
    J omega_dot = -omega × (J omega + h_w) - tau + M_aero + M_d
    delta_dot = (delta_c - delta)/tau_s,  tau_dot = (tau_c - tau)/tau_w,  h_w_dot = tau

气动力矩 M_aero = q_inf S [b Cl, cbar Cm, b Cn]，系数为 (alpha, beta, p, q, r, delta) 的线性示例模型；
alpha, beta 由机体系速度矢量反解（因此姿态与气流角真正耦合）。舵面顺序 [de, da, dr]，力矩顺序 [roll, pitch, yaw]。

MPC（nearspace/mpc3.py）：
* 姿态误差用误差四元数的向量部分 e(q) = 2 vec(q_ref^{-1} ⊗ q) = 2(w_r q_v - q_w v_r - v_r × q_v)，
  它是 q 的**精确线性函数**，可直接进 QP；
* 时域 [5×0.25, 6×0.5, 6×1.0] s（10.25 s，N=17）；
* 求解：SQP + 行缩放 OSQP（scaling=0，eps=1e-5，H 加 1e-4 I 正则）。

**四个必须记住的坑**：
1. 若加等式约束 q_ref^T q = 1 来锚定四元数范数，线性化 QP 会**不可行**（该约束在预测步内的可控方向太弱）→ 去掉，靠误差代价 + 真值模型归一化。
2. 预测步长 >1 s 时，30 km 处约 15 rad/s 的俯仰振荡使线性化离散模型条件数爆炸，OSQP 20000 次迭代不收敛 → 三轴时域最大步长取 1 s（俯仰轴可用 4 s）。
3. OSQP 关闭内部 scaling（我们自己做行归一化）反而更稳；eps 取 1e-5，过紧（1e-6）会伪"max iterations"。
4. QP 失败必须回退到"四元数 PD + 气动反解 + 残差给飞轮"，且注意舵面/力矩顺序映射，否则级联发散。

结果（150 s，msis2，5° 俯仰 + 5° 滚转 + 3° 偏航机动）：

| 算例 | 三轴 RMSE | max | roll | pitch | yaw | max abs(h) | 饱和 |
|---|---|---|---|---|---|---|---|
| T1 无飞轮 | 22.78° | 80.8° | 19.92° | 6.02° | 9.26° | 0 | – |
| T3 统一 MPC | **1.92°** | **6.00°** | **0.084°** | 1.92° | **0.020°** | 366 | 21% |

俯仰是判别轴：100 km 处气动权限约 12 N·m，5° 机动峰值需求约 70 N·m，只能靠飞轮补足，代价是动量接近饱和。

## 16. 下一步（按优先级）

1. 接入真实气动系数表（Cl/Cm/Cn 随 alpha, Ma, Kn 的插值）替换示例线性系数；
2. 慢层 h_ref 调度，把"主动预偏置"做成可展示的显式策略；
3. 三轴补齐 C2（分配基线）与 C4（失配/场景鲁棒），并降低机动段 QP 失败率；
4. 论文正文改写：§2–§7 → 第 2、4 节；§11 → 第 5 节。

## 17. 原下一步清单（保留）

1. **填数**：$I_y$、$S,\bar c$、$C_m$ 表、执行机构参数、弹道 $h(t),V(t),\gamma(t)$（§10.1）。
2. **建仿真**：环境 → 舵机 → 飞轮 → 刚体 → 统一 MPC 六模块，单文件可复现。
3. **先跑开环可行性**：验证 §10.2 权限表与 §10.3 功率表。
4. **再跑 C1–C4**，确认“提前预偏置”现象可复现（$h_w$ 先升后降、$\eta$ 始终为正）。
5. **最后写论文**：把 §2–§7 直接改写为第 2、4 节，§11 作为第 5 节。
