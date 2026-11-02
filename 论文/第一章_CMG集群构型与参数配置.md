# 第一章 CMG 集群构型与参数配置

## 1.1 CMG 工作原理

控制力矩陀螺（Control Moment Gyroscope，CMG）利用高速旋转转子的角动量，通过改变转子角动量方向产生控制力矩。与反作用飞轮通过改变转子自旋角速度调节角动量不同，单框架 CMG 通常保持转子转速近似恒定，依靠框架机构带动转子自旋轴转动，使转子角动量矢量在机体坐标系中改变方向，并对飞行器产生反作用力矩。

单台 CMG 由高速转子、框架及其驱动机构组成。设转子角动量大小为 $h_0$，框架角为 $\gamma$，转子自旋轴单位向量为 $\boldsymbol{s}(\gamma)$，则转子角动量可表示为

$$
\boldsymbol{h}(\gamma)=h_0\boldsymbol{s}(\gamma).
$$

框架转动时，转子角动量方向随之变化。对时间求导可得

$$
\dot{\boldsymbol{h}}
=\frac{\partial \boldsymbol{h}}{\partial \gamma}\dot{\gamma}.
$$

根据角动量定理，转子角动量变化率对应相互作用力矩。考虑转子与飞行器之间的反作用关系，CMG 对飞行器施加与转子角动量变化率方向相反的力矩。因此，CMG 的输出能力取决于转子角动量大小、框架角速度以及当前几何构型。

单台 CMG 的输出方向受到框架轴和当前自旋轴方向限制，通常不能独立提供任意方向的三轴力矩。为满足飞行器三轴姿态控制需求，需要将多台 CMG 按一定几何关系组合成集群。集群输出能力不仅取决于单台转子角动量和框架运动速度，还与设备空间布置及当前框架构型有关。

## 1.2 四台 SGCMG 金字塔构型

### 1.2.1 集群布置

本文采用四台单框架控制力矩陀螺（Single-Gimbal Control Moment Gyroscope，SGCMG）组成金字塔式集群。四台设备的框架轴沿方位角方向均匀分布，并相对于机体横向平面具有相同的倾斜角。该布置使四台 CMG 的角动量方向能够随各自框架角变化，在机体三轴方向形成耦合的力矩输出能力。

设机体坐标系为 $x$-$y$-$z$，四台 CMG 的方位角定义为

$$
\theta_i=\frac{2\pi(i-1)}{4},\qquad i=1,2,3,4.
$$

设金字塔斜置角为 $\beta$，则第 $i$ 台 CMG 的框架轴单位向量定义为

$$
\boldsymbol{g}_i=
\begin{bmatrix}
\sin\beta\cos\theta_i\\
\sin\beta\sin\theta_i\\
\cos\beta
\end{bmatrix}.
$$

在零框架角位置，转子自旋轴单位向量取为

$$
\boldsymbol{s}_{i,0}=
\begin{bmatrix}
-\sin\theta_i\\
\cos\theta_i\\
0
\end{bmatrix}.
$$

上述定义满足 $\boldsymbol{g}_i^\mathsf{T}\boldsymbol{s}_{i,0}=0$，即框架轴与转子初始自旋轴相互垂直。本文采用的金字塔斜置角为

$$
\beta=54.7356^\circ,
$$

该角度满足 $\cos^2\beta=1/3$，是当前四台金字塔构型采用的几何参数。

### 1.2.2 框架转动与转子角动量方向

设第 $i$ 台 CMG 的框架角为 $\gamma_i$。转子自旋轴绕框架轴 $\boldsymbol{g}_i$ 转动，其方向可写为

$$
\boldsymbol{s}_i(\gamma_i)
=
\cos\gamma_i\,\boldsymbol{s}_{i,0}
+
\sin\gamma_i
\left(\boldsymbol{g}_i\times\boldsymbol{s}_{i,0}\right).
$$

转子转速近似恒定时，单台转子的角动量为

$$
\boldsymbol{h}_i(\gamma_i)=h_0\boldsymbol{s}_i(\gamma_i),
$$

其中 $h_0$ 为单台转子的角动量大小。定义集群框架角向量为

$$
\boldsymbol{\gamma}
=
\begin{bmatrix}
\gamma_1&\gamma_2&\gamma_3&\gamma_4
\end{bmatrix}^{\mathsf{T}},
$$

则四台 CMG 的总角动量为

$$
\boldsymbol{H}(\boldsymbol{\gamma})
=
\sum_{i=1}^{4}\boldsymbol{h}_i(\gamma_i)
=
h_0\sum_{i=1}^{4}\boldsymbol{s}_i(\gamma_i).
$$

该式表明，集群总角动量不仅由单台转子角动量大小决定，还随四台框架角共同变化。调整框架角可以改变集群角动量在机体三个轴向上的分布，为后续姿态机动形成不同的角动量构型。

### 1.2.3 集群力矩映射关系

对集群角动量关于框架角求偏导，定义 CMG 集群角动量雅可比矩阵

$$
\boldsymbol{A}(\boldsymbol{\gamma})
=
\frac{\partial\boldsymbol{H}(\boldsymbol{\gamma})}
{\partial\boldsymbol{\gamma}}
\in\mathbb{R}^{3\times4}.
$$

其第 $i$ 列为

$$
\boldsymbol{A}_i(\boldsymbol{\gamma})
=
h_0\left(
\boldsymbol{g}_i\times\boldsymbol{s}_i(\gamma_i)
\right).
$$

令框架角速度向量为

$$
\boldsymbol{\nu}
=
\dot{\boldsymbol{\gamma}}
=
\begin{bmatrix}
\dot{\gamma}_1&\dot{\gamma}_2&\dot{\gamma}_3&\dot{\gamma}_4
\end{bmatrix}^{\mathsf{T}},
$$

则集群角动量变化率为

$$
\dot{\boldsymbol{H}}
=
\boldsymbol{A}(\boldsymbol{\gamma})\boldsymbol{\nu}.
$$

按照本文采用的机体力矩符号约定，CMG 集群施加于飞行器的控制力矩为

$$
\boxed{
\boldsymbol{\tau}_{\mathrm{CMG}}
=
-\boldsymbol{A}(\boldsymbol{\gamma})\boldsymbol{\nu}
}
$$

其中 $\boldsymbol{\tau}_{\mathrm{CMG}}\in\mathbb{R}^{3}$ 为三轴 CMG 输出力矩。该映射关系是后续姿态动力学建模和预测优化控制问题的基础。

四台 SGCMG 提供四个框架角速度自由度，而飞行器姿态力矩为三维量，因此集群存在冗余自由度。在满足当前力矩需求的同时，系统可能通过不同的框架运动方案到达不同的后续构型。这一性质为本文研究预测时域内的 CMG 提前构型迁移提供了基础。

## 1.3 CMG 实物参考与参数配置

### 1.3.1 实物产品参考

为使仿真中的 CMG 参数具有工程参考，本文选取 Honeywell Aerospace 的 M50 控制力矩陀螺作为参数量级参照。公开产品规格汇总资料列出了 M50 的角动量范围、最大输出力矩、转子转速、框架旋转范围、质量及功耗等信息。本文采用公开产品资料中的参数范围作为仿真配置参考，并非对 M50 产品进行完整复刻。

根据 SatNow 和 SatCatalog 的公开产品资料汇总，M50 的主要参数如下 [1,2]。

| 参数 | M50 公开资料 | 说明 |
|---|---:|---|
| 角动量范围 | 25–75 N·m·s | 产品资料给出的角动量范围 |
| 最大输出力矩 | 75 N·m | 产品规格指标 |
| 转子转速 | 4500–6500 rpm | 可选转速范围 |
| 框架旋转范围 | 360° | 产品资料列出的旋转范围 |
| 总质量 | 28 kg | 产品资料值 |
| 功耗 | 11–113 W | 不同工作状态下的资料值 |
| 带宽 | 10 Hz | 产品资料指标 |

上述产品数据来自二次规格汇总页面。正式工程定型时仍应优先取得 Honeywell 原厂数据手册，并核对各参数的定义、额定条件和版本。最大输出力矩、角动量和功耗可能对应不同工作条件，不应未经额定工况核对便视为能够同时达到的工作点。

### 1.3.2 仿真参数确定

结合 M50 的公开产品参数 [1,2] 及相关学术文献中的 M-50 CMG 仿真参数 [3,4]，本文采用的四台 SGCMG 集群参数如表 1-2 所示。

| 参数 | 仿真配置 | 单位 | 参数依据及说明 |
|---|---:|---|---|
| CMG 数量 | 4 | 台 | 四台金字塔式集群 |
| 金字塔斜置角 $\beta$ | 54.7356 | ° | 集群几何配置 |
| 单台转子角动量 $h_0$ | 50 | N·m·s | 位于 M50 公开角动量范围内 [1,2] |
| 转子转速 | 6000 | rpm | 位于 M50 公开转速范围内 [1,2] |
| 框架角速度上限 | 1.0 | rad/s | 公开 M-50 仿真文献参数 [3,4] |
| 框架角加速度上限 | 3.0 | rad/s² | 公开 M-50 仿真文献参数 [3,4] |
| 框架速率环时间常数 | 0.03 | s | 当前仿真模型设定 |
| 单台质量预算 | 30 | kg | 参考 M50 的 28 kg，留有预算余量 |
| 单台功率预算 | 200 | W | 仿真功率预算，不是 M50 实测功耗 |
| 框架角行程 | 不设置有限边界 | — | 仿真采用展开角；真实硬件需考虑连续旋转连接方案 |

其中，框架角速度和角加速度均为每台框架的约束上限，而非期望工作值：

$$
|\dot{\gamma}_i|\leq 1.0\ \mathrm{rad/s},
$$

$$
|\ddot{\gamma}_i|\leq 3.0\ \mathrm{rad/s^2}.
$$

在控制周期为 $0.2\ \mathrm{s}$ 时，框架角速度的单步变化量相应受到

$$
|\nu_{i,k}-\nu_{i,k-1}|
\leq 3.0\times0.2
=0.6\ \mathrm{rad/s}
$$

的离散约束。角加速度上限约束框架角速度的变化率，并不直接等同于框架电机额定转矩。实际电机选型还需要结合框架及负载转动惯量、摩擦、陀螺耦合载荷、驱动器电流和热约束进行核算。

### 1.3.3 转子参数的派生核算

转子角动量与转子自旋轴转动惯量及转速之间满足

$$
h_0=J_r\Omega_r.
$$

仿真转子转速取 6000 rpm，对应角速度为

$$
\Omega_r
=
6000\frac{2\pi}{60}
\approx628.32\ \mathrm{rad/s}.
$$

当单台转子角动量取 $h_0=50\ \mathrm{N\,m\,s}$ 时，可反推出转子自旋轴转动惯量

$$
J_r
=
\frac{h_0}{\Omega_r}
=
\frac{50}{628.32}
\approx0.0796\ \mathrm{kg\,m^2}.
$$

相应的单台转子旋转动能为

$$
E_r
=
\frac{1}{2}J_r\Omega_r^2
=
\frac{1}{2}h_0\Omega_r
\approx15.7\ \mathrm{kJ}.
$$

上述结果是基于理想刚性转子的派生参数，用于检查角动量与转速配置的一致性，不包含转子轮毂、轴、驱动器和包容结构质量，也不构成转子强度或破裂包容验证。

### 1.3.4 参数对应关系与适用范围

本文仿真配置与 M50 产品参数之间的对应关系需要作如下区分。

首先，单台转子角动量 50 N·m·s 和转速 6000 rpm 均处于公开 M50 参数范围内 [1,2]。其次，框架角速度上限 1.0 rad/s 和角加速度上限 3.0 rad/s² 参考公开学术论文中列出的 M-50 CMG 仿真参数 [3,4]，而不是由产品最大输出力矩反推得到。单台质量预算取 30 kg，与公开资料中的 28 kg 产品质量接近 [1,2]；单台功率预算取 200 W，是仿真中的设计预算，不代表 M50 的实际功耗。

对于理想单框架 SGCMG，单台瞬时陀螺力矩幅值近似满足

$$
|\tau_{\mathrm{CMG},i}|=h_0|\dot{\gamma}_i|.
$$

因此，当前模型在 $h_0=50\ \mathrm{N\,m\,s}$、$|\dot{\gamma}_i|\leq1.0\ \mathrm{rad/s}$ 时，单台理想力矩幅值上限约为 50 N·m。该值不能与 M50 产品资料中列出的 75 N·m 最大输出力矩直接等同 [1,2]。集群三轴输出能力还取决于当前构型矩阵 $\boldsymbol{A}(\boldsymbol{\gamma})$，不能将单台力矩简单乘以四并视为任意方向的保证输出能力。

此外，M50 产品资料列出的 360° 框架旋转范围 [1,2]，与本文仿真采用的无界展开角模型并不完全相同。仿真中不对框架角施加有限行程边界，允许框架角连续跨越多个 $2\pi$ 周期；这是一种便于研究连续构型迁移的数学假设。真实设备若要实现持续多圈旋转，需要采用滑环或等效的无缠绕供电与信号传输方案，并进一步验证其寿命、噪声和可靠性。因此，本文的无界框架角模型不能直接解释为 M50 已具备无限连续旋转能力。

综上，本文采用的四台 SGCMG 配置具有公开产品和学术文献中的参数量级参照，可作为后续姿态动力学建模与预测优化控制研究的仿真基准。该配置不是 Honeywell M50 的完整硬件复刻，也不意味着已完成电机、滑环、热设计、结构强度和长期可靠性验证。

## 参考文献与数据来源

[1] Honeywell Aerospace M50 CMG 产品规格汇总，SatNow.  
https://www.satnow.com/products/control-moment-gyroscopes/honeywell-aerospace/43-1202-m50

[2] Honeywell M50 CMG 产品资料汇总，SatCatalog.  
https://www.satcatalog.com/component/m50-cmg/

[3] *Mechanical Engineering Journal*, DOI: 10.1299/mej.14-00554. M-50 CMG 框架运动参数及相关仿真参数.  
https://doi.org/10.1299/mej.14-00554  
https://www.jstage.jst.go.jp/article/mej/2/5/2_14-00554/_pdf

[4] *Journal of System Design and Dynamics*. M-50 CMG 参数资料.  
https://www.jstage.jst.go.jp/article/jsdd/5/1/5_1_30/_pdf

---

**本章小结**

本章介绍了 SGCMG 的基本工作原理，建立了四台金字塔式 SGCMG 集群的几何描述，并给出了框架角、集群角动量及三轴力矩映射关系。在此基础上，参考 Honeywell M50 公开产品参数和 M-50 CMG 学术文献参数，确定了本文仿真所采用的转子角动量、转速、框架运动约束及质量和功率预算。上述构型与参数将作为第二章飞行器—舵面—CMG 耦合动力学建模的基础。
