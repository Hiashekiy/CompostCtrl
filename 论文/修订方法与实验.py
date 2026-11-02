from pathlib import Path
import re, importlib.util, sys

root = Path(__file__).resolve().parents[1]
paper = root/'论文/气动舵面与CMG协同姿态控制方法及实验.md'
s = paper.read_text(encoding='utf-8')
first = (root/'论文/第一章_CMG集群构型与参数配置.md').read_text(encoding='utf-8')
# 保留既有几何推导；不改原章文件。
geometry = first.split('## 1.1 CMG 工作原理')[1].split('## 1.3 CMG 实物参考')[0]
geometry = geometry.replace('## 1.2 四台 SGCMG 金字塔构型', '#### 金字塔式集群')
geometry = re.sub(r'### 1\.2\.\d+ ', '#### ', geometry)
geometry = '### 1.2 CMG工作原理与集群构型\n\n'+geometry
config = r'''### 1.3 CMG配置及参数来源

本文以Honeywell M50的公开资料作为设备量级参照，并结合已有M-50研究中的框架运动参数建立仿真配置。产品额定指标、文献仿真参数和本项目设定分别列于表A，避免将三类来源合并为同一组硬件指标。

| 参数 | 公开产品或文献依据 | 本项目采用值 | 来源及含义 |
|---|---|---|---|
| 单台转子角动量 | M50公开范围25–75 N·m·s | 50 N·m·s | [1]产品规格汇总；选用范围内的仿真工作值 |
| 单台最大输出力矩 | M50公开值75 N·m | 模型由 $h_0|\nu_i|$ 确定 | [1]；本模型单台理想上限50 N·m，未直接采用75 N·m |
| 转子转速 | 产品范围4500–6500 rpm | 6000 rpm | [1]；[3]表1与[4]表2也采用6000 rpm |
| 框架角速度上限 | 文献为±1.0 rad/s | 1.0 rad/s | [3]表1，[4]表2 |
| 框架角加速度上限 | 文献为3.0 rad/s² | 3.0 rad/s² | [3]表1，[4]表2 |
| 金字塔斜置角 | 文献约54.7° | 54.7356° | 几何布置参数 |
| 框架速率环时间常数 | 本项目执行模型 | 0.03 s | 仿真假设，用于P1执行动态 |
| 单台质量 | 产品汇总为28 kg | 30 kg预算 | [1,2]；预算未形成增装后的惯量重设计 |
| 单台功率 | 产品汇总给出最大113 W | 200 W预算 | [1]；预算不代表实测功耗 |
| 框架行程 | 产品资料列为360° | 无有限边界的展开角 | [1]；数学假设不代表产品可无限连续旋转 |

表A CMG仿真参数及其来源。

其中，框架速率和加速度直接参考原始论文中的M-50参数表[3,4]，不是由产品最大力矩反推得到。角动量和转速选在产品公开范围内，但不同规格不应在未核对额定工况时视为可同时达到的性能。产品汇总页属于二次资料，尚不能代替设备定型所需的原厂额定工况与试验数据。

转子角动量满足 $h_0=J_r\Omega_r$。取6000 rpm时，$\Omega_r\approx628.32$ rad/s，由50 N·m·s得到 $J_r\approx0.0796$ kg·m²，理想转子动能 $E_r=h_0\Omega_r/2\approx15.7$ kJ。这些派生量用于检查设定的一致性，不替代结构强度、驱动电机、热设计与破裂包容验证。

在0.2 s控制周期下，框架速率单步变化上限为 $3.0\times0.2=0.6$ rad/s。四台集群的任意轴向输出仍由当前雅可比矩阵决定，不能以四倍单机力矩作为保证值。本文将上述配置作为数值研究基准，并未复刻M50的完整机械、电气及寿命特性。

'''
old = s.split('### 1.2 姿态与CMG耦合模型')[1].split('### 1.3 有限时域联合优化')[0]
# 几何推导已在1.2完整给出，耦合模型不再重复简略推导。
a = old.index('设第 $i$ 台CMG')
b = old.index('由角动量定理')
old = old[:a] + old[b:]
detail = r'''
#### 角动量守恒与构型相关的输出能力

系统在机体坐标系中的总角动量为 $J\omega+H(\gamma)$。其惯性系变化率等于外力矩，因此有

$$
\tau_a+\tau_d=J\dot\omega+A(\gamma)\nu+\omega\times[J\omega+H(\gamma)].
$$

将内部角动量变化项移至等式右侧，即得到式（3）。式中的 $-A\nu$ 表示框架运动产生的反作用力矩，$-\omega\times H$ 则来自机体坐标系转动，二者共同描述CMG与本体的耦合，不能重复计算或只保留其中一个。

当前构型对应的可达反作用力矩集合为

$$
\mathcal T_c(\gamma)=\{-A(\gamma)\nu:\ |\nu_i|\leq\nu_{\max}\}.
$$

对于任意单位方向 $d$，该集合沿该方向的支撑值为 $\nu_{\max}\|A(\gamma)^{\mathsf T}d\|_1$，但它不等同于同时约束其他轴力矩为零时的纯轴输出极限。当 $A$ 行满秩时，四个框架输入对应三轴力矩，存在一维瞬时零空间；当最小奇异值减小时，部分方向的控制能力减弱。因此，构型改变可能改变后续力矩可达集，即使角动量模长较大，也不保证所有方向均具备较大控制裕度。本文记录最小奇异值作为诊断量，没有在目标函数中加入奇异性奖励。

#### 气动总力矩与舵面执行模型

给定弹道提供高度和速度，大气模型得到密度，进而计算动压。参数化气动模型采用

$$
\tau_a=\bar q S[bC_l,\ \bar c C_m,\ bC_n]^{\mathsf T},\qquad
C_m=C_{m0}+C_{m\alpha}\alpha+C_{mq}\frac{\bar c\omega_y}{2V}+C_{m\delta}\delta_e.
$$

其中，$S,b,\bar c$ 为参考几何量，$\alpha$ 为迎角，$\omega_y$ 为俯仰角速度，$\delta_e$ 为升降舵偏角。将其分解为 $\tau_a=\tau_{\rm nat}+B_\delta\delta$，可区分自然与阻尼力矩和舵面偏转贡献。本文优化变量始终是两者之和，因此式（3）不再额外加入 $\tau_{\rm nat}$，以避免重复计入。

P0将优化力矩直接作用于姿态模型。P1先依据气动力矩映射分配舵偏指令，再以一阶环节推进实际舵偏角。其未限幅模型与幅值、速率约束为

$$
\dot\delta=(\delta_c-\delta)/\tau_s,\qquad
|\delta_j|\leq\delta_{\max},\qquad |\dot\delta_j|\leq\dot\delta_{\max}.
$$

实际气动力矩由实际舵偏角重新计算，而非直接使用优化输出。P1还以框架速率状态 $v$ 描述CMG速率环：

$$
\dot\gamma=v,\qquad \dot v=(\nu_c-v)/\tau_g,\qquad \tau_{\rm CMG}=-A(\gamma)v.
$$

其中，$\tau_s=0.05$ s，$\tau_g=0.03$ s，指令和状态按执行器边界限制。控制器仍以 $\dot\gamma=\nu$ 的降阶模型预测，P1用于检查忽略执行滞后的影响。该参数化气动模型没有经完整风洞或CFD数据库标定，不能据大迎角仿真推断真实失速或分离流特性。

'''
start=s.index('### 1.2 姿态与CMG耦合模型')
end=s.index('### 1.3 有限时域联合优化')
s=s[:start]+geometry+'\n'+config+'### 1.4 飞行器与执行机构耦合模型\n'+old+detail+s[end:]
s=s.replace('### 1.3 有限时域联合优化','### 1.5 有限时域联合优化').replace('### 1.4 滚动求解与提前构型迁移','### 1.6 滚动求解与提前构型迁移')
# 编号采用普通段落，避免预览器把tag送入aligned环境。
s=re.sub(r'\\tag\{(\d+)\}', '', s)
blocks=list(re.finditer(r'\$\$\n(.*?)\n\$\$',s,re.S))
# 原式号保持仅用于原有公式，按内容识别。
keys=[('x=[q',1),('\\dot q&',3),('\\underline\\tau_{a,k}\\leq',4),('e_{R,k}=2',5),('\\mathcal J={}',6),('\\min_{X,U}',7),('\\varepsilon_{\\rm RMSE}',8)]
for m in reversed(blocks):
    n=next((n for key,n in keys if key in m[1]),None)
    if n: s=s[:m.end()]+f'\n\n（式{n}）'+s[m.end():]
# 14幅已有实验图保留，并修正A/B图中峰值的统计口径。
spec=importlib.util.spec_from_file_location('paper_plot',root/'scripts/plot_no_cmg_comparison.py')
mod=importlib.util.module_from_spec(spec); spec.loader.exec_module(mod)
src=(root/'scripts/plot_no_cmg_comparison.py').read_text(encoding='utf-8')
src=src.replace('np.nanmax(errA)', 'ma["stats"]["combined"]["max_err_deg"]').replace('np.nanmax(errB)', 'mb["stats"]["combined"]["max_err_deg"]')
src=src.replace('短弧姿态误差 [°]', '姿态误差向量范数 [°]')
src=src.replace('机动窗 RMSE %.3f°，峰值 %.2f°', '机动窗 RMSE %.3f°，窗内峰值 %.3f°')
exec(compile(src,str(root/'scripts/plot_no_cmg_comparison.py'),'exec'),mod.__dict__)
mod.RES=root/'results/torque_gap_no_cmg_comparison_rti2'; mod.TAG='_rti2'
out=root/'论文/figures/修订'; out.mkdir(exist_ok=True)
for phase in ['P0','P1']:
    for sc,a,b in mod.collect(phase):
        mod.fig_scenario(phase,sc,a,b,out/f'{phase}_{sc}.png')
def fig(n,file,title):
    return f'\n\n![图{n} {title}]({file.as_posix()})\n\n图{n} {title}。\n\n'
scenes=['S00_pitch30','S01_pitch_minus30','S02_pitch10','S03_pitch45','S04_roll30','S05_yaw30','S06_pitch30_roll20','S07_sequential']
titles=['正向30°俯仰','反向30°俯仰','10°俯仰','45°俯仰','30°滚转','30°偏航','俯仰–滚转耦合','连续机动']
p0='\n#### P0典型响应与多场景曲线\n\n图1–图8依次给出八种任务的完整响应。各图包含姿态、误差、力矩、框架运动、角动量与求解耗时；图例中的RMSE和峰值均采用机动窗口口径，曲线保留机动前后全过程。图1、图4用于观察准备阶段和机动后行为，图6、图7用于检查偏航及轴间耦合。\n'
for n,(sc,t) in enumerate(zip(scenes,titles),1):p0+=fig(n,out/f'P0_{sc}.png',f'P0模型{t}的A/B对照')
s=s.replace('### 3.2 舵面执行动态下的交叉验证',p0+'### 3.2 舵面执行动态下的交叉验证')
p1='\n图9–图11给出执行动态下的完整响应，与表3采用相同运行数据。实际舵面响应与理想优化输入之间存在偏差，CMG通过额外力矩参与闭环补偿；机动后的误差仍须结合全过程曲线判断。\n'
for n,i in enumerate([0,1,6],9):p1+=fig(n,out/f'P1_{scenes[i]}.png',f'P1模型{titles[i]}的A/B对照')
s=s.replace('### 3.3 预测时域与提前构型迁移',p1+'### 3.3 预测时域与提前构型迁移')
mech='\n图12–图14分别展示L、S、Z配置。原图标题中的A、B、C依次对应本文L、S、Z，不与主体有无CMG的A/B分组混用。迁移时刻采用本节10°阈值口径。\n'
for n,name,title in [(12,'图4-13_A_完整预测时域.png','完整预测时域L的框架迁移与力矩响应'),(13,'图4-14_B_短预测时域.png','短预测时域S的框架迁移与力矩响应'),(14,'图4-12_C_无未来机动参考.png','零机动Z的框架与角动量响应')]:mech+=fig(n,root/'论文/figures'/name,title)
s=s.replace('### 3.4 求解开销与结果可靠性',mech+'### 3.4 求解开销与结果可靠性')
s+='''\n\n## CMG参数参考来源\n\n[1] [Honeywell Aerospace M50产品规格汇总，SatNow](https://www.satnow.com/products/control-moment-gyroscopes/honeywell-aerospace/43-1202-m50)。产品范围与质量功耗参照，二次资料。\n\n[2] [Honeywell M50 CMG，SatCatalog](https://www.satcatalog.com/component/m50-cmg/)。产品信息与原厂资料入口，二次资料。\n\n[3] Nanamori Y, Takahashi M. [Steering law considering biased loads for control moment gyroscopes](https://doi.org/10.1299/mej.14-00554). Mechanical Engineering Journal, 2015, 2(5). 参数依据见表1（PDF第9页）。\n\n[4] Nanamori Y, Takahashi M. [Steering Law of Control Moment Gyros using Optimization of Initial Gimbal Angles for Satellite Attitude Control](https://www.jstage.jst.go.jp/article/jsdd/5/1/5_1_30/_pdf). Journal of System Design and Dynamics, 2011, 5(1):30起。参数依据见表2（PDF第8页）。\n'''
paper.write_text(s,encoding='utf-8')
print('updated',paper,'images',len(re.findall(r'!\[',s)))
