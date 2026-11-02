# 任务说明：S08 大迎角/动压退化分析 与 Simulink 移植

本文档总结本轮全部工作，包含**已完成、已验证的内容**、**未完成的内容**、**已知缺陷**、
复现方法、文件索引与 git 记录。凡涉及数字，均可在文中给出的路径中找到原始产物。

- 仓库：`https://github.com/Hiashekiy/CompostCtrl`，分支 `experiment/torque-gap-modulated-ocp`
- 环境：Windows 10 22H2 / Intel i5-12600KF / Python 3.14（numpy 2.5.3、scipy 1.18.1、
  casadi 3.8.2 位于 `.pydeps/`）/ MATLAB R2024a（Simulink 24.1）/ MinGW-w64 GCC 15.2（`D:\w64devkit`）
- 本轮**未**修改控制器、MPC 权重、CMG 构型、M50 框架限幅、舵面参数与任何仿真窗口

---

## 第一部分：S08 大迎角与动压耦合下的姿态控制能力退化分析

对应方案 `docs/S08_aero_degradation_experiment.md`。

### 1. 代码与模型审计（`docs/torque_gap/S08_code_audit.md`）

审计结论（决定了后面所有实验的设计与解读口径）：

| 审计对象 | 结论 |
|---|---|
| 状态方程 | **只有姿态自由度**（q、ω，B 组再加 γ）：`q̇=½q⊗[0,ω]`，`Jω̇=τa−A(γ)ν+τd−ω×(Jω+H)`。**没有平动方程**；高度/速度/密度来自解析跳跃弹道 + MSIS2 大气，是外部给定 |
| 迎角 | `α=atan2(w_b,u_b)`，`V_b=R(q)ᵀV_ned`；实测与 `θ−γ` 逐点一致（差值恒为 0），参考迎角 ≡ `alpha_trim` |
| 动压 | `q̄=½ρ|V_ned|²`；气动力矩、舵面能力盒、扰动幅值都按 q̄ 标度 |
| 气动模型 | `Cm=Cm0+Cm_α·α+Cm_q·q·ĉ+Cm_δe·δe`，**对 α 严格线性**；`Cm_δe` 与 α 无关；**无 CL/CD、无力方程**；**与马赫数/雷诺数/Knudsen 数无关**；无声明适用范围 |
| 舵面 | P0：虚拟总力矩 + 能力盒（盒半宽 = 0.7738×可达半宽，标定得）；P1：18 状态真值，有界最小二乘舵偏分配 + 0.05 s 舵机 + 60°/s 速率限 |
| CMG | 4 台金字塔，h0=50 N·m·s，|ν|≤1.0 rad/s，|ν̇|≤3.0 rad/s²，τg=0.03 s；σmin_home=40.82，奇异下限 12.25 |
| CMG 开关 | `cmg_present` 物理开关（A 组 7 状态/3 输入、cluster 为 None、无 H 无 A(γ)ν）与 `cmg_enabled` 解耦；用 γ=[0.3,0.1,−0.2,0.4]、|H|=30.76 证明"冻结框架 ≠ 无 CMG"（角加速度差 4.58e-5 rad/s²） |
| 日志 | 原日志缺 α、β、Mach、Cm、舵面利用率；本轮新增 `nearspace/aero_diagnostics.py` 与 20+ 个观测列（不进入任何方程），CL/CD 写空单元格并在 `log_conventions` 标注"未建模" |

### 2. 实验矩阵（72 次运行，全部通过）

| 分组 | 工况 | 臂 | 植物 | 运行数 |
|---|---|---|---|---|
| A1 迎角扫描 | α = −10/0/10/20/30/40°，q̄ = 18.147 Pa 固定 | A/B | P0+P1 | 12+12 |
| A2 动压扫描 | α = 2° 固定，q̄ = 9.07/18.15/36.29/72.59 Pa | A/B | P0+P1 | 8+8 |
| A3 耦合 | α ∈ {0°,30°} × q̄ ∈ {0.5,2}×Q0 | A/B | P0 | 8 |
| A4 复现 | 原始 S00(+30°)、S03(+45°) | A/B | P0+P1 | 4+4 |
| 种子 | A4 两工况 × 两臂 × 种子 1–4（扰动相位随机化） | A/B | P0 | 16 |

构造要点：A1–A3 用**平飞定常条件**（`amp=0`、`dVdt=0`）使 γ≡0、q̄/ρ/V/M/Kn 恒定；每个受控工况叠加
**同一条 +8° 俯仰凸起**（110–130 s，bump）以获得加载过程；外扰按原弹道物理规律折算
（`Md_amp × q̄_case/q̄_30km`），使同一 q̄ 下与 A4 看到的外扰一致。**这两点是边做边发现并修正的**
（不修正的话受控工况外扰被放大 ~1000 倍，且纯保持工况无梯度）。

### 3. 主要结果（详见 `docs/torque_gap/S08_results_analysis.md` 与 `results/torque_gap_s08/summary.md`）

- **复现**：A4 与历史运行**逐位一致**（3.3258 / 0.4331 / 8.2643 / 9.4900，差 0.0e+00）。
- **A1（固定 q̄）**：仅舵面臂 RMSE 0.044°(α=0) → 0.168°(10°) → 0.745°(20°，贴边 18.4%) →
  **47.67°(30°)、75.70°(40°) 崩溃**；CMG 臂在 α≤20° 保持 ≤0.005°，但 `|H|` 用到 56–82%、
  σmin 最低 0.05–0.1。舵面**效能恒定**（τ_half≡47.89 N·m），只有**需求**随 α 线性增长。
- **A2（固定 α）**：q̄ 提高 8 倍使跟踪从 0.559° **改善**到 0.000°；退化只在低动压端(≤0.5Q0)。
  → 方案假设 H2（动压↑→退化）在本模型中**不成立**。
- **A3（耦合）**：交互项 $I_{\alpha q}$(RMSE) = +55.52°(A) / +52.27°(B)，显著为正；
  但机制是"迎角决定是否越界、动压决定越界后幅度"，**不是**气动系数非线性（模型无此机制）。
- **A4（原始退化）**：失效发生在窗口内**低动压段**（q̄ 3–8 Pa，95–130 s）；S00 中 A 组 97.2 s 起持续贴边、
  102.8 s 起误差持续 >1°；B 组推迟到 148.4 s，代价是 `|H|` 80%、σmin 掉到奇异下限以下（最低 0.6）。
  S03 中 B 组全程反而更差（9.49° vs 8.26°）：98–111 s 把角动量与奇异裕度一次用尽。
- **P1（真实舵面）**：趋势一致但幅度放大（S00 A/B 6.26°/0.38°，S03 A/B 18.07°/7.64°）；
  即使 α≈0，仅舵面臂也有 ~2–3° 固有误差（舵机滞后+有界分配），CMG 补的正是这块动态缺口。
- **种子**：模型确定性（自跑复现 5e-9）；S03/A 对相位不敏感（极差 0.006°），S03/B 极差 2.22°（近奇异混沌）。

### 4. 交付判定（方案 §16 十问的答案见结果报告 §7）

结论分三层：**事实**（只有姿态自由度、α 线性、效能只随 q̄ 变、复现通过、阈值位置）、
**受控实验支持的物理解释**（退化=需求超能力；CMG 推迟失效但消耗储备）、
**仍需真实数据验证的推测**（真实失速/稀薄效应/平动耦合）。
**A5（非线性气动对照）未执行**：模型确是线性、满足前提，但仓库无任何可追溯气动数据，
方案 §9 禁止人为构造失速曲线，故以"缺少气动数据"交付。

### 5. 产物索引

| 内容 | 路径 |
|---|---|
| 审计 / 结果 / 配置审计 / 复现核查 | `docs/torque_gap/S08_code_audit.md`、`S08_results_analysis.md`、`results/torque_gap_s08/{config_audit.md,replay_check.md}` |
| 逐运行指标（72 行） | `results/torque_gap_s08/summary.{csv,md,json}`、`interaction.json` |
| 配置（48 个文件） | `configs/torque_gap_s08/{base.yaml,scenarios/*,cases/*,design.json}` |
| 原始数据 | `results/torque_gap_s08/<group>/<run_id>/{timeseries.csv,metrics.json,meta.json,config_resolved.yaml}` |
| 图（42 张 + 汇总） | `figures/torque_gap_s08/` |
| 脚本 | `scripts/{build_s08_matrix,check_s08_config,run_s08_matrix,replay_check_s08,analyze_s08_aero,summarize_s08_factorial,plot_s08_overview}.py` |

---

## 第二部分：Simulink 移植（MIL）

### 6. 实际实现方式（**这是我必须如实说明的地方**）

当前交付的 `.slx` 模型里只有 **3 个块**：

```
s08_replay_sfun ──► s08_plant_sfun ──► Outport(x)
```

- 两个块都是 **Level-2 MATLAB S-Function**（自定义 M 代码），状态存在 `Dwork(1)` 里；
- `s08_plant_sfun` 每个 0.2 s 周期内部做 8 次 RK4 子步（与 Python `ClosedLoop._step_P0` 同构）；
- 也就是说：**Simulink 在这里只承担"调度 + 日志"**，物理、CMG 几何、执行机构限幅、
  控制器全部在 M / Java / Python 代码中，模型本身**没有可视化结构**（没有 Integrator/Sum/Gain 子系统、
  没有气动/CMG/刚体姿态的可见分区、没有 Scope）。

这样做换来的是"可与 Python 逐位对比"，牺牲的是"看得懂的模型"。
**对"要一个真正的 Simulink 模型"这一诉求而言，该交付不合格。**

### 7. 已完成的验证（这部分是真实有效的）

| 判据 | 内容 | 结果 |
|---|---|---|
| **C1** 单步植物等价 | 7 个工况 × 8 个随机点，与 `ClosedLoop._step_P0` 比 | **max\|Δx\| = 2.2e-16** |
| **C3** 开环回放 | 用 Python 录制的 τa、ν 驱动 Simulink 植物 300 步 | **max 相对\|Δstate\| = 4.0e-10**，max\|Δ姿态误差\| = 9.8e-9° |
| **C2** 控制器等价 | 把录制状态喂回控制器服务器 | A 组 1.8e-05 N·m；B 组受非光滑 QP 影响分离 |
| **C4** 闭环 | 同一 Python RTI 控制器（TCP）+ 同一植物，6 个工况 | 非混沌工况 1e-9~1e-12；两个近奇异工况差异=模型自身敏感性本底 |
| **混沌本底归因** | 给**参考模型自己**注入 1e-9 rad 扰动 | S00/B 0.637°、S03/B 7.35°；移植闭环 0.68°、13.9° → 同量级，**移植无系统误差** |

报告：`results/simulink_port/report.md`；规格与调试记录：`docs/torque_gap/simulink_port_spec.md`。

### 8. 环境适配（踩过的坑，全部记录在 spec §6）

| 现象 | 处理 |
|---|---|
| `Fatal Startup Error: failed to load settings errors_warnings plugin` | 沙箱不允许写 `%APPDATA%\MathWorks`；`simulink/mlab.cmd` 把 `APPDATA/LOCALAPPDATA/TMP/TEMP/MATLAB_PREFDIR` 重定向到仓库内 `.matlab_*`（已 gitignore） |
| `matlab.System` 的 name-value 构造器异常（最小示例也复现） | 放弃 System object，改用 Level-2 M-S-Function |
| Level-2 块 mask `Parameters` 报 "no dialog parameters" | 改文件通道 `data/CURRENT_*.json` |
| `block.UserData` 在 `setup` 不可写 | 用函数 `persistent` 缓存参数 |
| `evalin('base')` / `From Workspace` 取不到数据 | 改为 `s08_replay_sfun` 从 CSV 回放 |
| `bdroot(block.BlockHandle)` 返回 `'?'` | 不再依赖模型名/句柄 |
| 中文路径 `D:\项目\飞轮` | 实测文件 I/O、`dlmread`、S-Function 均正常，仅控制台 GBK 乱码 |
| 闭环 300 步 87.6 s | TCP Nagle + 延迟 ACK；两侧 `TCP_NODELAY` 后 **39.4 s**（0.13 s/步，与进程内一致） |

### 9. 工具链（与最初结论相反，已更正）

`D:\w64devkit`（独立 MinGW-w64，**GCC 15.2.0**）**可用**：`simulink/mlab.cmd` 设
`MW_MINGW64_LOC`，MATLAB 自带 `mingw64.xml` 即可选中。实测：

- 纯 C MEX：`mex simulink/mexopts/s08_hello.c` → 运行输出 `gcc 15.2.0`、`2*21 = 42`；
- **CasADi → C → mex 全链路**：`mex probef_mex.c probef_plain.c` 成功，`probef_mex([3;0]) = 9`。
- 坑：`generate()` 只吃纯文件名（带目录触发 `check_name` 断言）；导出入口按 `Function` 名命名；
  win64 上 `casadi_int` 是 `long long int`；`arg/res` 尺寸要用 `<name>_work()` 查。
- HPIPM 现成：`.pydeps/casadi/libhpipm.dll`；w64devkit 带 `gendef/dlltool`。
  → 结论：`D2①(C+S-Function 控制器)` **环境不再是障碍，剩下的只是移植工作量**（预估 1–2 天）。

### 10. 产物索引

| 内容 | 路径 |
|---|---|
| 模型（13 个 .slx，7 回放 + 6 闭环） | `results/simulink_port/runs/*.slx` |
| 运行数据与指标 | `results/simulink_port/runs/*_{replay,closedloop}.{csv}` / `_metrics.json` |
| 报告 / 环境探测 | `results/simulink_port/{report.md,plant_check.json,controller_check.json,sensitivity_*.json,env_report.txt}` |
| 图（15 张） | `figures/simulink_port/` |
| MATLAB 侧代码 | `simulink/{mlab.cmd,s08_startup.m,open_case.m,run_case.m,check_plant_step.m,check_env.m}`、`simulink/blocks/*.m` |
| Python 侧 | `simulink/py/{export_for_simulink.py,ctrl_server.py,run_closedloop.py,check_controller_replay.py,sensitivity_check.py,make_report.py}` |
| 工具链验证 | `simulink/mexopts/s08_hello.c`、`simulink/mexopts/codegen/{probef_plain.c,probef_mex.c}` |

### 11. 手动运行方法

```matlab
cd D:\项目\飞轮
run('simulink/s08_startup.m')                          % 加路径 + 列出模型
open_case('A4_descent_replay_S00_with_cmg_P0','replay') % 刷新该工况输入并打开模型 → 点 Run
```

闭环模型需先起控制器（另开终端，端口 55901）：

```bash
python simulink/py/ctrl_server.py --case configs/torque_gap_s08/cases/A4_descent_replay_S00_with_cmg_P0.yaml --port 55901
```

> 必须走 `open_case`：模型从固定路径 `data/CURRENT_case.json`、`CURRENT_u.csv` 读输入，
> 直接双击 .slx 会复用上一次运行的工况快照（A 组 7 状态、B 组 11 状态，维度不符会报错）。

---

## 第三部分：未完成与已知缺陷

### 12. 明确的未完成项

| 项 | 状态 | 说明 |
|---|---|---|
| **可见结构的 Simulink 模型** | **未做**（本次最大缺陷） | 已设计：`飞行条件(查表)/气动/CMG 金字塔/刚体姿态(Integrator×3+Sum+Gain+Cross)/执行机构(Saturation/Rate Limiter)/控制器(0.2 s 离散)`，四元数乘与 CMG 几何用 MATLAB Function 块；连续求解器 ode4 0.005 s，与 Python 参考预期差 ~1e-6（不再是 1e-16）。**仅有起始脚本 `simulink/py/export_visible_model.py`（未跑通、未验证）** |
| P1 植物（18 状态真实舵机）移植 | 未做 | S08 的 P1 结论对"舵面是瓶颈"最关键，Simulink 侧缺失 |
| A2/A3 的闭环模型 | 未做 | 只做了 A1/A4 的 6 个闭环工况 |
| 代码生成 / PIL / HIL | 未做 | 需 Simulink Coder（未授权）与硬件 |
| Aerospace Blockset 主链 | 未做 | `license('test')=1` 但本安装 `load_system('aeroblks')` 等全部失败，块库在盘中却加载不了，未进一步排查 |
| A5（非线性气动） | 未做（有意） | 无可追溯气动数据，按方案 §9 不应构造失速曲线 |

### 13. 已知缺陷 / 需要接手者注意

1. **`.slx` 是"黑盒 + 调度器"，不是可读的框图**（见 §6）。除 C1/C3/C4 的等价性证据外，
   它对"看模型、改模型"这件事几乎没有价值。
2. 模型从**固定路径**读输入（`data/CURRENT_*.json/csv`），是单槽设计；并行跑两个工况会互相覆盖。
   这是为绕开本安装 Simulink 数据通道故障而做的妥协。
3. 闭环需要 **第二个进程**（Python RTI over TCP）；虽然保真等价，但不是"纯 Simulink"。
4. 植物是解释执行的 M-S-Function：回放 1–3 s、闭环 15–40 s（300 步），不可用于实时性结论。
5. MATLAB 会警告 w64devkit 的 GCC 版本不在官方支持列表（功能正常，非官方组合）。
6. 仓库里仍有 21 个**本轮之前就存在**的本地修改（`figures/torque_gap_no_cmg_comparison/*.png`、
   `scripts/plot_no_cmg_comparison.py` 等）与未跟踪目录 `论文/`，**我没有动它们**，也未提交。

---

## 第四部分：git 记录与复现

### 14. 本轮提交（均在 `experiment/torque-gap-modulated-ocp`，已推送）

| commit | 内容 |
|---|---|
| `2709cd5` | S08：审计 + A1–A4 矩阵 + 报告（472 文件） |
| `e4e9636` | Simulink MIL 移植：植物/控制器在环/等价性证据（110 文件） |
| `02a2301` | `open_case` 助手与 README 指针 |
| `9159af5` | 启用 w64devkit 工具链、修复 TCP 开销、文档更新 |
| `56dd1f3` | CasADi→mingw→mex 链路验证与坑记录 |
| `ccce859` | `s08_startup.m`、清理模型回调、README 手动运行说明 |
| `a020882` | 删除未采用的每模型快照与 PreLoadFcn 助手 |

### 15. 复现命令

```bash
# S08 实验（72 次运行）
python scripts/build_s08_matrix.py && python scripts/check_s08_config.py
python scripts/run_s08_matrix.py --phase P0   --jobs 8
python scripts/run_s08_matrix.py --phase P1   --jobs 6
python scripts/run_s08_matrix.py --phase seed --jobs 6
python scripts/replay_check_s08.py
python scripts/analyze_s08_aero.py && python scripts/summarize_s08_factorial.py
python scripts/plot_s08_overview.py --plant P0 && python scripts/plot_s08_overview.py --plant P1

# Simulink 移植（C1/C3/C4）
python simulink/py/export_for_simulink.py --all
simulink\mlab.cmd "addpath('simulink'); check_plant_step"
python simulink/py/check_controller_replay.py
python simulink/py/run_closedloop.py --all
python simulink/py/sensitivity_check.py --tag A4_descent_replay_S00_with_cmg_P0
python simulink/py/make_report.py
```

（`mlab.cmd` 需 `MATLAB_HOME` 指向 `D:\Program Files\MATLAB\R2024a`。）

### 16. 若继续这项工作，建议顺序

1. **先做可见结构模型**（本次缺陷的正解）：按 §12 的子系统划分搭框图，参数用 `Simulink.Parameter`，
   加 Scope/To Workspace；以现有 S-Function 版为回归基线，验收口径从 1e-16 放宽到 ~1e-6。
2. 移植 P1 植物（真实舵机 + 有界分配），它是 S08 结论里"舵面是瓶颈"的证据来源。
3. 有了工具链（GCC 15.2 + HPIPM DLL + codegen 已验证），再把控制器 C 化，去掉 Python 进程。
4. 最后才是代码生成 / HIL。
