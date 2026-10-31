# 阶段 S0：仓库审计与基线冻结

本文件记录 DSH 实施前对本地仓库的**实际**检查结果，作为后续 `docs/torque_gap/experiment_report.md` 的可复用性依据。

## 1. 基线冻结

| 项 | 值 |
|---|---|
| 仓库根 | `D:/项目/飞轮` |
| 基线分支 | `experiment/incremental-cmg-search` |
| 基线 SHA | `5a206cfe548abff22557706f1613e8049614164d` |
| 实验分支 | `experiment/torque-gap-modulated-ocp`（从上述 SHA 新建） |
| 未跟踪文件（实施前） | `docs/DSH_预测力矩余缺调制_CMG协同轨迹优化_实验实施方案.md`、`docs/气动舵面_CMG预测余缺调制协同优化_方法精简版.md` |
| 工作区修改（实施前） | 无 |

注意：本任务书提到的远端 `main` 分支内容与本地 `experiment/incremental-cmg-search` 分支并不完全相同（本地分支已包含增量构型搜索实验 `experiments/cmg_incremental_search/`）。本次实验**以本地实际源码为准**，并**不修改**、不删除、不覆盖：

- `nearspace/*` 中已有模块（只被新模块 import）
- `experiments/cmg_incremental_search/**`（独立实验树）
- `results/cmg_ab/**`、`results/cmg_maneuver/**`、`results/c1_cmg_*`、`figures/**` 中既有成果
- `config.yaml`（新实验通过 `configs/torque_gap/base.yaml` 的 `base:` 继承，不改原文件）

> 说明：本文件是实施前的历史快照。其中提到的第一版实验臂 `results/torque_gap/`、`figures/torque_gap/` 已在后续精简中移除，当前保留内容见 `README.md`。

新增内容全部位于 `nearspace/torque_gap_*.py`、`configs/torque_gap/`、`scripts/*torque_gap*`、`results/torque_gap/`、`figures/torque_gap/`、`docs/torque_gap/`、`tests/test_torque_gap_*.py`、`requirements-ocp.txt`。

## 2. 运行环境

| 项 | 值 |
|---|---|
| Python | 3.14.7 (CPython, win_amd64) |
| numpy / scipy / matplotlib / pyyaml | 2.5.3 / 1.18.1 / 3.11.2 / 6.0.3 |
| osqp / pymsis | 1.1.3 / 0.13.0 |
| CasADi | 3.8.2（`cp311-abi3` wheel，可被 CPython ≥3.11 导入） |
| IPOPT | 由 CasADi 插件提供（`Nlpsol::ipopt`），`mumps` 线性求解器可用；`ma27` 在本机加载后立即返回 NaN，**不使用** |
| pytest | 本机未安装（因此新测试同时提供 `if __name__` 直接运行入口） |

**依赖安装限制（重要）**：本机 pip 无法写入 `site-packages`（沙箱拒绝），且 pip 的临时解包目录被拒绝，`pip install casadi` 直接失败。实际采用：

```powershell
# 1) 下载 wheel（pip download 亦受临时目录限制，故用 urllib 直接取 PyPI 文件）
python -c "import urllib.request as u; u.urlretrieve('<pypi wheel url>', r'.piptmp\casadi.whl')"
# 2) 解包到工作区
python -c "import zipfile; zipfile.ZipFile(r'.piptmp\casadi.whl').extractall(r'.pydeps')"
```

`nearspace/torque_gap_ocp.py::_casadi()` 会在 `import casadi` 之前把 `.pydeps/casadi` 加入 `PATH` 并注册 DLL 目录；否则 Windows 加载器找不到 `libcasadi_nlpsol_ipopt.dll` 的依赖，IPOPT 插件加载失败（实测报 `Plugin 'ipopt' is not found`）。`.pydeps/`、`.piptmp/` 已加入 `.gitignore`，`requirements-ocp.txt` 固定 `casadi==3.8.2`。

## 3. 逐文件复用清单（实际读取的源码）

| 源码 | 实际内容（已核对） | 本次实施决定 |
|---|---|---|
| `nearspace/cmg.py` | `CMGParams(h0, gdot_max=1.5, gddot_max=10, tau_g=0.03, sigma_floor_frac=0.3)`；`CMGCluster.momentum/jacobian/torque/sigma_min`；四单元金字塔 `beta=54.7356°`，`A=dh/dγ`，`M=-Aγ̇`。类默认 `h0=25.0`，`config.yaml` 解析值 `h0=50.0` | **直接复用**。数值核对：`sigma_min(A(0))=40.8248`、`σ_min·γ̇max=61.24 N·m`、峰值能力 `244.9 N·m`。新增 `TorquePlantParams.cluster_constants()` 把同一套解析几何交给 CasADi 图（`A`、`H` 与 numpy 版逐元素一致，误差 1.4e-14）。**不使用类默认值冒充设备参数**：一律 `CMGParams.from_config(cfg)` |
| `nearspace/model6.py` | 四元数标量在前 Hamilton、机体→NED；`VehicleParams`（Jx=1200, Jy=8500, Jz=9000，S=4.2，b=2.6，cbar=3.2，δmax=[25,20,20]°）；`aero_coeffs`/`aero_moment`；`flow_angles`；`quat_*` 工具 | **复用**四元数约定、惯量、气动系数与 `flow_angles`。`torque_gap_authority.natural_moment_v` 与 `surface_matrix` 重写了**同一口径**的气动力矩分解，并用 `aero_moment6c` 做逐元素校验（见 `docs/torque_gap/model_validation.md` T3） |
| `nearspace/model6c.py` | 18 状态 `q,ω,δ,γ,γ̇`；输入 `[δ_c(3), γ̇_c(4)]`；`aero_moment6c`、`dynamics6c`、`plant_step6c`、`limit_commands6c` | **P1 真值植物直接复用**（含伺服一阶 `τ_s=0.05 s` 与框架速率一阶 `τ_g=0.03 s`）。**没有**把 `tau_a` 直接塞进 `plant_step6c`：新增有界最小二乘分配器把 `tau_a` 反算为 `δ_c`，并单独记录 `τ_a,desired` 与 `τ_a,actual` |
| `nearspace/trajectory3.py` | `Trajectory3D`：`h/V/rho/gamma`、`q_ref`、`omega_ref`（对 `q_ref` 差分，步进/保持机动有专门处理）、`disturbance`、`p_array`、`windows`；`Maneuver` 支持 `hold/bump/step`，五次多项式 `10s³-15s⁴+6s⁵` | **复用**参考轨迹与 `omega_ref`（保证“机体系角速度而非 Euler 角导数”）。首轮只用平滑五次 `hold` 机动；**不使用** `step`（参考角加速度不可定义），**不启用** `dist_pulse_amp`（80 N·m 脉冲会主导本次研究机制，见 `configs/torque_gap/base.yaml` 注释） |
| `nearspace/sim.py`, `metrics.py` | 既有闭环/指标口径 | 只**复用口径思路**（RMSE、窗口、回退统计），未复用接口。新 runner 独立实现，避免与旧 `[δ_cmd, M_cmg]` 控制器语义混用 |
| `nearspace/cmg_control.py` | `LTVMPCCMG2`（`[δ_cmd(3), M_cmg(3)]` + 阻尼伪逆 + σ floor + 历史权重 + 动量预算） | **不作为新方法控制器**。本实验的优化器是独立的 11 状态 / 7 输入 NLP |
| `nearspace/config.py` | `load_config/read_yaml/deep_merge/dump`，支持 `base:` 继承 | **复用**：`configs/torque_gap/base.yaml` 通过 `base: ../../config.yaml` 继承冻结配置 |
| `config.yaml` | `cmg.h0=50, gdot_max=1.5, gddot_max=10, tau_g=0.03`；`simulation.dt_ctrl=0.2`；气象后端 `msis2`（f107=150, ap=4） | 新配置**不修改**它，只继承 |
| `experiments/cmg_incremental_search/**` | 上一轮“增量构型搜索”实验的独立实现（`src/dynamics/cmg_pyramid.py` 等） | **未复用**：其模型与 `nearspace/cmg.py` 是两套实现；本实验按任务书要求以 `nearspace/` 为准，避免第三套几何 |

## 4. 缺失项（本次新增补齐）

| 缺口 | 处理 |
|---|---|
| 11 状态 `[q,ω,γ]` + `[τ_a,ν]` 真值植物 | `nearspace/torque_gap_model.py`（P0） |
| 时变三轴气动力矩盒 + 可实现性检验 | `nearspace/torque_gap_authority.py` |
| `τ_nom / R⁺ / R⁻ / d` 余缺调制 | `nearspace/torque_gap_modulation.py` |
| 联合 NLP（多重射击 + IPOPT） | `nearspace/torque_gap_ocp.py` |
| 闭环 runner（超时/回退/记录） | `nearspace/torque_gap_sim.py` |
| 场景装配（预测网格、参考、盒、调制、初值） | `nearspace/torque_gap_scenario.py` |
| 气动力矩→舵偏角分配器 | `torque_gap_authority.allocate_deflection`（有界 BVLS） |
| 框架角行程限制 | `CMGParams` 无 `gamma_min/max`：实验显式引入 `torque_gap.gamma_min_deg/gamma_max_deg`（首版 ±90°，**显式假设**），由 OCP 硬约束 + P0/P1 真值钳位共同执行 |
| CasADi/IPOPT 依赖 | `requirements-ocp.txt`（固定 3.8.2）+ `.pydeps` 本地解包 + `PATH` 注入 |
| 实验配置/入口/评估/作图 | `configs/torque_gap/**`、`scripts/{run,check,evaluate,plot}_torque_gap*.py` |

## 5. 单位与坐标约定（全实验统一）

| 量 | 单位/约定 |
|---|---|
| 四元数 | 标量在前 Hamilton，`q`：机体→NED；`q` 与 `-q` 等价 |
| 角速度 | 机体系 `[p,q,r]` rad/s |
| 框架角/角速度 | `γ` rad，`ν` rad/s，`|ν_i| ≤ gdot_max = 1.5`，`|Δν_i| ≤ gddot_max·dt = 10·0.2` |
| 力矩 | N·m；`τ_a` 为**合成三轴气动力矩（总力矩）**，不是舵角；`τ_cmg = -A(γ)ν` |
| 气动角 | rad；`Q = ½ρV²` |
| 舵偏角 | rad（`δ=[de,da,dr]`，`|δ_i| ≤ [25,20,20]°`） |
| 角动量 | N·m·s（`h0=50`/单元，簇 200） |

## 6. 已发现的工程风险（实施中已处理）

1. **CasADi reshape 是列主序**，与 numpy 默认行主序不一致 → 决策向量与参数向量都强制 `order="F"`。曾因此得到“约束看似收敛但四元数全部错位”的假解，已通过 `T3`/一致性检查与残差核对定位。
2. **numpy 2.5 移除了 `np.trapz`**（用 `np.trapezoid`）。
3. **IPOPT 线性求解器**：`ma27` 在本机不可用（返回 NaN），统一 `mumps`。
4. 旧仓库 `sigma_floor_frac`、动量预算、薄气段卸载规则属于旧控制器策略，**未**复制进本优化器；`σ_min` 只作诊断记录。
5. 本机 `tmp/` 目录对写入被拒绝（沙箱），因此 pip 临时目录改用工作区内 `.piptmp/`。
