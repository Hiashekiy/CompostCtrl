# `docs/torque_gap/` 与仓库现状索引

本仓库在 `7ee44e1` 之后做了一次精简：只保留 **①仅舵面对照 ②有 CMG 的当前配置
③提前构型迁移机制验证** 三组实验，以及支撑它们的模型、求解器与校验脚本。
其余历史实验臂、早期 CMG/MPC 遗留项目产物、过时报告已删除（全部仍可从 git 历史取回）。

## 1. 保留的实验（三组）

| 组 | 配置 | 数据 | 图 | 报告 |
|---|---|---|---|---|
| ① 仅舵面（CMG 冻结） | `configs/torque_gap_noCMG/cases/c1_pitch30.yaml` | `results/torque_gap_noCMG/C1_B0_P0_noCMG/` | `figures/torque_gap_noCMG/C1_surface_only_comparison.png/.pdf` | `C1_surface_only_comparison.md` |
| ② 有 CMG（当前 M50 配置） | `configs/torque_gap_m50/cases/c1_pitch30.yaml` | `results/torque_gap_m50/C1_B0_P0/` | `figures/torque_gap_m50/C1_M50_overview.png/.pdf`、`C1_M50_vs_original.png/.pdf` | `C1_M50_overview_analysis.md`、`C1_M50_rate_limit_comparison.md` |
| ② 对照：原假设 γ̇=1.5/γ̈=10 | `configs/torque_gap_unlimited/cases/c1_pitch30.yaml` | `results/torque_gap_unlimited/C1_B0_P0/` | `figures/torque_gap_unlimited/C1_control_overview.png/.pdf` | `C1_control_overview_analysis.md` |
| ③ 提前构型迁移机制（A/B/C） | `configs/torque_gap_prediction_mechanism/` | `results/torque_gap_prediction_mechanism/{A_full_horizon,B_short_horizon,C_no_future_maneuver}/` | `figures/torque_gap_prediction_mechanism/{A,B,C}_overview.png/.pdf`、`mechanism_comparison.png/.pdf` | `prediction_mechanism_experiment.md` |

② 中原假设臂（γ̇=1.5 rad/s、γ̈=10 rad/s²）之所以单独保留一次运行，是因为它是
“M50 vs 原假设”和“单舵面 vs 有 CMG”两张对比图的数据源。

## 2. 保留的辅助材料

| 文件 / 目录 | 作用 |
|---|---|
| `code_audit.md` | 实施前对本地仓库的检查记录（历史快照） |
| `model_validation.md` | 模型/权限/调制量的 43 项逐条校验（`scripts/check_torque_gap_model.py` 生成） |
| `modulation_removed_report.md` | 删除 `J_cfg` 调制机制的回归验证记录 |
| `C1_noMod_overview_analysis.md` + `results/torque_gap_noMod/C1_P0/` | 上述删除的对照运行（无调制代码） |
| `results/torque_gap_unlimited/model_validation.md` | 无限行程模型的 32 项校验（`scripts/check_torque_gap_unlimited.py` 生成） |
| `results/model_checks_bounded.json` | 43 项校验的机读结果（含气动权限标定 0.774） |
| `results/msis2_table_*.csv` | 大气表（`nearspace/atmosphere_real.py` 运行时读取） |
| `docs/references/AST-V1.pdf` | 参考文献 |
| `docs/CMG提前构型迁移与角动量预置机制验证实验方案.md` | 第 ③ 组实验的任务书 |
| `docs/DSH_预测力矩余缺调制_CMG协同轨迹优化_实验实施方案.md` | 最初的 S0–S5 任务书（其第一版实验臂已删除） |
| `docs/系统建模与统一MPC-数学形式.md` | 数学基础（11 状态模型、四元数约定、CMG 几何） |

## 3. 代码结构

```
nearspace/
  torque_gap_model.py       11 状态真值模型（动力学 / RK4 / 姿态误差）
  torque_gap_authority.py   舵面权限矩阵、矢量化舵偏分配（BVLS）、气动盒标定
  torque_gap_modulation.py  τ_nom、R⁺/R⁻、缺口 d、E_k —— 仅诊断量，不进目标函数
  torque_gap_scenario.py    预测网格、气动盒、参考轨迹、`cmg_enabled`/`gamma_limit`
  torque_gap_ocp.py         多打靶 NLP（CasADi + IPOPT，精确 Hessian，mumps）
  torque_gap_sim.py         闭环仿真（P0 虚拟力矩植物 / P1 真实舵偏）、日志与指标
  model6.py model6c.py cmg.py trajectory3.py atmosphere*.py config.py   基础模型
```

`scripts/` 只保留 9 个：`run_torque_gap.py`（单次运行）、两个 `check_*`（模型校验）、
`plot_c1_*` 三个（作图与派生报告）、`analyze_prediction_mechanism.py`、
`check_prediction_mechanism_configs.py`（配置审计）、`bench_ocp_modulation.py`（求解器基准）。

## 4. 复现

```powershell
$env:PYTHONPATH = "$PWD\.pydeps"          # casadi 位于 ./pydeps（见 .gitignore 注释）

# 模型校验
python scripts/check_torque_gap_model.py          # 43 项
python scripts/check_torque_gap_unlimited.py      # 32 项

# 单元测试
python tests/test_torque_gap_model.py
python tests/test_torque_gap_unlimited.py
python tests/test_atmosphere_real.py

# 三组实验的作图与派生报告（数据已在仓库内，无需重跑仿真）
python scripts/plot_c1_surface_only.py            # ① 仅舵面 vs 有 CMG
python scripts/plot_c1_m50_comparison.py           # ② M50 vs 原假设
python scripts/plot_c1_control_overview.py --run results/torque_gap_m50/C1_B0_P0 `
    --fig-root figures/torque_gap_m50 --stem C1_M50_overview --doc C1_M50_overview_analysis.md
python scripts/analyze_prediction_mechanism.py    # ③ A/B/C 指标、总图、对照图

# 需要重跑某次仿真时
python scripts/run_torque_gap.py --case c1_pitch30 --plant P0 `
    --config-root configs/torque_gap_m50/cases --out results/torque_gap_m50/C1_B0_P0
```

## 5. 已移除的内容（可回滚范围）

| 类别 | 内容 |
|---|---|
| 第一版 ±90° 限位实验 | `results/torque_gap/`（29 次运行）、`figures/torque_gap/`（89 张）、`docs/torque_gap/experiment_report.md` |
| 无限行程 20 臂 | `results/torque_gap_unlimited/` 中除 `C1_B0_P0` 外的 19 次；`figures/torque_gap_unlimited/` 中除 `C1_control_overview` 外的 109 张；`docs/torque_gap/unlimited_gimbal_experiment.md` |
| 早期 CMG/MPC 遗留项目 | `results/{cmg_ab,cmg_maneuver,c1_cmg_diagnostics,c1_cmg_reproduction,legacy_rw}`、`figures/cmg_ab/`、`experiments/`、`archive/`、`legacy/`、`tools/`、`configs/{cmg_ab,cmg_maneuver,c1_cmg_reproduction*}`、20 个旧脚本、9 个未使用模块（`mpc3` `qp` `cmg_control` `cmg_hier` `cmg_priority` `scenarios` `sim` `arms` `metrics`）、`tests/test_cmg_ab.py` |
| 过时报告 | `docs/` 根目录 18 份阶段性/对比报告 |
| S08 Python/TCP Simulink 移植链（原生迁移完成后清理） | `simulink/README.md`、`s08_startup.m`、`open_case.m`、`run_case.m`、`check_plant_step.m`、`probe2/3.m`、`simulink/blocks/`、`simulink/py/{ctrl_server,run_closedloop,make_report,export_for_simulink,export_visible_model,sensitivity_check,check_controller_replay}.py`、`simulink/mexopts/`、`docs/torque_gap/simulink_port_spec.md`、`docs/任务说明_S08退化分析与Simulink移植.md` |
| 一次性开发残留（原生迁移完成后清理） | 仓库根 `tmp_*.py`、`*.slxc`、`simulink/restart_license_check.m`、`simulink/hpipm/build_log.txt`、`simulink/results/{license_recheck,hpipm_interface}/`、`simulink/results/native_closedloop/development_failed/` |

删除项均在 git 历史中：精简前的最后一个完整提交是 `7ee44e1`；原生迁移的清理点见其后的
`Fix P0 CMG gimbal drive wiring` 与 `Prune legacy S08 port and development leftovers` 两个提交。

## 6. 说明

- `.pydeps/`（230 MB）是**必需依赖**：它装着本地解包的 `casadi 3.8.2` 与 IPOPT 插件，
  因为本机不允许写 site-packages。验证：不带 `PYTHONPATH=.pydeps` 时
  `import casadi` 直接 `ModuleNotFoundError`；带上则一切正常。**不要删**。
- `.piptmp/`（含 57.3 MB 的 `casadi.whl`）、`tmp/`（早期调试脚本与提交信息草稿）以及
  4 个 `pip-*` 临时目录都是历史残留，**已在精简中删除**；它们本就不参与版本控制
  （见 `.gitignore` 中的 `.pydeps/`、`.piptmp/`、`tmp/`、`pip-*/` 条目）。
- 部分早期报告在删除前引用了彼此（例如 `C1_control_overview_analysis.md` 曾引用
  `unlimited_gimbal_experiment.md` §5.2/§5.4）。这些引用已改为指向现存文档，
  相关结论（角动量储备 vs 瞬时力矩包络）现在见 `prediction_mechanism_experiment.md` §5、§9。

## 7. 截至本轮新增的内容（无 CMG 对照 + 求解速度）

| 内容 | 位置 |
|---|---|
| 无 CMG（A）/ 舵面+CMG（B）多场景对照实验报告 | `no_cmg_vs_cmg_scenario_experiment.md` |
| 该实验的代码审查与惯量审计 | `no_cmg_vs_cmg_code_audit.md` |
| MPC 求解速度优化（Tier 1/2/3 全报告） | `mpc_solver_speedup.md` |
| 配置（由脚本生成的 A/B） | `configs/torque_gap_no_cmg_comparison/` |
| 数据（当前唯一数据集：11 组 A/B，RTI + 信赖域） | `results/torque_gap_no_cmg_comparison_rti2/` |
| 图（逐组对照总图 + 跨场景汇总） | `figures/torque_gap_no_cmg_comparison/{P0_rti2,P1_rti2}/`、`cross_*_rti2_*` |

新增模块与脚本：

```
nearspace/torque_gap_rti.py     HPIPM 实时迭代（RTI / GN-SQP）后端，与 OCP 同接口
nearspace/hpipm_bindings.py     Windows 下加载 D:\Tools\hpipm 的 DLL 垫片
scripts/build_no_cmg_comparison.py   生成 A/B 配置（--check 检测漂移）
scripts/audit_no_cmg_configs.py      A/B 逐键差异审计
scripts/audit_no_cmg_references.py   参考姿态核验与出图
scripts/run_no_cmg_matrix.py         运行矩阵（--solver-profile / --backend / --results-root）
scripts/summarize_no_cmg_comparison.py  summary.csv / summary.md
scripts/plot_no_cmg_comparison.py     逐场景 A/B 对照总图（9 联：姿态°/误差°/误差分量°/力矩分解/能力包络/CMG 力矩/框架角°/角动量/求解耗时） 与跨场景图
scripts/report_no_cmg_rti2.py        由 summary.csv 直接生成实验报告（11 组）
tests/test_torque_gap_no_cmg.py       无 CMG 分支的 15 项测试
tests/test_torque_gap_solver_policy.py 采纳策略 / 热启动 / quat_norm 测试
tests/test_torque_gap_rti.py          HPIPM RTI 的 8 项测试
```

关键结论（详见 `mpc_solver_speedup.md`）：B 组单步求解从 baseline 的 781 ms（P95 2973 ms）
降到 Tier-1 的 252 ms（P95 648 ms），再到 HPIPM RTI 的 79 ms；**给 RTI 加上
`|δγ| ≤ 0.5 rad` / `|δω| ≤ 1 rad/s` 硬信赖域后**，108 ms 均值 / 139 ms P95 /
**173 ms 最坏**（每一步都在 0.2 s 内），而跟踪（8 场景均值 0.740°）与 IPOPT（0.733°）
只差 0.9%——实时性与精度同时成立。
