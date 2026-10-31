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

删除项均在 git 历史中：精简前的最后一个完整提交是 `7ee44e1`。

## 6. 说明

- `.pydeps/`（230 MB）是**必需依赖**：它装着本地解包的 `casadi 3.8.2` 与 IPOPT 插件，
  因为本机不允许写 site-packages。验证：不带 `PYTHONPATH=.pydeps` 时
  `import casadi` 直接 `ModuleNotFoundError`；带上则一切正常。**不要删**。
- `.piptmp/`（原 57.3 MB 的 `casadi.whl` 与 `x.whl` **已删除**）与 `tmp/`（7 个早期调试脚本 /
  提交信息草稿，约 84 KB）都是历史残留，**不参与版本控制**（见 `.gitignore`）。
  它们剩下的空目录与文件因 NTFS 权限（`tmp/` 下连新建文件都被拒）无法在本会话删除；
  如需彻底清除，用**管理员权限**的资源管理器或命令提示符删除即可：
  `rmdir /s /q "D:\项目\飞轮\.piptmp" "D:\项目\飞轮\tmp"`。
- 部分早期报告在删除前引用了彼此（例如 `C1_control_overview_analysis.md` 曾引用
  `unlimited_gimbal_experiment.md` §5.2/§5.4）。这些引用已改为指向现存文档，
  相关结论（角动量储备 vs 瞬时力矩包络）现在见 `prediction_mechanism_experiment.md` §5、§9。
