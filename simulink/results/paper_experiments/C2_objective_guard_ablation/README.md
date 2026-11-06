# C2目标函数拒步机制消融

仅新增一次60s联合MPC Simulink仿真；原始基线来自paper_final/V2_C2_3。本文批独立保存，不替换论文正式数据。

- 中文结论与限制：reports/实验报告.md
- 新旧逐事件对比：tables/event_comparison.csv；全程：tables/global_metrics.csv
- 优化器：tables/optimizer_diagnostics.csv、optimizer_steps.csv、optimizer_12_24_summary.csv
- 真实物理数据：tables/guard_on_physical_16_28_native.csv、guard_off_physical_16_28_native.csv
- 约束核验：tables/physical_constraints.csv、physical_verification.csv
- 三张图：figures/Fig1_C2_three_maneuvers、Fig2_C2_second_maneuver_detail、Fig3_C2_SQP_diagnostics（PDF及PNG）
- 新原始输出：raw/sim_output.mat；完整逐周期诊断：raw/*_authority.mat；分析中间完整数据：raw/analysis_native.mat
- 配置与模型：config/、models/、implementation/；原始/内嵌bank均保留
- SHA256：manifest.csv；验收：checks/acceptance.json

大型MAT和SLX保留本地，Git保存源码、配置、SHA、报告、CSV及图表。基线原始数据直接保留在paper_final，不重复仿真。本次代码默认enforce_objective_decrease=true，仅实验模型设置false。

从仓库根目录只复算、绘图（不会调用sim）：

```powershell
$env:MATLAB_HOME='D:/Program Files/MATLAB/R2024a'
& .\simulink\mlab.cmd "addpath('simulink'); init_paths; addpath('simulink/paper/c2_objective_guard'); analyze_c2_guard_ablation;"
python simulink/paper/c2_objective_guard/render_c2_guard.py
python simulink/paper/c2_objective_guard/report_c2_guard.py
```

run_c2_guard_ablation是本次单次仿真入口，simulation_invocation.json存在时会拒绝重跑。默认开关函数级回归位于checks/default_guard_regression.json；保持关闭拒步后的原stagnation退出，不强制每周期满4轮。图形显示按原始20ms分箱端点及各列极值选择真实点，不平滑；所有指标及物理约束用完整1ms输出计算。
