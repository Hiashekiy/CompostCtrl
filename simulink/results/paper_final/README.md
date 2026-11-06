# 论文专用实验数据集

今后论文写作只从本目录取数据。197个正式案例，E1=144、E2=16、E3=10、E4=27。原始MAT、参考bank与模型为真实文件复制，不是仅有旧路径的占位引用。

入口：[总览](reports/paper_experiment_overview.md)、[197例清单](manifest.csv)、[图表目录](reports/figure_registry.csv)、[逐文件SHA256](reports/file_sha256.csv)、[原始复算](reports/reproduction_checks.csv)。每组有summary.md，reports下亦有四份总结。

scenarios/<id>/保存未改动的原始case_config、metrics、build_config、provenance、reference_bank.mat及model.slx；raw/<id>/保存sim_output.mat和原控制/执行器诊断MAT。manifest记录本目录相对路径、Git来源、完成/有效域/原稳定判据。旧表/JSON中原有绝对路径为不改写的历史证据；论文使用manifest和reports/path_mapping.csv解析本地副本，不需要搜索历史目录。

reports/reproducibility/code保存相关原生模型、控制/分析源码与物理/预测数据依赖快照。源案例模型的旧绝对初始化路径作为历史证据保留，未为重仿修改模型；本次验证的是原始输出到指标/图表的后处理复现，未宣称跨机器无需配置即可重新仿真。MATLAB/Simulink版本和核验结果见reproduction_receipt.json。

Git字段区分：repository_snapshot_git/source_git_commit是本次归档读取仓库的快照；run_baseline_git是原始provenance记录的运行前基线，不能据此声称当时未提交的新代码已经存在于该提交。formal_selection_git与original_result_archive_git分别记录正式筛选与原始结果归档提交，实际运行版本由逐案例模型/bank/源码SHA和reports/source_evidence内原始来源清单约束。

## 后处理复现（零新增仿真）

在仓库根目录运行：

```powershell
$env:MATLAB_HOME='D:\Program Files\MATLAB\R2024a'
& .\simulink\mlab.cmd "addpath('simulink/results/paper_final/reports/reproducibility'); reproduce_paper_final('D:/项目/飞轮/simulink/results/paper_final'); export_e2_gimbal_pairs('D:/项目/飞轮/simulink/results/paper_final');"
python simulink/results/paper_final/reports/reproducibility/render_paper_final.py
python simulink/results/paper_final/reports/reproducibility/finalize_paper_final.py verify
```

归档内也保存组织/绘图脚本；Python绘图和哈希核验脚本可直接从本目录的reports/reproducibility运行，无需调用历史实验目录。绘图仅使用MATLAB从原始输出导出的真实点，20ms显示分箱保留各列极值，不做平滑；所有指标在完整1ms原始输出上复算。Python不模拟物理系统。

## 数据保留与清理

原始MAT、模型/数据依赖留在本地，Git仅归档清单、配置、源码、摘要与图表，避免上传约7GB原始实验。不要执行全仓git add。旧MAT与原始保护基线未删除。

清理仅针对cleanup_manifest.csv列出的已完整备份缓存及未跟踪重复启动日志。备份位于../paper_archive/20261106_paper_final_cleanup.zip；删除逐文件验证路径和SHA，不递归清空结果目录。具体删除状态及前后统计见reports/deleted_files.csv、cleanup_statistics.json和final_verification.json。
