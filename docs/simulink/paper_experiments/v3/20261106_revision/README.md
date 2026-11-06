# 2026-11-06 实验三/四增量修订

执行规格见本目录两份原始方案。新批次：`simulink/results/paper_experiments/20261106_e3_e4_revision/`。

- E3：5档预测时域×M/Z共10例，旧V3_S/V3_L/V3_Z只读复用，新增7例。只改变预测网格与是否存在未来机动，不调整控制器或物理模型。
- E4：固定1°峰值×9频率×3方法，直接读取旧27例，新增0次仿真。
- 旧批次`20261105_redesign_v1`保持只读，不调用旧整批报告/验收入口。

从仓库根目录在PowerShell运行（支持已完成同配置案例断点续跑）：

```powershell
$env:MATLAB_HOME='D:\Program Files\MATLAB\R2024a'
& .\simulink\mlab.cmd "addpath('simulink/paper/v3'); run_e3_horizon_pairs"
```

只重生成新批次报告，不运行仿真：

```matlab
addpath('simulink'); init_paths;
addpath('simulink/paper','simulink/paper/v3');
old=fullfile(repo_root,'simulink/results/paper_experiments/20261105_redesign_v1');
out=fullfile(repo_root,'simulink/results/paper_experiments/20261106_e3_e4_revision');
report_e4_1deg(old,out);
report_e3_horizon_pairs(old,out);
```

新批次入口：`reports/E3_revised.md`、`reports/E4_1deg.md`、各自acceptance JSON与`tables`摘要表。完整原始物理数据为`E3/cases/<新案例ID>/sim_output.mat`，保留1ms输出；同目录保留参考bank、控制器诊断MAT和CSV。旧三例及E4的原始数据仍在旧批次。

Git只归档代码、配置、摘要表、图及验收证据。新MAT、缓存和大体积逐时刻CSV保留本地并按本批次.gitignore排除，复现时重新生成。新增case模型位于`simulink/cases/paper_v3/`。

只读保护核验：`python scripts/audit_e3_e4_revision.py verify`。其快照仅覆盖本任务复用30例、旧报告/表/配置及相关正式物理和控制器输入，不做全仓预检。

## 本次执行结果

新增7例均完成60s物理仿真，无控制器回退。五组各含60001个1ms样本；独立Python复算五组迁移判据，与MATLAB一致；预览前M−Z构型、框架速度、角动量和实际CMG力矩差均为零。存盘控制器模板除`dts`外完全一致，1400份相关只读文件SHA-256核验通过。

| H(s) | 35–40s姿态RMSE(°) | 相对迁移提前量(s) |
|---:|---:|---:|
|5|2.517184|3.357|
|8|2.255163|6.678|
|12|1.993398|10.603|
|16|1.760107|14.560|
|23.2|1.317855|21.510|

E4三种方法辅助通过数保持1/9、4/9、8/9；三张主图分别为增益、相位和RMSE，所有27个频点保留。不能解释为严格LTI带宽。

完整性核验：`python scripts/verify_e3_e4_revision.py`。验收证据位于新批次`reports/independent_revision_verification.json`、`E3_controller_audit.json`、`read_only_verification.json`。

计算耗时包含真实离线开销，旧复用案例和新案例的执行负载可能不同，不据此宣称计算开销随H严格单调或具有硬实时保证。初次整合中报告脚本的括号语法错误已修复，后续仅重新生成报告，未增加任何额外仿真。
