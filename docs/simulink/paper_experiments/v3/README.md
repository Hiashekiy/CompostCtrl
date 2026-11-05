# 四组论文实验重新实施（v3）

本目录保存用户四份方案的原文副本、统一协调记录和新实现入口。实际数据和报告只写入独立批次 `simulink/results/paper_experiments/20261105_redesign_v1/`。批次名称使用开始运行日期，不因跨零点而改名。旧批次和原始方案保持不动。

结果入口：[综合报告](../../../../simulink/results/paper_experiments/20261105_redesign_v1/reports/综合报告.md)、[执行与数据验收](../../../../simulink/results/paper_experiments/20261105_redesign_v1/reports/验收状态.md)。运行尚未完成时，以报告中的实际数量和验收标志为准。

## 入口

在仓库根目录用 PowerShell 执行：

```powershell
& .\simulink\paper\v3\run.ps1 -Mode all -Stamp 20261105_redesign_v1
```

MATLAB 入口为 `paper_v3(mode,stamp)`，支持 `preflight`、`unit`、`smoke`、`tune`、`E1`、`E2`、`E3`、`E4`、`all`、`analyze`、`report`。恢复现有批次时读取保存的实验矩阵并核对配置哈希；不覆盖已经有有效原始输出且配置一致的单例。更改实验参数或控制器时应使用新批次。

完整独立验收在 MATLAB 中调用 `v3_acceptance(out)`，其中 `out` 为批次绝对路径。该入口重新读取原始输出，核对采样周期、实际执行器约束、参考一致性、文件哈希，再生成表格、图和报告。不能只看矩阵数量宣称所有控制任务成功；以 `reports/acceptance.json` 和各案例真实状态为准。

## 文件布局

|用途|路径|
|---|---|
|MATLAB 入口与辅助函数|`simulink/paper/v3/`|
|生成的独立 case 模型|`simulink/cases/paper_v3/`|
|环境、矩阵、控制器冻结、运行日志|批次 `config/` 与 `calibration/`|
|原始输出、实际输入、状态、指标|批次 `<group>/cases/<id>/`|
|汇总小表与独立重算|批次 `tables/`|
|论文图 PNG（300 dpi）与矢量 PDF|批次 `figures/`|
|分组分析、综合报告、验收证据|批次 `reports/`|

原始 `sim_output.mat` 保留完整物理日志。为减少重复磁盘占用，派生 `timeseries.csv`、`domain_diagnostics.csv` 约每20ms导出，额外保留每列极值和末点；指标始终在完整原始采样上计算。`csv_export.json` 标明这一导出口径。`actuator_timeseries_20ms.csv` 是实际执行器读数及明确标为支撑上界的几何量，不能将局部方向投影解释成其他轴力矩为零时的权限。

标定、结构预检、卸载验证、E4预检单独归档，不计入正式249例分母。历史E3时域扫描仅单独标注为历史，不计为本次运行。

## 并行与溯源

冻结PID后，E1与E4没有数据依赖，允许分MATLAB进程并行；E4按幅值分区的辅助入口为 `run_partition.ps1`。每个进程使用独立缓存，单例采用原子文件锁。实际计算耗时受同时运行任务影响，不作为独占硬件实时性能保证。批次 `config/parallel_execution.json` 记录并行范围。

本次对原生产生成器仅增加可选 case/result 目录参数，并在指定自定义输出目录时跳过重复模型缩略图与默认共享 bank 备份；原12参数调用行为保留。控制律和物理核心没有借此替换。新PID以单独S-function使用原端口，增益冻结后E2/E4共用，E4关闭卸载。

原始基线Git提交、实际模型和输入bank哈希在配置中；PID实现哈希随构建保存，后验分析代码单独留档。代码提交不包含每例大型MAT、模型缓存或完整轨迹CSV；本地数据仍完整保留。

从Git克隆只会取得代码和汇总证据，不等于取得338份原始输出。要重新实验请使用新的 `-Stamp`，并准备本机原生模型、HPIPM依赖及所需原始资产；不要把缺少原始MAT的旧批次目录当作新实验批次。冻结PID源文件和结果JSON保留原始字节，避免Git换行转换破坏已记录的哈希。
