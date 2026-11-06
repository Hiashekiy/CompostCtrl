# 实验二：C2只读诊断与动态权限卸载修订

本目录保留用户实施方案。新增实现全部位于`simulink/paper/v3/e2_revision/`；新批次为`simulink/results/paper_experiments/20261106_e2_unload_revision/`。原338例和实验一/三/四保持冻结。

## 已执行任务

- C2：读取旧`E2/cases/V2_C2_3/sim_output.mat`的60001个1ms样本；新增仿真0次。7段统计、14项执行器角度/速率/加速度限幅比例、三次机动边界及真实MPC诊断均单独导出。前两次机动后的统计窗口只有8s，第三次为10s，没有改历史口径。
- 原生轨迹可行性：A=2–12s，B=12–35s，C=35–40s（+5°、5s过渡），完整后观测40–50s，实际运行至60s。只做窗口标记，执行器没有时间驱动的权限修改。
- 非正式预检：M1/M2各16s，2次；物理、保持误差、预测和真实条件端点补偿检查通过。旧通用指标将其标为TRACKING_FAIL，因为16s尚未覆盖35s后的机动，不能当成正式任务失败/通过结果。
- 正式补测：M1/M2×PID-NoUnload/PID-DynamicUnload/Joint-MPC，6次全新独立60s六自由度仿真。初态、参考、扰动和约束配对一致。两PID仅卸载开关不同；冻结PID参数和基础分配不变。

|构型|无卸载机动RMSE(°)|动态卸载机动RMSE(°)|联合MPC机动RMSE(°)|卸载前置H模相对下降(Nms)|
|---|---:|---:|---:|---:|
|M1|2.288939|2.194236|0.500145|8.917572|
|M2|2.030638|2.515378|0.516915|7.212534|

四个PID案例均未通过原稳定判据，联合MPC两例通过。M1略有改善，M2恶化；H模下降不能替代后续性能验证。

## 数据与报告入口

- `reports/E2_修订综合结论.md`：主要结论、六例性能及限制。
- `reports/C2_第二次退化诊断.md`：12–24s优化全步连续拒绝的证据链及未决因果问题。
- `reports/M1M2_场景可行性.md`：真实飞行条件和A/B/C依据。
- `tables/M1M2_6cases.csv`、`M1M2_stage_metrics.csv`、`M1M2_paired_results.csv`：汇总和分阶段统计。
- `E2/cases/E2R_M1_1`等六目录的`sim_output.mat`：新正式原始物理输出。
- `E2_PRECHECK/cases/E2R_PC_M1_DU`及M2目录：两次非正式预检。
- 每例`source_manifest.json/build_config.json/provenance.json`记录真实执行代码、模型、bank与原始输出哈希；`dynamic_unload_ticks.csv`记录每周期请求、选用程度、补偿、余量和暂停原因。

完整原始MAT和1ms诊断CSV保留本地，Git仅提交代码、配置、摘要/控制周期证据、图和报告；本批次`.gitignore`已排除MAT、缓存、大体积物理CSV。新case模型可按脚本重建，位于`simulink/cases/paper_v3/`，没有修改正式SLX。

## 复现命令

从仓库根目录运行，MATLAB需有原生模型、HPIPM及旧批次原始数据：

```powershell
$env:MATLAB_HOME='D:\Program Files\MATLAB\R2024a'
& .\simulink\mlab.cmd "addpath('simulink/paper/v3/e2_revision'); inspect_e2_revision"
python simulink/paper/v3/e2_revision/write_c2_diagnosis.py
& .\simulink\mlab.cmd "addpath('simulink/paper/v3/e2_revision'); precheck_e2_revision"
& .\simulink\mlab.cmd "addpath('simulink/paper/v3/e2_revision'); run_e2_revision"
& .\simulink\mlab.cmd "addpath('simulink/paper/v3/e2_revision'); validate_e2_revision"
python simulink/paper/v3/e2_revision/write_c2_diagnosis.py
python simulink/paper/v3/e2_revision/verify_e2_revision.py
python simulink/paper/v3/e2_revision/audit_e2_revision.py verify
```

同配置已完成仿真不会重复执行；配置或执行源码变化会拒绝覆盖，须用明确的新case ID。`run_e2_revision`在正式仿真前强制读取场景和预检门控，失败即停止。独立核验与报告重生不会增加仿真。

## 实现与解释限制

实验专用副本取消2Nm固定cap及0.2Nm额外slew；沿H下降方向用物理速率/加速度窗口确定最大候选，再检验三轴舵面可分配性、20ms F20反馈/舵机及CMG真实驱动递推、原0.25Nm补偿容差和相对基础动作的严格H下降。实际最大选用额外力矩约0.377/0.445Nm，补偿/执行器约束仍起主要限幅作用，并未因删除固定cap而宣称大幅卸载。

“真实额外力矩”采用同一时刻物理状态下的无卸载条件端点作参照，并明确标注其反事实范围；不能用它替代六次真实飞行的后续误差比较。方向支撑只代表单步投影上界。C2的参考角加速度力矩代理不是MPC的真实请求，不用于唯一归因。

验收：1200个20ms CMG端点与真实输出一致（速率误差约1e-16rad/s）；正式案例采用的353个卸载候选通过H下降独立复核；六例物理/采样与配对协议通过；原338例、E3/E4修订和相关冻结文件5891项SHA核验无变化。执行/数据验收通过与PID跟踪失败分别记录。
