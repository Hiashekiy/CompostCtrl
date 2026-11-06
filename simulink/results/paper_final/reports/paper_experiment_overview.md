# 论文实验总览

唯一论文数据入口为paper_final。197例：E1=144，E2=16，E3=10，E4=27；全部包含实际复制的原始sim_output.mat、bank、配置、案例模型和SHA256。没有新增Simulink仿真。

建议正文顺序：E1先回答机动能力范围，E2比较传统控制并保留连续任务/卸载负面结果，E3展示预测时域与配对迁移的观察关系，E4用固定1°的三项周期跟踪指标收尾。不得将更早迁移写成已证明唯一原因，也不得将周期比较写成严格LTI带宽。

|章节|正文主表|正文主图|补充材料|
|---|---|---|---|
|E1|success_rates、E1_all、E1_min_duration|可行矩阵、最短时间、仅联合成功代表|两者成功、高迎角范围代表|
|E2|single_maneuver_metrics、events_from_raw、M1M2_paired_results|单次、C1/C2、卸载后机动|动压权限、构型、补偿、C2求解日志|
|E3|E3_10cases、E3_5pairs、pairs_from_raw|时域性能、五组相对迁移|局部正负方向支撑与计算限制|
|E4|E4_1deg_27cases|增益、相位、RMSE三图|状态、环境、耗时与饱和列|

读者应同时看到：C2第二次退化及原因未决、C1后观察窗口不足、M2卸载恶化、E1没有正式域中断点。详见四份分组总结、manifest.csv、figure_registry.csv和reproduction_checks.csv。

本资料属于实验事实底稿，不是已经由人类作者确认的论文正文。后续作者需核对论述与原始证据；未引入统计显著性、虚构重复样本或未测因果效应。
