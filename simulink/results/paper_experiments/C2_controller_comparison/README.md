# C2联合MPC与PID对比图

图中仅比较联合MPC、PID–CMG及俯仰参考，不再显示拒步开关对比。上图为实际俯仰，下图为四元数姿态角误差；灰色区间标记三次机动。

MPC读取C2_objective_guard_ablation中已完成的当前MPC原始输出；PID读取paper_final的V2_C2_1冻结PID无卸载原始输出。未新增仿真。两个原始bank参考一致，完整1ms重算RMSE与已保存指标差小于1e-8。绘图选择真实点，不平滑。

文件：figures/C2_MPC_vs_PID.png、figures/C2_MPC_vs_PID.pdf。原始数据路径与SHA256见reports/provenance.json，指标见tables/。

建议图注：C2连续俯仰机动中联合MPC与PID–CMG的跟踪响应及姿态误差对比（角度单位：°）。
