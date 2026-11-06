function paper_report_pid(f,out)
cal=fullfile(out,'pid_calibration','pid_tuned.json');if isfile(cal),pid=jsondecode(fileread(cal));fprintf(f,'## PID 与卸载验证\n\n反馈为 tau=Kp*eq+Ki*I-Kd*(omega-omega_ref)，仅用当前参考；两种 PID 共用增益。Kp=[%g,%g,%g]，Ki=[%g,%g,%g]，Kd=[%g,%g,%g]。独立校准候选见 pid_calibration/tuning.json。H_on=%g、H_off=%g N m s，k_H=%g，qbar_min=%g Pa。\n\n',pid.kp,pid.ki,pid.kd,pid.hon,pid.hoff,pid.kh,pid.qmin);end
fprintf(f,'```mermaid\nflowchart LR\nR[当前参考] --> P[四元数PID]\nP --> A[当前力矩分配]\nA --> F[F20舵面反馈]\nA --> C[CMG阻尼最小二乘与限幅]\nF --> S[原生舵机与气动]\nC --> D[原生CMG驱动]\nS --> B[六自由度机体]\nD --> B\nB --> P\nH[真实框架角与动量] --> U[阈值卸载和可行比例]\nU --> A\n```\n\n卸载补偿按真实可交付 CMG 增量计算：tau_a_unload=A*(nu-nu_base)，tau_cmg_unload=-tau_a_unload；无舵面权限或动压不足时不卸载。框架速度与相邻命令加速度均受原始物理参数限制。\n\n');
file=fullfile(out,'unload_acceptance','acceptance_v2.json');if isfile(file),a=jsondecode(fileread(file));fprintf(f,'独立卸载物理验收 passed=%d；原始数据与开关/补偿日志见 unload_acceptance/。不代表每个 E2 任务都触发卸载；触发次数见 extra_metrics.json。\n\n',a.passed);end
end



