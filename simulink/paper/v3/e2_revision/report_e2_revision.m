function report_e2_revision(old,out)
cases={};stages={};data=cell(2,3);logs=cell(2,2);sources={};protocol={};
for task=1:2,for method=1:3
 id=sprintf('E2R_M%d_%d',task,method);folder=fullfile(out,'E2/cases',id);d=load(fullfile(folder,'native_diagnostic.mat'),'D');D=d.D;data{task,method}=D;m=jsondecode(fileread(fullfile(folder,'metrics.json')));s=jsondecode(fileread(fullfile(folder,'case_config.json')));bc=jsondecode(fileread(fullfile(folder,'build_config.json')));b=load(fullfile(folder,'reference_bank.mat'));
 [~,k]=min(abs(D.time_s-34.8));m.pre_Hy_Nms=D.Hy(k);m.pre_Hnorm_Nms=D.Hnorm(k);m.pre_sigma_min=D.sigma_min(k);m.pre_positive_support_Nm=D.pitch_positive_support_Nm(k);m.pre_negative_support_Nm=D.pitch_negative_support_Nm(k);m.source_path=folder;m.task=task;m.variant=method;
 m.A_H_delta_Nms=sample(D,'Hnorm',12)-sample(D,'Hnorm',2);m.B_H_delta_Nms=sample(D,'Hnorm',35)-sample(D,'Hnorm',12);
 m.dynamic_active_ticks=0;m.dynamic_max_selected_extra_Nm=NaN;m.dynamic_actual_compensation_max_Nm=NaN;m.first_low_qbar_pause_s=NaN;m.dynamic_prediction_descent_pass=true;
 if method<=2
  [L,u]=e2r_unload_log(folder);logs{task,method}=L;m.dynamic_active_ticks=u.active_ticks;m.dynamic_max_selected_extra_Nm=u.max_selected_extra_Nm;v=u.actual_compensation_max_Nm;if ~isempty(v),m.dynamic_actual_compensation_max_Nm=v;end;m.dynamic_prediction_descent_pass=u.strict_predicted_momentum_descent;
  stop=find(strcmp(L.reason,'low_qbar'),1);if ~isempty(stop),m.first_low_qbar_pause_s=L.time_s(stop);end
 end
 d=load(fullfile(folder,'sim_output.mat'),'r');r=d.r;p=vehicle_params;
 m.physical_bounds_pass=~any(abs(D{:,{'nu1','nu2','nu3','nu4'}})>p.cmg.gdot_max+1e-6,'all') && ~any(abs(D{:,{'acc1','acc2','acc3','acc4'}})>p.cmg.gddot_max+1e-6,'all') && ~any(abs(D{:,{'delta1','delta2','delta3'}})>p.vehicle.delta_max(:)'+1e-6,'all') && ~any(abs(D{:,{'delta_dot1','delta_dot2','delta_dot3'}})>p.vehicle.delta_dot_max(:)'+1e-6,'all');
 m.native_period_pass=v3_period_matches(D.time_s,.001);f=dir(fullfile(folder,'*_torque_feedback.mat'));inner=load(fullfile(folder,f(1).name));m.inner_period_pass=v3_period_matches(inner.diagnostics.time,.02);m.inner_fallbacks=sum(inner.diagnostics.fallback);
 solver=[];if method==3,f=dir(fullfile(folder,'*_authority.mat'));solver=load(fullfile(folder,f(1).name));m.upper_period_pass=v3_period_matches(solver.control_times,.2);m.max_sqp_attempted=max(cellfun(@(z)z.sqp_attempted,solver.diagnostics));assert(m.max_sqp_attempted<=4);else,h=load(fullfile(folder,'pid_diagnostics.mat'));m.upper_period_pass=v3_period_matches(h.times,.2);m.max_sqp_attempted=NaN;end
 cases{end+1}=m;sources{end+1}=struct('id',id,'source_path',folder,'raw_sha256',paper_hash(fullfile(folder,'sim_output.mat')),'model_sha256',bc.model_sha256,'bank_sha256',bc.bank_sha256,'config_sha256',paper_hash(fullfile(folder,'case_config.json')),'status',m.status,'actual_end_s',m.actual_end_s);
 conditions=struct('id',id,'bank_x0',b.x0,'time',b.time,'q_ref',b.q_ref,'omega_ref',b.omega_ref,'tau_d',b.tau_d,'physical_flight0',squeeze(r.flight_servo_measured.Data(1,1:6)),'pid_config',s.pid,'method',s.method,'dts',s.dts);protocol{task,method}=conditions;
 win=[2 12;12 35;35 40;40 50];names={'A','B','C_maneuver','C_post10'};
 for j=1:4
  mask=D.time_s>=win(j,1)&D.time_s<win(j,2);if j==4,mask=D.time_s>40 & D.time_s<=50;end;Q=D(mask,:);
  v=struct('id',id,'task',task,'variant',method,'stage',names{j},'start_s',win(j,1),'end_s',win(j,2),'samples',height(Q),'attitude_rmse_deg',sqrt(mean(Q.attitude_error_deg.^2)),'attitude_peak_deg',max(Q.attitude_error_deg),'pitch_rmse_deg',sqrt(mean(Q.pitch_error_deg.^2)),'qbar_min_Pa',min(Q.qbar_Pa),'qbar_max_Pa',max(Q.qbar_Pa),'height_min_m',min(Q.height_m),'height_max_m',max(Q.height_m),'speed_min_mps',min(Q.speed_mps),'speed_max_mps',max(Q.speed_mps),'alpha_min_deg',min(Q.alpha_deg),'alpha_max_deg',max(Q.alpha_deg),'H_start_Nms',Q.Hnorm(1),'H_end_Nms',Q.Hnorm(end),'Hy_start_Nms',Q.Hy(1),'Hy_end_Nms',Q.Hy(end),'sigma_min_low',min(Q.sigma_min),'cmg_rate_sat_pct',100*mean(Q.cmg_rate_sat),'cmg_accel_sat_pct',100*mean(Q.cmg_accel_sat),'surface_angle_sat_pct',100*mean(Q.surface_angle_sat),'surface_rate_sat_pct',100*mean(Q.surface_rate_sat),'actual_aero_pitch_rms_Nm',sqrt(mean(Q.aero_y.^2)),'actual_cmg_pitch_rms_Nm',sqrt(mean(Q.cmg_y.^2)),'unload_active_ticks',0,'actual_compensation_rms_Nm',NaN,'actual_compensation_max_Nm',NaN,'fallback_count',0);
  if method<=2,L=logs{task,method};lm=L.time_s>=win(j,1)&L.time_s<win(j,2);active=lm & L.eta>0;v.unload_active_ticks=sum(active);errs=L.actual_compensation_error_Nm(active);errs=errs(isfinite(errs));if ~isempty(errs),v.actual_compensation_rms_Nm=sqrt(mean(errs.^2));v.actual_compensation_max_Nm=max(errs);end;else,sm=solver.control_times>=win(j,1)&solver.control_times<win(j,2);v.fallback_count=sum(cellfun(@(z)z.fallback,solver.diagnostics(sm)));end
  stages{end+1}=v;
 end
end;end
T=struct2table([cases{:}]);P=struct2table([stages{:}]);writetable(T,fullfile(out,'tables/M1M2_6cases.csv'));writetable(P,fullfile(out,'tables/M1M2_stage_metrics.csv'));
proof={};pairs={};
for task=1:2
 base=protocol{task,1};for method=2:3,Q=protocol{task,method};for field={'bank_x0','time','q_ref','omega_ref','tau_d','physical_flight0','dts'},key=field{1};assert(isequaln(base.(key),Q.(key)),'e2r:pair','Paired physical protocol differs: %s',key);end;end
 a=rmfield(base.pid_config,'unload');b=rmfield(protocol{task,2}.pid_config,'unload');assert(isequaln(a,b));assert(~base.pid_config.unload && protocol{task,2}.pid_config.unload);
 proof{end+1}=struct('task',task,'same_initial_state_reference_disturbance_and_grid',true,'PID_only_config_difference','unload flag','onset_s',35,'transition_s',5,'post_window_s',[40 50]);
 N=T(T.task==task&T.variant==1,:);U=T(T.task==task&T.variant==2,:);J=T(T.task==task&T.variant==3,:);
 z=struct('task',task,'PID_NoUnload_pre_H_Nms',N.pre_Hnorm_Nms,'PID_Dynamic_pre_H_Nms',U.pre_Hnorm_Nms,'MPC_pre_H_Nms',J.pre_Hnorm_Nms,'dynamic_H_reduction_vs_NoUnload_Nms',N.pre_Hnorm_Nms-U.pre_Hnorm_Nms,'NoUnload_maneuver_rmse_deg',N.maneuver_rmse_deg,'Dynamic_maneuver_rmse_deg',U.maneuver_rmse_deg,'MPC_maneuver_rmse_deg',J.maneuver_rmse_deg,'maneuver_rmse_improvement_deg',N.maneuver_rmse_deg-U.maneuver_rmse_deg,'NoUnload_post10_rmse_deg',N.post_rmse_deg,'Dynamic_post10_rmse_deg',U.post_rmse_deg,'MPC_post10_rmse_deg',J.post_rmse_deg,'post10_rmse_improvement_deg',N.post_rmse_deg-U.post_rmse_deg,'dynamic_active_ticks',U.dynamic_active_ticks,'dynamic_max_selected_extra_Nm',U.dynamic_max_selected_extra_Nm,'dynamic_actual_compensation_max_Nm',U.dynamic_actual_compensation_max_Nm,'all_three_complete',all(T.complete(T.task==task)));pairs{end+1}=z;
end
writetable(struct2table([pairs{:}]),fullfile(out,'tables/M1M2_paired_results.csv'));paper_json(fullfile(out,'reports/M1M2_protocol_audit.json'),struct('passed',true,'pairs',{proof}));
e2r_figures(data,logs,out);
physical=all(T.physical_bounds_pass & T.native_period_pass & T.inner_period_pass & T.upper_period_pass);cfg=jsondecode(fileread(fullfile(out,'config/frozen_pid.json')));
pred=all(T.dynamic_prediction_descent_pass);comp=all(T.dynamic_actual_compensation_max_Nm(T.variant==2)<=cfg.unload_compensation_tolerance+1e-6);
paper_json(fullfile(out,'reports/E2_acceptance.json'),struct('formal_expected',6,'formal_actual',height(T),'formal_complete',all(T.complete),'C2_new_simulations',0,'informal_prechecks',2,'formal_new_simulations',6,'physical_and_period_checks_passed',physical,'paired_protocol_passed',true,'strict_predicted_momentum_descent',pred,'actual_endpoint_compensation_within_tolerance',comp,'invalid_ids',{cellstr(string(T.id(ismember(string(T.status),["MODEL_DOMAIN_FAIL","CONTROLLER_FAIL","INCOMPLETE"]))))},'tracking_failed_ids',{cellstr(string(T.id(strcmp(T.status,'TRACKING_FAIL'))))},'sources',{sources},'performance_success_not_required_for_execution_acceptance',true));
fid=fopen(fullfile(out,'reports/E2_修订综合结论.md'),'w','n','UTF-8');cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'# 实验二修订综合结论\n\n本次完成C2原始MAT只读诊断（新增0次），M1/M2动态卸载非正式预检2次，正式六自由度仿真6次。实验一/三/四冻结。\n\n## C2证据链\n\n12–24s共60个QP求解成功但全步因objective_increase拒绝、接受步为0；第二次机动期间无执行器饱和且气动力矩跟随指令，因而优化停滞是直接可见证据。第三次前Hy已改变符号且机体有正俯仰角速度，优化又产生接受动作。不能归因为角动量电量耗尽，也不能唯一断言是主动构型恢复。详细证据、动力学项和未决原因见[C2诊断](C2_第二次退化诊断.md)。\n\n## 真实窗口与实现\n\n采用原生轨迹A=2–12s、B=12–35s、C=35–40s，机动后完整观测40–50s，运行至60s。窗口来自真实动压、飞行状态与耦合舵面补偿候选；控制执行不按秒数强制增减，不冻结物理状态。详见[场景可行性](M1M2_场景可行性.md)。\n\n冻结PID原文件未改。两PID使用同一实验副本，只差unload开关，kp/ki/kd和基础链式分配一致。取消2Nm固定卸载cap与0.2Nm额外slew，按物理框架速率/加速度可行射线和三轴舵面分配、真实20ms舵面/CMG驱动预测选最大可行程度；hon/hoff=80/60、残差门2Nm、qmin=1、补偿容差0.25Nm均保持。不存在用更大常数替代旧上限。MPC目标/权重、23.2s时域和最多4轮SQP不变。\n\n## 正式结果\n\n|构型|方法|状态|机动RMSE(°)|机动后10s RMSE(°)|34.8s H模(Nms)|34.8s正/负俯仰支撑(Nm)|\n|---|---|---|---:|---:|---:|---|\n');
for k=1:height(T),fprintf(fid,'|M%d|%s|%s|%.6f|%.6f|%.6f|%.4f / %.4f|\n',T.task(k),char(string(T.method(k))),char(string(T.status(k))),T.maneuver_rmse_deg(k),T.post_rmse_deg(k),T.pre_Hnorm_Nms(k),T.pre_positive_support_Nm(k),T.pre_negative_support_Nm(k));end
fprintf(fid,'\n动态卸载对比（正误差改善值表示误差降低）：\n\n');for k=1:2,z=pairs{k};fprintf(fid,'- M%d：机动前相对无卸载H模降低 %.6f Nms；机动RMSE改善 %.6f°，后10s RMSE改善 %.6f°；有效卸载%d步，最大选用额外力矩 %.6f Nm，实际20ms条件增量补偿最大误差 %.6f Nm。\n',k,z.dynamic_H_reduction_vs_NoUnload_Nms,z.maneuver_rmse_improvement_deg,z.post10_rmse_improvement_deg,z.dynamic_active_ticks,z.dynamic_max_selected_extra_Nm,z.dynamic_actual_compensation_max_Nm);end
fprintf(fid,'\n## 验收与解释限制\n\n六例完成=%d，物理/采样检查=%d，预测H相对同刻基础动作严格下降=%d，实际条件端点补偿在原容差内=%d。性能失败完整保留；执行/数据验收通过不等于姿态稳定判据通过。两次16s预检的旧分析状态保留TRACKING_FAIL（未观测35s之后的正式机动），其通过项仅为预先声明的保持/补偿/物理安全检查。\n\n实际附加力矩由真实20ms输出相对“同一时刻无卸载条件端点”计算，抵消同态自然气动力矩；它不是第二条反事实飞行轨迹。后续轨迹收益使用六次独立正式仿真的配对比较，不以指令理论相消代替物理补偿。\n\n删除固定cap并不保证大卸载量；实际幅值仍受20ms舵机/CMG动态、三轴耦合和原0.25Nm补偿容差约束。H模下降不自动意味着所有方向权限增加，方向支撑仍只是单步投影上界，不是纯俯仰三轴独立可达力矩。若后续误差没有改善或未过稳定判据，不宣称显著卸载收益或稳定性得到证明。\n\n',all(T.complete),physical,pred,comp);
for name={'M1M2_flight_authority_window','M1M2_unload_response','M1M2_following_maneuver'},fprintf(fid,'![%s](../figures/%s.png)\n\n',name{1},name{1});end
fprintf(fid,'[六例汇总](../tables/M1M2_6cases.csv)；[A/B/C阶段统计](../tables/M1M2_stage_metrics.csv)；[配对结果](../tables/M1M2_paired_results.csv)；[验收状态](E2_acceptance.json)。原始数据位于本批次E2/cases/E2R_M1_1等六个目录的sim_output.mat；两份预检位于E2_PRECHECK/cases。\n');
end
function value=sample(D,key,t)
[~,k]=min(abs(D.time_s-t));value=D.(key)(k);
end
function e2r_figures(data,logs,out)
f=figure('Visible','off');tiledlayout(5,2);colors=[0 .447 .741;.85 .325 .098;.494 .184 .556];
for task=1:2,for method=1:2
 D=data{task,method};label={'No unload','Dynamic unload'};col=colors(method,:);
 nexttile(task);hold on;plot(D.time_s,D.Hnorm,'Color',col,'DisplayName',label{method});ylabel('|H| (N m s)');title(sprintf('M%d',task));
 nexttile(task+2);hold on;plot(D.time_s,D.sigma_min,'Color',col,'DisplayName',label{method});ylabel('sigma min A');
 nexttile(task+4);hold on;plot(D.time_s,D.pitch_positive_support_Nm,'Color',col,'DisplayName',[label{method} ' positive']);plot(D.time_s,D.pitch_negative_support_Nm,'--','Color',col,'DisplayName',[label{method} ' negative']);ylabel('Pitch support (N m)');
 nexttile(task+6);hold on;plot(D.time_s,D.aero_y,'Color',col,'DisplayName',[label{method} ' aero']);plot(D.time_s,D.cmg_y,'--','Color',col,'DisplayName',[label{method} ' CMG']);ylabel('Actual pitch (N m)');
 nexttile(task+8);hold on;plot(D.time_s,D.attitude_error_deg,'Color',col,'DisplayName',label{method});ylabel('Attitude error (deg)');
end;end
axs=findall(f,'Type','axes');for k=1:numel(axs),axes(axs(k));grid on;xline(12,'k:','HandleVisibility','off');xline(35,'k--','HandleVisibility','off');xline(40,'k--','HandleVisibility','off');legend('Location','best');xlabel('Time (s)');end;v3_savefig(f,out,'M1M2_unload_response');
f=figure('Visible','off');tiledlayout(2,2);labels={'PID NoUnload','PID DynamicUnload','Joint MPC'};
for task=1:2,for method=1:3,D=data{task,method};nexttile(task);hold on;plot(D.time_s,D.pitch_deg,'Color',colors(method,:),'DisplayName',labels{method});if method==1,plot(D.time_s,D.ref_pitch_deg,'k--','DisplayName','reference');end;ylabel(sprintf('M%d pitch (deg)',task));nexttile(task+2);hold on;plot(D.time_s,D.attitude_error_deg,'Color',colors(method,:),'DisplayName',labels{method});ylabel('Attitude error (deg)');end;end
axs=findall(f,'Type','axes');for k=1:numel(axs),axes(axs(k));grid on;xlim([30 50]);xline(35,'k:','HandleVisibility','off');xline(40,'k:','HandleVisibility','off');legend('Location','best');xlabel('Time (s)');end;v3_savefig(f,out,'M1M2_following_maneuver');
end
