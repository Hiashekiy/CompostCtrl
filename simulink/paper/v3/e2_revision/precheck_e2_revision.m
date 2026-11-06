function precheck_e2_revision
check_e2_scenario;
old=fullfile(repo_root,'simulink/results/paper_experiments/20261105_redesign_v1');out=fullfile(repo_root,'simulink/results/paper_experiments/20261106_e2_unload_revision');
R=readtable(fullfile(out,'tables/M1M2_offline_feasibility.csv'));valid=true;proof={};
for task=1:2,Q=R(R.task==task,:);A=Q.time_s>=2 & Q.time_s<12;B=Q.time_s>=12 & Q.time_s<35;C=Q.time_s>=35 & Q.time_s<=40;
 z=struct('task',task,'A_samples',sum(A),'A_feasible_ticks',sum(Q.selected_scale(A)>0),'A_mean_extra_Nm',mean(Q.feasible_extra_norm_Nm(A)),'B_mean_extra_Nm',mean(Q.feasible_extra_norm_Nm(B)),'A_qbar_range_Pa',[min(Q.qbar_Pa(A)) max(Q.qbar_Pa(A))],'B_qbar_range_Pa',[min(Q.qbar_Pa(B)) max(Q.qbar_Pa(B))],'C_qbar_range_Pa',[min(Q.qbar_Pa(C)) max(Q.qbar_Pa(C))]);
 z.passed=all(Q.selected_scale(A)>0) && z.A_mean_extra_Nm>z.B_mean_extra_Nm && max(Q.qbar_Pa(C))<min(Q.qbar_Pa(A));proof{end+1}=z;valid=valid && z.passed;
end
paper_json(fullfile(out,'config/scenario_feasibility_gate.json'),struct('passed',valid,'proof',{proof},'not_a_benefit_requirement',true,'scope','Existing native trajectory with coupled candidate allocation and20ms surface+CMG drive prediction'));
assert(valid,'e2r:scenario','No credible A/B/C trajectory; stop formal simulations');
write_e2_scenario_report(out);
% No retuning: both PID variants use exactly the old frozen parameter struct.
cfg=jsondecode(fileread(fullfile(old,'calibration/pid_frozen.json')));results={};
Simulink.fileGenControl('set','CacheFolder',fullfile(out,'cache'),'CodeGenFolder',fullfile(out,'cache/codegen'),'createDir',true);
for task=1:2
 s=v3_spec;s.group='E2_PRECHECK';s.id=sprintf('E2R_PC_M%d_DU',task);s.pair_id=sprintf('E2R_PC_M%d',task);s.method='PID_DynamicUnload';s.onset=35;s.transition=5;s.duration=16;s.amplitude=5;s.phase='precheck';s.pid=cfg;s.pid.unload=true;if task==1,s.gamma0=[pi/2;pi/2;0;0];else,s.gamma0=ones(4,1)*1.5;end
 m=e2r_run_case(s,out);assert(~isempty(m),'e2r:infrastructure','Precheck failed');folder=fullfile(out,s.group,'cases',s.id);[D,~,~]=e2r_native(folder);[~,u]=e2r_unload_log(folder);p=vehicle_params;
 bounds=~any(abs(D{:,{'nu1','nu2','nu3','nu4'}})>p.cmg.gdot_max+1e-6,'all') && ~any(abs(D{:,{'acc1','acc2','acc3','acc4'}})>p.cmg.gddot_max+1e-6,'all') && ~any(abs(D{:,{'delta1','delta2','delta3'}})>p.vehicle.delta_max(:)'+1e-6,'all') && ~any(abs(D{:,{'delta_dot1','delta_dot2','delta_dot3'}})>p.vehicle.delta_dot_max(:)'+1e-6,'all');
 z=struct('id',s.id,'complete',m.complete,'status',m.status,'physical_bounds',bounds,'hold_peak_after2_deg',max(D.attitude_error_deg(D.time_s>=2)),'initial_H_Nms',D.Hnorm(1),'final_H_Nms',D.Hnorm(end),'unload',u);
 z.passed=m.complete && bounds && z.hold_peak_after2_deg<=1 && u.active_ticks>0 && u.strict_predicted_momentum_descent && u.predicted_compensation_max_Nm<=cfg.unload_compensation_tolerance+1e-8 && u.actual_compensation_max_Nm<=cfg.unload_compensation_tolerance+1e-6 && ~ismember(m.status,{'MODEL_DOMAIN_FAIL','CONTROLLER_FAIL'});
 results{end+1}=z;
end
passed=all(cellfun(@(z)z.passed,results));paper_json(fullfile(out,'reports/precheck_acceptance.json'),struct('passed',passed,'new_informal_simulations',2,'results',{results},'strict_attitude_guard_deg',1,'unchanged_compensation_tolerance_Nm',cfg.unload_compensation_tolerance));
fprintf('E2_PRECHECK_COMPLETE passed=%d\n',passed);
end
