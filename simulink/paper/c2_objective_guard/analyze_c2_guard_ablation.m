function analyze_c2_guard_ablation
% Full-resolution read-only analysis of exactly two saved C2 outputs.
init_paths;root=repo_root;addpath(fullfile(root,'simulink/paper'),fullfile(root,'simulink/paper/v3'));
out=fullfile(root,'simulink/results/paper_experiments/C2_objective_guard_ablation');
base=fullfile(root,'simulink/results/paper_final/E2_Controller_Comparison');
p=vehicle_params;polar=load(fullfile(root,'simulink/data/public_surrogate/force_polar.mat'));
bank=load(fullfile(out,'config/reference_bank.mat'));cfg=jsondecode(fileread(fullfile(out,'config/case_config.json')));
names={'guard_on','guard_off'};events={};globals={};constraints={};cycles={};steps={};summaries={};native={};checks={};
for arm=1:2
 method=names{arm};if arm==1,raw=fullfile(base,'raw/V2_C2_3');else,raw=fullfile(out,'raw');end
 d=load(fullfile(raw,'sim_output.mat'),'r');r=d.r;t=r.states.Time(:);x=squeeze(r.states.Data);assert(size(x,1)==numel(t));
 [qr,ref]=paper_reference_series(bank,t);q=paper_euler_quaternions(x(:,1:3));err=rad2deg(2*acos(min(1,abs(sum(q.*qr,2)))));
 ta=align(r.tau_actual,t,'linear');tc=align(r.cmg_torque,t,'linear');H=align(r.cmg_H,t,'linear');nu=align(r.nu_actual,t,'previous');acc=align(r.cmg_acceleration,t,'previous');delta=align(r.delta_actual,t,'previous');rate=align(r.delta_dot,t,'previous');air=align(r.flight_conditions,t,'linear');command=align(r.commands,t,'previous');
 complete=t(end)>=59.999&&isempty(r.ErrorMessage);overall=sqrt(mean(err.^2));globalrow=struct('method',method,'complete',complete,'actual_end_s',t(end),'rmse_observed_deg',overall,'rmse_full_60s_deg',NaN,'peak_attitude_error_deg',max(err),'pitch_rmse_observed_deg',sqrt(mean((ref(:,2)-rad2deg(x(:,2))).^2)),'simulation_error',r.ErrorMessage);if complete,globalrow.rmse_full_60s_deg=overall;end;globals{end+1}=globalrow;
 for j=1:3
  start=cfg.events(j,1);finish=start+cfg.events(j,2);next=inf;if j<3,next=cfg.events(j+1,1);end;stop=min([finish+10,next,t(end)]);during=t>=start-1e-9&t<=finish+1e-9;after=t>finish+1e-9&t<=stop+1e-9;
  events{end+1}=struct('method',method,'event_index',j,'onset_s',start,'finish_s',finish,'post_end_s',stop,'post_duration_s',max(0,stop-finish),'complete_maneuver',t(end)>=finish-.001,'maneuver_samples',sum(during),'maneuver_rmse_deg',rms_safe(err(during)),'maneuver_peak_deg',max_safe(err(during)),'post_rmse_deg',rms_safe(err(after)),'post_peak_deg',max_safe(err(after)),'full_60s_rmse_deg',globalrow.rmse_full_60s_deg);
 end
 derivedH=p.cmg.h0*(cos(x(:,7:10))*p.cmg.s0'+sin(x(:,7:10))*p.cmg.gxs0');
 derivedTorque=p.cmg.h0*((sin(x(:,7:10)).*nu)*p.cmg.s0'-(cos(x(:,7:10)).*nu)*p.cmg.gxs0');
 geometry_H=max(abs(derivedH-H),[],'all');geometry_torque=max(abs(derivedTorque-tc),[],'all');
 groups={delta,rate,nu,acc};limits={p.vehicle.delta_max(:)',p.vehicle.delta_dot_max(:)',repmat(p.cmg.gdot_max,1,4),repmat(p.cmg.gddot_max,1,4)};labels={'surface_angle_rad','surface_rate_radps','cmg_rate_radps','cmg_acceleration_radps2'};all_valid=true;
 for window=1:2
  mask=true(size(t));lo=0;hi=t(end);if window==2,lo=16;hi=28;mask=t>=lo&t<=hi;end
  for j=1:4
   values=abs(groups{j}(mask,:));limit=limits{j};violations=any(values>limit+1e-6,2);saturation=any(values>=limit-1e-5,2);all_valid=all_valid&&~any(violations);
   constraints{end+1}=struct('method',method,'quantity',labels{j},'window_start_s',lo,'window_end_s',hi,'samples',sum(mask),'limits',jsonencode(limit),'max_abs_per_channel',jsonencode(max(values,[],1)),'max_positive_excess',max([0;reshape(values-limit,[],1)]),'violation_samples',sum(violations),'saturation_samples',sum(saturation),'saturation_pct',100*mean(saturation));
  end
 end
 T=array2table([t,rad2deg(x(:,1:3)),ref,err,rad2deg(x(:,4:6)),ta,tc,x(:,7:10),nu,acc,H,vecnorm(H,2,2),delta,rate,air(:,1),rad2deg(air(:,3)),command], ...
  'VariableNames',{'time_s','roll_deg','pitch_deg','yaw_deg','ref_roll_deg','ref_pitch_deg','ref_yaw_deg','attitude_error_deg','p_degps','q_degps','r_degps','aero_x_Nm','aero_y_Nm','aero_z_Nm','cmg_x_Nm','cmg_y_Nm','cmg_z_Nm','gamma1_rad','gamma2_rad','gamma3_rad','gamma4_rad','nu1_radps','nu2_radps','nu3_radps','nu4_radps','acc1_radps2','acc2_radps2','acc3_radps2','acc4_radps2','Hx_Nms','Hy_Nms','Hz_Nms','Hnorm_Nms','delta1_rad','delta2_rad','delta3_rad','delta_rate1_radps','delta_rate2_radps','delta_rate3_radps','qbar_Pa','alpha_deg','aero_cmd_x_Nm','aero_cmd_y_Nm','aero_cmd_z_Nm','nu_cmd1_radps','nu_cmd2_radps','nu_cmd3_radps','nu_cmd4_radps'});
 native{arm}=T;ix=extrema_indices(table2array(T));writetable(T(ix,:),fullfile(out,'tables',[method '_plot_timeseries.csv']));writetable(T(t>=16&t<=28,:),fullfile(out,'tables',[method '_physical_16_28_native.csv']));
 files=dir(fullfile(raw,'*_authority.mat'));assert(numel(files)==1);a=load(fullfile(raw,files(1).name));certificate_ok=true;guard_ok=true;
 for k=1:numel(a.diagnostics)
  z=a.diagnostics{k};ct=a.control_times(k);reason=getv(z,'early_stop_reason','');fallback=getv(z,'fallback',false);deltas=getv(z,'objective_delta_by_step',[]);accepted=getv(z,'objective_step_accepted',[]);would=getv(z,'objective_would_reject',[]);
  would_count=sum(would);if arm==1,would_count=double(strcmp(reason,'objective_increase'));end
  physical_reject=getv(z,'physical_rejected_steps',double(startsWith(reason,'rejected: ')));
  code=0;if strcmp(reason,'objective_increase'),code=1;elseif physical_reject>0,code=2;elseif strcmp(reason,'qp_failed'),code=3;elseif fallback,code=4;end
  cycle=struct('method',method,'time_s',ct,'qp_attempts',numel(z.status_by_qp),'qp_successes',sum(z.status_by_qp==0),'qp_failures',sum(z.status_by_qp~=0),'last_qp_status',z.status,'sqp_iterations',z.iterations,'max_sqp_iterations',z.max_sqp_iterations,'accepted_steps',z.accepted_steps,'rejected_steps',z.rejected_steps,'physical_candidate_rejections',physical_reject,'would_objective_reject',would_count,'accepted_increase_steps',sum(logical(would)&logical(accepted)),'candidate_objectives_logged',numel(deltas),'objective_nominal',getv(z,'nominal_objective',NaN),'objective_final',z.objective,'candidate_delta_sum',NaN,'candidate_delta_max',max_safe(deltas),'objective_tol',z.objective_tol,'fallback',fallback,'fallback_reason',getv(z,'fallback_reason',''),'early_stop_reason',reason,'rejection_code',code,'elapsed_s',z.elapsed_s);
  if arm==2,cycle.candidate_delta_sum=sum(deltas);guard_ok=guard_ok&&isfield(z,'enforce_objective_decrease')&&~z.enforce_objective_decrease;end
  cycles{end+1}=cycle;
  for j=1:numel(deltas)
   steps{end+1}=struct('method',method,'time_s',ct,'sqp_iteration',z.objective_candidate_iteration(j),'objective_before',z.objective_before_by_step(j),'candidate_objective',z.candidate_objective_by_step(j),'delta_J',deltas(j),'objective_tol',z.objective_tol,'would_objective_reject',logical(would(j)),'accepted',logical(accepted(j)));
  end
  for j=1:numel(z.iteration_authority)
   aa=z.iteration_authority{j};certificate_ok=certificate_ok&&all(aa.certified)&&max(aa.vertex_residual,[],'all')<=1e-9&&max(abs(aa.allocation_residual),[],'all')<=1e-7;
  end
 end
 checks{end+1}=struct('method',method,'complete_60s',complete,'truth_1ms',max(abs(diff(t)-.001))<1e-8,'all_real_actuator_bounds_passed',all_valid,'logged_authority_certificates_passed',certificate_ok,'cmg_H_geometry_max_error',geometry_H,'cmg_torque_geometry_max_error',geometry_torque,'cmg_geometry_passed',geometry_H<1e-7&&geometry_torque<1e-7,'actual_alpha_min_deg',min(T.alpha_deg),'actual_alpha_max_deg',max(T.alpha_deg),'observed_model_domain_valid',all(T.alpha_deg>=rad2deg(min(polar.alpha_rad))-1e-8&T.alpha_deg<=rad2deg(max(polar.alpha_rad))+1e-8),'guard_setting_matches_experiment',guard_ok);
end
E=struct2table([events{:}]);G=struct2table([globals{:}]);C=struct2table([cycles{:}]);P=struct2table([constraints{:}]);K=struct2table([checks{:}]);
assert(all(C.max_sqp_iterations==4)&&all(C.sqp_iterations<=4));
baseline=E(strcmp(E.method,'guard_on'),:);expected=[.106867606969546;3.23261114673668;.557230079115791];assert(max(abs(baseline.maneuver_rmse_deg-expected))<1e-8);
for method=names
 mask=strcmp(C.method,method{1})&C.time_s>=12-1e-9&C.time_s<24-1e-9;R=C(mask,:);summaries{end+1}=struct('method',method{1},'window_start_s',12,'window_end_exclusive_s',24,'control_cycles',height(R),'qp_attempts',sum(R.qp_attempts),'qp_successes',sum(R.qp_successes),'qp_failures',sum(R.qp_failures),'accepted_steps',sum(R.accepted_steps),'would_objective_reject',sum(R.would_objective_reject),'accepted_increase_steps',sum(R.accepted_increase_steps),'physical_candidate_rejections',sum(R.physical_candidate_rejections),'fallbacks',sum(R.fallback),'sqp_iterations_mean',mean(R.sqp_iterations),'sqp_iterations_max',max(R.sqp_iterations),'objective_delta_sum_observed',sum(R.candidate_delta_sum),'objective_stagnation_stops',sum(strcmp(R.early_stop_reason,'objective_stagnation')));
end
S=struct2table([summaries{:}]);assert(S.control_cycles(1)==60&&S.qp_successes(1)==60&&S.accepted_steps(1)==0&&S.would_objective_reject(1)==60);
comparison={};for j=1:3,A=E(strcmp(E.method,'guard_on')&E.event_index==j,:);B=E(strcmp(E.method,'guard_off')&E.event_index==j,:);comparison{end+1}=struct('event_index',j,'onset_s',A.onset_s,'finish_s',A.finish_s,'baseline_post_end_s',A.post_end_s,'ablation_post_end_s',B.post_end_s,'baseline_maneuver_rmse_deg',A.maneuver_rmse_deg,'ablation_maneuver_rmse_deg',B.maneuver_rmse_deg,'rmse_change_deg',B.maneuver_rmse_deg-A.maneuver_rmse_deg,'baseline_peak_deg',A.maneuver_peak_deg,'ablation_peak_deg',B.maneuver_peak_deg,'baseline_post_rmse_deg',A.post_rmse_deg,'ablation_post_rmse_deg',B.post_rmse_deg,'baseline_full_rmse_deg',A.full_60s_rmse_deg,'ablation_full_rmse_deg',B.full_60s_rmse_deg);end
writetable(E,fullfile(out,'tables/event_metrics.csv'));writetable(struct2table([comparison{:}]),fullfile(out,'tables/event_comparison.csv'));writetable(G,fullfile(out,'tables/global_metrics.csv'));writetable(C,fullfile(out,'tables/optimizer_diagnostics.csv'));writetable(S,fullfile(out,'tables/optimizer_12_24_summary.csv'));writetable(P,fullfile(out,'tables/physical_constraints.csv'));writetable(K,fullfile(out,'tables/physical_verification.csv'));
if ~isempty(steps),writetable(struct2table([steps{:}]),fullfile(out,'tables/optimizer_steps.csv'));end
% Preserve exact full-resolution extracted series for independent rechecking.
save(fullfile(out,'raw/analysis_native.mat'),'native','E','G','C','P','K','S','-v7.3');
receipt=struct('baseline_metrics_match',true,'baseline_60_successful_qps_and_zero_accepted_reproduced',true,'new_simulations',1,'max_sqp_iterations_unchanged',true,'all_physical_constraints_passed',all(K.all_real_actuator_bounds_passed),'all_saved_certificates_passed',all(K.logged_authority_certificates_passed),'all_cmg_geometry_checks_passed',all(K.cmg_geometry_passed),'guard_off_was_used',all(K.guard_setting_matches_experiment),'complete_60s',K.complete_60s(2),'baseline_candidate_deltas_recorded',false,'plot_sampling','native20ms-bin endpoints and every-column extrema; metrics use all native1ms samples');paper_json(fullfile(out,'checks/analysis_receipt.json'),receipt);
disp(struct2table([comparison{:}]));disp(S);disp(K);fprintf('C2_GUARD_ANALYSIS_COMPLETE\n');
end
function y=getv(x,key,default),y=default;if isfield(x,key),y=x.(key);end;end
function v=rms_safe(x),v=NaN;if ~isempty(x),v=sqrt(mean(x.^2));end;end
function v=max_safe(x),v=NaN;if ~isempty(x),v=max(x);end;end
function a=align(ts,t,method),a=interp1(ts.Time,squeeze(ts.Data),t,method,'extrap');end
function idx=extrema_indices(V)
n=size(V,1);m=floor(n/20);idx=[1:20:n,n];for j=2:size(V,2),A=reshape(V(1:20*m,j),20,m);[~,lo]=min(A,[],1);[~,hi]=max(A,[],1);idx=[idx,(0:m-1)*20+lo,(0:m-1)*20+hi];end;idx=unique(idx);
end
