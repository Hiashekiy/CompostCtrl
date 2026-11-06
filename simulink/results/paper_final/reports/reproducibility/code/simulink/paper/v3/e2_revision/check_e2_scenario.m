function check_e2_scenario
addpath('simulink');init_paths;addpath('simulink/paper','simulink/paper/v3','simulink/paper/v3/e2_revision');
old=fullfile(repo_root,'simulink/results/paper_experiments/20261105_redesign_v1');out=fullfile(repo_root,'simulink/results/paper_experiments/20261106_e2_unload_revision');
assert(isfile(fullfile(out,'reports/C2_第二次退化诊断.md')),'Finish C2 diagnosis before unloading work');
p=vehicle_params;supplement_c2_dynamics(out,p);c=jsondecode(fileread(fullfile(old,'calibration/pid_frozen.json')));c.unload=false;rows={};equivalence=[];
for task=1:2
 d=load(fullfile(out,'config',sprintf('M%d_native_basis.mat',task)));D=d.D;r=d.r;b=d.b;t=(0:.2:60)';x=interp1(r.states.Time,squeeze(r.states.Data),t,'previous');flight=interp1(r.flight_servo_measured.Time,squeeze(r.flight_servo_measured.Data),t,'previous');rates=interp1(r.nu_actual.Time,squeeze(r.nu_actual.Data),t,'previous');qr=interp1(b.time(:),squeeze(b.q_ref(:,1,:))',t,'linear');wr=interp1(b.time(:),squeeze(b.omega_ref(:,1,:))',t,'linear');z0=[];z1=[];local={};maxdifference=0;
 for k=1:numel(t)
  previous=zeros(4,1);if ~isempty(z0),previous=z0.nu;end
  [u0,z0]=v3_pid_step(x(k,:)',qr(k,:)',wr(k,:)',flight(k,:)',c,z0);
  [u1,z1]=e2r_pid_step(x(k,:)',qr(k,:)',wr(k,:)',flight(k,:)',c,z1,rates(k,:)');maxdifference=max(maxdifference,max(abs(u0-u1)));
  air=online_flight_condition(flight(k,1:3)',flight(k,4:6)');
  [~,~,choice]=e2r_select_unload(x(k,7:10)',rates(k,:)',previous,z0.nu_base,z0.tau_a_request,flight(k,:)',x(k,4:6)',air,c);
  [nat,B]=online_aero_moment(p.vehicle,air,x(k,4:6)',flight(k,7:9)');[dl,du]=online_surface_reachability(p.vehicle,flight(k,7:9)',.2,flight(k,10:12)');
  % Signed pitch projection headroom is descriptive only; full vector candidates decide feasibility.
  cy=B(2,:);high=nat(2)+sum(max(cy',0).*du+min(cy',0).*dl);low=nat(2)+sum(max(cy',0).*dl+min(cy',0).*du);
  v=struct('task',task,'time_s',t(k),'qbar_Pa',air(1),'height_m',-flight(k,3),'speed_mps',air(2),'alpha_deg',rad2deg(air(3)),'Hnorm_Nms',norm(z0.H),'base_residual_Nm',norm(z0.residual),'aero_pitch_positive_headroom_Nm',high-z0.tau_a_request(2),'aero_pitch_negative_headroom_Nm',z0.tau_a_request(2)-low,'feasible_extra_norm_Nm',choice.selected_extra_Nm,'selected_scale',choice.scale,'predicted_compensation_error_Nm',choice.compensation_error_endpoint_Nm,'predicted_H_delta_vs_base_Nms',choice.H_delta_vs_base_Nms,'surface_command_margin_rad',choice.surface_command_margin_rad,'extra_x_Nm',choice.extra_a(1),'extra_y_Nm',choice.extra_a(2),'extra_z_Nm',choice.extra_a(3),'reason',choice.reason);
  local{end+1}=v;rows{end+1}=v;
 end
 assert(maxdifference<1e-10,'No-unload copy changed frozen baseline');equivalence(task)=maxdifference;
end
R=struct2table([rows{:}]);writetable(R,fullfile(out,'tables/M1M2_offline_feasibility.csv'));
f=figure('Visible','off');tiledlayout(3,2);for task=1:2,Q=R(R.task==task,:);nexttile(task);plot(Q.time_s,Q.qbar_Pa);ylabel(sprintf('M%d qbar (Pa)',task));nexttile(task+2);plot(Q.time_s,Q.aero_pitch_positive_headroom_Nm,Q.time_s,Q.aero_pitch_negative_headroom_Nm);ylabel('Pitch headroom (N m)');legend('positive projection','negative projection');nexttile(task+4);plot(Q.time_s,Q.feasible_extra_norm_Nm);ylabel('Coupled feasible unload (N m)');end
axs=findall(f,'Type','axes');for k=1:numel(axs),axes(axs(k));grid on;xline(2,'k:');xline(12,'k:');xline(35,'k--');xline(40,'k--');xline(50,'k:');xlabel('Physical time (s)');end;v3_savefig(f,out,'M1M2_flight_authority_window');
env=load(fullfile(repo_root,'simulink/data/native_environment.mat'));
paper_json(fullfile(out,'config/scenario_offline.json'),struct('source','Original no-unload M1/M2 native60s trajectories, not new simulations','source_initial_height_m',env.h0,'source_initial_velocity_NED_mps',env.V_ned0,'proposed_A_s',[2 12],'proposed_B_s',[12 35],'proposed_C_maneuver_s',[35 40],'proposed_C_observation_s',[40 50],'duration_s',60,'no_unload_replay_max_command_difference',equivalence,'dynamic_candidate_source_sha256',paper_hash(which('e2r_select_unload')),'warning','Offline local feasibility is conditional on baseline state; closed-loop prechecks required. Projection headroom alone is not coupled feasibility. qbar>=1 is only a coarse stop gate.'));
fprintf('E2_SCENARIO_OFFLINE_COMPLETE no-unload-equivalence=%g\n',max(equivalence));
end
