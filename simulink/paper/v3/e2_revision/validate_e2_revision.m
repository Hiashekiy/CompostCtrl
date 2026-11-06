function validate_e2_revision
addpath('simulink');init_paths;addpath('simulink/paper','simulink/paper/v3','simulink/paper/v3/e2_revision');
old=fullfile(repo_root,'simulink/results/paper_experiments/20261105_redesign_v1');out=fullfile(repo_root,'simulink/results/paper_experiments/20261106_e2_unload_revision');p=vehicle_params;proof={};
for task=1:2,for method=1:2
 folder=fullfile(out,'E2/cases',sprintf('E2R_M%d_%d',task,method));h=load(fullfile(folder,'pid_diagnostics.mat'));d=load(fullfile(folder,'sim_output.mat'),'r');r=d.r;maxrate=0;maxangle=0;sensore=0;count=0;active=0;maxHerror=0;
 for j=1:numel(h.times)
  t=h.times(j);z=h.history{j};if t+.02>r.states.Time(end)+1e-9,continue;end
  i=round(t*1000)+1;in=round((t+.02)*1000)+1;states=squeeze(r.states.Data);rates=squeeze(r.nu_actual.Data);assert(abs(r.states.Time(i)-t)<1e-8 && abs(r.states.Time(in)-(t+.02))<1e-8);
  x=states(i,:)';nu=rates(i,:)';sensore=max(sensore,max(abs(z.actual_rate-nu)));
  [gp,np]=e2r_drive_endpoint(x(7:10),nu,z.nu,.02);xn=states(in,:)';nn=rates(in,:)';maxrate=max(maxrate,max(abs(np-nn)));maxangle=max(maxangle,max(abs(gp-xn(7:10))));count=count+1;
  if z.eta>0
   active=active+1;c=z.dynamic_choice;Hb=H(x(7:10)+.2*z.nu_base,p);Hn=H(x(7:10)+.2*z.nu,p);assert(norm(Hn)<norm(Hb)-1e-7);maxHerror=max(maxHerror,abs(norm(Hn)-norm(Hb)-c.H_delta_vs_base_Nms));
  end
 end
 z=struct('task',task,'variant',method,'validated_endpoints',count,'active_descent_checks',active,'measured_rate_input_max_error_radps',sensore,'drive_endpoint_max_rate_error_radps',maxrate,'drive_endpoint_max_angle_error_rad',maxangle,'independent_H_delta_max_error_Nms',maxHerror);
 z.passed=sensore<1e-8 && maxrate<1e-8 && maxangle<1e-8 && maxHerror<1e-8;assert(z.passed,'e2r:drive','Physical drive prediction mismatch');proof{end+1}=z;
end;end
% Full templates must agree, including all MPC weights, grid and SQP settings.
templates={};for task=1:2
 folders={fullfile(old,'E2/cases',sprintf('V2_M%d_3',task)),fullfile(out,'E2/cases',sprintf('E2R_M%d_3',task))};base=[];
 for k=1:2,b=jsondecode(fileread(fullfile(folders{k},'build_config.json')));load_system(b.model_path);mw=get_param(b.model,'ModelWorkspace');v=getVariable(mw,'native_controller_template');close_system(b.model,0);if k==1,base=v;else,assert(isequaln(base,v),'MPC template changed');end;end
 templates{end+1}=struct('task',task,'entire_template_identical',true,'horizon_s',sum(v.dts),'sqp_limit',v.max_sqp_iterations);
end
paper_json(fullfile(out,'reports/independent_physical_verification.json'),struct('passed',true,'drive_and_descent_checks',{proof},'MPC_templates',{templates},'no_extra_simulations',true));
report_e2_revision(old,out); % postprocessing only, including final short JSON key and titles
R=readtable(fullfile(out,'tables/M1M2_offline_feasibility.csv'));f=figure('Visible','off');tiledlayout(3,2);
for task=1:2,Q=R(R.task==task,:);nexttile(task);plot(Q.time_s,Q.qbar_Pa);title(sprintf('M%d: A 2-12s; B 12-35s; C 35-40s',task));ylabel('qbar (Pa)');nexttile(task+2);plot(Q.time_s,Q.aero_pitch_positive_headroom_Nm,Q.time_s,Q.aero_pitch_negative_headroom_Nm);ylabel('Pitch headroom (N m)');legend('positive projection','negative projection');nexttile(task+4);plot(Q.time_s,Q.feasible_extra_norm_Nm);ylabel('Coupled feasible unload (N m)');end
axs=findall(f,'Type','axes');for k=1:numel(axs),axes(axs(k));grid on;xline(2,'k:');xline(12,'k:');xline(35,'k--');xline(40,'k--');xline(50,'k:');xlabel('Time (s)');end;v3_savefig(f,out,'M1M2_flight_authority_window');
fprintf('E2_INDEPENDENT_PHYSICAL_VERIFICATION_COMPLETE\n');
end
function h=H(g,p)
h=sum(p.cmg.h0*(p.s0.*cos(g')+p.gxs0.*sin(g')),2);
end
