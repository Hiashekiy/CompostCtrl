function report_dt01_sqp4(folders)
% Named raw logs only. Same-time aerodynamic moment error and SO(3) attitude
% distance, compared with the same interpolated original reference as before.
root=repo_root;base=fullfile(root,'simulink','results','online_authority');
out=fullfile(base,'dt01_sqp4');if ~exist(out,'dir'),mkdir(out);end
bank=load(fullfile(root,'simulink','data','online_plan_bank.mat'));
rows={};
for j=1:numel(folders)
 folder=fullfile(base,folders{j});
 for arm={'A','B'}
  files=dir(fullfile(folder,[arm{1} '_surrogate_flight_*.mat']));if isempty(files),continue;end
  assert(numel(files)==1,'Ambiguous result folder');
  d=load(fullfile(folder,files.name));r=d.r;
  evidence=dir(fullfile(folder,'*_authority.mat'));assert(numel(evidence)==1);
  log=load(fullfile(folder,evidence.name));infos=log.diagnostics;
  period=median(diff(log.control_times));
  assert(all(abs(diff(log.control_times)-period)<1e-8));
  assert(numel(infos)==round(r.states.Time(end)/period)+1);
  t=r.commands.Time;assert(isequal(t,r.tau_actual.Time));
  u=values(r.commands);u=u(:,1:3);actual=values(r.tau_actual);error=u-actual;
  x=values(r.states);q=eul2quat(x(:,[3 2 1]));
  ref=interp1(bank.time(:),squeeze(bank.q_ref(:,1,:))',r.states.Time,'linear');
  ref=ref./vecnorm(ref,2,2);
  attitude=rad2deg(2*acos(min(1,abs(sum(q.*ref,2)))));
  elapsed=cellfun(@(i)i.elapsed_s,infos);sorted=sort(elapsed);
  row=struct('folder',folders{j},'arm',arm{1},'control_period_s',period, ...
   'cycles',numel(infos),'duration_s',t(end),'max_sqp_iterations',infos{1}.max_sqp_iterations, ...
   'moment_command_rms_Nm',sqrt(mean(u.^2)), ...
   'moment_error_rms_Nm',sqrt(mean(error.^2)), ...
   'moment_error_peak_Nm',max(abs(error)), ...
   'moment_relative_percent',100*sqrt(sum(error.^2)./sum(u.^2)), ...
   'moment_combined_relative_percent',100*norm(error,'fro')/norm(u,'fro'), ...
   'attitude_rms_deg',sqrt(mean(attitude.^2)),'attitude_peak_deg',max(attitude), ...
   'mean_control_s',mean(elapsed),'median_control_s',median(elapsed), ...
   'p95_control_s',sorted(ceil(.95*numel(sorted))),'max_control_s',max(elapsed), ...
   'first_control_s',elapsed(1),'max_after_first_control_s',max(elapsed(2:end)), ...
   'over_budget_cycles',sum(elapsed>period),'over_budget_percent',100*mean(elapsed>period), ...
   'qp_per_cycle_max',max(cellfun(@(i)i.qp_solved,infos)), ...
   'hpipm_internal_mean_per_cycle_s',mean(cellfun(@(i)sum(i.qp_solver_time_s),infos)), ...
   'fallbacks',sum(cellfun(@(i)i.fallback,infos)), ...
   'max_vertex_residual_Nm',0,'max_servo_vertex_residual_rad',0);
  qptimes=cellfun(@(i)i.qp_solver_time_s(:),infos,'UniformOutput',false);
  qptimes=vertcat(qptimes{:});row.hpipm_internal_mean_per_qp_s=mean(qptimes);
  for k=1:numel(infos)
   a=infos{k}.authority;if isempty(a),continue;end
   row.max_vertex_residual_Nm=max(row.max_vertex_residual_Nm,max(a.vertex_residual,[],'all'));
   row.max_servo_vertex_residual_rad=max(row.max_servo_vertex_residual_rad,max(a.servo_vertex_residual));
   assert(all(abs(a.axis_scale(2,:)-1)<1e-10));
  end
  assert(row.qp_per_cycle_max<=row.max_sqp_iterations);
  assert(row.max_vertex_residual_Nm<1e-9 && row.max_servo_vertex_residual_rad<1e-9);
  if abs(period-.1)<1e-8
   assert(row.max_sqp_iterations==4 && row.fallbacks==0);
   for k=1:numel(infos)
    assert(abs(infos{k}.control_period_s-.1)<1e-12 && abs(infos{k}.prediction_dts(1)-.1)<1e-12);
   end
  end
  rows{end+1}=row; %#ok<AGROW>
  save(fullfile(out,[folders{j} '_' arm{1} '_metrics.mat']),'row','attitude','error','t','elapsed');
 end
end
assert(~isempty(rows));rows=[rows{:}];
fid=fopen(fullfile(out,'comparison.json'),'w');fprintf(fid,'%s',jsonencode(rows,'PrettyPrint',true));fclose(fid);
writetable(struct2table(rows),fullfile(out,'comparison.csv'));
disp(jsonencode(rows,'PrettyPrint',true));
end
function value=values(ts)
value=squeeze(ts.Data);if size(value,1)~=numel(ts.Time),value=value';end
end
