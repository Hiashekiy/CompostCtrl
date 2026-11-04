function report_online_authority_closedloop
% Report complete runs only, using simultaneous command/actual moment samples.
root=repo_root;base=fullfile(root,'simulink','results','online_authority');
bank=load(fullfile(root,'simulink','data','online_plan_bank.mat'));p=vehicle_params;v=p.vehicle;
folders=[dir(fullfile(base,'L0_*'));dir(fullfile(base,'L1_*'));dir(fullfile(base,'L2_*'))];rows=struct([]);initial=struct;
for f=1:numel(folders)
 folder=fullfile(folders(f).folder,folders(f).name);
 for arm={'A','B'}
  files=dir(fullfile(folder,[arm{1} '_surrogate_flight_*.mat']));if isempty(files),continue;end
  file=fullfile(folder,files(end).name);d=load(file);r=d.r;t=r.states.Time;
  x=values(r.states);u=values(r.commands);actual=values(r.tau_actual);
  if isfield(initial,arm{1})
   assert(max(abs(x(1,:)-initial.(arm{1})))<1e-12,'Ablation initial states differ');
  else,initial.(arm{1})=x(1,:);end
  delta=values(r.delta_actual);rate=values(r.delta_dot);
  q=eul2quat(x(:,[3 2 1]));ref=interp1(bank.time(:),squeeze(bank.q_ref(:,1,:))',t,'linear');ref=ref./vecnorm(ref,2,2);
  error=rad2deg(2*acos(min(1,abs(sum(q.*ref,2)))));torque_error=u(:,1:3)-actual;
  weights=[diff(t);0];
  row=struct('folder',folders(f).name,'arm',arm{1},'duration_s',t(end), ...
   'attitude_rms_deg',sqrt(mean(error.^2)),'attitude_peak_deg',max(error), ...
   'moment_error_rms_Nm',sqrt(mean(torque_error.^2,1)),'moment_error_peak_Nm',max(abs(torque_error),[],1), ...
   'deflection_saturation_s',sum((abs(delta)>=v.delta_max(:)'-.00001).*weights,1), ...
   'rate_saturation_s',sum((abs(rate)>=v.delta_dot_max(:)'-.00001).*weights,1));
  m='NearSpace_CMG_FlightControl';if strcmp(arm{1},'A'),m=[m '_SurfaceOnly'];end
  evidence=dir(fullfile(folder,[m '*_authority.mat']));
  if strcmp(arm{1},'B'),evidence=evidence(~contains({evidence.name},'SurfaceOnly'));end
  if isempty(evidence),continue;end
  d=load(fullfile(folder,evidence(end).name));infos=d.diagnostics;
  assert(numel(d.control_times)==numel(infos));
  assert(all(abs(diff(d.control_times)-.2)<1e-8),'MPC was not scheduled at 200 ms');
  certified=cellfun(@(i)isfield(i,'authority') && ~isempty(i.authority) && isfield(i.authority,'servo_vertex_residual'),infos);
  legacy=startsWith(folders(f).name,'L0_');
  if ~legacy && ~any(certified),continue;end % preliminary runs predate the final certificate schema
  row.fallbacks=sum(cellfun(@(i)i.fallback,infos));
  row.control_updates=numel(infos);
  row.worst_control_s=max(cellfun(@(i)i.elapsed_s,infos));
  row.mean_control_s=mean(cellfun(@(i)i.elapsed_s,infos));
  row.rti_iterations=sum(cellfun(@(i)optional(i,'iterations'),infos));
  row.rejected_candidates=sum(cellfun(@(i)optional(i,'rejected'),infos));
  row.repaired_candidates=sum(cellfun(@(i)optional(i,'repaired_candidates'),infos));
  row.prediction_step_s=0;
  row.max_allocation_residual_Nm=0;row.max_vertex_residual_Nm=0;row.max_servo_vertex_residual_rad=0;
  row.future_height_max_error_m=0;row.future_qbar_max_error_Pa=0;row.future_V_max_error_mps=0;row.future_alpha_max_error_rad=0;
  fc=values(r.flight_conditions);
  if ~legacy,measured=values(r.flight_servo_measured);end
  template=load(fullfile(root,'simulink','tests','05_python_matlab_qp_equivalence','production_online_qp.mat'));
  nodes=[0;cumsum(template.dts(:))];
  for k=1:numel(infos)
   if ~certified(k),continue;end
   a=infos{k}.authority;
   if isfield(a,'max_step_s'),row.prediction_step_s=a.max_step_s;end
   row.max_allocation_residual_Nm=max(row.max_allocation_residual_Nm,max(abs(a.allocation_residual),[],'all'));
   row.max_vertex_residual_Nm=max(row.max_vertex_residual_Nm,max(a.vertex_residual,[],'all'));
   row.max_servo_vertex_residual_rad=max(row.max_servo_vertex_residual_rad,max(a.servo_vertex_residual));
   future=d.control_times(k)+nodes(1:end-1);valid=future<=t(end);
   measured_future=interp1(t,[-measured(:,3) fc(:,1:3)],future(valid));
   predicted=a.air([1 4 2 5],valid)';difference=abs(predicted-measured_future);
   if ~isempty(difference)
    row.future_height_max_error_m=max(row.future_height_max_error_m,max(difference(:,1)));
    row.future_qbar_max_error_Pa=max(row.future_qbar_max_error_Pa,max(difference(:,2)));
    row.future_V_max_error_mps=max(row.future_V_max_error_mps,max(difference(:,3)));
    row.future_alpha_max_error_rad=max(row.future_alpha_max_error_rad,max(difference(:,4)));
   end
  end
  if legacy
   row.max_allocation_residual_Nm=NaN;row.max_vertex_residual_Nm=NaN;row.max_servo_vertex_residual_rad=NaN;
   row.future_height_max_error_m=NaN;row.future_qbar_max_error_Pa=NaN;row.future_V_max_error_mps=NaN;row.future_alpha_max_error_rad=NaN;
  end
  if strcmp(arm{1},'B')
   row.gamma_peak_rad=max(abs(x(:,7:10)),[],1);row.nu_peak_radps=max(abs(u(:,4:7)),[],1);
   assert(all(row.gamma_peak_rad>1e-3) && all(row.nu_peak_radps>1e-6),'CMG path is inactive');
  else,row.gamma_peak_rad=zeros(1,4);row.nu_peak_radps=zeros(1,4);end
  if isempty(rows),rows=row;else,rows(end+1)=orderfields(row,rows);end %#ok<AGROW>
  save(fullfile(folder,[arm{1} '_metrics.mat']),'row','t','error','torque_error');
  fid=fopen(fullfile(folder,[arm{1} '_metrics.json']),'w');fprintf(fid,'%s',jsonencode(row));fclose(fid);
  figure('Visible','off');subplot(2,1,1);plot(t,error);ylabel('attitude error (deg)');grid on;
  subplot(2,1,2);plot(t,torque_error);ylabel('command - actual (Nm)');xlabel('time (s)');grid on;
  exportgraphics(gcf,fullfile(folder,[arm{1} '_errors.png']));close(gcf);
 end
end
if ~isempty(rows)
 all_runs=struct2table(rows);writetable(all_runs,fullfile(base,'closedloop_metrics.csv'));
 selected=all_runs([],:);
 for mode={'L0','L1','L2'}
  for arm={'A','B'}
   for duration=[2 10 60]
    match=startsWith(all_runs.folder,[mode{1} '_']) & strcmp(all_runs.arm,arm{1}) & all_runs.duration_s==duration;
    if ~strcmp(mode{1},'L0'),match=match & all_runs.prediction_step_s==.1;end
    index=find(match,1,'last');if ~isempty(index),selected=[selected;all_runs(index,:)];end %#ok<AGROW>
   end
  end
 end
 writetable(selected,fullfile(base,'final_comparison.csv'));
 disp(selected(:,{'folder','arm','duration_s','attitude_rms_deg','attitude_peak_deg','moment_error_rms_Nm','fallbacks','worst_control_s'}));
end
end
function data=values(ts)
data=squeeze(ts.Data);if size(data,1)~=numel(ts.Time),data=data';end
end
function value=optional(s,name)
value=0;if isfield(s,name),value=s.(name);end
end
