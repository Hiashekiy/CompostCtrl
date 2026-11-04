function report_five_sqp_comparison(folders)
% Metric rows for a named set of online result folders, written into five_sqp/.
% Same formulas as report_online_authority_closedloop, but it never rewrites the
% historical folders, so an old run cannot be silently replaced while comparing.
root=repo_root;base=fullfile(root,'simulink','results','online_authority');
out=fullfile(base,'five_sqp');if ~exist(out,'dir'),mkdir(out);end
if nargin<1 || isempty(folders),error('report_five_sqp_comparison: folders required');end
bank=load(fullfile(root,'simulink','data','online_plan_bank.mat'));p=vehicle_params;v=p.vehicle;
template=load(fullfile(root,'simulink','tests','05_python_matlab_qp_equivalence','production_online_qp.mat'));
nodes=[0;cumsum(template.dts(:))];rows={};names={};
for f=1:numel(folders)
 folder=fullfile(base,folders{f});
 for arm={'A','B'}
  files=dir(fullfile(folder,[arm{1} '_surrogate_flight_*.mat']));if isempty(files),continue;end
  d=load(fullfile(folder,files(end).name));r=d.r;t=r.states.Time;
  x=values(r.states);u=values(r.commands);actual=values(r.tau_actual);
  delta=values(r.delta_actual);rate=values(r.delta_dot);
  q=eul2quat(x(:,[3 2 1]));ref=interp1(bank.time(:),squeeze(bank.q_ref(:,1,:))',t,'linear');
  ref=ref./vecnorm(ref,2,2);error=rad2deg(2*acos(min(1,abs(sum(q.*ref,2)))));
  torque_error=u(:,1:3)-actual;weights=[diff(t);0];
  row=struct('folder',folders{f},'arm',arm{1},'duration_s',t(end), ...
   'attitude_rms_deg',sqrt(mean(error.^2)),'attitude_peak_deg',max(error), ...
   'moment_error_rms_Nm',[sqrt(mean(torque_error.^2,1))], ...
   'moment_error_peak_Nm',[max(abs(torque_error),[],1)], ...
   'deflection_saturation_s',sum((abs(delta)>=v.delta_max(:)'-.00001).*weights,1), ...
   'rate_saturation_s',sum((abs(rate)>=v.delta_dot_max(:)'-.00001).*weights,1));
  m='NearSpace_CMG_FlightControl';if strcmp(arm{1},'A'),m=[m '_SurfaceOnly'];end
  evidence=dir(fullfile(folder,[m '*_authority.mat']));
  if strcmp(arm{1},'B'),evidence=evidence(~contains({evidence.name},'SurfaceOnly'));end
  if isempty(evidence)
   row.fallbacks=NaN;row.control_updates=NaN;row.mean_control_s=NaN;row.worst_control_s=NaN;
   row.rti_iterations=NaN;row.rejected_candidates=NaN;row.repaired_candidates=NaN;
   row.max_allocation_residual_Nm=NaN;row.max_vertex_residual_Nm=NaN;row.max_servo_vertex_residual_rad=NaN;
   row.future_height_max_error_m=NaN;row.future_qbar_max_error_Pa=NaN;
   row.future_V_max_error_mps=NaN;row.future_alpha_max_error_rad=NaN;
   row.gamma_peak_rad=zeros(1,4);row.nu_peak_radps=zeros(1,4);
  else
   e=load(fullfile(folder,evidence(end).name));infos=e.diagnostics;
   assert(numel(e.control_times)==numel(infos));
   period=median(diff(e.control_times));row.control_period_s=period;
   assert(all(abs(diff(e.control_times)-period)<1e-8),'Irregular MPC schedule');
   nodes=[0;cumsum(template.dts(:))];
   if isfield(infos{1},'prediction_dts'),nodes=[0;cumsum(infos{1}.prediction_dts(:))];end
   row.fallbacks=sum(cellfun(@(i)i.fallback,infos));
   row.control_updates=numel(infos);
   row.mean_control_s=mean(cellfun(@(i)i.elapsed_s,infos));
   row.worst_control_s=max(cellfun(@(i)i.elapsed_s,infos));
   row.rti_iterations=sum(cellfun(@(i)optional(i,'iterations'),infos));
   row.rejected_candidates=sum(cellfun(@(i)optional(i,'rejected'),infos));
   row.repaired_candidates=sum(cellfun(@(i)optional(i,'repaired_candidates'),infos));
   row.max_allocation_residual_Nm=0;row.max_vertex_residual_Nm=0;row.max_servo_vertex_residual_rad=0;
   row.future_height_max_error_m=0;row.future_qbar_max_error_Pa=0;
   row.future_V_max_error_mps=0;row.future_alpha_max_error_rad=0;
   measured=values(r.flight_servo_measured);fc=values(r.flight_conditions);
   certified=cellfun(@(i)isfield(i,'authority') && ~isempty(i.authority) && ...
    isfield(i.authority,'servo_vertex_residual'),infos);
   for k=1:numel(infos)
    if ~certified(k),continue;end
    a=infos{k}.authority;
    row.max_allocation_residual_Nm=max(row.max_allocation_residual_Nm,max(abs(a.allocation_residual),[],'all'));
    row.max_vertex_residual_Nm=max(row.max_vertex_residual_Nm,max(a.vertex_residual,[],'all'));
    row.max_servo_vertex_residual_rad=max(row.max_servo_vertex_residual_rad,max(a.servo_vertex_residual));
    future=e.control_times(k)+nodes(1:end-1);valid=future<=t(end);
    if ~any(valid),continue;end
    measured_future=interp1(t,[-measured(:,3) fc(:,1:3)],future(valid));
    difference=abs(a.air([1 4 2 5],valid)'-measured_future);
    row.future_height_max_error_m=max(row.future_height_max_error_m,max(difference(:,1)));
    row.future_qbar_max_error_Pa=max(row.future_qbar_max_error_Pa,max(difference(:,2)));
    row.future_V_max_error_mps=max(row.future_V_max_error_mps,max(difference(:,3)));
    row.future_alpha_max_error_rad=max(row.future_alpha_max_error_rad,max(difference(:,4)));
   end
   if size(x,2)>=10,row.gamma_peak_rad=max(abs(x(:,7:10)),[],1);else,row.gamma_peak_rad=zeros(1,4);end
   if size(u,2)>=7,row.nu_peak_radps=max(abs(u(:,4:7)),[],1);else,row.nu_peak_radps=zeros(1,4);end
  end
  rows{end+1}=row;names=union(names,fieldnames(row),'stable'); %#ok<AGROW>
 end
end
if isempty(rows),fprintf('FIVE_SQP_COMPARISON_NONE\n');return;end
for k=1:numel(rows)
 for j=1:numel(names)
  if ~isfield(rows{k},names{j}),rows{k}.(names{j})=0;end
 end
 rows{k}=orderfields(rows{k},names);
end
rows=cell2mat(rows);
table_=struct2table(rows);writetable(table_,fullfile(out,'five_sqp_comparison.csv'));
fid=fopen(fullfile(out,'five_sqp_comparison.json'),'w');
fprintf(fid,'%s',jsonencode(rows,'PrettyPrint',true));fclose(fid);
disp(table_(:,{'folder','arm','duration_s','attitude_rms_deg','attitude_peak_deg', ...
 'fallbacks','mean_control_s','worst_control_s','rti_iterations','rejected_candidates','repaired_candidates'}));
fprintf('FIVE_SQP_COMPARISON %s\n',out);
end
function data=values(ts)
data=squeeze(ts.Data);if size(data,1)~=numel(ts.Time),data=data';end
end
function value=optional(s,name)
value=0;if isfield(s,name),value=s.(name);end
end
