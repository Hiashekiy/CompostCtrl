function diagnose_torque_tracking
% Exact same-time algebraic decomposition, without changing either controller.
root=repo_root;base=fullfile(root,'simulink','results','online_authority');
report=loadjson(fullfile(base,'pitch_scale_repair','closedloop_report.json'));p=vehicle_params;v=p.vehicle;
result=struct;
for arm={'A','B'}
 name=arm{1};folder=fullfile(base,report.(name).folder);
 d=load(fullfile(folder,[name '_surrogate_flight_60s.mat']));r=d.r;t=r.commands.Time;
 u=series(r.commands);u=u(:,1:3);actual=series(r.tau_actual);delta=series(r.delta_actual);
 flight=series(r.flight_servo_measured);air=series(r.flight_conditions);state=series(r.states);
 period=.2;dt=median(diff(t));control=0:period:t(end);indices=round(control/dt)+1;
 cmd=zeros(numel(control),3);allocated=zeros(numel(control),3);old_vs_measured=zeros(numel(control),3);
 necessary_violation=zeros(numel(control),3);
 for k=1:numel(control)
  j=indices(k);[nat,B]=online_aero_moment(v,air(j,:)',state(j,4:6)',delta(j,:)');
  previous=flight(j,10:12)';
  lo=max(-v.delta_max(:),previous-v.delta_dot_max(:)*period);
  hi=min(v.delta_max(:),previous+v.delta_dot_max(:)*period);
  cmd(k,:)=bounded_control_allocation(B,u(j,:)'-nat,lo,hi)';
  allocated(k,:)=(nat+B*cmd(k,:)')';old_vs_measured(k,:)=previous'-delta(j,:);
  necessary_violation(k,:)=max(0,abs(cmd(k,:)-delta(j,:))-v.delta_dot_max(:)'*period);
 end
 stages=min(floor((t+1e-9)/period)+1,numel(control));
 allocation=zeros(size(u));hold_change=zeros(size(u));servo=zeros(size(u));formula=zeros(size(u));
 for j=1:numel(t)
  k=stages(j);[nat,B]=online_aero_moment(v,air(j,:)',state(j,4:6)',delta(j,:)');
  quasi=(nat+B*cmd(k,:)')';formula(j,:)=(nat+B*delta(j,:)')';
  allocation(j,:)=u(j,:)-allocated(k,:);
  hold_change(j,:)=allocated(k,:)-quasi;
  servo(j,:)=quasi-formula(j,:);
 end
 error=u-actual;closure=error-allocation-hold_change-servo;
 rms=@(z)sqrt(mean(z.^2,1));
 z=struct('total_rms_Nm',rms(error),'allocation_rms_Nm',rms(allocation), ...
 'held_workpoint_change_rms_Nm',rms(hold_change),'servo_tracking_rms_Nm',rms(servo), ...
 'formula_actual_max_Nm',max(abs(formula-actual),[],'all'), ...
 'decomposition_closure_max_Nm',max(abs(closure),[],'all'), ...
 'previous_command_actual_gap_peak_deg',rad2deg(max(abs(old_vs_measured),[],1)), ...
 'necessary_rate_bound_excess_peak_deg',rad2deg(max(necessary_violation,[],1)), ...
 'necessary_rate_bound_excess_ticks',sum(necessary_violation>1e-8,1));
 % Validate reconstructed commands against their real one-cycle-delayed feedback.
 z.command_reconstruction_max_rad=max(abs(cmd(1:end-1,:)-flight(indices(2:end),10:12)),[],'all');
 assert(z.command_reconstruction_max_rad<1e-8 && z.decomposition_closure_max_Nm<1e-8);
 result.(name)=z;
end
file=fullfile(base,'pitch_scale_repair','tracking_diagnosis.json');fid=fopen(file,'w');fprintf(fid,'%s',jsonencode(result));fclose(fid);disp(jsonencode(result));
end
function data=series(ts)
data=squeeze(ts.Data);if size(data,1)~=numel(ts.Time),data=data';end
end
function data=loadjson(file)
data=jsondecode(fileread(file));
end
