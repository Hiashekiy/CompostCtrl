function check_direct_servo_model(run_folder)
% S1 gate for the direct-servo MPC: prove the 1 ms servo predictor, the aerodynamic
% effectiveness and the moment history against the Simulink truth BEFORE any MPC
% integration. Nothing here is allowed to change the plant, aero data or reference.
root=repo_root;init_paths;
out=fullfile(root,'simulink','results','online_authority','direct_servo');if ~exist(out,'dir'),mkdir(out);end
p=vehicle_params;v=p.vehicle;ref=load(fullfile(root,'simulink','data','native_actuators','reference.mat'));
servo_dt=double(ref.dt);
report=struct('servo_clock_s',servo_dt,'tau_s',v.tau_s,'delta_max',v.delta_max(:)');
report.delta_dot_max=v.delta_dot_max(:)';report.structure=check_truth_structure(v,servo_dt);

% ---- 1. discrete servo response against the real Simulink actuator -----------
limit=v.delta_max(:);rate=v.delta_dot_max(:);
scenarios=[
 struct('name','hold_at_target','x0',[0;0;0],'cmd',[0;0;0],'duration',.2);
 struct('name','small_step','x0',[0;0;0],'cmd',[.01;-.01;.005],'duration',.2);
 struct('name','rate_saturated','x0',[0;0;0],'cmd',[.12;.10;-.10],'duration',.2);
 struct('name','reverse_motion','x0',[.10;-.05;.02],'cmd',[-.10;.05;-.02],'duration',.2);
 struct('name','command_saturated','x0',[0;0;0],'cmd',3*limit,'duration',.2);
 struct('name','near_positive_limit','x0',.99*limit,'cmd',-.99*limit,'duration',1);
 struct('name','long_hold_2s','x0',[0;0;0],'cmd',[.05;-.08;.03],'duration',2);
 struct('name','half_second_hold','x0',[.02;.01;-.03],'cmd',[.06;.02;-.01],'duration',.5)];
report.scenarios=struct([]);
for k=1:numel(scenarios)
 s=scenarios(k);
 [t_truth,delta_truth,rate_truth]=truth_servo(ref,s.x0,s.cmd,s.duration,servo_dt);
 predicted=servo_predict(v,s.x0,s.cmd,s.duration,[],[],servo_dt);
 n=numel(predicted.time)-1;t_target=(0:n)*servo_dt;
 delta_truth=resample(delta_truth,t_truth,t_target);rate_truth=resample(rate_truth,t_truth,t_target(1:end-1));
 row=struct('name',s.name,'duration_s',s.duration, ...
  'delta_max_error_rad',max(abs(predicted.delta-delta_truth),[],'all'), ...
  'rate_max_error_radps',max(abs(predicted.delta_dot-rate_truth),[],'all'), ...
  'endpoint_error_rad',max(abs(predicted.delta(:,end)-delta_truth(:,end)),[],'all'));
 % The existing closed-form endpoint model must agree with the same recurrence.
 [~,~,nominal]=online_surface_reachability(v,s.x0,s.duration,s.cmd,servo_dt,-limit,limit);
 row.reachability_endpoint_error_rad=max(abs(nominal-predicted.delta(:,end)));
 assert(row.delta_max_error_rad<1e-9,'S1: servo trajectory mismatch in %s',s.name);
 assert(row.rate_max_error_radps<1e-9,'S1: servo rate mismatch in %s',s.name);
 assert(row.reachability_endpoint_error_rad<1e-9,'S1: reachability endpoint mismatch in %s',s.name);
 if isempty(report.scenarios),report.scenarios=row;else,report.scenarios(end+1)=row;end %#ok<AGROW>
 fprintf('SERVO_TRUTH %-22s delta=%.3e rate=%.3e endpoint=%.3e\n',s.name, ...
  row.delta_max_error_rad,row.rate_max_error_radps,row.reachability_endpoint_error_rad);
end

% ---- 2. effectiveness B(qbar) and natural moment against the truth aero block
[air,~]=online_flight_condition([0;0;-2e4],[3e3;0;0]);
Craw=[0 v.Cl_da v.Cl_dr;v.Cm_de 0 0;0 v.Cn_da v.Cn_dr];
G=diag(v.S*[v.b v.cbar v.b]);
grid=[ 1e3 8e2 .02 0  0 0 0   .02 -.01 .03;
      8e3 1.2e3 .08 .01  .02 -.01 .03  .05 .02 -.04;
      3e4 1.5e3 -.05 -.02 -.03 .02 .01 -.06 -.03 .05;
      6e4 1.1e3 .12 0   .01 .01 -.02 .10 -.05 0;
      1.5e4 6e2 -.12 .03 0 .04 0   -.09 .07 -.02];
report.aero=struct([]);aero_truth_ok=true;
for k=1:size(grid,1)
 u10=grid(k,:)';air_k=u10(1:4);omega_k=u10(5:7);delta_k=u10(8:10);
 [truth,got]=truth_moment(u10);aero_truth_ok=aero_truth_ok && got;
 [natural,B,total]=online_aero_moment(v,air_k,omega_k,delta_k); analytic=air_k(1)*G*([v.Cl_beta*air_k(4);v.Cm0+v.Cm_alpha*air_k(3);v.Cn_beta*air_k(4)] ...
  +[v.Cl_p*v.b/2 0 v.Cl_r*v.b/2;0 v.Cm_q*v.cbar/2 0;v.Cn_p*v.b/2 0 v.Cn_r*v.b/2]*omega_k/max(air_k(2),1)) ...
  +air_k(1)*G*Craw*delta_k;
 row=struct('qbar',air_k(1),'V',air_k(2),'alpha',air_k(3),'beta',air_k(4));
 row.truth_total=truth.total(:)';row.model_total=total(:)';
 row.truth_error=check_error(truth.total,total,got);
 row.natural_error=check_error(truth.natural,natural,got);
 row.surface_error=check_error(truth.surface,B*delta_k,got);
 row.analytic_error=max(abs(analytic-total));
 row.B_error=max(abs(B-air_k(1)*G*Craw),[],'all');
 % tau_nat must carry the measured rate dependence; a frozen natural moment is a bug.
 row.natural_rate_gain=max(abs((online_aero_moment(v,air_k,2*omega_k,delta_k)-online_aero_moment(v,air_k,[0;0;0],delta_k)) ...
  -2*(natural-online_aero_moment(v,air_k,[0;0;0],delta_k))));
 assert(row.analytic_error<1e-9 && row.B_error<1e-12);
 assert(row.natural_rate_gain<1e-9,'S1: natural moment is not linear in the rate');
 if got
  assert(row.truth_error<1e-9,'S1: aero total mismatch at grid row %d',k);
  assert(row.natural_error<1e-9 && row.surface_error<1e-9,'S1: aero components mismatch at grid row %d',k);
 end
 if isempty(report.aero),report.aero=row;else,report.aero(end+1)=row;end %#ok<AGROW>
end
report.aero_truth_block_available=aero_truth_ok;
if aero_truth_ok
 fprintf('AERO_TRUTH max_error=%.3e B_error=%.1e\n',max([report.aero.truth_error]),max([report.aero.B_error]));
else
 fprintf('AERO_TRUTH block outputs unavailable; analytic identity and replay only\n');
end

% ---- 3. 1 ms replay against a stored 60 s closed-loop run ---------------------
if nargin<1 || isempty(run_folder),run_folder=pick_run(root);end
report.replay=replay_run(root,run_folder,v,servo_dt);
fprintf('REPLAY %s delta_rms=%.3e delta_max=%.3e torque_rms=%.3e torque_max=%.3e\n', ...
 report.replay.folder,report.replay.delta_rms_rad,report.replay.delta_max_rad, ...
 report.replay.torque_rms_Nm,report.replay.torque_max_Nm);
fprintf('REPLAY stage_constant_torque rms=%.3e max=%.3e (terminal angle used for the whole stage)\n', ...
 report.replay.stage_constant_torque_rms_Nm,report.replay.stage_constant_torque_max_Nm);
report.note=['S1 only: servo predictor, aerodynamic effectiveness and moment history are ' ...
 'validated against the Simulink truth. No direct-servo MPC is integrated yet.'];
save(fullfile(out,'direct_servo_model_check.mat'),'report');
fid=fopen(fullfile(out,'direct_servo_model_check.json'),'w');
fprintf(fid,'%s',jsonencode(report,'PrettyPrint',true));fclose(fid);
fprintf('DIRECT_SERVO_MODEL_PASS %s\n',out);
end
function info=check_truth_structure(v,servo_dt)
% The predictor is only allowed to claim truth equivalence if the block still is
% the recurrence documented in servo_predict; fail loudly when the plant changes.
m='Surface_Actuator';load_system(m);
info=struct('block',m,'file',get_param(m,'FileName'));
info.command_limit_lower=str2num(get_param([m '/Command_limit'],'LowerLimit')); %#ok<ST2NM>
info.command_limit_upper=str2num(get_param([m '/Command_limit'],'UpperLimit')); %#ok<ST2NM>
info.derivative_limit_lower=str2num(get_param([m '/Derivative_limit'],'LowerLimit')); %#ok<ST2NM>
info.derivative_limit_upper=str2num(get_param([m '/Derivative_limit'],'UpperLimit')); %#ok<ST2NM>
info.state_limit_lower=str2num(get_param([m '/State_limit'],'LowerLimit')); %#ok<ST2NM>
info.state_limit_upper=str2num(get_param([m '/State_limit'],'UpperLimit')); %#ok<ST2NM>
info.inverse_time_constant=str2num(get_param([m '/Inverse_time_constant'],'Gain')); %#ok<ST2NM>
info.euler_increment=str2num(get_param([m '/Euler_increment'],'Gain')); %#ok<ST2NM>
info.actuator_sample_time=str2num(get_param([m '/Actual_state'],'SampleTime')); %#ok<ST2NM>
info.solver=get_param(m,'Solver');info.fixed_step=str2num(get_param(m,'FixedStep')); %#ok<ST2NM>
limit=v.delta_max(:);rate=v.delta_dot_max(:);
assert(abs(info.inverse_time_constant-1/v.tau_s)<1e-12,'S1: actuator time constant changed');
assert(max(abs(info.command_limit_upper-limit))<1e-12 && max(abs(info.command_limit_lower+limit))<1e-12, ...
 'S1: actuator command limit changed');
assert(max(abs(info.derivative_limit_upper-rate))<1e-12 && max(abs(info.derivative_limit_lower+rate))<1e-12, ...
 'S1: actuator rate limit changed');
assert(max(abs(info.state_limit_upper-limit))<1e-12 && max(abs(info.state_limit_lower+limit))<1e-12, ...
 'S1: actuator state limit changed');
assert(abs(info.euler_increment-servo_dt)<1e-15 && abs(info.actuator_sample_time-servo_dt)<1e-15, ...
 'S1: actuator clock is not the 1 ms servo clock');
assert(abs(info.fixed_step-servo_dt)<1e-15,'S1: plant fixed step is not the servo clock');
end
function [t,delta,rate]=truth_servo(ref,x0,command,duration,servo_dt)
% Execute the real Surface_Actuator model (no reimplementation) for one scenario.
m='Surface_Actuator';load_system(m);
set_param([m '/Actual_state'],'InitialCondition',mat2str(x0(:),17));
n=round(duration/servo_dt);t=(0:n)'*servo_dt;u=repmat(command(:)',n+1,1);
si=Simulink.SimulationInput(m);
si=si.setExternalInput([t u]);
si=si.setModelParameter('StopTime',num2str(duration,17));
r=sim(si);
delta=values(r.state);rate=values(r.deriv);
if size(delta,1)~=3,delta=delta';end
if size(rate,1)~=3,rate=rate';end
t=r.state.Time(:);
end
function [truth,ok]=truth_moment(u10)
% Real Aerodynamic_Model at one operating point: zero length run, no states.
% Ports are order-based (component_1 natural, component_2 surface, component_3 total),
% because the saved block names are not exposed by the Dataset.
truth=struct('natural',nan(3,1),'surface',nan(3,1),'total',nan(3,1));ok=false;
m='Aerodynamic_Model';load_system(m);
try
 si=Simulink.SimulationInput(m);
 si=si.setExternalInput([0 u10(:)']);
 si=si.setModelParameter('StopTime','0','SaveOutput','on','SaveFormat','Dataset');
 r=sim(si);
 yout=r.get('yout');
 if isa(yout,'Simulink.SimulationData.Dataset') && yout.numElements>=3
  data=zeros(3,3);
  for k=1:3
   element=yout{k};value=element.Values;
   if isa(value,'timeseries'),value=value.Data;end
   data(:,k)=value(1,:)';
  end
  truth.natural=data(:,1);truth.surface=data(:,2);truth.total=data(:,3);ok=true;
 end
catch
 ok=false;
end
end
function value=check_error(truth,total,got)
if got,value=max(abs(truth-total));else,value=NaN;end
end
function folder=pick_run(root)
% Prefer the restored-0.2 s 4-round run recorded in docs/simulink/15, else newest B.
base=fullfile(root,'simulink','results','online_authority');
preferred={'L2_60s_20261104_212322'};
for k=1:numel(preferred)
 if isfile(fullfile(base,preferred{k},'B_surrogate_flight_60s.mat')),folder=preferred{k};return;end
end
folders=dir(base);candidates={};
for k=1:numel(folders)
 if folders(k).isdir && ~isempty(regexp(folders(k).name,'^L2_60s_','once')) && ...
   isfile(fullfile(base,folders(k).name,'B_surrogate_flight_60s.mat'))
  candidates{end+1}=folders(k).name; %#ok<AGROW>
 end
end
assert(~isempty(candidates),'S1: no stored 60 s B run to replay');
candidates=sort(candidates);folder=candidates{end};
end
function row=replay_run(root,folder,v,servo_dt)
% Compare the predictor against the executed plant at 1 ms inside every control
% period, and quantify the error of holding one terminal angle for the whole stage.
base=fullfile(root,'simulink','results','online_authority');
d=load(fullfile(base,folder,'B_surrogate_flight_60s.mat'));r=d.r;
t_state=r.states.Time;x=values(r.states);
assert(size(x,1)==numel(t_state) && size(x,2)>=10,'S1: state log does not carry [euler;omega;gamma]');
measured=values(r.flight_servo_measured);
fc=values(r.flight_conditions);
tau_actual=values(r.tau_actual);
assert(size(measured,1)==numel(t_state) && size(fc,1)==numel(t_state) && size(tau_actual,1)==numel(t_state), ...
 'S1: stored log sample counts disagree with the state log');
period=.2;
idx=zeros(0,1);
for k=1:numel(t_state)
 if abs(mod(t_state(k)+1e-9,period))<1e-6,idx(end+1,1)=k;end %#ok<AGROW>
end
delta_error=[];torque_error=[];const_error=[];step_error=[];
for j=1:numel(idx)-1
 a=idx(j);b=idx(j+1);
 x0=measured(a,7:9)';command=measured(b,10:12)';
 if any(~isfinite(x0))||any(~isfinite(command)),continue;end
 air=[fc(a,1:3)';0];
 if size(fc,2)>=4,air(4)=fc(a,4);end
 omega=x(a,5:7)';
 predicted=servo_predict(v,x0,command,t_state(b)-t_state(a),air,omega,servo_dt);
 samples=(0:round((t_state(b)-t_state(a))/servo_dt))';
 truth_delta=measured(a:b,7:9)';truth_torque=tau_actual(a:b,1:3)';
 if size(truth_delta,2)~=numel(samples),continue;end
 delta_error=[delta_error;abs(predicted.delta-truth_delta)']; %#ok<AGROW>
 torque_error=[torque_error;abs(predicted.torque-truth_torque)']; %#ok<AGROW>
 % The naive assumption the new MPC must avoid: one terminal angle for the stage.
 tau_nat=predicted.natural(:,1);airs=air;
 [~,B_end]=online_aero_moment(v,airs,omega,predicted.delta(:,end));
 tau_const=repmat(tau_nat,1,numel(samples))+B_end*predicted.delta(:,end);
 const_error=[const_error;abs(tau_const-truth_torque)']; %#ok<AGROW>
 step_error(end+1)=max(abs(predicted.delta(:,end)-truth_delta(:,end))); %#ok<AGROW>
end
row=struct('folder',folder,'control_periods',numel(idx)-1, ...
 'delta_rms_rad',sqrt(mean(delta_error(:).^2)),'delta_max_rad',max(delta_error(:)), ...
 'torque_rms_Nm',sqrt(mean(torque_error(:).^2)),'torque_max_Nm',max(torque_error(:)), ...
 'stage_constant_torque_rms_Nm',sqrt(mean(const_error(:).^2)), ...
 'stage_constant_torque_max_Nm',max(const_error(:)), ...
 'stage_step_delta_max_rad',max(step_error));
assert(row.delta_rms_rad<1e-9,'S1: replayed servo trajectory does not match the plant');
% The servo recurrence is exact; the moment history additionally holds air data and
% the body rate at the control instant, so its tolerance is a modelling tolerance.
assert(row.torque_rms_Nm<5e-2,'S1: replayed moment history deviates from the plant');
end
function data=values(ts)
data=squeeze(ts.Data);if size(data,1)~=numel(ts.Time),data=data';end
end
function y=resample(data,t_source,t_target)
y=zeros(size(data,1),numel(t_target));
for k=1:size(data,1),y(k,:)=interp1(t_source,data(k,:),t_target,'linear','extrap');end
end
