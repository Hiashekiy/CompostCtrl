function native_torque_feedback_allocation_sfun(block)
% 20 ms torque-tracking inner loop (method F).  It replaces the 0.2 s open-loop surface
% allocation ONLY in the experiment models built with torque_inner_mode=
% 'feedback_predictive'; the production 0.2 s allocator is untouched.
%
% Inputs
%   1  tau_cmd(3)        upper MPC torque command, zero-order held over its 0.2 s period
%   2  delta_actual(3)   MEASURED surface position (Aerodynamic_Surface/1), never the
%                        previous command
%   3  air(4)            [qbar;V;alpha;beta] from the same measured flight condition the
%                        controller uses
%   4  omega(3)          measured body rate
% Output
%   1  delta_cmd(3)      servo command sent to the real 1 ms actuator
%
% The plant servo keeps its own delta_max / delta_dot_max; the inner loop only limits
% its own command rate to delta_dot_max*0.02 and logs when that limit binds, so the
% "command slew" limit is never reported as a physical actuator limit.
%
% Every tick is logged; the diagnostics MAT and a JSON summary are written into
% native_results_dir on Terminate.
block.NumDialogPrms=0;
period=.02;
try
 mw=get_param(bdroot(block.BlockHandle),'ModelWorkspace');
 if hasVariable(mw,'native_inner_period'),period=getVariable(mw,'native_inner_period');end
catch
end
assert(isscalar(period) && period>0 && period<=.05,'online:innerPeriod', ...
 'inner period must be a small positive number (got %g)',period);
block.NumInputPorts=4;block.NumOutputPorts=1;block.SetPreCompPortInfoToDefaults;
block.InputPort(1).Dimensions=3;block.InputPort(2).Dimensions=3;
block.InputPort(3).Dimensions=4;block.InputPort(4).Dimensions=3;
for k=1:4,block.InputPort(k).DirectFeedthrough=true;end
block.OutputPort(1).Dimensions=3;
block.SampleTimes=[period 0];block.SimStateCompliance='HasNoSimState';
block.RegBlockMethod('Start',@(b)step('start',b));
block.RegBlockMethod('Outputs',@(b)step('output',b));
block.RegBlockMethod('Terminate',@(b)step('terminate',b));
end
function step(action,b)
persistent state
if isempty(state),state=containers.Map('KeyType','double','ValueType','any');end
key=double(b.BlockHandle);
if strcmp(action,'start')
 p=vehicle_params;c.v=p.vehicle;
 c.period=b.SampleTimes(1);c.time=-inf;
 c.u=zeros(3,1);c.ticks=0;c.fallbacks=0;c.slew=0;c.max_iterations=0;
 c.model=get_param(bdroot(b.BlockHandle),'Name');
 c.dir='';mw=get_param(bdroot(b.BlockHandle),'ModelWorkspace');
 if hasVariable(mw,'native_results_dir'),c.dir=getVariable(mw,'native_results_dir');end
 c.options=struct('dt',c.period,'lambda',1e-3,'max_iterations',4,'backtracks',4, ...
  'weight',ones(3,1),'servo_dt',.001);
 c.times=zeros(0,1);c.tau_cmd=zeros(0,3);c.delta_actual=zeros(0,3);
 c.hat_tau=zeros(0,3);c.delta_cmd=zeros(0,3);c.predicted=zeros(0,3);
 c.achievable=zeros(0,3);c.residual=zeros(0,3);c.iterations=zeros(0,1);
 c.cost_initial=zeros(0,1);c.cost_final=zeros(0,1);
 c.fallback_flag=false(0,1);c.slew_flag=false(0,1);c.compute_s=zeros(0,1);
 state(key)=c;return;
end
if strcmp(action,'terminate')
 if ~isKey(state,key),return;end
 c=state(key);remove(state,key);
 summary=struct('model',c.model,'inner_period_s',c.period,'ticks',c.ticks, ...
  'fallbacks',c.fallbacks,'slew_saturated_ticks',c.slew,'max_iterations_used',c.max_iterations, ...
  'compute_ms_mean',1000*mean_or_nan(c.compute_s), ...
  'compute_ms_p95',1000*p95_(c.compute_s),'compute_ms_max',1000*max_or_nan(c.compute_s), ...
  'over_20ms',sum(c.compute_s>c.period));
 folder=c.dir;
 if isempty(folder)
  root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
  folder=fullfile(root,'simulink','results','online_authority','torque_margin_feedback');
 end
 try
  if ~exist(folder,'dir'),mkdir(folder);end
  diagnostics=struct('time',c.times,'tau_cmd',c.tau_cmd,'delta_actual',c.delta_actual, ...
   'hat_tau',c.hat_tau,'delta_cmd',c.delta_cmd,'predicted_endpoint',c.predicted, ...
   'achievable_torque',c.achievable,'residual_Nm',c.residual,'iterations',c.iterations, ...
   'cost_initial',c.cost_initial,'cost_final',c.cost_final, ...
   'fallback',c.fallback_flag,'slew_saturated',c.slew_flag,'compute_s',c.compute_s);
  save(fullfile(folder,[c.model '_torque_feedback.mat']),'diagnostics','summary');
  fid=fopen(fullfile(folder,[c.model '_torque_feedback.json']),'w');
  if fid>0,fprintf(fid,'%s',jsonencode(summary,'PrettyPrint',true));fclose(fid);end
  fprintf('INNER_TORQUE_FEEDBACK ticks=%d fallbacks=%d slew=%d mean=%.3fms max=%.3fms\n', ...
   c.ticks,c.fallbacks,c.slew,summary.compute_ms_mean,summary.compute_ms_max);
 catch err
  warning('INNER_TORQUE_FEEDBACK_LOG_SKIPPED %s',err.message);
 end
 return;
end
c=state(key);
if b.CurrentTime>c.time+1e-9
 tau_cmd=b.InputPort(1).Data(:);delta_actual=b.InputPort(2).Data(:);
 air=b.InputPort(3).Data(:);omega=b.InputPort(4).Data(:);
 started=tic;
 if all(isfinite([tau_cmd;delta_actual;air;omega]))
  try
   [u,info]=online_torque_feedback_step(c.v,tau_cmd,delta_actual,air,omega,c.u,c.options);
  catch err
   u=c.u;info=struct('predicted_endpoint',nan(3,1),'achievable_torque',nan(3,1), ...
    'residual_Nm',nan(3,1),'iterations',0,'cost_initial',NaN,'cost_final',NaN, ...
    'fallback',true,'slew_saturated',false,'natural',nan(3,1),'B',nan(3));
   warning('INNER_TORQUE_FEEDBACK_FAILED %s',err.message);
  end
 else
  u=c.u;info=struct('predicted_endpoint',nan(3,1),'achievable_torque',nan(3,1), ...
   'residual_Nm',nan(3,1),'iterations',0,'cost_initial',NaN,'cost_final',NaN, ...
   'fallback',true,'slew_saturated',false,'natural',nan(3,1),'B',nan(3));
 end
 elapsed=toc(started);
 if ~all(isfinite(u))
  u=c.u;info.fallback=true;
 end
 c.u=u(:);c.time=b.CurrentTime;c.ticks=c.ticks+1;
 c.fallbacks=c.fallbacks+double(info.fallback);c.slew=c.slew+double(info.slew_saturated);
 c.max_iterations=max(c.max_iterations,info.iterations);
 c.times(end+1,1)=b.CurrentTime;
 c.tau_cmd(end+1,:)=tau_cmd.';c.delta_actual(end+1,:)=delta_actual.';
 c.hat_tau(end+1,:)=(info.natural(:)+info.B*delta_actual).';
 c.delta_cmd(end+1,:)=c.u.';
 c.predicted(end+1,:)=info.predicted_endpoint(:).';
 c.achievable(end+1,:)=info.achievable_torque(:).';
 c.residual(end+1,:)=info.residual_Nm(:).';
 c.iterations(end+1,1)=info.iterations;
 c.cost_initial(end+1,1)=info.cost_initial;c.cost_final(end+1,1)=info.cost_final;
 c.fallback_flag(end+1,1)=logical(info.fallback);
 c.slew_flag(end+1,1)=logical(info.slew_saturated);
 c.compute_s(end+1,1)=elapsed;
 state(key)=c;
end
b.OutputPort(1).Data=c.u;
end
function value=mean_or_nan(data)
if isempty(data),value=NaN;else,value=mean(data);end
end
function value=max_or_nan(data)
if isempty(data),value=NaN;else,value=max(data);end
end
function value=p95_(data)
if isempty(data),value=NaN;return;end
data=sort(data(:));value=data(max(1,min(numel(data),ceil(.95*numel(data)))));
end
