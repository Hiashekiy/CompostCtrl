function [command,state,info]=native_online_rti_step(s,x,state,force_failure)
% Online authority branch: bounded sequential SQP over one control period.
% Every round solves exactly ONE HPIPM QP at the current accepted nominal point,
% makes exactly ONE full-step proposal and re-certifies it on its own trajectory.
% There is no alpha line search and no reuse of a torque box certified elsewhere.
if nargin<4,force_failure=false;end
started=tic;N=double(s.N);nx=double(s.nx);s.nu_previous=state.nu_previous;
max_sqp=online_max_sqp_iterations(s);
if nx==11,E=@native_E_B;else,E=@native_E_A;end
% Warm start: s.dts is non-uniform, so the plan is shifted in TIME, not by index.
if isempty(state.U),U=zeros(double(s.nu),N);
else,U=online_shift_controls(state.U,s.dts,s.dt_ctrl);end
info=struct('fallback',false,'fallback_reason','','status',0,'objective',inf, ...
 'iterations',0,'sqp_attempted',0,'qp_solved',0,'accepted_steps',0,'rejected_steps',0, ...
 'full_step_checks',0,'restoration_calls',0,'certificate_count',0,'rejected',0, ...
 'repaired_candidates',0,'status_by_qp',[],'qp_solver_time_s',[],'qp_total_time_s',[], ...
 'rollout_time_s',[],'qp_delta_norm',[],'early_stop_reason','','max_sqp_iterations',max_sqp, ...
 'objective_tol',0,'authority',[]);
info.control_period_s=s.dt_ctrl;info.prediction_dts=s.dts;
info.iteration_authority={};
try
 [X,U,a,ok]=try_rollout(U,true);
 assert(ok,'online:restoration','Unable to restore feasible nominal control');
 assert(verify_authority_candidate(s,x,X,U,a,s.flight_state),'online:restoration', ...
  'Restored nominal control failed independent certification');
 obj=objective(X);objective_tol=1e-9*max(1,abs(obj));info.objective_tol=objective_tol;
 accepted=struct('X',X,'U',U,'a',a,'objective',obj);
 info.iteration_authority{end+1}=a;
 for iteration=1:max_sqp
  info.iterations=iteration;info.sqp_attempted=iteration;
  if force_failure,error('native:forcedFailure','Injected failure');end
  if isfield(state,'inject_failure_at') && any(state.inject_failure_at(:)==iteration)
   info.status=-1;info.status_by_qp(end+1)=-1;info.early_stop_reason='injected_qp_failure';break;
  end
  % The QP is linearised AND bounded at the currently accepted nominal point.
  work=s;work.tau_min=a.tau_min;work.tau_max=a.tau_max;
  qp_started=tic;qp=assemble_native_qp(work,X,U);qp.validate_readback=false;
  r=solve_hpipm_snapshot(qp);info.qp_total_time_s(end+1)=toc(qp_started);
  info.qp_solver_time_s(end+1)=r.time_s;info.status=r.status;info.status_by_qp(end+1)=r.status;
  if r.status~=0 || ~all(isfinite(r.U(:))) || ~all(isfinite(r.X(:)))
   info.early_stop_reason='qp_failed';break;
  end
  info.qp_solved=info.qp_solved+1;
  step=max(abs(r.U),[],'all');info.qp_delta_norm(end+1)=step;
  if step<1e-4,info.early_stop_reason='control_converged';break;end
  proposal=U+r.U;                          % r.U is the increment on this sequence
  [Xt,Ut,at,ok,message]=try_rollout(proposal,false);
  info.full_step_checks=info.full_step_checks+1;
  if ~ok
   % Causal restoration from the SAME unmodified full-step input, then re-certify
   % the restored trajectory. The proposal is never clipped against an old box.
   info.restoration_calls=info.restoration_calls+1;
   [Xt,Ut,at,ok,message]=try_rollout(proposal,true);
   if ok
    ok=verify_authority_candidate(s,x,Xt,Ut,at,s.flight_state);
    if ok
     info.repaired_candidates=info.repaired_candidates+1;
    else
     message='restored proposal failed certification';
    end
   end
  end
  if ~ok
   if isempty(message),message='full-step proposal failed certification';end
   info.rejected_steps=info.rejected_steps+1;info.rejected=info.rejected_steps;
   info.early_stop_reason=['rejected: ' message];break;
  end
  Jt=objective(Xt);
  if Jt>obj+objective_tol
   info.rejected_steps=info.rejected_steps+1;info.rejected=info.rejected_steps;
   info.early_stop_reason='objective_increase';break;
  end
  improvement=obj-Jt;scale=max(1,abs(obj));
  X=Xt;U=Ut;a=at;obj=Jt;
  accepted=struct('X',X,'U',U,'a',a,'objective',obj);
  info.accepted_steps=info.accepted_steps+1;info.iteration_authority{end+1}=a;
  if improvement<=1e-6*scale,info.early_stop_reason='objective_stagnation';break;end
 end
 if isempty(info.early_stop_reason),info.early_stop_reason='max_iterations';end
 X=accepted.X;U=accepted.U;a=accepted.a;
 [valid,detail]=online_verify_first_command(s,U,a,s.flight_state);
 assert(valid,'online:first','First-command check failed: %s',detail);
 command=U(:,1);state.U=U;state.last=command;
 info.objective=accepted.objective;info.authority=a;
catch err
 % A failed future prediction cannot reuse an old command at a new air state.
 p=vehicle_params;v=p.vehicle;flight=s.flight_state;
 air=online_flight_condition(flight(1:3),flight(4:6));
 [natural,B]=online_aero_moment(v,air,x(5:7),flight(7:9));
 lo=max(-v.delta_max(:),flight(10:12)-v.delta_dot_max(:)*s.dt_ctrl);
 hi=min(v.delta_max(:),flight(10:12)+v.delta_dot_max(:)*s.dt_ctrl);
 held=min(max(flight(7:9),lo),hi);tau=natural+B*held;
 command=tau;
 if nx==11
  nu=min(max(zeros(4,1),max(-s.gdot,state.nu_previous-s.gddot*s.dt_ctrl)),min(s.gdot,state.nu_previous+s.gddot*s.dt_ctrl));
  command=[tau;nu];
 end
 state.U=[];info.fallback=true;info.failure=err.message;info.fallback_reason=err.message;
 info.status=-1;info.authority=[];
end
if nx==11,state.nu_previous=command(4:7);end
info.cycle_time_s=toc(started);info.elapsed_s=info.cycle_time_s;
 function [Xt,Ut,at,ok,message]=try_rollout(value,restore)
  Xt=[];Ut=[];at=[];ok=false;message='';timer=tic;
  try
   [Xt,Ut,at,ok]=online_authority_rollout(s,x,value,s.flight_state,restore);
  catch err
   ok=false;message=err.message;
  end
  info.rollout_time_s(end+1)=toc(timer);info.certificate_count=info.certificate_count+1;
 end
 function cost=objective(trajectory)
  W=s.weights;weight=s.dts(:)/sum(s.dts);cost=0;
  for j=1:N+1
   [e,~]=E(s.q_ref(:,j),trajectory(:,j));
   if j<=N
    dw=trajectory(5:7,j)-s.omega_ref(:,j);
    cost=cost+weight(j)*(W.w_att/W.e_scale^2*(e'*e)+W.w_rate/W.omega_scale^2*(dw'*dw));
   else,cost=cost+W.q_terminal/W.e_scale^2*(e'*e);end
  end
 end
end
function value=online_max_sqp_iterations(s)
% Default four rounds; explicit five-round experiments remain supported.
value=4;if isfield(s,'max_sqp_iterations'),value=double(s.max_sqp_iterations);end
assert(isscalar(value) && isfinite(value) && ismember(value,[4 5]),'online:sqp', ...
 'max_sqp_iterations must be 4 or 5 (got %g)',value);
end
