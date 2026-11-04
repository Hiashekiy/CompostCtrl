function [command,state,info]=direct_servo_mpc_step(s,x,state,force_failure)
% Direct-servo MPC: at most four SQP rounds per control period, exactly one HPIPM QP
% per round, one full-step proposal per round (no alpha search). The optimised control
% is the servo command itself, so the plant's own actuator dynamics are part of the
% prediction and no lower-level torque allocator is used.
if nargin<3 || isempty(state)
 state=struct('U',[],'last',[],'nu_previous',zeros(4,1),'delta_command',s.flight_state(10:12));
end
if ~isfield(state,'delta_command'),state.delta_command=s.flight_state(10:12);end
if ~isfield(state,'nu_previous'),state.nu_previous=zeros(4,1);end
if nargin<4,force_failure=false;end
persistent v
if isempty(v),p=vehicle_params;v=p.vehicle;end
started=tic;N=double(s.N);nx=double(s.nx_direct);nu=double(s.nu_direct);
max_sqp=4;if isfield(s,'max_sqp_iterations'),max_sqp=double(s.max_sqp_iterations);end
assert(ismember(max_sqp,[4 5]),'direct:sqp','max_sqp_iterations must be 4 or 5');
s.nu_previous=state.nu_previous;s.direct_previous_command=state.delta_command(:);
x=x(:);if numel(x)==nx-3,x=[x;s.flight_state(7:9)];end;x(nx-2:nx)=s.flight_state(7:9);
% Warm start: the command plan is shifted in absolute time, and the servo state is
% measured, never inherited from the previous prediction.
if isempty(state.U),U=zeros(nu,N);U(1:3,:)=repmat(state.delta_command(:),1,N);
else,U=online_shift_controls(state.U,s.dts,s.dt_ctrl);end
if nu==7,U(4:7,:)=min(max(U(4:7,:),-s.gdot),s.gdot);end
U(1:3,:)=min(max(U(1:3,:),-v.delta_max(:)),v.delta_max(:));
info=struct('fallback',false,'fallback_reason','','status',0,'objective',inf,'iterations',0, ...
 'qp_solved',0,'accepted_steps',0,'rejected_steps',0,'full_step_checks',0, ...
 'restoration_calls',0,'policy_rejections',0,'max_sqp_iterations',max_sqp, ...
 'objective_tol',0,'authority',[],'qp_delta_norm',[],'qp_solver_time_s',[], ...
 'qp_total_time_s',[],'rollout_time_s',[],'status_by_qp',[],'early_stop_reason','', ...
 'rejected',0,'repaired_candidates',0,'trust_scale',[],'trust_radius_rad',0);
trust_rad=.25*min(v.delta_dot_max(:))*s.dt_ctrl;
if isfield(s,'direct_trust_rad'),trust_rad=double(s.direct_trust_rad);end
info.trust_radius_rad=trust_rad;
info.iteration_authority={};
try
 [X,U,a,ok,reason]=direct_servo_propagate(s,x,U,s.flight_state,true);
 assert(ok,'direct:restoration','Unable to restore a feasible command plan: %s',reason);
 [valid,detail]=direct_servo_verify(s,x,U,a,s.flight_state);
 assert(valid,'direct:restoration','Restored plan failed certification: %s',detail);
 cost=objective(X,U);objective_tol=1e-9*max(1,abs(cost));info.objective_tol=objective_tol;
 accepted=struct('X',X,'U',U,'a',a,'cost',cost);
 info.iteration_authority{end+1}=a;
 for iteration=1:max_sqp
  info.iterations=iteration;
  if force_failure,error('direct:forcedFailure','Injected failure');end
  if isfield(state,'inject_failure_at') && any(state.inject_failure_at(:)==iteration)
   info.status=-1;info.status_by_qp(end+1)=-1;info.early_stop_reason='injected_qp_failure';break;
  end
  qp_timer=tic;
  work=assemble_direct_servo_qp(s,X,U,a);
  work.validate_readback=false;
  r=solve_hpipm_snapshot(work);
  info.qp_total_time_s(end+1)=toc(qp_timer);
  info.qp_solver_time_s(end+1)=r.time_s;info.status=r.status;info.status_by_qp(end+1)=r.status;
  if r.status~=0||~all(isfinite(r.U(:)))||~all(isfinite(r.X(:)))
   info.early_stop_reason='qp_failed';break;
  end
  info.qp_solved=info.qp_solved+1;
  step_norm=max(abs(r.U(1:3,:)),[],'all');info.qp_delta_norm(end+1)=max(abs(r.U),[],'all');
  if max(abs(r.U),[],'all')<1e-4,info.early_stop_reason='control_converged';break;end
  % Fixed trust region on the servo COMMAND. The linear servo block is the exponential
  % solution (exact only while the rate limit is inactive, i.e. |step| < rate*tau_s), so a
  % step far beyond the rate-limited span is inconsistent with the plant and is truncated
  % deterministically. This is a trust region, not an alpha candidate search.
  scale=1;
  if step_norm>trust_rad,scale=trust_rad/step_norm;end
  info.trust_scale(end+1)=scale;
  proposal=U+scale*r.U;
  timer=tic;
  [Xt,Ut,at,ok,reason]=direct_servo_propagate(s,x,proposal,s.flight_state,false);
  info.rollout_time_s(end+1)=toc(timer);info.full_step_checks=info.full_step_checks+1;
  if ~ok
   info.restoration_calls=info.restoration_calls+1;
   [Xt,Ut,at,ok,reason]=direct_servo_propagate(s,x,proposal,s.flight_state,true);
   if ok
    [ok,detail]=direct_servo_verify(s,x,Ut,at,s.flight_state);
    if ok,info.repaired_candidates=info.repaired_candidates+1;else,reason=detail;end
   end
  end
  if ~ok
   info.rejected_steps=info.rejected_steps+1;info.rejected=info.rejected_steps;
   info.early_stop_reason=['rejected: ' reason];break;
  end
  cost_try=objective(Xt,Ut);
  if cost_try>cost+objective_tol
   info.rejected_steps=info.rejected_steps+1;info.rejected=info.rejected_steps;
   info.early_stop_reason='objective_increase';break;
  end
  improvement=cost-cost_try;scale=max(1,abs(cost));
  X=Xt;U=Ut;a=at;cost=cost_try;
  accepted=struct('X',X,'U',U,'a',a,'cost',cost);
  info.accepted_steps=info.accepted_steps+1;info.iteration_authority{end+1}=a;
  if improvement<=1e-6*scale,info.early_stop_reason='objective_stagnation';break;end
 end
 if isempty(info.early_stop_reason),info.early_stop_reason='max_iterations';end
 X=accepted.X;U=accepted.U;a=accepted.a;
 [valid,detail]=direct_servo_verify(s,x,U,a,s.flight_state);
 assert(valid,'direct:first','First-command gate failed: %s',detail);
 command=U(:,1);state.U=U;state.last=command;state.delta_command=command(1:3);
 info.objective=accepted.cost;info.authority=a;
 if nu==7,state.nu_previous=command(4:7);end
catch err
 % Safe, physically valid command: hold the measured surface command (rate-bounded from
 % the measured angle) instead of replaying an old torque-level fallback.
 flight=s.flight_state;limit=v.delta_max(:);rate=v.delta_dot_max(:);
 held=min(max(flight(10:12),flight(7:9)-rate*s.dt_ctrl),flight(7:9)+rate*s.dt_ctrl);
 held=min(max(held,-limit),limit);
 command=held;
 if nu==7
  nu=min(max(zeros(4,1),max(-s.gdot,state.nu_previous-s.gddot*s.dt_ctrl)),min(s.gdot,state.nu_previous+s.gddot*s.dt_ctrl));
  command=[held;nu];
  state.nu_previous=nu;
 end
 state.U=[];state.delta_command=held;info.fallback=true;info.failure=err.message;
 info.fallback_reason=err.message;info.status=-1;info.authority=[];
end
info.cycle_time_s=toc(started);info.elapsed_s=info.cycle_time_s;
 function value=objective(trajectory,control)
  weight=s.dts(:)/sum(s.dts);value=0;Wc=s.direct_weights;natt_local=nx-3;
  for j=1:N+1
   if natt_local==11,[e,~]=native_E_B(s.q_ref(:,j),trajectory(1:11,j));
   else,[e,~]=native_E_A(s.q_ref(:,j),trajectory(1:7,j));end
   if j<=N
    dw=trajectory(5:7,j)-s.omega_ref(:,j);
    value=value+weight(j)*(s.weights.w_att/s.weights.e_scale^2*(e'*e)+ ...
     s.weights.w_rate/s.weights.omega_scale^2*(dw'*dw));
   else
    value=value+s.weights.q_terminal/s.weights.e_scale^2*(e'*e);
   end
  end
  for k=1:N
   value=value+weight(k)*sum(Wc.cmd*(control(1:3,k)'./v.delta_max(:)').^2);
   if k==1,previous=s.direct_previous_command;else,previous=control(1:3,k-1);end
   value=value+weight(k)*sum(Wc.dcmd*((control(1:3,k)-previous)'./v.delta_max(:)').^2);
  end
 end
end
