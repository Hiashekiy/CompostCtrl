function [command,state,info]=direct_servo_mpc_step(s,x,state,force_failure)
% Direct-servo MPC: at most four SQP rounds per control period, exactly one HPIPM QP per
% round. The optimised control is the servo command itself, so the plant's actuator
% dynamics are part of the prediction and no lower-level torque allocator exists.
%
% s.acceptance_mode selects ONLY the acceptance rule (everything else is identical):
%   hard_stop       : reject and stop the cycle when a feasible proposal raises J
%   no_descent_gate : accept any feasible, finite proposal; J is recorded, not gated
%   continue_best   : keep iterating through J increases, emit the best feasible candidate
% s.model_mode: 'fixed' (default) or 'legacy' (bcd96fd cost and linearisation).
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
mode='hard_stop';if isfield(s,'acceptance_mode'),mode=s.acceptance_mode;end
assert(ismember(mode,{'hard_stop','no_descent_gate','continue_best'}),'direct:acceptance', ...
 'unknown acceptance_mode %s',mode);
s.nu_previous=state.nu_previous;s.direct_previous_command=state.delta_command(:);
x=x(:);if numel(x)==nx-3,x=[x;s.flight_state(7:9)];end;x(nx-2:nx)=s.flight_state(7:9);
if isempty(state.U),U=zeros(nu,N);U(1:3,:)=repmat(state.delta_command(:),1,N);
else,U=online_shift_controls(state.U,s.dts,s.dt_ctrl);end
if nu==7,U(4:7,:)=min(max(U(4:7,:),-s.gdot),s.gdot);end
U(1:3,:)=min(max(U(1:3,:),-v.delta_max(:)),v.delta_max(:));
info=struct('fallback',false,'fallback_reason','','status',0,'objective',inf,'iterations',0, ...
 'qp_solved',0,'accepted_steps',0,'rejected_steps',0,'full_step_checks',0, ...
 'restoration_calls',0,'policy_rejections',0,'max_sqp_iterations',max_sqp, ...
 'acceptance_mode',mode,'objective_tol',0,'authority',[],'qp_delta_norm',[], ...
 'qp_solver_time_s',[],'qp_total_time_s',[],'rollout_time_s',[],'status_by_qp',[], ...
 'early_stop_reason','','rejected',0,'repaired_candidates',0,'trust_scale',[], ...
 'trust_radius_rad',0,'rounds',[]);
trust_rad=.25*min(v.delta_dot_max(:))*s.dt_ctrl;
if isfield(s,'direct_trust_rad'),trust_rad=double(s.direct_trust_rad);end
info.trust_radius_rad=trust_rad;
info.iteration_authority={};
try
 [X,U,a,ok,reason]=direct_servo_propagate(s,x,U,s.flight_state,true);
 assert(ok,'direct:restoration','Unable to restore a feasible command plan: %s',reason);
 [valid,detail]=direct_servo_verify(s,x,U,a,s.flight_state);
 assert(valid,'direct:restoration','Restored plan failed certification: %s',detail);
 [J,terms]=direct_servo_objective(s,X,U,s.direct_previous_command,s.nu_previous);
 objective_tol=1e-9*max(1,abs(J));info.objective_tol=objective_tol;
 accepted=struct('X',X,'U',U,'a',a,'J',J,'terms',terms);
 best_candidate=accepted;
 info.iteration_authority{end+1}=a;
 for iteration=1:max_sqp
  info.iterations=iteration;
  if force_failure,error('direct:forcedFailure','Injected failure');end
  if isfield(state,'inject_failure_at') && any(state.inject_failure_at(:)==iteration)
   info.status=-1;info.status_by_qp(end+1)=-1;info.early_stop_reason='injected_qp_failure';break;
  end
  round=struct('iteration',iteration,'qp_status',0,'qp_delta_inf',0, ...
   'cmd_before',U(1:3,1),'cmd_proposed',U(1:3,1),'cmd_accepted',U(1:3,1), ...
   'J_old_total',J,'J_try_total',NaN,'J_old_terms',terms,'J_try_terms',terms, ...
   'qp_predicted_delta_J',NaN,'actual_delta_J',NaN,'predicted_vs_actual_ratio',NaN, ...
   'trust_scale',1,'proposal_feasible',false,'restore_applied',false, ...
   'restore_change_inf',0,'accepted',false,'rejection_reason','', ...
   'qp_setup_ms',0,'qp_solve_ms',0,'linearize_ms',0,'nonlinear_rollout_ms',0, ...
   'objective_eval_ms',0,'round_total_ms',0);
  round_started=tic;
  qp_timer=tic;
  work=assemble_direct_servo_qp(s,X,U,a);
  work.validate_readback=false;
  round.qp_setup_ms=1000*toc(qp_timer);
  r=solve_hpipm_snapshot(work);
  round.qp_solve_ms=1000*r.time_s;
  info.qp_total_time_s(end+1)=toc(qp_timer);
  info.qp_solver_time_s(end+1)=r.time_s;info.status=r.status;info.status_by_qp(end+1)=r.status;
  round.qp_status=r.status;
  if r.status~=0||~all(isfinite(r.U(:)))||~all(isfinite(r.X(:)))
   info.early_stop_reason='qp_failed';round.rejection_reason='qp_failed';
   info.rounds{end+1}=round;break;
  end
  info.qp_solved=info.qp_solved+1;
  step_norm=max(abs(r.U(1:3,:)),[],'all');info.qp_delta_norm(end+1)=max(abs(r.U),[],'all');
  round.qp_delta_inf=max(abs(r.U),[],'all');
  if max(abs(r.U),[],'all')<1e-4
   info.early_stop_reason='control_converged';round.rejection_reason='control_converged';
   info.rounds{end+1}=round;break;
  end
  % Fixed trust region: the linear servo block is exact only while the rate limit is
  % inactive, so a step far beyond the rate-limited span is truncated deterministically
  % (a trust region, not an alpha candidate search).
  scale=1;
  if step_norm>trust_rad,scale=trust_rad/step_norm;end
  info.trust_scale(end+1)=scale;round.trust_scale=scale;
  proposal=U+scale*r.U;
  timer=tic;
  [Xt,Ut,at,ok,reason]=direct_servo_propagate(s,x,proposal,s.flight_state,false);
  info.rollout_time_s(end+1)=toc(timer);info.full_step_checks=info.full_step_checks+1;
  round.restore_applied=~ok;round.restore_change_inf=0;
  if ~ok
   info.restoration_calls=info.restoration_calls+1;
   [Xt,Ut,at,ok,reason]=direct_servo_propagate(s,x,proposal,s.flight_state,true);
   if ok
    [ok,detail]=direct_servo_verify(s,x,Ut,at,s.flight_state);
    if ok,info.repaired_candidates=info.repaired_candidates+1;else,reason=detail;end
   end
  end
  round.nonlinear_rollout_ms=1000*sum(info.rollout_time_s(end));
  if ~ok
   info.rejected_steps=info.rejected_steps+1;info.rejected=info.rejected_steps;
   info.early_stop_reason=['rejected: ' reason];round.rejection_reason=['infeasible: ' reason];
   info.rounds{end+1}=round;break;
  end
  round.proposal_feasible=true;
  round.cmd_proposed=Ut(1:3,1);
  Jt_timer=tic;
  [Jt,terms_t]=direct_servo_objective(s,Xt,Ut,s.direct_previous_command,s.nu_previous);
  round.objective_eval_ms=1000*toc(Jt_timer);
  round.J_try_total=Jt;round.J_try_terms=terms_t;round.actual_delta_J=Jt-J;
  round.qp_predicted_delta_J=r.objective;
  if isfinite(r.objective)&&abs(Jt-J)>0
   round.predicted_vs_actual_ratio=r.objective/(Jt-J);
  end
  if strcmp(mode,'hard_stop')&&Jt>J+objective_tol
   info.rejected_steps=info.rejected_steps+1;info.rejected=info.rejected_steps;
   info.early_stop_reason='objective_increase';
   round.rejection_reason='objective_increase';
   info.rounds{end+1}=round;break;
  end
  % Accepted nominal point for the next round (all three modes reach here only when the
  % proposal is finite and physically feasible).
  improvement=J-Jt;
  X=Xt;U=Ut;a=at;J=Jt;terms=terms_t;
  round.accepted=true;round.cmd_accepted=U(1:3,1);
  round.round_total_ms=1000*toc(round_started);
  accepted=struct('X',X,'U',U,'a',a,'J',J,'terms',terms);
  if J<best_candidate.J,best_candidate=accepted;end
  info.accepted_steps=info.accepted_steps+1;info.iteration_authority{end+1}=a;
  info.rounds{end+1}=round;
  if strcmp(mode,'continue_best')
   % No early exit on cost: the cycle ends only at the round budget or a hard failure.
   if improvement<=1e-12,info.early_stop_reason='objective_stagnation';break;end
  else
   if improvement<=1e-6*max(1,abs(Jt)),info.early_stop_reason='objective_stagnation';break;end
  end
 end
 if isempty(info.early_stop_reason),info.early_stop_reason='max_iterations';end
 if strcmp(mode,'continue_best')
  % Final selection over every feasible candidate of this cycle, including the start point.
  best=best_candidate;
  X=best.X;U=best.U;a=best.a;J=best.J;terms=best.terms;
 else
  X=accepted.X;U=accepted.U;a=accepted.a;J=accepted.J;
 end
 [valid,detail]=direct_servo_verify(s,x,U,a,s.flight_state);
 assert(valid,'direct:first','First-command gate failed: %s',detail);
 command=U(:,1);state.U=U;state.last=command;state.delta_command=command(1:3);
 info.objective=J;info.authority=a;info.objective_terms=terms;
 if nu==7,state.nu_previous=command(4:7);end
catch err
 flight=s.flight_state;limit=v.delta_max(:);rate=v.delta_dot_max(:);
 held=min(max(flight(10:12),flight(7:9)-rate*s.dt_ctrl),flight(7:9)+rate*s.dt_ctrl);
 held=min(max(held,-limit),limit);
 command=held;
 if nu==7
  nu=min(max(zeros(4,1),max(-s.gdot,state.nu_previous-s.gddot*s.dt_ctrl)),min(s.gdot,state.nu_previous+s.gddot*s.dt_ctrl));
  command=[held;nu];state.nu_previous=nu;
 end
 state.U=[];state.delta_command=held;info.fallback=true;info.failure=err.message;
 info.fallback_reason=err.message;info.status=-1;info.authority=[];
end
info.cycle_time_s=toc(started);info.elapsed_s=info.cycle_time_s;
end
