function [u,info]=online_torque_feedback_step(v,tau_cmd,delta_actual,air,omega,u_previous,options)
% One 20 ms inner decision: the three servo commands that put the aerodynamic torque
% at the END of the next 20 ms as close as possible to the torque the upper MPC asked
% for, using the measured surface deflection and the same online aero model.
%
%   min_u  || tau_cmd - tau_nat - B F_dt(delta_actual,u) ||_W^2 + lambda*||D^-1 (u-u_prev) ||^2
%   s.t.   u in [-delta_max, delta_max] and |u-u_prev| <= delta_dot_max*dt
%
% F_dt is the 20 ms endpoint of the EXISTING 1 ms servo recurrence
% (online_servo_endpoint).  The box is the inner COMMAND slew/saturation limit; the
% physical servo keeps its own angle and rate limits unchanged.
%
% Only three variables, and each axis is independent inside F, so the Jacobian is
% diagonal and at most four damped Gauss-Newton iterations with projection and
% backtracking are enough.  The commanded deflection enters through F, which is the
% servo-lag compensation a static allocation cannot express.
%
% W_tau is the identity (explicit: no per-axis normalisation, nothing tuned on the
% test output).  lambda is a small regulariser; with W=I and lambda=1e-3 it only damps
% the step.  Every failure path falls back to the bounded static allocation at the
% same measured operating point, i.e. still inside the physical servo limits.
if nargin<7,options=struct();end
opts=options;
if ~isfield(opts,'dt'),opts.dt=.02;end
if ~isfield(opts,'lambda'),opts.lambda=1e-3;end
if ~isfield(opts,'max_iterations'),opts.max_iterations=4;end
if ~isfield(opts,'backtracks'),opts.backtracks=4;end
if ~isfield(opts,'weight'),opts.weight=ones(3,1);end
if ~isfield(opts,'servo_dt'),opts.servo_dt=.001;end
assert(isscalar(opts.dt) && opts.dt>0 && isscalar(opts.lambda) && opts.lambda>=0);
assert(isscalar(opts.max_iterations) && opts.max_iterations>=1 && opts.max_iterations<=4);
tau_cmd=tau_cmd(:);delta_actual=delta_actual(:);omega=omega(:);u_previous=u_previous(:);air=air(:);
assert(numel(tau_cmd)==3 && numel(delta_actual)==3 && numel(omega)==3 && numel(u_previous)==3);
assert(numel(air)==4 && all(isfinite([tau_cmd;delta_actual;omega;air;u_previous])), ...
 'online:inner','non-finite inner-loop input');
w=opts.weight(:);
assert(numel(w)==3 && all(w>0) && all(isfinite(w)));
limit=v.delta_max(:);rate=v.delta_dot_max(:);
assert(all(abs(delta_actual)<=limit+1e-12),'online:inner','measured deflection outside its own limit');
[natural,B]=online_aero_moment(v,air,omega,delta_actual);
target=tau_cmd-natural;
lower=max(-limit,u_previous-rate*opts.dt);
upper=min(limit,u_previous+rate*opts.dt);
assert(all(lower<=upper+1e-15),'online:inner','empty inner command set');
[static_u,static_residual]=bounded_control_allocation(B,target,lower,upper);
info=struct('natural',natural,'B',B,'target',target,'lower',lower,'upper',upper, ...
 'static_command',static_u,'static_residual_Nm',static_residual,'iterations',0, ...
 'cost_initial',NaN,'cost_final',NaN,'fallback',false,'note','', ...
 'predicted_endpoint',nan(3,1),'achievable_torque',nan(3,1),'residual_Nm',nan(3,1), ...
 'slew_saturated',false,'gradient_norm',NaN,'commanded_delta_change',nan(3,1));
if rcond(B)<=1e-10 || ~all(isfinite(B(:)))
 % Ill-conditioned effectiveness: hold the bounded static allocation, never a raw solve.
 u=static_u;info=record(info,u,'ill_conditioned');return;
end
u=static_u;
cost=cost_of(u);
info.cost_initial=cost;
for iteration=1:opts.max_iterations
 info.iterations=iteration;
 [endpoint,gain]=online_servo_endpoint(v,delta_actual,u,opts.dt,opts.servo_dt);
 S=B.*gain.';                                  % d(B F)/du = B*diag(gain)
 residual=target-B*endpoint;
 slope=-2*(S.'*(w.*residual))+2*opts.lambda*((u-u_previous)./limit.^2);
 hessian=2*(S.'*(w.*S)+opts.lambda*diag(1./limit.^2));
 info.gradient_norm=norm(slope,inf);
 if ~all(isfinite(hessian(:))) || ~all(isfinite(slope))
  u=static_u;info=record(info,u,'nonfinite_newton_step');return;
 end
 step=-(hessian+1e-12*eye(3))\slope;
 if ~all(isfinite(step))
  u=static_u;info=record(info,u,'nonfinite_newton_step');return;
 end
 if norm(step,inf)<1e-10,break;end
 improved=false;
 for attempt=1:opts.backtracks
  trial=min(max(u+step./2^(attempt-1),lower),upper);
  trial_cost=cost_of(trial);
  if all(isfinite(trial)) && trial_cost<=cost
   u=trial;cost=trial_cost;improved=true;break;
  end
 end
 if ~improved,break;end
end
info.cost_final=cost_of(u);
if info.cost_final>info.cost_initial+1e-12
 % A projected Gauss-Newton step must never leave the inner problem worse than its own
 % feed-forward initialisation; if it did, keep the initialisation and say so.
 u=static_u;info.cost_final=cost_of(u);info.fallback=true;
 info.note='cost_increase_reverted';info=record(info,u,info.note);return;
end
info=record(info,u,'');
if ~all(isfinite(u)) || any(u<lower-1e-12) || any(u>upper+1e-12)
 u=static_u;info=record(info,u,'constraint_violation');
end
 function value=cost_of(candidate)
  e=online_servo_endpoint(v,delta_actual,candidate,opts.dt,opts.servo_dt);
  r=target-B*e;
  value=sum(w.*r.^2)+opts.lambda*sum(((candidate-u_previous)./limit).^2);
 end
 function meta=record(meta,value,reason)
  out=min(max(value(:),lower),upper);
  [e,g]=online_servo_endpoint(v,delta_actual,out,opts.dt,opts.servo_dt);
  meta.predicted_endpoint=e;meta.gain=g;
  meta.achievable_torque=natural+B*e;
  meta.residual_Nm=tau_cmd-meta.achievable_torque;
  meta.slew_saturated=any(abs(out-u_previous)>=rate*opts.dt-1e-9);
  meta.commanded_delta_change=out-u_previous;
  if ~isempty(reason),meta.fallback=true;meta.note=reason;end
 end
end
