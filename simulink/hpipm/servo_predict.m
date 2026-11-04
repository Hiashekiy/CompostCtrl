function r=servo_predict(v,delta0,command,duration,air,omega,servo_dt)
% Truth-consistent discrete surface servo, one 1 ms step at a time.
%
% Same block order as simulink/models/Surface_Actuator.slx (build_actuators_aero:
% make_drive), so this is a re-implementation of the plant, not a simplification:
%   cmd   = clip(delta_cmd, +-delta_max)                      Command_limit
%   rate  = clip((cmd - delta)/tau_s, +-delta_dot_max)         Inverse_time_constant, Derivative_limit
%   delta = clip(delta + servo_dt*rate, +-delta_max)           Euler_increment, Next_state, State_limit
% The actuator clock (UnitDelay, servo_dt) is independent of the MPC stage length,
% so a stage of 0.2/0.5/1/2 s always reuses the command as a zero-order hold and the
% angle keeps moving inside the stage.
%
% r.time(k)=(k-1)*servo_dt, r.delta(3,n+1), r.delta_dot(3,n). With air=[qbar;V;alpha;beta]
% supplied, r.torque(3,n+1)=tau_nat(omega,delta)+B_delta(qbar)*delta is the per-millisecond
% aerodynamic moment history (not the stage-end value).
if nargin<5,air=[];end
if nargin<6,omega=[];end
if nargin<7,servo_dt=.001;end
delta0=delta0(:);command=command(:);
assert(numel(delta0)==3 && numel(command)==3 && all(isfinite([delta0;command])));
assert(isscalar(duration) && isfinite(duration) && duration>=0);
assert(isscalar(servo_dt) && servo_dt>0 && servo_dt<=v.tau_s,'Servo step must keep the recurrence monotone');
n=round(duration/servo_dt);
assert(abs(n*servo_dt-duration)<1e-10,'online:servo','Duration must align with the servo clock');
limit=v.delta_max(:);rate_max=v.delta_dot_max(:);
assert(all(abs(delta0)<=limit+1e-12),'online:servo','Initial angle outside the physical limit');
cmd=min(max(command,-limit),limit);
delta=zeros(3,n+1);delta_dot=zeros(3,n);delta(:,1)=delta0;
for k=1:n
 delta_dot(:,k)=min(max((cmd-delta(:,k))/v.tau_s,-rate_max),rate_max);
 delta(:,k+1)=min(max(delta(:,k)+servo_dt*delta_dot(:,k),-limit),limit);
end
r.time=(0:n)*servo_dt;r.delta=delta;r.delta_dot=delta_dot;r.command=cmd;
r.torque=[];r.natural=[];
if ~isempty(air)
 air=air(:);assert(numel(air)==4 && all(isfinite(air)));
 if isempty(omega),omega=zeros(3,1);end
 omega=omega(:);assert(numel(omega)==3 && all(isfinite(omega)));
 r.torque=zeros(3,n+1);r.natural=zeros(3,n+1);
 for k=1:n+1
  [natural,B]=online_aero_moment(v,air,omega,delta(:,k));
  r.natural(:,k)=natural;r.torque(:,k)=natural+B*delta(:,k);
 end
end
end
