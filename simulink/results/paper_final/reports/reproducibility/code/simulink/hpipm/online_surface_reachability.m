function [lower,upper,nominal]=online_surface_reachability(v,actual,duration,command,servo_dt,command_lower,command_upper)
% Exact endpoint intervals for the existing monotone discrete servo recurrence.
% Endpoint reachability is NOT an instantaneous torque guarantee during a hold.
% Uses measured actual state, never the allocator's previous command.
if nargin<5,servo_dt=.001;end
actual=actual(:);command=command(:);limit=v.delta_max(:);rate=v.delta_dot_max(:);
if nargin<6,command_lower=-limit;command_upper=limit;end
command_lower=command_lower(:);command_upper=command_upper(:);
assert(all(command_lower>=-limit & command_upper<=limit & command_lower<=command_upper));
assert(numel(actual)==3 && numel(command)==3 && all(isfinite([actual;command])));
assert(isscalar(duration) && isfinite(duration) && duration>=0);
assert(isscalar(servo_dt) && isfinite(servo_dt) && servo_dt>0);
assert(all(abs(actual)<=limit+1e-12) && all(rate>=0) && v.tau_s>0);
assert(servo_dt<=v.tau_s,'Servo recurrence must be monotone');
n=round(duration/servo_dt);
assert(abs(n*servo_dt-duration)<1e-10,'Duration must align with the actual servo clock');
command=min(max(command,command_lower),command_upper);
lower=advance(actual,command_lower);upper=advance(actual,command_upper);nominal=advance(actual,command);
 function next=advance(state,target)
  % Closed form of the same discrete recurrence, including its rate-limited prefix.
  distance=abs(target-state);threshold=rate*v.tau_s;
  steps=zeros(3,1);moving=rate>0;
  steps(moving)=min(n,max(0,ceil((distance(moving)-threshold(moving))./(servo_dt*rate(moving)))));
  remaining=max(0,distance-steps.*servo_dt.*rate);
  remaining=remaining.*(1-servo_dt/v.tau_s).^(n-steps);
  next=target-sign(target-state).*remaining;next(~moving)=state(~moving);
  next=min(max(next,-limit),limit);
 end
end
