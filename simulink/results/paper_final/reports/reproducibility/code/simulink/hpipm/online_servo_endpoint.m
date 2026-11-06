function [endpoint,gain,steps]=online_servo_endpoint(v,actual,command,duration,servo_dt)
% Endpoint of the EXISTING 1 ms servo recurrence after `duration`, for a command that
% is held constant over that interval:
%     rate_k = sat((command-delta_k)/tau_s, +-delta_dot_max)
%     delta_{k+1} = sat(delta_k + dt*rate_k, +-delta_max)
% Uses the same closed form as online_surface_reachability (a separate implementation
% of the same physics is cross-checked in check_torque_margin_feedback), and returns
% the per-axis slope d(endpoint)/d(command) of the current linear piece, which is the
% only derivative the 20 ms inner problem needs.
if nargin<5,servo_dt=.001;end
actual=actual(:);command=command(:);
limit=v.delta_max(:);rate=v.delta_dot_max(:);
assert(numel(actual)==3 && numel(command)==3 && all(isfinite([actual;command])), ...
 'online:servoEndpoint','non-finite servo state or command');
assert(isscalar(duration) && isfinite(duration) && duration>=0);
assert(isscalar(servo_dt) && isfinite(servo_dt) && servo_dt>0 && servo_dt<v.tau_s, ...
 'online:servoEndpoint','servo step must be smaller than the servo time constant');
n=round(duration/servo_dt);
assert(abs(n*servo_dt-duration)<1e-10,'online:servoEndpoint','duration must align with the servo clock');
command=min(max(command,-limit),limit);
distance=abs(command-actual);
moving=rate>0;
steps=zeros(3,1);
steps(moving)=min(n,max(0,ceil((distance(moving)-rate(moving)*v.tau_s)./(servo_dt*rate(moving)))));
remaining=max(0,distance-steps.*servo_dt.*rate);
decay=(1-servo_dt/v.tau_s).^(n-steps);
endpoint=command-sign(command-actual).*(remaining.*decay);
endpoint(~moving)=actual(~moving);
endpoint=min(max(endpoint,-limit),limit);
% decay in [0,1] because steps<=n, so the slope stays in [0,1] by construction.
gain=1-decay;
gain(~moving)=0;
gain=min(max(gain,0),1);
end
