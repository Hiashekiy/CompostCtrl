function delta=direct_servo_state_at(v,delta0,command,t)
% Exact 1 ms discrete servo state at time t (t must be a multiple of the servo clock).
% Same recurrence as servo_predict/online_surface_reachability, closed form: the
% command is held, so the angle moves at the rate limit first and then decays
% exponentially towards the (already angle-limited) command. No overshoot => exact.
assert(isscalar(t) && t>=0,'direct:servo','time must be a nonnegative scalar');
% Positions may be requested between servo ticks (e.g. a substep midpoint); snap to the
% nearest 1 ms tick, which is the clock the plant itself runs on.
n=round(t/.001);
if n<1,n=1;end
delta0=delta0(:);command=command(:);
limit=v.delta_max(:);rate=v.delta_dot_max(:);
cmd=min(max(command,-limit),limit);
distance=abs(cmd-delta0);threshold=rate*v.tau_s;
steps=zeros(3,1);moving=rate>0;
steps(moving)=min(n,max(0,ceil((distance(moving)-threshold(moving))./(.001*rate(moving)))));
remaining=max(0,distance-steps*.001.*rate).*(1-.001/v.tau_s).^(n-steps);
delta=cmd-sign(cmd-delta0).*remaining;
delta(~moving)=delta0(~moving);
delta=min(max(delta,-limit),limit);
end
