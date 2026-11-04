function certificate=online_fast_servo_certificate(v,actual,duration,targets,lower,upper)
% Monotone-interval replacement for the 45-step bisection of every torque vertex.
%
% For a held command u the plant's 1 ms servo map F_n(u;delta0) is nondecreasing in u
% (first-order lag with a symmetric rate clip, 1 ms <= tau_s, nonnegative rate). Hence
% the set of achievable end angles over the admissible command interval [lower,upper] is
% exactly the interval [F_n(lower), F_n(upper)], and every vertex target inside it is
% reachable by SOME constant command. Both endpoints are the same closed form that
% online_surface_reachability already evaluates, so no bisection is needed.
%
% This proves EXISTENCE of a constant command per vertex; it does not produce that
% command (the production first control still comes from the allocator chain), so the
% command/residual fields are left empty/NaN instead of being filled with zeros:
%   commands_evaluated = false, max_residual_rad = NaN, command = []
% Any failed precondition returns certified=false; the caller must then use the legacy
% bisection certificate unchanged.
persistent rate_checked
if isempty(rate_checked)
 assert(v.tau_s>0 && all(v.delta_dot_max(:)>=0),'fast:servo','Servo parameters are outside the monotonicity assumptions');
 rate_checked=true;
end
servo_dt=.001;
n=round(duration/servo_dt);
assert(abs(n*servo_dt-duration)<1e-10,'fast:servo','Duration must align with the servo clock');
assert(servo_dt<=v.tau_s,'fast:servo','Servo step must keep the recurrence monotone');
actual=actual(:);lower=lower(:);upper=upper(:);
limit=v.delta_max(:);rate=v.delta_dot_max(:);
certified=false;
certificate=struct('certified',false,'certificate_type','analytic_monotone_interval', ...
 'commands_evaluated',false,'command',[],'max_residual_rad',NaN, ...
 'endpoint_margin',NaN,'endpoint_low',nan(3,1),'endpoint_high',nan(3,1), ...
 'interior_vertices',0,'total_vertices',size(targets,2));
if ~isequal(size(targets,1),3) || ~all(isfinite(targets(:))) || ~all(isfinite(actual))
 return;
end
if any(lower<max(-limit,-v.delta_max(:))) || any(upper>min(limit,v.delta_max(:))) || any(lower>upper)
 return;
end
endpoint_low=advance(lower);endpoint_high=advance(upper);
certificate.endpoint_low=endpoint_low;certificate.endpoint_high=endpoint_high;
tol_delta=1e-12*max(1,max(abs(limit)));
inside=all(targets>=endpoint_low-tol_delta & targets<=endpoint_high+tol_delta,1);
all_inside=all(inside);
margin=min([targets-endpoint_low;endpoint_high-targets],[],1);
certificate.endpoint_margin=min(margin);
certificate.interior_vertices=nnz(inside);
certificate.certified=all_inside;
certificate.certificate_type='analytic_monotone_interval';
 function endpoint=advance(command)
  % Same closed form as online_surface_reachability: rate-limited prefix, then geometric
  % decay. n steps of the exact 1 ms recurrence, no for-loop.
  command=min(max(command(:),lower),upper);
  distance=abs(command-actual);threshold=rate*v.tau_s;
  steps=zeros(3,1);moving=rate>0;
  steps(moving)=min(n,max(0,ceil((distance(moving)-threshold(moving))./(servo_dt*rate(moving)))));
  remaining=max(0,distance-steps*servo_dt.*rate).*(1-servo_dt/v.tau_s).^(n-steps);
  endpoint=command-sign(command-actual).*remaining;
  endpoint(~moving)=actual(~moving);
  endpoint=min(max(endpoint,-limit),limit);
 end
end
