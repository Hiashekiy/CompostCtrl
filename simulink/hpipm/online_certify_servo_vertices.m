function certificate=online_certify_servo_vertices(v,actual,duration,targets,lower,upper)
% Invert the monotone discrete servo for all eight endpoint targets.
% Constant commands imply monotone state paths: endpoints plus command/rate
% limits certify the entire interval, not merely an unchecked terminal sample.
n=round(duration/.001);assert(abs(n*.001-duration)<1e-10);
lo=repmat(lower(:),1,size(targets,2));hi=repmat(upper(:),1,size(targets,2));
actual=actual(:);rate=v.delta_dot_max(:);
for k=1:45
 command=(lo+hi)/2;endpoint=advance(command);
 below=endpoint<targets;lo(below)=command(below);hi(~below)=command(~below);
end
command=(lo+hi)/2;endpoint=advance(command);
residual=max(abs(endpoint-targets),[],'all');
assert(residual<1e-9,'online:servoVertex','Torque vertex cannot be reached by the discrete servo');
assert(all(abs(command)<=v.delta_max(:)+1e-12,'all'));
assert(all(abs(actual)<=v.delta_max(:)+1e-12));
assert(all(abs(endpoint)<=v.delta_max(:)+1e-12,'all'));
certificate=struct('command',command,'endpoint',endpoint,'max_residual_rad',residual, ...
 'state_path_min',min(actual,endpoint),'state_path_max',max(actual,endpoint));
 function endpoint=advance(target)
  distance=abs(target-actual);
  steps=min(n,max(0,ceil((distance-rate*v.tau_s)./(.001*max(rate,realmin)))));
  remaining=max(0,distance-steps.*.001.*rate).*(1-.001/v.tau_s).^(n-steps);
  endpoint=target-sign(target-actual).*remaining;
  stationary=rate==0;endpoint(stationary,:)=repmat(actual(stationary),1,size(target,2));
 end
end
