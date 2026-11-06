function [g,rate]=e2r_drive_endpoint(g,rate,command,duration)
% Exact recurrence copied from the existing physical CMG drive graph.
p=vehicle_params;command=min(max(command(:),-p.cmg.gdot_max),p.cmg.gdot_max);g=g(:);rate=rate(:);
for j=1:round(duration/.001)
 nextg=g+.001*rate;acc=min(max((command-rate)/p.cmg.tau_g,-p.cmg.gddot_max),p.cmg.gddot_max);rate=min(max(rate+.001*acc,-p.cmg.gdot_max),p.cmg.gdot_max);g=nextg;
end
end
