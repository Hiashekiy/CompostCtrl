function [valid,detail]=online_verify_first_command(s,U,authority,flight)
% Final gate before the first control of this cycle is executed. It re-checks that
% the stored plan, the authority certified for it and the MEASURED flight/servo
% feedback describe the same instant, and that the allocator can realise stage one
% from the actual surface position within one control period.
persistent v
if isempty(v),p=vehicle_params;v=p.vehicle;end
valid=false;detail='';N=double(s.N);
if ~isequal(size(U),[double(s.nu) N]),detail='control plan has the wrong size';return;end
if ~all(isfinite(U(:))) || ~all(isfinite(authority.position(:))) || ~all(isfinite(authority.natural(:)))
 detail='non-finite control or prediction';return;
end
if norm(authority.position(:,1)-flight(1:3),inf)>1e-10,detail='prediction does not start at the measured position';return;end
if norm(authority.delta(:,1)-flight(7:9),inf)>1e-10,detail='prediction does not start at the measured surface angle';return;end
if max(abs(authority.nominal_tau(:,1)-U(1:3,1)))>1e-12,detail='stage-one torque is not the certified one';return;end
if any(U(1:3,1)<authority.tau_min(:,1)-1e-8) || any(U(1:3,1)>authority.tau_max(:,1)+1e-8)
 detail='stage-one torque is outside its own certified box';return;
end
if max(abs(authority.allocation_residual(:,1)))>1e-7,detail='stage-one allocation residual';return;end
command=authority.allocator_command(:,1);
lo=max(-v.delta_max(:),flight(10:12)-v.delta_dot_max(:)*s.dt_ctrl);
hi=min(v.delta_max(:),flight(10:12)+v.delta_dot_max(:)*s.dt_ctrl);
if any(command<lo-1e-8) || any(command>hi+1e-8)
 detail='stage-one surface command is unreachable from the previous command';return;
end
if double(s.nx)==11
 previous=zeros(4,1);if isfield(s,'nu_previous'),previous=s.nu_previous;end
 lo=max(-s.gdot,previous-s.gddot*s.dt_ctrl);hi=min(s.gdot,previous+s.gddot*s.dt_ctrl);
 if any(U(4:7,1)<lo-1e-8) || any(U(4:7,1)>hi+1e-8),detail='stage-one CMG rate outside the cone';return;end
end
valid=true;
end
