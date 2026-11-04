function [valid,detail]=direct_servo_verify(s,x,U,a,flight)
% Acceptance gate for a direct-servo command plan. It replaces the old torque-box
% certificate with checks on the QUANTITY THAT IS NOW EXECUTED (the servo command) plus
% the physical servo and CMG envelopes, so no unverified command can reach the plant.
persistent v
if isempty(v),p=vehicle_params;v=p.vehicle;end
valid=false;detail='';N=double(s.N);nx=double(s.nx_direct);nu=double(s.nu_direct);
limit=v.delta_max(:);rate=v.delta_dot_max(:);
if ~isequal(size(U),[nu N]),detail='command plan has the wrong size';return;end
if ~isequal(size(a.command),[3 N])||~isequal(size(a.delta),[3 N+1]),detail='diagnostics have the wrong size';return;end
if ~all(isfinite(U(:)))||~all(isfinite(a.air(:)))||~all(isfinite(a.position(:)))
 detail='non-finite command or prediction';return;
end
if norm(a.position(:,1)-flight(1:3),inf)>1e-10,detail='prediction does not start at the measured position';return;end
if norm(a.delta(:,1)-flight(7:9),inf)>1e-10,detail='prediction does not start at the measured angle';return;end
if norm(x(nx-2:nx)-flight(7:9),inf)>1e-10,detail='state does not start at the measured angle';return;end
if any(abs(U(1:3,:))>limit+1e-12,'all'),detail='command outside the physical angle limit';return;end
previous=flight(10:12);
for k=1:N
 span=rate*s.dt_ctrl;
 if any(U(1:3,k)>min(limit,previous+span)+1e-12)||any(U(1:3,k)<max(-limit,previous-span)-1e-12)
  detail=sprintf('stage %d command step is not reachable from the previous command',k);return;
 end
 previous=U(1:3,k);
end
if any(abs(a.delta)>limit+1e-12,'all'),detail='predicted angle violates the physical limit';return;end
if nu==7
 cmg_previous=zeros(4,1);if isfield(s,'nu_previous'),cmg_previous=s.nu_previous;end
 for k=1:N
  span=s.gddot*s.dts(k);if k==1,span=s.gddot*s.dt_ctrl;end
  if any(U(4:7,k)>min(s.gdot,cmg_previous+span)+1e-12)||any(U(4:7,k)<max(-s.gdot,cmg_previous-span)-1e-12)
   detail=sprintf('stage %d CMG rate command is outside the achievable cone',k);return;
  end
  cmg_previous=U(4:7,k);
 end
end
valid=true;
end
