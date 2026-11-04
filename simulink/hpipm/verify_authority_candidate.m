function valid=verify_authority_candidate(s,x,X,U,a,flight)
% Check certificates from the SAME causally restored X/U/Z trajectory.
% No clipping and no stale previous-trajectory box is used here.
% In the analytic certificate modes the servo vertex residual and the inverse commands are
% not evaluated (no bisection is run), so the gate uses the analytic flag and the endpoint
% margin instead of pretending a numeric residual of zero.
N=double(s.N);
mode='legacy';if isfield(a,'certificate_mode'),mode=a.certificate_mode;end
analytic=strcmp(mode,'analytic_box')||strcmp(mode,'analytic_full');
valid=isequal(size(U),[double(s.nu) N]) && isequal(size(X),[double(s.nx) N+1]) ...
 && all(isfinite([X(:);U(:);a.air(:);a.position(:)])) ...
 && norm(X(:,1)-x,inf)<1e-10 && norm(a.position(:,1)-flight(1:3),inf)<1e-10 ...
 && norm(a.delta(:,1)-flight(7:9),inf)<1e-10 ...
 && all(a.tau_min<=a.tau_max,'all') ...
 && all(U(1:3,:)>=a.tau_min-1e-8 & U(1:3,:)<=a.tau_max+1e-8,'all') ...
 && max(a.vertex_residual,[],'all')<=1e-9 ...
 && max(abs(a.allocation_residual),[],'all')<=1e-7;
if analytic
 % Per stage: a legacy fallback (singular matrix, unprovable box, unsatisfied monotone
 % interval) is legitimate and is judged by its own numeric residual; an analytic stage is
 % judged by its certificate flag and endpoint margin. Both are stricter than "0 residual".
 valid=valid && all(a.certified);
 for k=1:N
  if isfinite(a.servo_vertex_residual(k))
   valid=valid && a.servo_vertex_residual(k)<=1e-9;
  else
   valid=valid && a.certified(k) && a.endpoint_margin(k)>=-1e-12;
  end
 end
else
 valid=valid && max(a.servo_vertex_residual)<=1e-9 && all(isfinite(a.servo_vertex_residual));
end
if double(s.nx)==11
 previous=zeros(4,1);if isfield(s,'nu_previous'),previous=s.nu_previous;end
 for k=1:N
  dt=s.dts(k);if k==1,dt=s.dt_ctrl;end
  valid=valid && all(abs(U(4:7,k))<=s.gdot+1e-8) && all(abs(U(4:7,k)-previous)<=s.gddot*dt+1e-8);
  previous=U(4:7,k);
 end
end
end
