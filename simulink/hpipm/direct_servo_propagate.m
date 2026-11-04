function [X,U,a,ok,reason]=direct_servo_propagate(s,x0,U,flight,restore)
% Nonlinear propagation of the direct-servo model along a command sequence, with the
% external flight state (position/velocity/air data) predicted outside the optimiser
% and updated stage by stage, and the servo resolved inside every stage.
% flight=[r_NED;v_body;delta_actual;previous_command] from the measured plant.
persistent v
if isempty(v),p=vehicle_params;v=p.vehicle;end
assert(numel(flight)==12 && all(isfinite(flight)),'direct:feedback','Missing measured flight/servo feedback');
N=double(s.N);nx=double(s.nx_direct);nu=double(s.nu_direct);
assert(isequal(size(U),[nu N]),'direct:control','Command sequence has the wrong size');
X=zeros(nx,N+1);X(:,1)=x0;
z=[flight(1:3);online_quaternion_rotation(x0(1:4))*flight(4:6)];
a=struct('air',zeros(6,N),'position',zeros(3,N+1),'delta',zeros(3,N+1), ...
 'command',zeros(3,N),'torque',zeros(3,N),'natural',zeros(3,N), ...
 'slew_slack',zeros(3,N),'cmg_slack',zeros(4,N),'B_delta',zeros(3,3,N),'max_violation',0);
a.position(:,1)=z(1:3);a.delta(:,1)=flight(7:9);
% The servo state is measured, never assumed: the plant's actual angle starts this stage.
X(nx-2:nx,1)=flight(7:9);
previous=flight(10:12);nu_previous=zeros(4,1);
if isfield(s,'nu_previous'),nu_previous=s.nu_previous;end
limit=v.delta_max(:);rate=v.delta_dot_max(:);
ok=true;reason='';
for k=1:N
 R=online_quaternion_rotation(X(1:4,k));[air,detail]=online_flight_condition(z(1:3),R'*z(4:6));
 cmd=U(1:3,k);
 lower_cmd=max(-limit,previous-rate*s.dt_ctrl);upper_cmd=min(limit,previous+rate*s.dt_ctrl);
 violation=max([0;lower_cmd-cmd;cmd-upper_cmd]);
 if violation>0
  if restore
   cmd=min(max(cmd,lower_cmd),upper_cmd);U(1:3,k)=cmd;
  else
   ok=false;reason=sprintf('stage %d: command step exceeds the servo rate limit',k);return;
  end
 end
 if double(s.nx_direct)==14
  lo=max(-s.gdot,nu_previous-s.gddot*s.dts(k));hi=min(s.gdot,nu_previous+s.gddot*s.dts(k));
  if k==1,lo=max(-s.gdot,nu_previous-s.gddot*s.dt_ctrl);hi=min(s.gdot,nu_previous+s.gddot*s.dt_ctrl);end
  nu_cmd=U(4:7,k);
  if any(nu_cmd<lo-1e-12)||any(nu_cmd>hi+1e-12)
   if restore
    U(4:7,k)=min(max(nu_cmd,lo),hi);
   else
    ok=false;reason=sprintf('stage %d: CMG rate command outside the achievable cone',k);return;
   end
  end
  a.cmg_slack(:,k)=min(U(4:7,k)-lo,hi-U(4:7,k));
  nu_previous=U(4:7,k);
 end
 a.air(:,k)=detail;
 [natural,B,total]=online_aero_moment(v,air,X(5:7,k),X(nx-2:nx,k));
 a.natural(:,k)=natural;a.torque(:,k)=total;a.B_delta(:,:,k)=B;
 a.command(:,k)=U(1:3,k);
 a.slew_slack(:,k)=min(U(1:3,k)-lower_cmd,upper_cmd-U(1:3,k));
 X(:,k+1)=direct_servo_stage(s,X(:,k),U(:,k),s.dts(k),air,s.tau_d(:,k));
 a.delta(:,k+1)=X(nx-2:nx,k+1);
 previous=U(1:3,k);
 prediction=online_translation_prediction(z,[0;s.dts(k)],X(1:4,k:k+1),s.authority_max_step);
 z=prediction.state(:,end);a.position(:,k+1)=z(1:3);
end
if ~all(isfinite(X(:))),ok=false;reason='non-finite prediction';return;end
end
