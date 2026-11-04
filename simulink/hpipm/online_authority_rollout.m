function [X,U,authority,feasible]=online_authority_rollout(s,x,U,flight,restore)
% Reset at measured r/v/delta on EVERY call, including line-search candidates.
% flight=[r_NED;v_body;delta_actual;allocator_previous_command].
% Only attitude/CMG states remain HPIPM decision variables.
persistent v
if isempty(v),p=vehicle_params;v=p.vehicle;end
assert(numel(flight)==12 && all(isfinite(flight)),'online:feedback','Missing measured flight/servo feedback');
N=double(s.N);X=zeros(double(s.nx),N+1);X(:,1)=x;
if double(s.nx)==11,F=@native_F_B;else,F=@native_F_A;end
z=[flight(1:3);online_quaternion_rotation(x(1:4))*flight(4:6)];delta=flight(7:9);previous=flight(10:12);
authority=struct('tau_min',zeros(3,N),'tau_max',zeros(3,N),'air',zeros(6,N), ...
 'natural',zeros(3,N),'delta',zeros(3,N+1),'allocation_residual',zeros(3,N), ...
 'tracking_residual',zeros(3,N),'vertex_residual',zeros(8,N),'scale',zeros(1,N),'position',zeros(3,N+1));
authority.servo_vertex_residual=zeros(1,N);authority.vertex_commands=zeros(3,8,N);
authority.mode=s.authority_mode;authority.max_step_s=s.authority_max_step;
authority.nominal_tau=zeros(3,N);authority.allocator_command=zeros(3,N);
authority.axis_scale=zeros(3,N);
authority.delta(:,1)=delta;authority.position(:,1)=z(1:3);feasible=true;
nu_previous=zeros(4,1);if isfield(s,'nu_previous'),nu_previous=s.nu_previous;end
for k=1:N
 R=online_quaternion_rotation(X(1:4,k));[air,detail]=online_flight_condition(z(1:3),R'*z(4:6));
 if strcmp(s.authority_mode,'online_current')
  % L1 diagnostic ablation ONLY: freeze measured air data over the horizon.
  [air,detail]=online_flight_condition(s.flight_state(1:3),s.flight_state(4:6));
 end
 [natural,B]=online_aero_moment(v,air,X(5:7,k),delta);
 lower_cmd=max(-v.delta_max(:),previous-v.delta_dot_max(:)*s.dt_ctrl);
 upper_cmd=min(v.delta_max(:),previous+v.delta_dot_max(:)*s.dt_ctrl);
 [lower,upper]=online_surface_reachability(v,delta,s.dts(k),previous,.001,lower_cmd,upper_cmd);
 lower=max(lower,lower_cmd);upper=min(upper,upper_cmd);
 if any(lower>upper),feasible=false;return;end
 center=(lower+upper)/2;
 box=online_certified_torque_box(natural,B,lower,upper,center);
 certificate=online_certify_servo_vertices(v,delta,s.dts(k),box.vertex_delta,lower_cmd,upper_cmd);
 outside=any(U(1:3,k)<box.tau_min-1e-8 | U(1:3,k)>box.tau_max+1e-8);
 if restore,U(1:3,k)=min(max(U(1:3,k),box.tau_min),box.tau_max);
 elseif outside,feasible=false;return;end
 if double(s.nx)==11
  lo=max(-s.gdot,nu_previous-s.gddot*s.dts(k));hi=min(s.gdot,nu_previous+s.gddot*s.dts(k));
  if k==1,lo=max(-s.gdot,nu_previous-s.gddot*s.dt_ctrl);hi=min(s.gdot,nu_previous+s.gddot*s.dt_ctrl);end
  if restore,U(4:7,k)=min(max(U(4:7,k),lo),hi);
  elseif any(U(4:7,k)<lo-1e-8 | U(4:7,k)>hi+1e-8),feasible=false;return;end
  nu_previous=U(4:7,k);
 end
 target=U(1:3,k)-natural;
 if rcond(B)>1e-10
  command=B\target;
  if all(command>=lower_cmd-1e-10 & command<=upper_cmd+1e-10)
   command=min(max(command,lower_cmd),upper_cmd);residual=B*command-target;
  else,[command,residual]=bounded_control_allocation(B,target,lower_cmd,upper_cmd);end
 else,[command,residual]=bounded_control_allocation(B,target,lower_cmd,upper_cmd);end
 if norm(residual,inf)>1e-7,feasible=false;return;end
 [~,~,next_delta]=online_surface_reachability(v,delta,s.dts(k),command,.001,lower_cmd,upper_cmd);
 X(:,k+1)=F(X(:,k),U(:,k),s.tau_d(:,k),s.dts(k));
 prediction=online_translation_prediction(z,[0;s.dts(k)],X(1:4,k:k+1),s.authority_max_step);
 z=prediction.state(:,end);
 authority.tau_min(:,k)=box.tau_min;authority.tau_max(:,k)=box.tau_max;
 authority.air(:,k)=detail;authority.natural(:,k)=natural;
 authority.vertex_residual(:,k)=box.vertex_residual(:);authority.scale(k)=box.scale;
 authority.axis_scale(:,k)=box.axis_scale;
 authority.servo_vertex_residual(k)=certificate.max_residual_rad;
 authority.vertex_commands(:,:,k)=certificate.command;
 authority.nominal_tau(:,k)=U(1:3,k);authority.allocator_command(:,k)=command;
 authority.allocation_residual(:,k)=residual;authority.tracking_residual(:,k)=natural+B*next_delta-U(1:3,k);
 delta=next_delta;previous=command;authority.delta(:,k+1)=delta;authority.position(:,k+1)=z(1:3);
end
end
