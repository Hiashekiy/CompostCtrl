function y=direct_servo_stage(s,x,uk,dt,air,tau_d,substep)
% One prediction stage of the direct-servo model, propagated with the servo clock
% resolved inside the stage instead of being replaced by the stage-end angle.
%
% State  B: [q(4); omega(3); gamma(4); delta(3)] in R^14,  A: [q(4); omega(3); delta(3)] in R^10
% Control B: [delta_cmd(3); nu(4)] in R^7,                  A: delta_cmd(3) in R^3
% Attitude/CMG propagation reuses the validated native_F_* discretisation; the only
% new physics is that the aerodynamic moment is evaluated from the moving servo
% angle at every substep, using the closed-form 1 ms servo solution.
if nargin<7,substep=.02;end
persistent v
if isempty(v),p=vehicle_params;v=p.vehicle;end
nx=double(s.nx_direct);nsub=max(1,round(dt/substep));h=dt/nsub;
delta=x(nx-2:nx);cmd=uk(1:3);rate=v.delta_dot_max(:);limit=v.delta_max(:);
assert(all(isfinite(x)) && all(isfinite(uk)),'direct:state','Nonfinite state or control');
if double(s.nx_direct)==14
 att=x(1:11);nu=uk(4:7);
 for j=1:nsub
  delta_mid=direct_servo_state_at(v,delta,cmd,h/2);
  delta=direct_servo_state_at(v,delta,cmd,h);
  torque=online_aero_moment(v,air,att(5:7),delta_mid);
  att=native_F_B(att,[torque;nu],tau_d,h);
 end
 y=[att;delta];
else
 att=x(1:7);
 for j=1:nsub
  delta_mid=direct_servo_state_at(v,delta,cmd,h/2);
  delta=direct_servo_state_at(v,delta,cmd,h);
  torque=online_aero_moment(v,air,att(5:7),delta_mid);
  att=native_F_A(att,torque,tau_d,h);
 end
 y=[att;delta];
end
assert(all(isfinite(y)),'direct:state','Nonfinite prediction');
end
