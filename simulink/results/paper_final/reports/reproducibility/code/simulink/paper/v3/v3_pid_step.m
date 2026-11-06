function [u,z,d]=v3_pid_step(x,qr,wr,flight,c,z)
p=vehicle_params;if isempty(z),z=struct('I',zeros(3,1),'nu',zeros(4,1),'gamma',x(7:10),'unload',false,'extra',zeros(3,1));end
q=paper_quat(x(1:3));sg=sign(q'*qr);if sg==0,sg=1;end;e=2*sg*(q(1)*qr(2:4)-qr(1)*q(2:4)-cross(q(2:4),qr(2:4)));target=c.kp.*e+c.ki.*z.I-c.kd.*(x(4:6)-wr);
air=online_flight_condition(flight(1:3),flight(4:6));[nat,B]=online_aero_moment(p.vehicle,air,x(4:6),flight(7:9));[dl,du,nom]=online_surface_reachability(p.vehicle,flight(7:9),.2,flight(10:12));
[delta,~]=bounded_control_allocation([c.Wa*B;sqrt(c.lambda_delta)*eye(3)],[c.Wa*(target-nat);sqrt(c.lambda_delta)*nom],dl,du);ta0=nat+B*delta;
g=x(7:10);A=p.cmg.h0*(-p.s0.*sin(g')+p.gxs0.*cos(g'));H=sum(p.cmg.h0*(p.s0.*cos(g')+p.gxs0.*sin(g')),2);prev_rate=(g-z.gamma)/.2; % measured-angle average, not instantaneous drive truth
lo=max(-p.cmg.gdot_max,z.nu-p.cmg.gddot_max*.2);hi=min(p.cmg.gdot_max,z.nu+p.cmg.gddot_max*.2);
if norm(target-ta0)<1e-5,nu0=min(max(zeros(4,1),lo),hi);else
[nu0,~]=bounded_control_allocation([-c.Wc*A;sqrt(c.lambda_nu)*eye(4);sqrt(c.lambda_change)*eye(4)],[c.Wc*(target-ta0);zeros(4,1);sqrt(c.lambda_change)*z.nu],lo,hi);end
nu=nu0;ta=ta0;eta=0;reason='disabled';extra=zeros(3,1);base_residual=target-(ta0-A*nu0);
if norm(H)>c.hon,z.unload=true;elseif norm(H)<c.hoff,z.unload=false;end
if c.unload
 reason='below_threshold';if air(1)<c.qmin,reason='low_qbar';elseif norm(base_residual)>c.residual_gate,reason='attitude_residual';elseif z.unload
 reason='no_feasible_candidate';desired=c.kh*max(0,norm(H)-c.hoff)*H/max(norm(H),eps);desired=desired*min(1,c.unload_tau_cap/max(norm(desired),eps));desired=min(max(desired,-z.extra-c.unload_slew),-z.extra+c.unload_slew);
 direction=-A'*((A*A'+.01*eye(3))\desired);
 for scale=[1 .5 .25 .125 .0625 .03125 .015625]
 n=min(max(nu0+scale*direction,lo),hi);ex=A*(n-nu0);ta_candidate=ta0+ex;
 [de,rr]=bounded_control_allocation(B,ta_candidate-nat,dl,du);if norm(rr)>.001,continue;end
 % Check the true 20 ms servo endpoint predicted from current measured deflection.
 [dbase,ib]=online_torque_feedback_step(p.vehicle,ta0,flight(7:9),air,x(4:6),flight(10:12));
 [dnew,inew]=online_torque_feedback_step(p.vehicle,ta_candidate,flight(7:9),air,x(4:6),flight(10:12));
 eb=online_servo_endpoint(p.vehicle,flight(7:9),dbase,.02);en=online_servo_endpoint(p.vehicle,flight(7:9),dnew,.02);
 if norm(B*(en-eb)-ex)>c.unload_compensation_tolerance,continue;end
 Hnew=momentum(g+.2*n);Hbase=momentum(g+.2*nu0);if norm(Hnew)>=norm(Hbase)-1e-7,continue;end
 nu=n;ta=ta_candidate;delta=de;extra=ex;eta=scale;reason='active';break;
 end
 end
end
residual=target-(ta-A*nu);integrate=abs(residual)<.01 | e.*residual<=0;z.I=min(max(z.I+.2*e.*integrate,-c.imax),c.imax);z.nu=nu;z.gamma=g;z.extra=extra;
z.tau_ref=target;z.tau_a_request=ta;z.tau_cmg_request=-A*nu;z.tau_total_request=ta-A*nu;z.residual=residual;z.H=H;z.eta=eta;z.unload_a=extra;z.unload_c=-extra;z.reason=reason;z.delta_star=delta;z.delta_lower=dl;z.delta_upper=du;z.nu_base=nu0;z.rate_observation=prev_rate;z.rate_observation_note='Mean rate over preceding 0.2s, constraints relative to previous command';z.qbar=air(1);z.low_qbar=air(1)<c.qmin;z.sigma_min=min(svd(A));z.axis_rate_box=sum(abs(A),2)*p.cmg.gdot_max;z.air=air;
u=[ta;nu];d=[nat+max(B,0)*dl+min(B,0)*du;nat+max(B,0)*du+min(B,0)*dl;eta;norm(residual)];
function h=momentum(gamma),h=sum(p.cmg.h0*(p.s0.*cos(gamma')+p.gxs0.*sin(gamma')),2);end
end
