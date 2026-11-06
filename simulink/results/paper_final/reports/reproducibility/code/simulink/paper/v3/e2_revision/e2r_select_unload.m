function [nu,ta,d]=e2r_select_unload(g,actual_rate,previous_command,nu0,ta0,flight,omega,air,c)
% Largest feasible candidate along a momentum-descent ray; no fixed torque/slew cap.
p=vehicle_params;g=g(:);actual_rate=actual_rate(:);nu=nu0;ta=ta0;
d=struct('scale',0,'reason','no_descent_direction','extra_a',zeros(3,1),'extra_c',zeros(3,1),'requested_extra_Nm',0,'selected_extra_Nm',0,'compensation_error_endpoint_Nm',NaN,'surface_command_margin_rad',NaN,'H_base_next_Nms',NaN,'H_candidate_next_Nms',NaN,'H_delta_vs_base_Nms',0,'physical_ray_length_radps',0,'candidates_tested',0,'predicted_base_aero_Nm',zeros(3,1),'predicted_candidate_aero_Nm',zeros(3,1),'predicted_base_cmg_Nm',zeros(3,1),'predicted_candidate_cmg_Nm',zeros(3,1));
A=p.cmg.h0*(-p.s0.*sin(g')+p.gxs0.*cos(g'));
gb=g+.2*nu0;Hb=momentum(gb);Ab=p.cmg.h0*(-p.s0.*sin(gb')+p.gxs0.*cos(gb'));direction=-Ab'*(Hb/max(norm(Hb),eps));
d.H_base_next_Nms=norm(Hb);d.H_candidate_next_Nms=norm(Hb);
if norm(direction)<1e-10,return;end;direction=direction/norm(direction);
lo=max(max(-p.cmg.gdot_max,previous_command-p.cmg.gddot_max*.2),actual_rate-p.cmg.gddot_max*.2);
hi=min(min(p.cmg.gdot_max,previous_command+p.cmg.gddot_max*.2),actual_rate+p.cmg.gddot_max*.2);
if any(nu0<lo-1e-9 | nu0>hi+1e-9),d.reason='base_outside_measured_accel_window';return;end
positive=direction>1e-12;negative=direction< -1e-12;ray=min([(hi(positive)-nu0(positive))./direction(positive);(lo(negative)-nu0(negative))./direction(negative)]);
if isempty(ray)||ray<=1e-12,d.reason='actuator_limit';return;end
d.physical_ray_length_radps=ray;d.requested_extra_Nm=norm(A*(ray*direction));
[nat,B]=online_aero_moment(p.vehicle,air,omega,flight(7:9));[dl,du]=online_surface_reachability(p.vehicle,flight(7:9),.2,flight(10:12));
[ub,ib]=online_torque_feedback_step(p.vehicle,ta0,flight(7:9),air,omega,flight(10:12));eb=online_servo_endpoint(p.vehicle,flight(7:9),ub,.02);d.predicted_base_aero_Nm=nat+B*eb;d.base_surface_endpoint_rad=eb;
[gb20,nb20]=e2r_drive_endpoint(g,actual_rate,nu0,.02);Ab20=p.cmg.h0*(-p.s0.*sin(gb20')+p.gxs0.*cos(gb20'));d.predicted_base_cmg_Nm=-Ab20*nb20;
for scale=2.^-(0:12)
 d.candidates_tested=d.candidates_tested+1;n=nu0+scale*ray*direction;extra=A*(n-nu0);target=ta0+extra;
 [de,res]=bounded_control_allocation(B,target-nat,dl,du);if norm(res)>.001,d.reason='surface_allocation_infeasible';continue;end
 [un,info]=online_torque_feedback_step(p.vehicle,target,flight(7:9),air,omega,flight(10:12));en=online_servo_endpoint(p.vehicle,flight(7:9),un,.02);
 [gn20,nn20]=e2r_drive_endpoint(g,actual_rate,n,.02);An20=p.cmg.h0*(-p.s0.*sin(gn20')+p.gxs0.*cos(gn20'));candidate_cmg=-An20*nn20;
 compensation=max(norm(B*(en-eb)-extra),norm(B*(en-eb)+candidate_cmg-d.predicted_base_cmg_Nm));
 if ib.fallback || info.fallback,d.reason='inner_prediction_fallback';continue;end
 if compensation>c.unload_compensation_tolerance,d.reason='compensation_insufficient';continue;end
 Hn=momentum(g+.2*n);if norm(Hn)>=norm(Hb)-1e-7,d.reason='no_momentum_decrease';continue;end
 nu=n;ta=target;d.scale=scale;d.reason='active';d.extra_a=extra;d.extra_c=-extra;d.selected_extra_Nm=norm(extra);d.compensation_error_endpoint_Nm=compensation;d.surface_command_margin_rad=min(p.vehicle.delta_max(:)-abs(un));d.H_candidate_next_Nms=norm(Hn);d.H_delta_vs_base_Nms=norm(Hn)-norm(Hb);d.predicted_candidate_aero_Nm=nat+B*en;d.predicted_candidate_cmg_Nm=candidate_cmg;return;
end
end
function H=momentum(g)
p=vehicle_params;H=sum(p.cmg.h0*(p.s0.*cos(g')+p.gxs0.*sin(g')),2);
end
