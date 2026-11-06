function [D,r,b]=e2r_native(folder)
% All precise metrics originate in full 1 ms Simulink outputs, never display CSV.
d=load(fullfile(folder,'sim_output.mat'),'r');r=d.r;b=load(fullfile(folder,'reference_bank.mat'));p=vehicle_params;
t=r.states.Time(:);x=squeeze(r.states.Data);assert(max(abs(diff(t)-.001))<1e-8);
[qr,ref]=paper_reference_series(b,t);q=paper_euler_quaternions(x(:,1:3));err=rad2deg(2*acos(min(1,abs(sum(q.*qr,2)))));
wr=interp1(b.time(:),squeeze(b.omega_ref(:,1,:))',t,'linear');td=interp1(b.time(:),squeeze(b.tau_d(:,1,:))',t,'linear');
flight=align(r.flight_servo_measured,t,'previous');air=align(r.flight_conditions,t,'linear');ta=align(r.tau_actual,t,'linear');tc=align(r.cmg_torque,t,'linear');H=align(r.cmg_H,t,'linear');nu=align(r.nu_actual,t,'previous');ac=align(r.cmg_acceleration,t,'previous');delta=align(r.delta_actual,t,'previous');drate=align(r.delta_dot,t,'previous');cmd=align(r.commands,t,'previous');
natural=zeros(numel(t),3);support=zeros(numel(t),2);sig=zeros(numel(t),1);geomerr=zeros(numel(t),1);Bpitch=zeros(numel(t),1);
for k=1:numel(t)
 [nat,B]=online_aero_moment(p.vehicle,air(k,:)',x(k,4:6)',delta(k,:)');natural(k,:)=nat';Bpitch(k)=B(2,1);
 g=x(k,7:10);A=p.cmg.h0*(-p.s0.*sin(g)+p.gxs0.*cos(g));sig(k)=min(svd(A));lo=max(-p.cmg.gdot_max,nu(k,:)-p.cmg.gddot_max*.2);hi=min(p.cmg.gdot_max,nu(k,:)+p.cmg.gddot_max*.2);c=-A(2,:);support(k,:)=[sum(max(c,0).*hi+min(c,0).*lo),-sum(max(c,0).*lo+min(c,0).*hi)];geomerr(k)=norm((-A*nu(k,:)')'-tc(k,:));
end
assert(max(geomerr)<1e-7);gyro=-cross(x(:,4:6),H,2);rigid=-cross(x(:,4:6),(p.J*x(:,4:6)')',2);net=ta+tc+td+gyro+rigid;wdot=(p.J\net')';
names={'time_s','roll_deg','pitch_deg','yaw_deg','ref_roll_deg','ref_pitch_deg','ref_yaw_deg','attitude_error_deg','p_radps','q_radps','r_radps','ref_p_radps','ref_q_radps','ref_r_radps','g1','g2','g3','g4','nu1','nu2','nu3','nu4','acc1','acc2','acc3','acc4','Hx','Hy','Hz','Hnorm','sigma_min','pitch_positive_support_Nm','pitch_negative_support_Nm','aero_x','aero_y','aero_z','cmg_x','cmg_y','cmg_z','natural_x','natural_y','natural_z','disturbance_x','disturbance_y','disturbance_z','gyro_x','gyro_y','gyro_z','rigid_x','rigid_y','rigid_z','net_x','net_y','net_z','qdot_radps2','delta1','delta2','delta3','delta_dot1','delta_dot2','delta_dot3','qbar_Pa','speed_mps','alpha_deg','height_m','aero_cmd_x','aero_cmd_y','aero_cmd_z','nu_cmd1','nu_cmd2','nu_cmd3','nu_cmd4','Bpitch_Nm_per_rad'};
D=array2table([t rad2deg(x(:,1:3)) ref err x(:,4:6) wr x(:,7:10) nu ac H vecnorm(H,2,2) sig support ta tc natural td gyro rigid net wdot(:,2) delta drate air(:,1:2) rad2deg(air(:,3)) -flight(:,3) cmd Bpitch],'VariableNames',names);
D.pitch_error_deg=D.ref_pitch_deg-D.pitch_deg;D.pitch_aero_surface_Nm=D.aero_y-D.natural_y;D.pitch_control_total_Nm=D.aero_y+D.cmg_y;
D.cmg_rate_sat=any(abs(nu)>=p.cmg.gdot_max-1e-5,2);D.cmg_accel_sat=any(abs(ac)>=p.cmg.gddot_max-1e-5,2);D.surface_angle_sat=any(abs(delta)>=p.vehicle.delta_max(:)'-1e-5,2);D.surface_rate_sat=any(abs(drate)>=p.vehicle.delta_dot_max(:)'-1e-5,2);
end
function a=align(ts,t,method)
a=interp1(ts.Time,squeeze(ts.Data),t,method,'extrap');
end
