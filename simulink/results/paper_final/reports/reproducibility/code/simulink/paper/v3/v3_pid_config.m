function c=v3_pid_config(wn,zeta)
p=vehicle_params;J=diag(p.J);c=struct('kp',J*wn^2,'ki',J*wn^2*.02,'kd',2*zeta*wn*J,'imax',.3,'lambda_delta',1e-8,'lambda_nu',1e-7,'lambda_change',1e-7,'Wa',eye(3),'Wc',eye(3),'unload',false,'hon',80,'hoff',60,'kh',.1,'qmin',1,'residual_gate',2,'unload_tau_cap',2,'unload_slew',.2,'unload_compensation_tolerance',.25);
end
