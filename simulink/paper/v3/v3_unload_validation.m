function v3_unload_validation(out,cfg)
records={};for trial=1:3,s=v3_spec;s.group='UNLOAD_VALIDATION';s.phase='validation';s.id=sprintf('VU_%d',trial);s.duration=20;s.amplitude=0;s.gamma0=[pi/2;pi/2;0;0];s.method='PID_surface_first';s.pid=cfg;s.pid.unload=trial~=1;if trial==3,s.onset=0;s.transition=.5;s.amplitude=8;end;m=v3_run_case(s,out);assert(~isempty(m),'v3:unload','Missing physical validation');records{end+1}=m;end
% Acceptance is deliberately multidimensional; no claim of later benefit is made here.
a=records{2};b=records{1};valid=a.complete && b.complete && ~ismember(a.status,{'MODEL_DOMAIN_FAIL','CONTROLLER_FAIL'}) && ~ismember(b.status,{'MODEL_DOMAIN_FAIL','CONTROLLER_FAIL'});
observed=valid && a.unload_active_ticks>0 && a.H_final_Nms<a.H_initial_Nms && a.H_final_Nms<b.H_final_Nms;cost=a.rmse_full_deg-b.rmse_full_deg;
result=struct('completed',true,'physical_response_observed',observed,'rms_attitude_cost_deg',cost,'no_unload_H_final',b.H_final_Nms,'unload_H_final',a.H_final_Nms,'automatic_pause_observed',records{3}.unload_paused_ticks>0,'full_mechanism_acceptance',false,'note','Actual compensation and later maneuver benefit assessed from paired true torque traces and E2 M1/M2; H decrease alone never gives full acceptance');paper_json(fullfile(out,'calibration','unload_validation.json'),result);
% E4 only needs the chain allocator and frozen no-unload PID, not a positive unloading-benefit claim.
end
