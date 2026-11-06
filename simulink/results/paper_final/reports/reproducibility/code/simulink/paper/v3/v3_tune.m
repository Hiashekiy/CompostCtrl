function cfg=v3_tune(out)
folder=fullfile(out,'calibration');if ~isfolder(folder),mkdir(folder);end;frozen=fullfile(folder,'pid_frozen.json');if isfile(frozen),cfg=jsondecode(fileread(frozen));return;end
candidates=[];for wn=[.35 .7 1.2],for zeta=[.7 1 1.3],candidates(end+1,:)=[wn,zeta];end;end
best=inf;cfg=[];scores={};k=1;bestpair=[];
while k<=size(candidates,1)
 wn=candidates(k,1);zeta=candidates(k,2);c=v3_pid_config(wn,zeta);values=[];
 for scenario=1:4,s=v3_spec;s.id=sprintf('VT_%02d_%d',k,scenario);s.group='CALIBRATION';s.phase='calibration';s.method='PID_surface_first';s.pid=c;s.duration=40;amps=[5 8 12 5];transitions=[4 8 12 8];s.amplitude=amps(scenario);s.transition=transitions(scenario);if scenario==4,s.events=[8 8 3 5 0];end
 m=v3_run_case(s,out);if isempty(m) || ~m.complete || ismember(m.status,{'MODEL_DOMAIN_FAIL','CONTROLLER_FAIL'}),score=1e6;else,score=m.rmse_full_deg+.15*m.peak_attitude_error_deg+.001*m.actual_allocation_residual_rms_Nm+.01*m.surface_sat_pct+10*~strcmp(m.status,'SUCCESS');end;values(end+1)=score;
 end
 value=mean(values);scores{end+1}=struct('candidate',k,'wn',wn,'zeta',zeta,'score',value,'scenario_scores',values);if value<best,best=value;cfg=c;bestpair=[wn zeta];end
 if k==9,assert(~isempty(cfg) && best<1e6,'v3:tune','No completed admissible calibration candidate');w=bestpair(1);z=bestpair(2);candidates=[candidates;w*.8,z;w*1.2,z;w,z*.8;w,z*1.2];end
 paper_json(fullfile(folder,'tuning_scores.json'),scores);paper_json(fullfile(folder,'candidate_grid.json'),candidates);k=k+1;
end
assert(best<1e6);paper_json(frozen,cfg);save(fullfile(folder,'pid_frozen.mat'),'cfg','best','scores');paper_json(fullfile(folder,'freeze.json'),struct('pid_step_sha256',paper_hash(which('v3_pid_step')),'bounded_solver_sha256',paper_hash(which('bounded_control_allocation')),'selection_score',best,'calibration_scenarios','5deg4s,8deg8s,12deg12s,coupled3roll5pitch8s; duration40s','objective','RMSE+.15peak+.001actualTorqueResidual+.01surfaceSat+10trackingFailure; invalid=1e6'));
end
