function run_e2_revision
addpath('simulink');init_paths;addpath('simulink/paper','simulink/paper/v3','simulink/paper/v3/e2_revision');
old=fullfile(repo_root,'simulink/results/paper_experiments/20261105_redesign_v1');out=fullfile(repo_root,'simulink/results/paper_experiments/20261106_e2_unload_revision');
supplement_c2_dynamics(out,vehicle_params);
gate=jsondecode(fileread(fullfile(out,'reports/precheck_acceptance.json')));assert(gate.passed,'e2r:precheck','Prechecks did not pass; do not run formal six');
scenario=jsondecode(fileread(fullfile(out,'config/scenario_feasibility_gate.json')));assert(scenario.passed);
cfg=jsondecode(fileread(fullfile(old,'calibration/pid_frozen.json')));plan={};
for task=1:2,for method=1:3
 s=v3_spec;s.group='E2';s.id=sprintf('E2R_M%d_%d',task,method);s.pair_id=sprintf('E2R_M%d',task);s.onset=35;s.transition=5;s.duration=60;s.amplitude=5;s.pid=cfg;s.phase='formal';
 if task==1,s.gamma0=[pi/2;pi/2;0;0];else,s.gamma0=ones(4,1)*1.5;end
 if method==1,s.method='PID_NoUnload';s.pid.unload=false;elseif method==2,s.method='PID_DynamicUnload';s.pid.unload=true;else,s.method='MPC';s.pid=struct();end
 assert(abs(sum(s.dts)-23.2)<1e-10);plan{end+1}=s;
end;end
paper_json(fullfile(out,'config/formal_matrix.json'),plan);paper_json(fullfile(out,'config/frozen_pid.json'),cfg);
Simulink.fileGenControl('set','CacheFolder',fullfile(out,'cache'),'CodeGenFolder',fullfile(out,'cache/codegen'),'createDir',true);
for k=1:numel(plan)
 m=e2r_run_case(plan{k},out);assert(~isempty(m),'e2r:infrastructure','Repair infrastructure before proceeding, preserving failure');
 folder=fullfile(out,'E2/cases',plan{k}.id);[D,~,~]=e2r_native(folder);save(fullfile(folder,'native_diagnostic.mat'),'D','-v7.3');
 if startsWith(plan{k}.method,'PID'),e2r_unload_log(folder);end
end
report_e2_revision(old,out);
fprintf('E2_REVISION_FORMAL_COMPLETE new=6 C2 reruns=0\n');
end
