function run_e3_horizon_pairs
% Incremental revision only. Never invokes the legacy batch/report entrypoint.
addpath('simulink');init_paths;addpath('simulink/paper','simulink/paper/v3');
old=fullfile(repo_root,'simulink/results/paper_experiments/20261105_redesign_v1');
out=fullfile(repo_root,'simulink/results/paper_experiments/20261106_e3_e4_revision');
for n={'config','reports','tables','figures','cache'},if ~isfolder(fullfile(out,n{1})),mkdir(fullfile(out,n{1}));end;end
Simulink.fileGenControl('set','CacheFolder',fullfile(out,'cache'),'CodeGenFolder',fullfile(out,'cache/codegen'),'createDir',true);
manifest=jsondecode(fileread(fullfile(old,'config/source_manifest.json')));checks={};
for j=1:numel(manifest.files)
 a=manifest.files(j);[~,name]=fileparts(a.path);
 if ismember(name,{'v3_build','v3_reference','native_mpc_sfun','native_online_rti_step','online_torque_feedback_step','bounded_control_allocation','vehicle_params','build_closedloop','ocp_qp_create','ocp_qp_solver_solve'})
 assert(strcmp(paper_hash(a.path),a.sha256),'revision:compatibility','Source changed: %s',a.path);checks{end+1}=a;
 end
end
s=v3_spec;s.group='E3';s.onset=35;s.duration=60;s.phase='formal';s.masked=false;s.pid=struct();s.pair_id='migration';
for name={'S','L','Z'}
 folder=fullfile(old,'E3/cases',['V3_' name{1}]);c=jsondecode(fileread(fullfile(folder,'case_config.json')));b=jsondecode(fileread(fullfile(folder,'build_config.json')));
 assert(c.onset==35 && c.duration==60 && c.transition==5 && strcmp(c.method,'MPC') && c.cmg && ~c.masked);
 assert(b.dt_ctrl==.2 && b.inner_period==.02 && b.fixed_step==.001 && b.sqp_limit==4);
 assert(strcmp(paper_hash(b.model_path),b.model_sha256) && strcmp(paper_hash(fullfile(folder,'reference_bank.mat')),b.bank_sha256));
 v=s;v.id=c.id;v.amplitude=c.amplitude;v.dts=c.dts;newbank=v3_reference(v,v.dts);bank=load(fullfile(folder,'reference_bank.mat'));
 for field={'x0','time','q_ref','omega_ref','tau_d'},key=field{1};assert(isequaln(bank.(key),newbank.(key)),'revision:bank','Incompatible bank %s/%s',c.id,key);end
end
paper_json(fullfile(out,'config/E3_compatibility.json'),struct('compatible',true,'sources',{checks},'reused_ids',{{'V3_S','V3_L','V3_Z'}},'bank_checks',{{'x0','time','q_ref','omega_ref','tau_d'}},'physical_step_s',.001,'inner_step_s',.02,'upper_step_s',.2,'sqp_limit',4));
plan={};for H=[5 8 12 16],for mode={'M','Z'}
 if H==5 && strcmp(mode{1},'M'),continue;end
 v=s;v.id=sprintf('V3_H%02d_%s',H,mode{1});v.pair_id=sprintf('H%02d',H);v.amplitude=10*strcmp(mode{1},'M');v.dts=make_paper_grid(s.dts,H);plan{end+1}=v;
end;end
paper_json(fullfile(out,'config/E3_incremental_matrix.json'),plan);
report_e4_1deg(old,out);
for k=1:numel(plan),m=v3_run_case(plan{k},out);assert(~isempty(m),'revision:run','Infrastructure/analysis failure: %s',plan{k}.id);end
report_e3_horizon_pairs(old,out);
fprintf('REVISION_COMPLETE: E3 new=7 reused=3; E4 new=0 reused=27\n');
end
