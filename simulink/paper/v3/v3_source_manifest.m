function v3_source_manifest(out)
paths=dir(fullfile(repo_root,'simulink','paper','v3','*.m'));rows={};for k=1:numel(paths),path=fullfile(paths(k).folder,paths(k).name);rows{end+1}=struct('path',path,'sha256',paper_hash(path));end
for name={'native_mpc_sfun','native_online_rti_step','online_torque_feedback_step','bounded_control_allocation','vehicle_params','v3_pid_step','v3_reference','build_closedloop'},path=which(name{1});rows{end+1}=struct('path',path,'sha256',paper_hash(path));end
hpipm_startup;for name={'ocp_qp_create','ocp_qp_solver_solve'},path=which(name{1});if ~isempty(path),rows{end+1}=struct('path',path,'sha256',paper_hash(path));end;end
p=jsondecode(fileread(fullfile(out,'config/preflight.json')));paper_json(fullfile(out,'config/source_manifest.json'),struct('upstream_baseline_git_sha',p.git_sha,'captured_at',char(datetime('now','TimeZone','UTC','Format','yyyy-MM-dd''T''HH:mm:ssXXX')),'role','Final source and postprocessing revision; baseline Git SHA does not claim new uncommitted v3 files existed in baseline. Per-case bank/model/PID implementation hashes bind actual runs.','files',{rows}));
end
