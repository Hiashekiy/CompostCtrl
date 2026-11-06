function paper_finish(stamp,force)
addpath('simulink');init_paths;addpath('simulink/paper');out=fullfile(repo_root,'simulink','results','paper_experiments',stamp);
if nargin<2,force=false;end;signature=[paper_hash('simulink/paper/evaluate_paper_runs.m') paper_hash('simulink/paper/paper_extra_metrics.m')];marker=fullfile(out,'batch_completed.json');if ~force && isfile(marker),done=jsondecode(fileread(marker));if isfield(done,'source_signature') && strcmp(done.source_signature,signature),return;end;end
run_paper_followup(out);
try,paper_regression(out);catch err,paper_json(fullfile(out,'regression_error.json'),struct('error',getReport(err)));end
% Re-analyze saved raw logs using the latest metric implementation.
files=dir(fullfile(out,'E*','*','sim_output.mat'));for k=1:numel(files),folder=files(k).folder;try,d=load(fullfile(folder,'sim_output.mat'));s=jsondecode(fileread(fullfile(folder,'case_config.json')));metric=evaluate_paper_runs(d.r,s,folder);statusfile=fullfile(folder,'run_status.json');st=jsondecode(fileread(statusfile));if ~isfield(st,'legacy_status'),st.legacy_status=st.status;end;st.status=metric.status;st.criteria_version=metric.criteria_version;paper_json(statusfile,st);paper_extra_metrics(d.r,s,folder);catch err,paper_json(fullfile(folder,'analysis_error.json'),struct('error',getReport(err)));end;end
run_paper_suite('report',stamp);paper_comparison_figures(out);paper_final_report(out);copyfile(fullfile(out,'四组论文实验结果.md'),fullfile(repo_root,'docs','simulink','论文四组实验_实施与结果.md'));paper_json(fullfile(out,'batch_completed.json'),struct('source_signature',signature,'batch_finished',true,'paper_acceptance_complete',false,'note','Every configured attempt recorded; inspect method acceptance, infrastructure/domain failures and missing deliverables separately.'));
end







