function inspect_e2_revision
addpath('simulink');init_paths;addpath('simulink/paper','simulink/paper/v3','simulink/paper/v3/e2_revision');
old=fullfile(repo_root,'simulink/results/paper_experiments/20261105_redesign_v1');out=fullfile(repo_root,'simulink/results/paper_experiments/20261106_e2_unload_revision');
for n={'config','reports','tables','figures','cache'},if ~isfolder(fullfile(out,n{1})),mkdir(fullfile(out,n{1}));end;end
diagnose_e2_c2(old,out);
% Save a precise, full-state basis for scenario feasibility; no controller modified.
for task=1:2
 folder=fullfile(old,'E2/cases',sprintf('V2_M%d_1',task));[D,r,b]=e2r_native(folder);save(fullfile(out,'config',sprintf('M%d_native_basis.mat',task)),'D','r','b','folder','-v7.3');
 f=dir(fullfile(folder,'pid_diagnostics.mat'));d=load(fullfile(folder,f(1).name));paper_json(fullfile(out,'config',sprintf('M%d_pid_fields.json',task)),struct('raw_fields',{r.who},'pid_fields',{fieldnames(d.history{1})}));
end
fprintf('E2_READONLY_INSPECTION_COMPLETE\n');
end
