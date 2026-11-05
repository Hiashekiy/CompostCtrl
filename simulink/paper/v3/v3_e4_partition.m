function v3_e4_partition(stamp,amplitude)
addpath('simulink');init_paths;addpath('simulink/paper');addpath('simulink/paper/v3');out=fullfile(repo_root,'simulink/results/paper_experiments',stamp);assert(ismember(amplitude,[1 3 5]));Simulink.fileGenControl('set','CacheFolder',fullfile(out,'cache',sprintf('E4_A%d',amplitude)),'CodeGenFolder',fullfile(out,'cache',sprintf('E4_A%d',amplitude),'codegen'),'createDir',true);
frozen=jsondecode(fileread(fullfile(out,'calibration','freeze.json')));assert(strcmp(frozen.pid_step_sha256,paper_hash(which('v3_pid_step'))));p=jsondecode(fileread(fullfile(out,'config','E4_matrix.json')));if ~iscell(p),p=num2cell(p);end
if amplitude==3,p=p(end:-1:1);end % Opposite traversal lets the main worker fill low frequencies independently.
for k=1:numel(p),s=p{k};if s.amplitude~=amplitude,continue;end;m=v3_run_case(s,out);assert(~isempty(m),'v3:partition','Repair infrastructure/analysis failure before continuing');end
fprintf('V3_E4_PARTITION_COMPLETE amplitude=%g\n',amplitude);
end
