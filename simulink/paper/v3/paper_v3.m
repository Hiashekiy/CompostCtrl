function paper_v3(mode,stamp)
if nargin<1,mode='all';end;if nargin<2,stamp='20261105_redesign_v1';end
addpath('simulink');init_paths;addpath(fullfile(repo_root,'simulink','paper'));addpath(fullfile(repo_root,'simulink','paper','v3'));out=fullfile(repo_root,'simulink','results','paper_experiments',stamp);for name={'config','reports','tables','figures','cache'},if ~isfolder(fullfile(out,name{1})),mkdir(fullfile(out,name{1}));end;end
Simulink.fileGenControl('set','CacheFolder',fullfile(out,'cache',mode),'CodeGenFolder',fullfile(out,'cache',mode,'codegen'),'createDir',true);
if ismember(mode,{'report','analyze'}),if strcmp(mode,'analyze'),v3_verify(out);end;v3_report(out);return;end
if ~isfile(fullfile(out,'config','preflight.json')),paper_preflight(fullfile(out,'config'));end
v3_prepare_protection(out);
if strcmp(mode,'preflight'),return;end
v3_unit(fullfile(out,'config'));if strcmp(mode,'unit'),return;end
v3_smoke(out);if strcmp(mode,'smoke'),return;end
if ismember(mode,{'E1','E3'}),cfg=v3_pid_config(.7,1);else,cfg=v3_tune(out);if strcmp(mode,'tune'),return;end;v3_unload_validation(out,cfg);end
for group={'E2','E3','E1','E4'}
 g=group{1};if ~strcmp(mode,'all') && ~strcmp(mode,g),continue;end
 matrixpath=fullfile(out,'config',[g '_matrix.json']);if isfile(matrixpath),saved=jsondecode(fileread(matrixpath));if iscell(saved),plan=saved;else,plan=num2cell(saved);end;else,plan=v3_plan(g,cfg);paper_json(matrixpath,plan);end
 if strcmp(g,'E4')
 frozen=jsondecode(fileread(fullfile(out,'calibration','freeze.json')));assert(strcmp(frozen.pid_step_sha256,paper_hash(which('v3_pid_step'))),'Frozen PID implementation changed');
 % 27 separate full-duration PRECHECK cases: 3 amplitudes x 3 sentinel frequencies x 3 methods.
 for k=1:numel(plan),s=plan{k};if ismember(s.frequency,[.02 .1 .2]),s.group='E4_PRECHECK';s.phase='precheck';s.id=['P' s.id];m=v3_run_case(s,out);assert(~isempty(m),'v3:E4precheck','Repair infrastructure/analysis failure before formal sampling');end;end
 end
 for k=1:numel(plan),m=v3_run_case(plan{k},out);if isempty(m),v3_report(out);error('v3:repair','Infrastructure or analysis failure; repair and resume before continuing');end;end
 v3_report(out);
end
v3_report(out);
if strcmp(mode,'all'),v3_acceptance(out);end
end
