function paper_reference_manifest(out)
p=jsondecode(fileread(fullfile(out,'preflight.json')));files=dir(fullfile(out,'E*','*','case_config.json'));entries={};
for k=1:numel(files)
folder=files(k).folder;s=jsondecode(fileread(fullfile(folder,files(k).name)));bank=fullfile(folder,'reference_bank.mat');if ~isfile(bank),continue;end
m='NearSpace_CMG_FlightControl';if ~s.cmg,m=[m '_SurfaceOnly'];end;m=[m '_' s.id];model=fullfile(repo_root,'simulink','cases',[m '.slx']);e=struct('case_id',s.id,'physical_base','S00 archived initial state and native vehicle/environment','reference_profile',s.profile,'prediction_dts',s.dts,'task_onset',s.onset,'seed',0,'source_git_sha',p.git_sha,'input_bank_path',bank,'input_bank_sha256',paper_hash(bank),'model_path',model,'model_sha256','');if isfile(model),e.model_sha256=paper_hash(model);end
if strcmp(s.profile,'archive') && isfield(s,'archived_task'),e.physical_base=s.archived_task;end
entries{end+1}=e;
end
paper_json(fullfile(out,'reference_manifest.json'),entries);
end
