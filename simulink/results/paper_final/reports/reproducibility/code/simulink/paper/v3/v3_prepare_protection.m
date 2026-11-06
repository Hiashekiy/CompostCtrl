function v3_prepare_protection(out)
target=fullfile(out,'config/protected_old_raw.json');if isfile(target),return;end
files=dir(fullfile(repo_root,'simulink/results/paper_experiments/20261105_paper_v2','**','sim_output.mat'));rows={};for k=1:numel(files),path=fullfile(files(k).folder,files(k).name);rows{end+1}=struct('path',path,'sha256',paper_hash(path));end;paper_json(target,rows);
end
