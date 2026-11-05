function v3_compact_derived(out)
files=dir(fullfile(out,'**','metrics.json'));for k=1:numel(files),folder=files(k).folder;stpath=fullfile(folder,'run_status.json');if ~isfile(stpath) || isfile(fullfile(folder,'.run.lock')),continue;end;st=jsondecode(fileread(stpath));if strcmp(st.status,'RUNNING'),continue;end
for name={'timeseries.csv','domain_diagnostics.csv'},path=fullfile(folder,name{1});if isfile(path),T=readtable(path);if height(T)>1 && median(diff(T{:,1}))<.01,v3_write_csv(T,path);end;end;end
fprintf('V3_COMPACT %d %s\n',k,folder);
end
end
