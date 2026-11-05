function v3_write_csv(T,path)
% Derived display CSV at 20ms; preserve all column extrema and endpoints.
% Full 1ms native samples remain immutable in sim_output.mat; metrics use them all.
n=height(T);stride=max(1,round(.02/median(diff(T{:,1}))));idx=unique([1:stride:n,n]);for j=2:width(T),[~,lo]=min(T{:,j});[~,hi]=max(T{:,j});idx=union(idx,[lo hi]);end
temp=[path '.compact.csv'];writetable(T(idx,:),temp);movefile(temp,path,'f');
paper_json(fullfile(fileparts(path),'csv_export.json'),struct('native_output','sim_output.mat','native_period_s',.001,'derived_CSV_period_s',.02,'all_column_extrema_and_final_sample_preserved',true,'performance_metrics_use_full_native_samples',true));
end
