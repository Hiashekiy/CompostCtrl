function report_five_sqp_performance(folders)
% Per-cycle cost, QP count and stop-reason statistics for online 5-round SQP runs.
% folders: explicit L2_* result folder names under results/online_authority, newest
% first by default. Writes JSON/CSV next to results/online_authority/five_sqp.
root=repo_root;base=fullfile(root,'simulink','results','online_authority');
out=fullfile(base,'five_sqp');if ~exist(out,'dir'),mkdir(out);end
if nargin<1
 folders={};
 for f=dir(base)'
  if f.isdir && ~isempty(regexp(f.name,'^L2_\d+s_\d{8}_\d{6}$','once'))
   folders{end+1}=f.name; %#ok<AGROW>
  end
 end
 folders=sort(folders);
 if numel(folders)>2,folders=folders(end-1:end);end
end
rows={};
for f=1:numel(folders)
 folder=fullfile(base,folders{f});
 for arm={'A','B'}
  m='NearSpace_CMG_FlightControl';if strcmp(arm{1},'A'),m=[m '_SurfaceOnly'];end
  evidence=dir(fullfile(folder,[m '*_authority.mat']));
  if strcmp(arm{1},'B'),evidence=evidence(~contains({evidence.name},'SurfaceOnly'));end
  if isempty(evidence),continue;end
  d=load(fullfile(folder,evidence(end).name));infos=d.diagnostics;
  elapsed=cellfun(@(i)i.elapsed_s,infos)';times=d.control_times(:)';
  stop=cellfun(@(i)stop_reason(i),infos,'UniformOutput',false);
  qp_num=cellfun(@(i)count_field(i,'qp_solved'),infos)';
  iters=cellfun(@(i)i.iterations,infos)';
  accepted=cellfun(@(i)count_field(i,'accepted_steps'),infos)';
  rejected=cellfun(@(i)count_field(i,'rejected_steps'),infos)';
  restor=cellfun(@(i)count_field(i,'restoration_calls'),infos)';
  certs=cellfun(@(i)count_field(i,'certificate_count'),infos)';
  qp_total=cellfun(@(i)sum_optional(i,'qp_total_time_s'),infos)';
  qp_solver=cellfun(@(i)sum_optional(i,'qp_solver_time_s'),infos)';
  rollout=cellfun(@(i)sum_optional(i,'rollout_time_s'),infos)';
  fallbacks=sum(cellfun(@(i)i.fallback,infos));
  period=median(diff(times));
  row=struct('folder',folders{f},'arm',arm{1},'cycles',numel(infos), ...
   'duration_s',times(end),'fallbacks',fallbacks, ...
   'control_s_mean',mean(elapsed),'control_s_median',median(elapsed), ...
   'control_s_p95',percentile(elapsed,.95),'control_s_max',max(elapsed), ...
   'control_period_s',period,'over_budget_cycles',sum(elapsed>period), ...
   'qp_per_cycle_mean',mean(qp_num),'qp_per_cycle_max',max(qp_num),'qp_total',sum(qp_num), ...
   'rounds_per_cycle_mean',mean(iters),'rounds_per_cycle_max',max(iters), ...
   'accepted_per_cycle_mean',mean(accepted),'rejected_steps',sum(rejected), ...
   'restoration_calls',sum(restor),'certificates',sum(certs), ...
   'qp_total_s_mean',mean(qp_total),'qp_solver_s_mean',mean(qp_solver), ...
   'rollout_s_mean',mean(rollout), ...
   'qp_share',sum(qp_total)/sum(elapsed),'rollout_share',sum(rollout)/sum(elapsed));
  names=unique(stop,'stable');
  for k=1:numel(names)
   field=['stop_' regexprep(names{k},'[^A-Za-z0-9]','_')];
   row.(field)=sum(strcmp(stop,names{k}));
  end
  rows{end+1}=row; %#ok<AGROW>
 end
end
if isempty(rows),fprintf('FIVE_SQP_PERFORMANCE_NONE\n');return;end
names={};for k=1:numel(rows),names=union(names,fieldnames(rows{k}),'stable');end
for k=1:numel(rows)
 for j=1:numel(names)
  if ~isfield(rows{k},names{j}),rows{k}.(names{j})=0;end
 end
 rows{k}=orderfields(rows{k},names);
end
rows=cell2mat(rows);
table_=struct2table(rows);writetable(table_,fullfile(out,'five_sqp_performance.csv'));
fid=fopen(fullfile(out,'five_sqp_performance.json'),'w');
fprintf(fid,'%s',jsonencode(rows,'PrettyPrint',true));fclose(fid);
disp(table_(:,{'folder','arm','cycles','fallbacks','control_s_mean','control_s_median', ...
 'control_s_p95','control_s_max','over_budget_cycles','qp_per_cycle_mean','qp_per_cycle_max', ...
 'rounds_per_cycle_mean','accepted_per_cycle_mean','rejected_steps'}));
fprintf('FIVE_SQP_PERFORMANCE %s\n',out);
end
function value=stop_reason(info)
value='';if isfield(info,'early_stop_reason'),value=info.early_stop_reason;end
if isempty(value),value='unspecified';end
value=regexprep(value,'[: ].*','');
end
function value=count_field(info,name)
value=0;if isfield(info,name),value=info.(name);end
end
function value=sum_optional(info,name)
value=0;if isfield(info,name) && ~isempty(info.(name)),value=sum(info.(name));end
end
function value=percentile(data,p)
data=sort(data(:));if isempty(data),value=0;return;end
value=data(max(1,min(numel(data),ceil(p*numel(data)))));
end
