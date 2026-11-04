function report_direct_servo_s0(folders)
% S0 accounting for the direct-servo study: per-phase cost of a control period in
% the CURRENT (torque-level) controller, so the new architecture is measured the
% same way. Reads the saved per-cycle diagnostics only; never rewrites them.
root=repo_root;base=fullfile(root,'simulink','results','online_authority');
out=fullfile(base,'direct_servo');if ~exist(out,'dir'),mkdir(out);end
if nargin<1 || isempty(folders),folders=newest_runs(base,2);end
phases={'air','aero','reachability','box','certificate','allocation','propagation','translation'};
rows={};
for f=1:numel(folders)
 folder=fullfile(base,folders{f});
 for arm={'A','B'}
  m='NearSpace_CMG_FlightControl';if strcmp(arm{1},'A'),m=[m '_SurfaceOnly'];end
  evidence=dir(fullfile(folder,[m '*_authority.mat']));
  if strcmp(arm{1},'B'),evidence=evidence(~contains({evidence.name},'SurfaceOnly'));end
  if isempty(evidence),continue;end
  d=load(fullfile(folder,evidence(end).name));infos=d.diagnostics;
  elapsed=cellfun(@(i)i.elapsed_s,infos);
  cert=cellfun(@(i)isfield(i,'authority') && ~isempty(i.authority) && isfield(i.authority,'phase_time_s'),infos);
  row=struct('folder',folders{f},'arm',arm{1},'cycles',numel(infos), ...
   'fallbacks',sum(cellfun(@(i)i.fallback,infos)), ...
   'control_s_mean',mean(elapsed),'control_s_median',median(elapsed), ...
   'control_s_p95',percentile(elapsed,.95),'control_s_max',max(elapsed), ...
   'over_budget_cycles',sum(elapsed>.2),'over_budget_steady',sum(elapsed(3:end)>.2), ...
   'qp_solved_mean',mean(cellfun(@(i)field_or(i,'qp_solved'),infos)), ...
   'qp_solved_max',max(cellfun(@(i)field_or(i,'qp_solved'),infos)), ...
   'qp_solver_s_mean',mean(cellfun(@(i)sum_list(i,'qp_solver_time_s'),infos)), ...
   'qp_total_s_mean',mean(cellfun(@(i)sum_list(i,'qp_total_time_s'),infos)), ...
   'rollout_s_mean',mean(cellfun(@(i)sum_list(i,'rollout_time_s'),infos)));
  for k=1:numel(phases)
   value=zeros(1,numel(infos));
   for j=1:numel(infos)
    if cert(j),value(j)=infos{j}.authority.phase_time_s.(phases{k});end
   end
   row.(['phase_' phases{k} '_s_mean'])=mean(value);
  end
  rows{end+1}=row; %#ok<AGROW>
 end
end
if isempty(rows),error('report_direct_servo_s0: no diagnostics found');end
names={};for k=1:numel(rows),names=union(names,fieldnames(rows{k}),'stable');end
for k=1:numel(rows)
 for j=1:numel(names),if ~isfield(rows{k},names{j}),rows{k}.(names{j})=0;end,end
 rows{k}=orderfields(rows{k},names);
end
rows=cell2mat(rows);t=struct2table(rows);
writetable(t,fullfile(out,'s0_phase_timing.csv'));
fid=fopen(fullfile(out,'s0_phase_timing.json'),'w');fprintf(fid,'%s',jsonencode(rows,'PrettyPrint',true));fclose(fid);
disp(t(:,{'folder','arm','cycles','fallbacks','control_s_mean','control_s_p95','control_s_max', ...
 'over_budget_cycles','over_budget_steady','qp_solved_mean','qp_solver_s_mean','qp_total_s_mean'}));
for k=1:numel(rows)
 fprintf('%s %s phases(ms): ',rows(k).folder,rows(k).arm);
 for j=1:numel(phases),fprintf('%s=%.2f ',phases{j},1000*rows(k).(['phase_' phases{j} '_s_mean']));end
 fprintf('\n');
end
fprintf('DIRECT_SERVO_S0 %s\n',out);
end
function folders=newest_runs(base,count)
d=dir(base);candidates={};
for k=1:numel(d)
 if d(k).isdir && ~isempty(regexp(d(k).name,'^L2_60s_\d{8}_\d{6}$','once')),candidates{end+1}=d(k).name;end %#ok<AGROW>
end
candidates=sort(candidates);folders=candidates(max(1,end-count+1):end);
end
function value=field_or(info,name)
value=0;if isfield(info,name),value=info.(name);end
end
function value=sum_list(info,name)
value=0;if isfield(info,name) && ~isempty(info.(name)),value=sum(info.(name));end
end
function value=percentile(data,p)
data=sort(data(:));value=data(max(1,min(numel(data),ceil(p*numel(data)))));
end
