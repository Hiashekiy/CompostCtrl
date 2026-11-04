function diagnose_direct_servo_sqp(folders)
% Aggregate the ablation summaries into one comparison table (and CSV/JSON).
% Reads only saved summaries; no simulation.
root=repo_root;base=fullfile(root,'simulink','results','online_authority','direct_servo','acceptance_ablation');
if nargin<1||isempty(folders)
 d=dir(base);folders={};
 for k=1:numel(d),if d(k).isdir&&~strncmp(d(k).name,'.',1),folders{end+1}=d(k).name;end,end %#ok<AGROW>
end
rows={};names={};
for k=1:numel(folders)
 file=fullfile(base,folders{k},'summary.json');
 if ~isfile(file),continue;end
 s=jsondecode(fileread(file));
 row=struct('tag',folders{k},'acceptance_mode',s.acceptance_mode,'model_mode',s.model_mode, ...
  'arm',s.arm,'duration_s',s.duration_s,'cycles',s.cycles,'fallbacks',s.fallbacks, ...
  'accepted_mean',s.accepted_steps_mean,'zero_accept_cycles',s.cycles_with_zero_accept, ...
  'first_cmd_changed',s.first_command_changed_cycles,'first_cmd_max_rad',s.first_command_max_rad, ...
  'qp_per_cycle',s.qp_per_cycle_mean,'cycle_ms_mean',s.cycle_ms_mean,'cycle_ms_p95',s.cycle_ms_p95, ...
  'cycle_ms_max',s.cycle_ms_max,'over_200ms',s.over_200ms,'J_start',s.J_start,'J_end',s.J_end, ...
  'objective_increase',s.objective_increase);
 rows{end+1}=row; %#ok<AGROW>
 names=union(names,fieldnames(row),'stable');
end
if isempty(rows),fprintf('DIAGNOSE_NONE\n');return;end
for k=1:numel(rows)
 for j=1:numel(names),if ~isfield(rows{k},names{j}),rows{k}.(names{j})=0;end,end
 rows{k}=orderfields(rows{k},names);
end
rows=cell2mat(rows);t=struct2table(rows);
writetable(t,fullfile(base,'acceptance_comparison.csv'));
fid=fopen(fullfile(base,'acceptance_comparison.json'),'w');fprintf(fid,'%s',jsonencode(rows,'PrettyPrint',true));fclose(fid);
disp(t(:,{'tag','arm','duration_s','accepted_mean','zero_accept_cycles','first_cmd_changed', ...
 'qp_per_cycle','cycle_ms_mean','over_200ms','J_start','J_end'}));
fprintf('DIAGNOSE_DONE %s\n',base);
end
