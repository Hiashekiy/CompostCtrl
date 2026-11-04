function summarize_direct_servo_ablation_metrics
% Attitude tracking of every ablation run, computed from the saved closed-loop output.
% Kept separate from diagnose_direct_servo_sqp so the run summaries stay cheap to read.
root=repo_root;base=fullfile(root,'simulink','results','online_authority','direct_servo','acceptance_ablation');
bank=load(fullfile(root,'simulink','data','online_plan_bank.mat'));
d=dir(base);rows={};names={};
for k=1:numel(d)
 if ~d(k).isdir||strncmp(d(k).name,'.',1),continue;end
 folder=fullfile(base,d(k).name);
 files=dir(fullfile(folder,'*_closedloop.mat'));
 if isempty(files),continue;end
 x=load(fullfile(folder,files(1).name));r=x.r;t=r.states.Time;
 data=squeeze(r.states.Data);if size(data,1)~=numel(t),data=data';end
 q=eul2quat(data(:,[3 2 1]));
 ref=interp1(bank.time(:),squeeze(bank.q_ref(:,1,:))',t,'linear');
 ref=ref./vecnorm(ref,2,2);
 err=rad2deg(2*acos(min(1,abs(sum(q.*ref,2)))));
 segment=@(a,b) sqrt(mean(err(t>=a&t<b).^2));
 row=struct('tag',d(k).name,'attitude_rms_deg',sqrt(mean(err.^2)),'attitude_peak_deg',max(err), ...
  'rms_0_15',segment(0,15),'rms_15_30',segment(15,30),'rms_30_45',segment(30,45), ...
  'rms_45_60',segment(45,61));
 rows{end+1}=row; %#ok<AGROW>
 names=union(names,fieldnames(row),'stable');
end
if isempty(rows),fprintf('METRICS_NONE\n');return;end
for k=1:numel(rows)
 for j=1:numel(names),if ~isfield(rows{k},names{j}),rows{k}.(names{j})=0;end,end
 rows{k}=orderfields(rows{k},names);
end
rows=cell2mat(rows);t=struct2table(rows);
writetable(t,fullfile(base,'attitude_metrics.csv'));
fid=fopen(fullfile(base,'attitude_metrics.json'),'w');fprintf(fid,'%s',jsonencode(rows,'PrettyPrint',true));fclose(fid);
disp(t);
fprintf('METRICS_DONE %s\n',base);
end
