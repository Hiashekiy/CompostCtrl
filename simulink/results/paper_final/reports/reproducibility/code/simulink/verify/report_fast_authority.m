function report_fast_authority
% Aggregates the certificate-ablation runs: acceptance of the analytic certificate,
% per-phase timing, cycle-time distribution and attitude tracking. Reads saved results only.
root=repo_root;base=fullfile(root,'simulink','results','online_authority','fast_certificate','ablation');
bank=load(fullfile(root,'simulink','data','online_plan_bank.mat'));
d=dir(base);rows={};names={};
for k=1:numel(d)
 if ~d(k).isdir||strncmp(d(k).name,'.',1),continue;end
 file=fullfile(base,d(k).name,'summary.json');
 if ~isfile(file),continue;end
 s=jsondecode(fileread(file));
 row=struct('tag',d(k).name,'certificate_mode',s.certificate_mode,'arm',s.arm, ...
  'duration_s',s.duration_s,'cycles',s.cycles,'fallbacks',s.fallbacks, ...
  'cycle_ms_mean',s.cycle_ms_mean,'cycle_ms_p50',s.cycle_ms_p50,'cycle_ms_p95',s.cycle_ms_p95, ...
  'cycle_ms_p99',s.cycle_ms_p99,'cycle_ms_max',s.cycle_ms_max,'over_200ms',s.over_200ms, ...
  'over_200ms_steady',s.over_200ms_steady,'qp_per_cycle',s.qp_per_cycle_mean, ...
  'box_ms',phase_of(s,'box'),'servo_certificate_ms',phase_of(s,'servo_certificate'), ...
  'translation_ms',phase_of(s,'translation'),'allocation_ms',phase_of(s,'allocation'), ...
  'analytic_stages',analytic_share(s));
 attitude=attitude_of(fullfile(base,d(k).name),bank);
 row.attitude_rms_deg=attitude.rms;row.attitude_peak_deg=attitude.peak;
 row.rms_0_15=attitude.seg(1);row.rms_15_30=attitude.seg(2);
 row.rms_30_45=attitude.seg(3);row.rms_45_60=attitude.seg(4);
 rows{end+1}=row; %#ok<AGROW>
 names=union(names,fieldnames(row),'stable');
end
if isempty(rows),fprintf('FAST_REPORT_NONE\n');return;end
for k=1:numel(rows)
 for j=1:numel(names),if ~isfield(rows{k},names{j}),rows{k}.(names{j})=NaN;end,end
 rows{k}=orderfields(rows{k},names);
end
rows=cell2mat(rows);t=struct2table(rows);
writetable(t,fullfile(base,'fast_authority_comparison.csv'));
fid=fopen(fullfile(base,'fast_authority_comparison.json'),'w');
fprintf(fid,'%s',jsonencode(rows,'PrettyPrint',true));fclose(fid);
disp(t(:,{'tag','certificate_mode','arm','duration_s','attitude_rms_deg','fallbacks', ...
 'cycle_ms_mean','cycle_ms_p95','cycle_ms_max','over_200ms','box_ms','servo_certificate_ms', ...
 'analytic_stages'}));
fprintf('FAST_REPORT_DONE %s\n',base);
end
function value=phase_of(s,name)
value=NaN;
if ~isfield(s,'phase_ms_mean'),return;end
names=s.phase_ms_mean.names;values=s.phase_ms_mean.values;
if ischar(names),names={names};end
values=values(:)';
index=find(strcmp(names,name),1);
if ~isempty(index),value=values(index);end
end
function value=analytic_share(s)
value=0;
if ~isfield(s,'certificate_type_counts'),return;end
names=s.certificate_type_counts.names;counts=s.certificate_type_counts.counts;
if ischar(names),names={names};end
total=sum(counts);
analytic=0;
for k=1:numel(names)
 if contains(names{k},'analytic'),analytic=analytic+counts(k);end
end
if total>0,value=analytic/total;end
end
function out=attitude_of(folder,bank)
out=struct('rms',NaN,'peak',NaN,'seg',nan(1,4));
files=dir(fullfile(folder,'*_closedloop.mat'));
if isempty(files),return;end
d=load(fullfile(folder,files(1).name));r=d.r;t=r.states.Time;
x=squeeze(r.states.Data);if size(x,1)~=numel(t),x=x';end
q=eul2quat(x(:,[3 2 1]));
ref=interp1(bank.time(:),squeeze(bank.q_ref(:,1,:))',t,'linear');
ref=ref./vecnorm(ref,2,2);
err=rad2deg(2*acos(min(1,abs(sum(q.*ref,2)))));
out.rms=sqrt(mean(err.^2));out.peak=max(err);
edges=[0 15 30 45 61];
for k=1:4
 mask=t>=edges(k)&t<edges(k+1);
 if any(mask),out.seg(k)=sqrt(mean(err(mask).^2));end
end
end
