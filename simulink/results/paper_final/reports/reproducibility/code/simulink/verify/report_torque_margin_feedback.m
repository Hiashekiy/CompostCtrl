function report=report_torque_margin_feedback(duration,suffix)
% Aggregates the torque-margin / 20 ms torque-feedback ablation from the archived runs.
% Reads the saved runs only and writes new files into the same results folder.
%
% For every group (B0/M10/F20/MF10 and the optional M05/MF05) and every arm (A/B):
%   * three-axis target-torque RMS, absolute error RMS, relative RMS, peak error,
%     three-axis composite RMS, and the 0-15/15-30/30-45/45-60 s segment RMS
%   * attitude RMS/peak (same quaternion minimum-angle convention as reports 09-19),
%     body-rate error, and the closed-loop stability of the run
%   * command/actual deflection and rate saturation, command jitter, safe fallbacks
%   * MPC cycle time and inner-loop decision time
%   * the contrasts M10-B0, F20-B0, MF10-B0 and the interaction MF10-M10-F20+B0
if nargin<1,duration=60;end
if nargin<2,suffix='';end
root=repo_root;init_paths;
p=vehicle_params;v=p.vehicle;
base=fullfile(root,'simulink','results','online_authority','torque_margin_feedback');
bank=load(fullfile(root,'simulink','data','online_plan_bank.mat'));
groups={'B0','M10','F20','MF10','M05','MF05'};
rows={};entries={};report=struct('duration_s',duration,'suffix',suffix);
for g=1:numel(groups)
 for arm={'A','B'}
  tag=sprintf('%s_%s_%gs',groups{g},arm{1},duration);
  if ~isempty(suffix),tag=[tag '_' suffix];end
  folder=fullfile(base,tag);
  if ~isfolder(folder),continue;end
  file=fullfile(folder,sprintf('%s_closedloop.mat',arm{1}));
  if ~isfile(file),continue;end
  summary=jsondecode(fileread(fullfile(folder,'summary.json')));
  d=load(file);r=d.r;
  [t,cmd]=values(r.commands);[~,actual]=values(r.tau_actual);
  cmd=cmd(:,1:3);actual=actual(:,1:3);
  assert(numel(t)==numel(r.tau_actual.Time) && max(abs(t-r.tau_actual.Time(:)))<1e-12, ...
   'report:align','%s is not on one clock',tag);
  err=cmd-actual;
  rms=@(z)sqrt(mean(z.^2,1));
  cmd_rms=rms(cmd);err_rms=rms(err);err_max=max(abs(err),[],1);
  rel=100*err_rms./max(cmd_rms,realmin);
  [~,delta]=values(r.delta_actual);delta=delta(:,1:3);
  [~,rate]=values(r.delta_dot);rate=rate(:,1:3);
  [~,x]=values(r.states);
  % r is a Simulink.SimulationOutput object, so isfield() cannot see its logged
  % signals; probe the field instead.
  cmd_t=[];cmdlog=[];
  if has_signal(r,'delta_cmd')
   [cmd_t,cmdlog]=values(r.delta_cmd);cmdlog=cmdlog(:,1:3);
  end
  limit=v.delta_max(:)';rate_limit=v.delta_dot_max(:)';
  deflection_sat=100*mean(abs(delta)>=limit-1e-5,1);
  rate_sat=100*mean(abs(rate)>=rate_limit-1e-5,1);
  jitter=NaN(1,3);cmd_rate_sat=NaN(1,3);decisions=0;
  if ~isempty(cmdlog)
   % The command log is held on the 1 ms grid; only the change instants are actual
   % inner-loop decisions, so the command slew fraction is measured per decision.
   step=diff(cmdlog,1,1);moved=any(abs(step)>1e-12,2);packet=step(moved,:);
   decisions=size(packet,1);
   jitter=sum(abs(packet),1)./max(cmd_t(end)-cmd_t(1),realmin);
   if decisions>0
    cmd_rate_sat=100*mean(abs(packet)>=repmat(rate_limit*.02,decisions,1)-1e-9,1);
   end
  end
  % The 60 001-sample error trace is needed for the figures only; it is removed
  % before the JSON export so the summary stays a summary.
  attitude_full=attitude_error(t,x,bank);
  attitude=rmfield(attitude_full,'err_deg');
  torque=struct('cmd_rms_Nm',cmd_rms,'err_rms_Nm',err_rms,'rel_rms_pct',rel, ...
   'err_max_Nm',err_max,'composite_err_rms_Nm',rms(err(:)), ...
   'composite_rel_pct',100*rms(err(:))/max(rms(cmd(:)),realmin), ...
   'segments',segment_rms(t,err),'usable_axes',cmd_rms>1e-3);
  entry=struct('tag',tag,'group',groups{g},'arm',arm{1},'shrink_ratio',summary.shrink_ratio, ...
   'inner_mode',summary.inner_mode,'cycles',summary.cycles,'fallbacks',summary.fallbacks, ...
   'all_certified',summary.all_certified,'cycle_ms_mean',summary.cycle_ms_mean, ...
   'cycle_ms_p95',summary.cycle_ms_p95,'cycle_ms_p99',summary.cycle_ms_p99, ...
   'cycle_ms_max',summary.cycle_ms_max,'over_200ms',summary.over_200ms, ...
   'over_200ms_steady',summary.over_200ms_steady,'qp_per_cycle',summary.qp_per_cycle_mean, ...
   'accepted_steps_mean',summary.accepted_steps_mean, ...
   'rejected_steps_total',summary.rejected_steps_total, ...
   'accepted_steps_zero_cycles',summary.accepted_steps_zero_cycles, ...
   'shrink_restored_stages',summary.shrink_restored_stages, ...
   'shrink_stages',summary.shrink_stages,'shrink_max_abs_Nm',summary.shrink_max_abs_Nm, ...
   'box_margin_min',summary.box_margin_min,'box_margin_mean',summary.box_margin_mean, ...
   'torque',torque,'attitude',attitude,'surface',struct( ...
    'deflection_saturation_pct',deflection_sat,'rate_saturation_pct',rate_sat, ...
    'deflection_abs_max_rad',max(abs(delta),[],1),'rate_abs_max_radps',max(abs(rate),[],1), ...
    'command_jitter_radps',jitter,'command_rate_saturation_pct',cmd_rate_sat), ...
   'inner',summary.inner,'command_decisions',decisions,'wall_s',summary.wall_s);
  entries{end+1}=entry; %#ok<AGROW>
  rows=append_rows(rows,entry);
  plot_group(folder,tag,arm{1},t,cmd,actual,delta,cmd_t,cmdlog,attitude_full.err_deg,limit);
 end
end
if isempty(entries)
 fprintf('TMF_REPORT_NONE for duration %gs\n',duration);return;
end
report.groups=cell2mat(entries);report.differences=contrasts(report.groups);
table_rows=cell2mat(rows);
writetable(struct2table(table_rows),fullfile(base,sprintf('main_comparison_%gs.csv',duration)));
fid=fopen(fullfile(base,sprintf('main_comparison_%gs.json',duration)),'w');
fprintf(fid,'%s',jsonencode(report,'PrettyPrint',true));fclose(fid);
fid=fopen(fullfile(base,'main_comparison.json'),'w');
fprintf(fid,'%s',jsonencode(report,'PrettyPrint',true));fclose(fid);
disp(struct2table(table_rows));
for g=1:numel(report.groups)
 e=report.groups(g);
 fprintf(['%-16s arm %s: rel %s %%; err %s N.m; att RMS %.3f deg; defl sat %s %%; ' ...
  'rate sat %s %%; MPC %.1f/%.1f ms; cycles %d; fallbacks %d\n'],e.tag,e.arm, ...
  sprintf('%.2f ',e.torque.rel_rms_pct),sprintf('%.4g ',e.torque.err_rms_Nm), ...
  e.attitude.rms_deg,sprintf('%.1f ',e.surface.deflection_saturation_pct), ...
  sprintf('%.1f ',e.surface.rate_saturation_pct),e.cycle_ms_mean,e.cycle_ms_p95, ...
  e.cycles,e.fallbacks);
end
fprintf('TMF_REPORT_DONE %s duration=%gs runs=%d\n',base,duration,numel(report.groups));
end
function rows=append_rows(rows,entry)
for axis=1:3
 rows{end+1}=struct('group',entry.group,'arm',entry.arm,'axis',axis, ...
  'cmd_rms_Nm',entry.torque.cmd_rms_Nm(axis),'err_rms_Nm',entry.torque.err_rms_Nm(axis), ...
  'rel_rms_pct',entry.torque.rel_rms_pct(axis),'err_max_Nm',entry.torque.err_max_Nm(axis), ...
  'seg_0_15_Nm',entry.torque.segments(1,axis),'seg_15_30_Nm',entry.torque.segments(2,axis), ...
  'seg_30_45_Nm',entry.torque.segments(3,axis),'seg_45_60_Nm',entry.torque.segments(4,axis), ...
  'deflection_saturation_pct',entry.surface.deflection_saturation_pct(axis), ...
  'rate_saturation_pct',entry.surface.rate_saturation_pct(axis), ...
  'attitude_rms_deg',entry.attitude.rms_deg,'attitude_peak_deg',entry.attitude.peak_deg, ...
  'mpc_mean_ms',entry.cycle_ms_mean,'mpc_p95_ms',entry.cycle_ms_p95, ...
  'inner_mean_ms',inner_field(entry,'compute_ms_mean'), ...
  'inner_p95_ms',inner_field(entry,'compute_ms_p95'),'fallbacks',entry.fallbacks); %#ok<AGROW>
end
end
function value=has_signal(output,name)
value=false;
try
 value=~isempty(output.(name));
catch
 value=false;
end
end
function value=inner_field(entry,name)
value=NaN;if isstruct(entry.inner)&&isfield(entry.inner,name),value=entry.inner.(name);end
end
function out=contrasts(list)
items={};
arms=unique({list.arm});
for a=1:numel(arms)
 index=containers.Map();
 for tag={'B0','M10','F20','MF10'}
  hit=find(strcmp({list.arm},arms{a})&strcmp({list.group},tag{1}),1);
  if ~isempty(hit),index(tag{1})=hit;end
 end
 if index.Count<4,continue;end
 pick=@(name)list(index(name));
 b=pick('B0');m=pick('M10');f=pick('F20');mf=pick('MF10');
 entry=struct('arm',arms{a});
 entry.torque_err_rms_Nm=contrast( ...
  m.torque.err_rms_Nm-b.torque.err_rms_Nm,f.torque.err_rms_Nm-b.torque.err_rms_Nm, ...
  mf.torque.err_rms_Nm-b.torque.err_rms_Nm, ...
  mf.torque.err_rms_Nm-m.torque.err_rms_Nm-f.torque.err_rms_Nm+b.torque.err_rms_Nm);
 entry.torque_rel_rms_pct=contrast( ...
  m.torque.rel_rms_pct-b.torque.rel_rms_pct,f.torque.rel_rms_pct-b.torque.rel_rms_pct, ...
  mf.torque.rel_rms_pct-b.torque.rel_rms_pct, ...
  mf.torque.rel_rms_pct-m.torque.rel_rms_pct-f.torque.rel_rms_pct+b.torque.rel_rms_pct);
 entry.composite_err_rms_Nm=contrast( ...
  m.torque.composite_err_rms_Nm-b.torque.composite_err_rms_Nm, ...
  f.torque.composite_err_rms_Nm-b.torque.composite_err_rms_Nm, ...
  mf.torque.composite_err_rms_Nm-b.torque.composite_err_rms_Nm, ...
  mf.torque.composite_err_rms_Nm-m.torque.composite_err_rms_Nm- ...
  f.torque.composite_err_rms_Nm+b.torque.composite_err_rms_Nm);
 entry.attitude_rms_deg=contrast(m.attitude.rms_deg-b.attitude.rms_deg, ...
  f.attitude.rms_deg-b.attitude.rms_deg,mf.attitude.rms_deg-b.attitude.rms_deg, ...
  mf.attitude.rms_deg-m.attitude.rms_deg-f.attitude.rms_deg+b.attitude.rms_deg);
 entry.attitude_peak_deg=contrast(m.attitude.peak_deg-b.attitude.peak_deg, ...
  f.attitude.peak_deg-b.attitude.peak_deg,mf.attitude.peak_deg-b.attitude.peak_deg, ...
  mf.attitude.peak_deg-m.attitude.peak_deg-f.attitude.peak_deg+b.attitude.peak_deg);
 entry.deflection_saturation_pct=contrast( ...
  m.surface.deflection_saturation_pct-b.surface.deflection_saturation_pct, ...
  f.surface.deflection_saturation_pct-b.surface.deflection_saturation_pct, ...
  mf.surface.deflection_saturation_pct-b.surface.deflection_saturation_pct, ...
  mf.surface.deflection_saturation_pct-m.surface.deflection_saturation_pct- ...
  f.surface.deflection_saturation_pct+b.surface.deflection_saturation_pct);
 items{end+1}=entry; %#ok<AGROW>
end
if isempty(items),out=struct([]);else,out=cell2mat(items);end
end
function s=contrast(m10,f20,mf10,interaction)
s=struct('m10_minus_b0',m10,'f20_minus_b0',f20,'mf10_minus_b0',mf10, ...
 'interaction',interaction);
end
function out=attitude_error(t,x,bank)
% Same convention as report_fast_authority and docs/simulink/09..19: quaternion
% minimum rotation angle between the measured attitude and the reference bank.
q=eul2quat(x(:,[3 2 1]));
ref=interp1(bank.time(:),squeeze(bank.q_ref(:,1,:))',t,'linear');
ref=ref./vecnorm(ref,2,2);
err=rad2deg(2*acos(min(1,abs(sum(q.*ref,2)))));
omega_ref=interp1(bank.time(:),squeeze(bank.omega_ref(:,1,:))',t,'linear');
rate_err=x(:,4:6)-omega_ref;
out=struct('rms_deg',sqrt(mean(err.^2)),'peak_deg',max(err),'err_deg',err, ...
 'segment_deg',segment_rms(t,err),'rate_rms_radps',sqrt(mean(rate_err.^2,1)), ...
 'rate_peak_radps',max(abs(rate_err),[],1));
end
function seg=segment_rms(t,err)
edges=[0 15 30 45 61];seg=zeros(4,size(err,2));
for k=1:4
 mask=t>=edges(k)&t<edges(k+1);
 if any(mask),seg(k,:)=sqrt(mean(err(mask,:).^2,1));end
end
end
function [t,data]=values(ts)
data=squeeze(ts.Data);
if size(data,1)~=numel(ts.Time),data=data.';end
t=ts.Time(:);
end
function plot_group(folder,tag,arm,t,cmd,actual,delta,cmd_t,cmdlog,err_deg,limit)
axis_names={'roll','pitch','yaw'};
figure('Visible','off');
for axis=1:3
 subplot(3,1,axis);
 plot(t,cmd(:,axis),'b','LineWidth',1);hold on;plot(t,actual(:,axis),'r','LineWidth',1);grid on;
 ylabel(sprintf('%s (N.m)',axis_names{axis}));
 legend('tau cmd (MPC)','tau actual (surface)','Location','best');
end
xlabel('time (s)');
sgtitle(sprintf('%s arm %s: command vs actual aerodynamic torque (N.m, 1 ms truth, ZOH 0.2 s command)',tag,arm));
export_plot(fullfile(folder,sprintf('%s_torque.png',tag)));
figure('Visible','off');
for axis=1:3
 subplot(3,1,axis);plot(t,cmd(:,axis)-actual(:,axis),'k');grid on;
 ylabel(sprintf('%s err (N.m)',axis_names{axis}));
end
xlabel('time (s)');sgtitle(sprintf('%s arm %s: torque tracking error',tag,arm));
export_plot(fullfile(folder,sprintf('%s_error.png',tag)));
figure('Visible','off');
subplot(2,1,1);plot(t,err_deg,'k');grid on;ylabel('attitude err (deg)');
title(sprintf('%s arm %s: quaternion minimum-angle error',tag,arm));
subplot(2,1,2);
mask=err_deg>=0;plot(t(mask),movmean_(err_deg(mask),501),'m');grid on;
ylabel('attitude err, 0.5 s mean (deg)');xlabel('time (s)');
export_plot(fullfile(folder,sprintf('%s_attitude.png',tag)));
figure('Visible','off');
for axis=1:3
 subplot(3,1,axis);
 if ~isempty(cmdlog)
  stairs(cmd_t,cmdlog(:,axis),'b');hold on;
 else
  plot(t,delta(:,axis)*0,'b');hold on;
 end
 plot(t,delta(:,axis),'r');grid on;
 saturated=abs(delta(:,axis))>=limit(axis)-1e-5;
 yl=ylim;
 if any(saturated)
  area(t,saturated*yl(2),yl(2),'FaceAlpha',.1,'FaceColor','y','EdgeColor','none');
 end
 ylabel(sprintf('%s (rad)',axis_names{axis}));
 legend('delta cmd','delta actual','deflection limit active','Location','best');
end
xlabel('time (s)');sgtitle(sprintf('%s arm %s: servo command vs actual deflection',tag,arm));
export_plot(fullfile(folder,sprintf('%s_surface.png',tag)));
end
function y=movmean_(x,n)
y=filter(ones(1,n)/n,1,x);
end
function export_plot(file)
try
 exportgraphics(gcf,file);
catch err
 warning('TMF_PLOT_SKIPPED %s',err.message);
end
close(gcf);
end
