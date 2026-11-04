function out=run_torque_margin_feedback_ablation(group,arm,duration,suffix)
% One closed-loop run of the torque-margin / torque-feedback ablation.
%
% Groups (fixed 2x2 main design, A and B for every group):
%   B0    0 %  shrink, 0.2 s open-loop allocation   (baseline of this run environment)
%   M10  10 %  shrink, 0.2 s open-loop allocation   (method M only)
%   F20   0 %  shrink, 0.02 s torque feedback       (method F only)
%   MF10 10 %  shrink, 0.02 s torque feedback       (M + F)
%   M05/MF05  5 % sensitivity runs, optional, never used to replace M10/MF10
%
% The production models are never overwritten: every run builds its own case-named
% model under simulink/cases and archives into its own result folder.
if nargin<1,group='B0';end
if nargin<2,arm='A';end
if nargin<3,duration=2;end
if nargin<4,suffix='';end
config=group_config(group);
assert(ismember(arm,{'A','B'}),'arm must be A or B');
assert(isscalar(duration) && duration>0 && isfinite(duration));
root=repo_root;init_paths;
if isempty(suffix),tag=sprintf('%s_%s_%gs',group,arm,duration);
else,tag=sprintf('%s_%s_%gs_%s',group,arm,duration,suffix);end
out=fullfile(root,'simulink','results','online_authority','torque_margin_feedback',tag);
if ~exist(out,'dir'),mkdir(out);end
case_name=sprintf('TMF_%s_%s_%g',group,arm,duration);
if ~isempty(suffix),case_name=[case_name '_' suffix];end
assert(numel(case_name)<=24,'case name is too long for the 63-character model limit');
enabled=strcmp(arm,'B');
started=now;wall_started=tic;
build_closedloop(duration,true,enabled,case_name,'online_predicted',false,4,.2, ...
 'analytic_full',config.shrink,config.inner_mode);
m='NearSpace_CMG_FlightControl';
if ~enabled,m=[m '_SurfaceOnly'];end
m=[m '_' case_name];
casemodel=fullfile(root,'simulink','cases',[m '.slx']);
assert(isfile(casemodel),'tmf:build','model was not saved: %s',casemodel);
load_system(casemodel);
mw=get_param(m,'ModelWorkspace');
assignin(mw,'native_results_dir',out);
si=Simulink.SimulationInput(m);
si=si.setModelParameter('StopTime',num2str(duration));
si=si.setVariable('native_results_dir',out,'Workspace',m);
r=sim(si);
wall_s=toc(wall_started);
try
 save(fullfile(out,[arm '_closedloop.mat']),'r','wall_s');
catch err
 warning('TMF_SAVE_SKIPPED %s',err.message);
end
close_system(m,0);
% ---- hard output gates ------------------------------------------------------
t=r.commands.Time(:);cmd=values(r.commands);actual=values(r.tau_actual);
delta=values(r.delta_actual);rate=values(r.delta_dot);
assert(all(isfinite(cmd(:))) && all(isfinite(actual(:))),'tmf:nonfinite','non-finite torque log');
assert(numel(t)==numel(r.tau_actual.Time) && max(abs(t(:)-r.tau_actual.Time(:)))<1e-12, ...
 'tmf:align','the command and the physical torque are not on one clock');
x=values(r.states);
assert(~isempty(x) && all(isfinite(x(:))),'tmf:nonfinite','non-finite state log');
try
 dc_log=values(r.delta_cmd);
 assert(all(isfinite(dc_log(:))),'tmf:nonfinite','non-finite command log');
catch err
 if ~contains(err.identifier,'nonfinite'),warning('TMF_NO_DELTA_CMD_LOG %s',err.message);else,rethrow(err);end
end
p=vehicle_params;v=p.vehicle;
assert(max(abs(delta(:)))<=max(v.delta_max)*1.0001,'tmf:limit','physical deflection left its limit');
assert(max(abs(rate(:)))<=max(v.delta_dot_max)*1.0001,'tmf:limit','physical servo rate left its limit');
% ---- MPC diagnostics --------------------------------------------------------
files=dir(fullfile(out,'*_authority.mat'));
assert(numel(files)==1,'tmf:diag','expected one MPC diagnostic file, found %d',numel(files));
d=load(fullfile(out,files(1).name));infos=d.diagnostics;
cycle=cellfun(@(i)i.elapsed_s,infos);
fallbacks=0;certified=true(1,numel(infos));types={};
restored=0;stages=0;shrink_max=0;margin=[];sqp=[];accepted=[];rejected=[];qp=[];stops={};
phase_names={'air','aero','reachability','box','servo_certificate','allocation','attitude','translation'};
phase=zeros(1,numel(phase_names));
for k=1:numel(infos)
 i=infos{k};fallbacks=fallbacks+double(i.fallback);
 sqp(end+1)=field_or(i,'iterations');accepted(end+1)=field_or(i,'accepted_steps'); %#ok<AGROW>
 rejected(end+1)=field_or(i,'rejected_steps');qp(end+1)=field_or(i,'qp_solved'); %#ok<AGROW>
 if isfield(i,'early_stop_reason'),stops{end+1}=i.early_stop_reason;end %#ok<AGROW>
 if isfield(i,'authority')&&~isempty(i.authority)
  a=i.authority;types=[types,a.certificate_type];
  if isfield(a,'certified'),certified(k)=all(a.certified);end
  if isfield(a,'restored'),restored=restored+nnz(a.restored);end
  if isfield(a,'shrink_abs_Nm')
   stages=stages+numel(a.scale);shrink_max=max(shrink_max,max(a.shrink_abs_Nm,[],'all'));
   half=(a.tau_max-a.tau_min)/2;
   if any(isfield(a,{'nominal_tau'}))
    span=min(a.nominal_tau-a.tau_min,a.tau_max-a.nominal_tau);
    safe=half>0;margin=[margin;(span(safe)./half(safe))]; %#ok<AGROW>
   end
  end
  for j=1:numel(phase_names)
   if isfield(a,'phase_time_s')&&isfield(a.phase_time_s,phase_names{j})
    phase(j)=phase(j)+a.phase_time_s.(phase_names{j});
   end
  end
 end
end
[unique_types,~,which]=unique(types);
summary=struct('group',group,'arm',arm,'duration_s',duration,'suffix',suffix, ...
 'shrink_ratio',config.shrink,'inner_mode',config.inner_mode, ...
 'certificate_mode','analytic_full','sqp_iterations',4,'control_period_s',.2, ...
 'git_sha',git_sha(),'branch',current_branch(),'model',m,'output',out, ...
 'cycles',numel(infos),'fallbacks',fallbacks,'all_certified',all(certified), ...
 'cycle_ms_mean',1000*mean(cycle),'cycle_ms_p50',1000*median(cycle), ...
 'cycle_ms_p95',1000*percentile(cycle,.95),'cycle_ms_p99',1000*percentile(cycle,.99), ...
 'cycle_ms_max',1000*max(cycle),'over_200ms',sum(cycle>.2), ...
 'over_200ms_steady',sum(cycle(min(3,end):end)>.2), ...
 'qp_per_cycle_mean',mean(qp),'sqp_iterations_mean',mean(sqp), ...
 'accepted_steps_mean',mean(accepted),'rejected_steps_total',sum(rejected), ...
 'accepted_steps_zero_cycles',sum(accepted==0),'early_stop',stop_counts(stops), ...
 'certificate_type_counts',struct('names',{unique_types},'counts', ...
  {accumarray(which(:),1)'}), ...
 'shrink_stages',stages,'shrink_restored_stages',restored,'shrink_max_abs_Nm',shrink_max, ...
 'box_margin_min',min_or_nan(margin),'box_margin_mean',mean_or_nan(margin), ...
 'phase_ms_mean',struct('names',{phase_names},'values',{1000*phase/max(1,numel(infos))}), ...
 'wall_s',wall_s);
% ---- inner-loop diagnostics (20 ms arm only) --------------------------------
fb=dir(fullfile(out,'*_torque_feedback.mat'));
if ~isempty(fb)
 f=load(fullfile(out,fb(1).name));tick=f.diagnostics;
 summary.inner=struct('ticks',numel(tick.time),'fallbacks',f.summary.fallbacks, ...
  'slew_saturated_ticks',f.summary.slew_saturated_ticks, ...
  'max_iterations_used',f.summary.max_iterations_used, ...
  'compute_ms_mean',f.summary.compute_ms_mean,'compute_ms_p95',f.summary.compute_ms_p95, ...
  'compute_ms_max',f.summary.compute_ms_max,'over_20ms',f.summary.over_20ms, ...
  'cost_initial_mean',mean(tick.cost_initial),'cost_final_mean',mean(tick.cost_final), ...
  'predicted_torque_error_rms_Nm',sqrt(mean(sum(tick.residual_Nm.^2,2))), ...
  'command_slew_change_total_rad',sum(sum(abs(diff(tick.delta_cmd,1,1)))), ...
  'inner_period_s',f.summary.inner_period_s);
 assert(numel(tick.time)==round(duration/.02)+1,'tmf:innerTicks','inner ticks %d, expected %d', ...
  numel(tick.time),round(duration/.02)+1);
else
 summary.inner=[];
end
summary.torque_alignment=struct('samples',numel(t),'dt_median',median(diff(t)), ...
 'time_identical',max(abs(r.commands.Time(:)-r.tau_actual.Time(:)))<1e-12, ...
 'command_changes',sum(any(abs(diff(cmd(:,1:3)))>1e-12,2)));
fid=fopen(fullfile(out,'summary.json'),'w');fprintf(fid,'%s',jsonencode(summary,'PrettyPrint',true));fclose(fid);
fid=fopen(fullfile(out,'run_config.json'),'w');
fprintf(fid,'%s',jsonencode(struct('group',group,'arm',arm,'duration_s',duration, ...
 'suffix',suffix,'shrink_ratio',config.shrink,'inner_mode',config.inner_mode, ...
 'inner_period_s',double(config.inner_period),'certificate_mode','analytic_full', ...
 'sqp_iterations',4,'control_period_s',.2,'physics_step_s',.001, ...
 'disturbance','physical reference bank only (no constant external torque)', ...
 'git_sha',git_sha(),'branch',current_branch(),'model',m,'case_file',casemodel, ...
 'output',out,'started',datestr(started,31),'wall_s',wall_s),'PrettyPrint',true));fclose(fid);
fprintf(['TMF_DONE %s shrink=%.2f inner=%s cycles=%d fallbacks=%d cyc=%.1fms p95=%.1f ' ...
 'max=%.1f over200=%d wall=%.1fs\n'],tag,config.shrink,config.inner_mode,numel(infos), ...
 fallbacks,summary.cycle_ms_mean,summary.cycle_ms_p95,summary.cycle_ms_max, ...
 summary.over_200ms,wall_s);
end
function config=group_config(group)
switch upper(group)
 case 'B0',config=struct('shrink',0,'inner_mode','legacy','inner_period',.2);
 case 'M10',config=struct('shrink',.10,'inner_mode','legacy','inner_period',.2);
 case 'M05',config=struct('shrink',.05,'inner_mode','legacy','inner_period',.2);
 case 'F20',config=struct('shrink',0,'inner_mode','feedback_predictive','inner_period',.02);
 case 'MF10',config=struct('shrink',.10,'inner_mode','feedback_predictive','inner_period',.02);
 case 'MF05',config=struct('shrink',.05,'inner_mode','feedback_predictive','inner_period',.02);
 otherwise,error('unknown ablation group %s (B0/M10/F20/MF10, optional M05/MF05)',group);
end
end
function counts=stop_counts(list)
names={};values=[];
for k=1:numel(list)
 key=list{k};
 if isempty(key),key='none';end
 index=find(strcmp(names,key),1);
 if isempty(index),names{end+1}=key;values(end+1)=1;else,values(index)=values(index)+1;end %#ok<AGROW>
end
counts=struct('names',{names},'counts',values);
end
function data=values(ts)
data=squeeze(ts.Data);
if size(data,1)~=numel(ts.Time),data=data.';end
end
function value=field_or(info,name)
value=0;if isfield(info,name),value=double(info.(name));end
end
function value=percentile(data,p)
data=sort(data(:));value=data(max(1,min(numel(data),ceil(p*numel(data)))));
end
function value=min_or_nan(data)
if isempty(data),value=NaN;else,value=min(data);end
end
function value=mean_or_nan(data)
if isempty(data),value=NaN;else,value=mean(data);end
end
function value=git_sha()
[status,value]=system('git rev-parse HEAD');if status~=0,value='unknown';end
value=strtrim(value);
end
function value=current_branch()
[status,value]=system('git rev-parse --abbrev-ref HEAD');if status~=0,value='unknown';end
value=strtrim(value);
end
