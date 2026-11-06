function out=run_fast_authority_ablation(certificate_mode,arm,duration)
% One closed-loop run for the certificate ablation (torque-level MPC, 4 SQP rounds,
% 0.2 s). The production models are never overwritten: the run uses a case-named variant.
if nargin<1,certificate_mode='legacy';end
if nargin<2,arm='B';end
if nargin<3,duration=2;end
assert(ismember(certificate_mode,{'legacy','analytic_box','analytic_full','shadow'}));
assert(ismember(arm,{'A','B'}));
root=repo_root;init_paths;
tag=sprintf('%s_%s_%gs',certificate_mode,arm,duration);
out=fullfile(root,'simulink','results','online_authority','fast_certificate','ablation',tag);
if ~exist(out,'dir'),mkdir(out);end
enabled=strcmp(arm,'B');
switch certificate_mode
 case 'legacy',code='LE';
 case 'analytic_box',code='AB';
 case 'analytic_full',code='AF';
 otherwise,code='SH';
end
case_name=sprintf('FC_%s_%s_%g',code,arm,duration);
build_closedloop(duration,true,enabled,case_name,'online_predicted',false,4,.2,certificate_mode);
m='NearSpace_CMG_FlightControl';
if ~enabled,m=[m '_SurfaceOnly'];end
m=[m '_' case_name];
casemodel=fullfile(root,'simulink','cases',[m '.slx']);
assert(isfile(casemodel),'fast:ablation','model was not saved: %s',casemodel);
load_system(casemodel);
mw=get_param(m,'ModelWorkspace');
assignin(mw,'native_results_dir',out);
si=Simulink.SimulationInput(m);
si=si.setModelParameter('StopTime',num2str(duration));
si=si.setVariable('native_results_dir',out,'Workspace',m);
wall=tic;r=sim(si);wall_s=toc(wall);
try
 save(fullfile(out,[arm '_closedloop.mat']),'r','wall_s');
catch err
 warning('FAST_ABLATION_SAVE_SKIPPED %s',err.message);
end
close_system(m,0);
% --- diagnostics ---------------------------------------------------------
files=dir(fullfile(out,'*_authority.mat'));
assert(~isempty(files),'fast:ablation','no diagnostic file written to %s',out);
d=load(fullfile(out,files(1).name));infos=d.diagnostics;
cycle=cellfun(@(i)i.elapsed_s,infos);
types={};fallbacks=0;certified=true(1,numel(infos));
phase_names={'air','aero','reachability','box','servo_certificate','allocation','attitude','translation'};
phase=zeros(1,numel(phase_names));
for k=1:numel(infos)
 fallbacks=fallbacks+double(infos{k}.fallback);
 if isfield(infos{k},'authority')&&~isempty(infos{k}.authority)
  a=infos{k}.authority;
  types=[types,a.certificate_type];
  if isfield(a,'certified'),certified(k)=all(a.certified);end
  for j=1:numel(phase_names)
   if isfield(a,'phase_time_s')&&isfield(a.phase_time_s,phase_names{j})
    phase(j)=phase(j)+a.phase_time_s.(phase_names{j});
   end
  end
 end
end
[unique_types,~,which]=unique(types);
counts=accumarray(which(:),1)';
summary=struct('certificate_mode',certificate_mode,'arm',arm,'duration_s',duration, ...
 'git_sha',git_sha(),'cycles',numel(infos),'fallbacks',fallbacks, ...
 'all_certified',all(certified),'cycle_ms_mean',1000*mean(cycle), ...
 'cycle_ms_p50',1000*median(cycle),'cycle_ms_p95',1000*percentile(cycle,.95), ...
 'cycle_ms_p99',1000*percentile(cycle,.99),'cycle_ms_max',1000*max(cycle), ...
 'over_200ms',sum(cycle>.2),'over_200ms_steady',sum(cycle(3:end)>.2), ...
 'qp_per_cycle_mean',mean(cellfun(@(i)field_or(i,'qp_solved'),infos)), ...
 'certificate_type_counts',struct('names',{unique_types},'counts',counts), ...
 'phase_ms_mean',struct('names',{phase_names},'values',1000*phase/numel(infos)), ...
 'wall_s',wall_s);
fid=fopen(fullfile(out,'summary.json'),'w');fprintf(fid,'%s',jsonencode(summary,'PrettyPrint',true));fclose(fid);
fid=fopen(fullfile(out,'run_config.json'),'w');
fprintf(fid,'%s',jsonencode(struct('certificate_mode',certificate_mode,'arm',arm, ...
 'duration_s',duration,'sqp_iterations',4,'control_period_s',.2,'git_sha',git_sha(), ...
 'branch',current_branch(),'model',m,'output',out),'PrettyPrint',true));fclose(fid);
fprintf('FAST_ABLATION_DONE %s cycles=%d fallbacks=%d cycle=%.1fms p95=%.1f max=%.1f over200=%d\n', ...
 tag,numel(infos),fallbacks,summary.cycle_ms_mean,summary.cycle_ms_p95,summary.cycle_ms_max, ...
 summary.over_200ms);
end
function value=field_or(info,name)
value=0;if isfield(info,name),value=info.(name);end
end
function value=percentile(data,p)
data=sort(data(:));value=data(max(1,min(numel(data),ceil(p*numel(data)))));
end
function value=git_sha()
[status,value]=system('git rev-parse HEAD');if status~=0,value='unknown';end
value=strtrim(value);
end
function value=current_branch()
[status,value]=system('git rev-parse --abbrev-ref HEAD');if status~=0,value='unknown';end
value=strtrim(value);
end
