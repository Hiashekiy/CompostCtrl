function out=run_direct_servo_acceptance_ablation(acceptance_mode,model_mode,arm,duration)
% One command per ablation run. Builds the direct-servo model variant (production models are
% never overwritten), applies ONLY the acceptance/model switch, simulates, and stores the
% configuration, per-cycle diagnostics and per-round table under
% results/online_authority/direct_servo/acceptance_ablation/<mode>_<model>_<arm>_<dur>s/.
if nargin<1,acceptance_mode='hard_stop';end
if nargin<2,model_mode='fixed';end
if nargin<3,arm='B';end
if nargin<4,duration=2;end
assert(ismember(acceptance_mode,{'hard_stop','no_descent_gate','continue_best'}));
assert(ismember(model_mode,{'legacy','fixed'}));
assert(ismember(arm,{'A','B'}));
root=repo_root;init_paths;
tag=sprintf('%s_%s_%s_%gs',acceptance_mode,model_mode,arm,duration);
out=fullfile(root,'simulink','results','online_authority','direct_servo','acceptance_ablation',tag);
if ~exist(out,'dir'),mkdir(out);end
enabled=strcmp(arm,'B');
switch acceptance_mode
 case 'hard_stop',mode_code='HS';
 case 'no_descent_gate',mode_code='NG';
 otherwise,mode_code='CB';
end
% Simulink model names are limited to 63 characters, so the model carries a short code
% while the result folder keeps the full descriptive tag.
case_name=sprintf('DA_%s_%s_%s_%g',mode_code,upper(model_mode(1)),arm,duration);
build_closedloop(duration,true,enabled,case_name,'online_predicted',false,4,.2,'direct_servo');
m=['NearSpace_CMG_FlightControl'];
if ~enabled,m=[m '_SurfaceOnly'];end
m=[m '_' case_name];
casemodel=fullfile(root,'simulink','cases',[m '.slx']);
assert(isfile(casemodel),'direct:ablation','model was not saved: %s',casemodel);
load_system(casemodel);
mw=get_param(m,'ModelWorkspace');
template=getVariable(mw,'native_controller_template');
template.acceptance_mode=acceptance_mode;template.model_mode=model_mode;
assignin(mw,'native_controller_template',template);
assignin(mw,'native_results_dir',out);
si=Simulink.SimulationInput(m);
si=si.setModelParameter('StopTime',num2str(duration));
si=si.setVariable('native_results_dir',out,'Workspace',m);
wall=tic;r=sim(si);wall_s=toc(wall);
try
 save(fullfile(out,[arm '_closedloop.mat']),'r','wall_s');
catch err
 warning('ABLATION_SAVE_SKIPPED %s',err.message);
end
close_system(m,0);
% --- diagnostics ---------------------------------------------------------
files=dir(fullfile(out,'*_direct_authority.mat'));
assert(~isempty(files),'direct:ablation','no diagnostic file was written to %s',out);
d=load(fullfile(out,files(1).name));infos=d.diagnostics;t=d.control_times(:)';
cycles=numel(infos);
accepted=cellfun(@(i)i.accepted_steps,infos);
qp=cellfun(@(i)i.qp_solved,infos);
first=zeros(3,cycles);J=zeros(1,cycles);stop=cell(1,cycles);
for k=1:cycles
 if ~isempty(infos{k}.authority),first(:,k)=infos{k}.authority.command(:,1);end
 if isfield(infos{k},'objective'),J(k)=infos{k}.objective;end
 if isfield(infos{k},'early_stop_reason'),stop{k}=infos{k}.early_stop_reason;end
end
moved=[false (vecnorm(diff(first,1,2),inf)>1e-9)];
rounds=[];names=fieldnames(infos{1}.rounds{1});
for k=1:cycles
 for j=1:numel(infos{k}.rounds)
  rj=infos{k}.rounds{j};
  row=struct('cycle',k,'t',t(k),'iteration',j);
  for f=1:numel(names)
   value=rj.(names{f});
   if isnumeric(value)&&isscalar(value),row.(names{f})=value;end
  end
  row.J_old_att=rj.J_old_terms.att;row.J_old_rate=rj.J_old_terms.rate;
  row.J_old_terminal=rj.J_old_terms.terminal;row.J_old_cmd=rj.J_old_terms.cmd;
  row.J_old_dcmd=rj.J_old_terms.dcmd;row.J_old_nu=rj.J_old_terms.nu;row.J_old_dnu=rj.J_old_terms.dnu;
  row.J_try_att=rj.J_try_terms.att;row.J_try_rate=rj.J_try_terms.rate;
  row.J_try_terminal=rj.J_try_terms.terminal;row.J_try_cmd=rj.J_try_terms.cmd;
  row.J_try_dcmd=rj.J_try_terms.dcmd;row.J_try_nu=rj.J_try_terms.nu;row.J_try_dnu=rj.J_try_terms.dnu;
  row.cmd_before_1=rj.cmd_before(1);row.cmd_proposed_1=rj.cmd_proposed(1);
  row.cmd_accepted_1=rj.cmd_accepted(1);
  rounds=[rounds;row]; %#ok<AGROW>
 end
end
writetable(struct2table(rounds),fullfile(out,'iteration_diagnostics.csv'));
stop_names=unique(stop);counts=zeros(1,numel(stop_names));
for k=1:numel(stop_names),counts(k)=sum(strcmp(stop,stop_names{k}));end
summary=struct('acceptance_mode',acceptance_mode,'model_mode',model_mode,'arm',arm, ...
 'duration_s',duration,'cycles',cycles,'git_sha',git_sha(),'matlab_version',version, ...
 'wall_s',wall_s,'fallbacks',sum(cellfun(@(i)i.fallback,infos)), ...
 'accepted_steps_mean',mean(accepted),'accepted_steps_max',max(accepted), ...
 'cycles_with_zero_accept',sum(accepted==0),'first_command_changed_cycles',sum(moved), ...
 'first_command_max_rad',max(abs(first(:))),'first_command_mean_rad',mean(abs(first(:))), ...
 'qp_per_cycle_mean',mean(qp),'qp_per_cycle_max',max(qp), ...
 'cycle_ms_mean',1000*mean(cellfun(@(i)i.elapsed_s,infos)), ...
 'cycle_ms_p95',1000*percentile(cellfun(@(i)i.elapsed_s,infos),.95), ...
 'cycle_ms_max',1000*max(cellfun(@(i)i.elapsed_s,infos)), ...
 'over_200ms',sum(cellfun(@(i)i.elapsed_s,infos)>.2), ...
 'J_start',J(1),'J_end',J(end),'stop_reasons',struct('names',{stop_names},'counts',counts), ...
 'objective_increase',sum(~cellfun(@isempty,strfind(stop,'objective_increase'))), ...
 'trust_radius_rad',infos{1}.trust_radius_rad);
fid=fopen(fullfile(out,'summary.json'),'w');fprintf(fid,'%s',jsonencode(summary,'PrettyPrint',true));fclose(fid);
fid=fopen(fullfile(out,'run_config.json'),'w');
fprintf(fid,'%s',jsonencode(struct('acceptance_mode',acceptance_mode,'model_mode',model_mode, ...
 'arm',arm,'duration_s',duration,'sqp_iterations',4,'control_period_s',.2, ...
 'trust_radius_rad',infos{1}.trust_radius_rad,'git_sha',git_sha(),'branch',current_branch(), ...
 'model',m,'output',out),'PrettyPrint',true));fclose(fid);
fprintf('ABLATION_DONE %s accepted=%.2f zero=%d changed=%d qp=%.2f cycle=%.0fms J %.4g->%.4g\n', ...
 tag,summary.accepted_steps_mean,summary.cycles_with_zero_accept, ...
 summary.first_command_changed_cycles,summary.qp_per_cycle_mean,summary.cycle_ms_mean, ...
 summary.J_start,summary.J_end);
end
function value=git_sha()
[status,value]=system('git rev-parse HEAD');
if status~=0,value='unknown';end
value=strtrim(value);
end
function value=current_branch()
[status,value]=system('git rev-parse --abbrev-ref HEAD');
if status~=0,value='unknown';end
value=strtrim(value);
end
function value=percentile(data,p)
data=sort(data(:));value=data(max(1,min(numel(data),ceil(p*numel(data)))));
end
