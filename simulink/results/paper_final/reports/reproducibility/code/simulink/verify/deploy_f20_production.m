function report=deploy_f20_production(mode)
% Deploy the already validated F20-B + analytic_full configuration to the PRODUCTION
% model simulink/models/NearSpace_CMG_FlightControl.slx via
%
%   build_closedloop(60,true,true,'','online_predicted',false,4,.2,'analytic_full',0, ...
%                    'feedback_predictive')
%
% Nothing here reinvents the controller: the 11-parameter build_closedloop generator and
% the verified online_torque_feedback_step / native_torque_feedback_allocation_sfun path
% are reused unchanged.  case_name='' is what makes it write the production .slx.
%
% Modes
%   'all'    (default) preflight -> isolated equivalence -> production build -> checks
%   'check'  only the preflight snapshot of the CURRENT production model
%   'verify' only the isolated DEP_F20CHK 60 s equivalence run
%   'deploy' preflight + verify gates, then the production build and acceptance runs
%
% Gates (asserts, never printed numbers):
%   G1 the current production model is the legacy 0.2 s / bisection-certificate system
%   G2 an isolated 60 s build of the target configuration reproduces the archived
%      F20_B_60s commands AND states to 1e-9, with 301/3001 triggers
%   G3 the regenerated production model carries the expected model workspace and blocks
%      (Torque_feedback_allocation with 4 connected inputs, no Surface_control_allocation,
%      Previous_Allocation_Command at 0.02 s, CMG path and 1 ms servo untouched)
%   G4 the production file ITSELF, reloaded from simulink/models, reproduces the archived
%      F20-B run to 1e-9 (2 s and 60 s), with every axis inside its own limits
%   Any failure after the production file was touched restores the timestamped backup
%   and re-checks its checksum.
assert(ismember(mode,{'all','check','verify','deploy','accept','accept'}),'deploy:gate', ...
 'unknown mode %s',mode);
root=repo_root;init_paths;
p=vehicle_params;v=p.vehicle;
production=fullfile(root,'simulink','models','NearSpace_CMG_FlightControl.slx');
archive=fullfile(root,'simulink','results','online_authority','torque_margin_feedback', ...
 'F20_B_60s','B_closedloop.mat');
stamp=datestr(now,'yyyymmdd_HHMMSS');
out=fullfile(root,'simulink','results','deploy_f20',stamp);
if ~exist(out,'dir'),mkdir(out);end
report=struct('mode',mode,'stamp',stamp,'output',out,'production_model',production, ...
 'archive',archive,'git_sha',git_sha(),'branch',current_branch());
fprintf('DEPLOY_F20 mode=%s out=%s\n',mode,out);
if ismember(mode,{'all','check','deploy'})
 report.preflight=preflight(production);
end
if ismember(mode,{'all','verify','deploy'})
 report.equivalence=verify_isolated(root,archive,out);
end
if ismember(mode,{'all','deploy'})
 assert(isfield(report,'equivalence') && report.equivalence.passed,'deploy:gate', ...
  'G2 failed: the isolated build does not reproduce F20-B, production .slx NOT touched');
 report.deployment=deploy_production(root,production,archive,out,v,report.preflight);
end
if strcmp(mode,'accept')
 % Acceptance only, on the production file as it stands (no rebuild, no G1/G2 gates).
 load_system(production);[~,m]=fileparts(production);
 report.structure=inspect_structure(m,v);
 safe_close(m);
 report.deployment=struct('touched_production',false,'mode','accept', ...
  'production_checksum',checksum_of(production));
 report.deployment.smoke_2s=run_and_compare(production,2,archive,fullfile(out,'production_2s'),v);
 report.deployment.acceptance_60s=run_and_compare(production,60,archive,fullfile(out,'production_60s'),v);
 report.deployment.passed=true;
end
fid=fopen(fullfile(out,'deployment_record.json'),'w');
fprintf(fid,'%s',jsonencode(report,'PrettyPrint',true));fclose(fid);
fprintf('DEPLOY_F20_DONE %s\n',out);
end
% ------------------------------------------------------------------------------
function s=preflight(production)
s=struct('path',production,'exists',isfile(production));
assert(s.exists,'deploy:gate','production model missing: %s',production);
d=dir(production);s.bytes=d.bytes;
s.checksum=checksum_of(production);
[~,m]=fileparts(production);
load_system(production);
s.model=m;s.stop_time=get_param(m,'StopTime');s.fixed_step=get_param(m,'FixedStep');
s.solver=get_param(m,'Solver');s.solver_type=get_param(m,'SolverType');
mw=get_param(m,'ModelWorkspace');
w=mw.whos;
if isempty(w),s.workspace_variables={};else,s.workspace_variables={w.name};end
s.block_count=numel(find_system(m,'SearchDepth',1,'Type','Block'));
t=getVariable(mw,'native_controller_template');
s.template=struct('authority_mode',t.authority_mode, ...
 'authority_certificate_mode',field_or(t,'authority_certificate_mode','<absent>'), ...
 'authority_torque_shrink_ratio',field_or(t,'authority_torque_shrink_ratio',-1), ...
 'max_sqp_iterations',field_or(t,'max_sqp_iterations',-1),'dt_ctrl',double(t.dt_ctrl), ...
 'nx',double(t.nx),'N',double(t.N));
if hasVariable(mw,'native_inner_mode'),s.inner_mode=getVariable(mw,'native_inner_mode');
else,s.inner_mode='<absent>';end
if hasVariable(mw,'native_inner_period'),s.inner_period=getVariable(mw,'native_inner_period');
else,s.inner_period=-1;end
s.has_feedback_block=~isempty(find_system(m,'SearchDepth',1,'Name','Torque_feedback_allocation'));
s.has_legacy_block=~isempty(find_system(m,'SearchDepth',1,'Name','Surface_control_allocation'));
if s.has_legacy_block
 s.previous_command_sample_time=str2double( ...
  get_param([m '/Previous_Allocation_Command'],'SampleTime'));
else
 s.previous_command_sample_time=NaN;
end
s.cmg_present=~isempty(find_system([m '/Native_Plant/CMG'],'SearchDepth',0));
if ~s.has_feedback_block
 echo_block_inventory(m);
end
safe_close(m);
% G1: the model being replaced must be the legacy 0.2 s / bisection-certificate system.
assert(~s.has_feedback_block,'deploy:gate','G1: the production model already has the F20 block');
assert(s.has_legacy_block,'deploy:gate','G1: no Surface_control_allocation in the production model');
assert(abs(s.previous_command_sample_time-.2)<1e-12,'deploy:gate', ...
 'G1: Previous_Allocation_Command is %g s, expected the 0.2 s legacy cache', ...
 s.previous_command_sample_time);
assert(abs(s.template.dt_ctrl-.2)<1e-12,'deploy:gate','G1: dt_ctrl is not 0.2 s');
assert(s.template.max_sqp_iterations==4,'deploy:gate','G1: max_sqp_iterations is not 4');
assert(abs(str2double(s.fixed_step)-.001)<1e-9,'deploy:gate','G1: the physics step is not 1 ms');
assert(s.cmg_present,'deploy:gate','G1: the CMG subsystem is missing');
assert(strcmp(s.template.authority_mode,'online_predicted'),'deploy:gate', ...
 'G1: the production model is not the online flight controller');
fprintf(['PREFLIGHT ok: %s bytes=%d stop=%s step=%s template(cert=%s shrink=%g sqp=%g ' ...
 'dt=%g nx=%g N=%g) inner=%s legacy_block=%d cmg=%d\n'],s.model,s.bytes,s.stop_time, ...
 s.fixed_step,s.template.authority_certificate_mode,s.template.authority_torque_shrink_ratio, ...
 s.template.max_sqp_iterations,s.template.dt_ctrl,s.template.nx,s.template.N, ...
 s.inner_mode,s.has_legacy_block,s.cmg_present);
end
% ------------------------------------------------------------------------------
function echo_block_inventory(m)
names=find_system(m,'SearchDepth',1,'Type','Block');
fprintf('PREFLIGHT inventory (%d blocks): %s\n',numel(names),strjoin(names,', '));
end
% ------------------------------------------------------------------------------
function s=verify_isolated(root,archive,out)
s=struct('passed',false,'case_name','DEP_F20CHK','tolerance',1e-9);
folder=fullfile(out,'isolated_60s');if ~exist(folder,'dir'),mkdir(folder);end
assert(isfile(archive),'deploy:gate','archived F20-B run is missing: %s',archive);
build_closedloop(60,true,true,'DEP_F20CHK','online_predicted',false,4,.2, ...
 'analytic_full',0,'feedback_predictive');
m='NearSpace_CMG_FlightControl_DEP_F20CHK';
casemodel=fullfile(root,'simulink','cases',[m '.slx']);
assert(isfile(casemodel),'deploy:gate','the isolated model was not saved: %s',casemodel);
load_system(casemodel);
mw=get_param(m,'ModelWorkspace');assignin(mw,'native_results_dir',folder);
si=Simulink.SimulationInput(m);
si=si.setModelParameter('StopTime','60');
si=si.setVariable('native_results_dir',folder,'Workspace',m);
r=sim(si);
save(fullfile(folder,'isolated_closedloop.mat'),'r');
safe_close(m);
fresh=load(fullfile(folder,'isolated_closedloop.mat'));
ref=load(archive);
[dcmd,dstate]=compare_prefix(fresh.r,ref.r,0);
s.max_command_difference=dcmd;s.max_state_difference=dstate;
s.samples=numel(fresh.r.commands.Time);
s.counts=trigger_counts(folder);
assert(dcmd<=s.tolerance,'deploy:gate', ...
 'G2: the isolated build differs from the archived F20-B commands by %.3e',dcmd);
assert(dstate<=s.tolerance,'deploy:gate', ...
 'G2: the isolated build differs from the archived F20-B states by %.3e',dstate);
assert(s.counts.mpc_cycles==301,'deploy:gate','G2: %d upper cycles, expected 301', ...
 s.counts.mpc_cycles);
assert(s.counts.inner_ticks==3001,'deploy:gate','G2: %d inner ticks, expected 3001', ...
 s.counts.inner_ticks);
assert(s.counts.fallbacks==0 && s.counts.inner_fallbacks==0,'deploy:gate', ...
 'G2: safe fallbacks appeared (%d/%d)',s.counts.fallbacks,s.counts.inner_fallbacks);
s.passed=true;
fprintf(['VERIFY ok: command diff %.3e, state diff %.3e, samples %d, cycles %d, ' ...
 'inner %d, fallbacks %d/%d\n'],dcmd,dstate,s.samples,s.counts.mpc_cycles, ...
 s.counts.inner_ticks,s.counts.fallbacks,s.counts.inner_fallbacks);
end
% ------------------------------------------------------------------------------
function s=deploy_production(root,production,archive,out,v,preflight)
s=struct('touched_production',false,'restored',false, ...
 'backup_checksum',preflight.checksum,'backup_bytes',preflight.bytes);
try
 build_closedloop(60,true,true,'','online_predicted',false,4,.2, ...
  'analytic_full',0,'feedback_predictive');
 s.touched_production=true;
 d=dir(production);s.production_bytes=d.bytes;
 s.production_checksum=checksum_of(production);
 assert(~strcmp(s.production_checksum,preflight.checksum),'deploy:gate', ...
  'the production .slx is unchanged: the build did not write it');
 % ---- G3: structure and model workspace of the regenerated production model -----
 load_system(production);
 [~,m]=fileparts(production);
 s.structure=inspect_structure(m,v);
 safe_close(m);
 % ---- G4a: 2 s smoke on the production file itself -----------------------------
 s.smoke_2s=run_and_compare(production,2,archive,fullfile(out,'production_2s'),v);
 % ---- G4b: 60 s acceptance run on the production file itself -------------------
 s.acceptance_60s=run_and_compare(production,60,archive,fullfile(out,'production_60s'),v);
 s.passed=true;
catch err
 s.error=err.message;s.error_id=err.identifier;
 fprintf('DEPLOY_F20_FAILED %s\n',err.message);
 for j=1:min(10,numel(err.stack))
  fprintf('  AT %s line %d\n',err.stack(j).file,err.stack(j).line);
 end
 if s.touched_production
  % The model may still be open in this session, which locks the file: close it first.
  [~,mm]=fileparts(production);safe_close(mm);
  failed=fullfile(out,'NearSpace_CMG_FlightControl.failed.slx');
  try,copyfile(production,failed);catch,copyfile(production,strrep(failed,'.slx','_copy.slx'));end
  backup=backup_path(root);
  s.backup_path=backup;
  try
   copyfile(backup,production);
   restored=checksum_of(production);
   s.restored_checksum=restored;
   s.restored=strcmp(restored,preflight.checksum);
   fprintf('RESTORE from %s: checksum match %d\n',backup,s.restored);
  catch restore_err
   s.restore_error=restore_err.message;s.restored=false;
   fprintf('RESTORE_FAILED %s\n',restore_err.message);
  end
  try
   fid=fopen(fullfile(out,'rollback.json'),'w');
   fprintf(fid,'%s',jsonencode(s,'PrettyPrint',true));fclose(fid);
  catch
  end
  assert(s.restored,'deploy:gate','restoring the production model from the backup failed: %s', ...
   field_or(s,'restore_error','checksum mismatch'));
 end
 rethrow(err);
end
end
function path=backup_path(root)
d=dir(fullfile(root,'tmp','deploy_f20_backup','*','NearSpace_CMG_FlightControl.before.slx'));
assert(~isempty(d),'deploy:gate','no timestamped backup of the production model found');
[~,order]=sort([d.datenum]);
d=d(order(end));
path=fullfile(d(end).folder,d(end).name);
end
% ------------------------------------------------------------------------------
function s=inspect_structure(m,v)
s=struct('model',m,'blocks',{{}});
s.has_feedback=~isempty(find_system(m,'SearchDepth',1,'Name','Torque_feedback_allocation'));
s.has_legacy=~isempty(find_system(m,'SearchDepth',1,'Name','Surface_control_allocation'));
assert(s.has_feedback,'deploy:gate','G3: Torque_feedback_allocation is missing');
assert(~s.has_legacy,'deploy:gate','G3: the legacy Surface_control_allocation is still present');
s.function_name=get_param([m '/Torque_feedback_allocation'],'FunctionName');
assert(strcmp(s.function_name,'native_torque_feedback_allocation_sfun'),'deploy:gate', ...
 'G3: the feedback block calls %s',s.function_name);
ph=get_param([m '/Torque_feedback_allocation'],'PortHandles');
assert(numel(ph.Inport)==4 && numel(ph.Outport)==1,'deploy:gate', ...
 'G3: the feedback block has %d inputs and %d outputs',numel(ph.Inport),numel(ph.Outport));
for j=1:4
 line=get_param(ph.Inport(j),'Line');
 assert(line>0,'deploy:gate','G3: input %d of the feedback block is unconnected',j);
 src=get_param(line,'SrcPortHandle');
 assert(src>0,'deploy:gate','G3: input %d has no source port',j);
 s.blocks{end+1}=get_param(get_param(src,'Parent'),'Name');
end
assert(get_param(ph.Outport(1),'Line')>0,'deploy:gate','G3: the feedback output is unconnected');
s.previous_command_sample_time=str2double( ...
 get_param([m '/Previous_Allocation_Command'],'SampleTime'));
assert(abs(s.previous_command_sample_time-.02)<1e-12,'deploy:gate', ...
 'G3: Previous_Allocation_Command is %g s, expected 0.02 s in the F20 arm', ...
 s.previous_command_sample_time);
s.cmg_present=~isempty(find_system([m '/Native_Plant/CMG'],'SearchDepth',0));
assert(s.cmg_present,'deploy:gate','G3: the CMG subsystem is missing');
assert(strcmp(get_param([m '/Control_split'],'BlockType'),'Demux'),'deploy:gate', ...
 'G3: Control_split is not the B-arm demux');
lh=get_param([m '/Control_split'],'LineHandles');
nu_dst=get_param(lh.Outport(2),'DstPortHandle');
assert(any(nu_dst>0),'deploy:gate','G3: the CMG rate command is not connected');
s.cmg_command_port=get_param(get_param(nu_dst(1),'Parent'),'Name');
s.fixed_step=get_param(m,'FixedStep');
assert(abs(str2double(s.fixed_step)-.001)<1e-9,'deploy:gate', ...
 'G3: the physics step changed to %s',s.fixed_step);
try
 s.servo_sample_time=str2double(get_param( ...
  [m '/Aerodynamic_Surface/Surface_Actuator/Actual_state'],'SampleTime'));
 assert(abs(s.servo_sample_time-.001)<1e-12,'deploy:gate','G3: the servo state step changed');
catch err
 s.servo_sample_time=NaN;s.servo_check_error=err.message;
end
mw=get_param(m,'ModelWorkspace');
t=getVariable(mw,'native_controller_template');
s.template=struct('authority_mode',t.authority_mode, ...
 'authority_certificate_mode',t.authority_certificate_mode, ...
 'authority_torque_shrink_ratio',double(t.authority_torque_shrink_ratio), ...
 'max_sqp_iterations',double(t.max_sqp_iterations),'dt_ctrl',double(t.dt_ctrl));
for name={'native_inner_mode','native_inner_period','native_control_period','native_torque_shrink_ratio'}
 assert(hasVariable(mw,name{1}),'deploy:gate','G3: model workspace variable %s is missing',name{1});
end
s.inner_mode=getVariable(mw,'native_inner_mode');
s.inner_period=double(getVariable(mw,'native_inner_period'));
s.control_period=double(getVariable(mw,'native_control_period'));
s.shrink=double(getVariable(mw,'native_torque_shrink_ratio'));
assert(strcmp(s.template.authority_certificate_mode,'analytic_full'),'deploy:gate', ...
 'G3: certificate mode is %s',s.template.authority_certificate_mode);
assert(s.template.authority_torque_shrink_ratio==0 && s.shrink==0,'deploy:gate', ...
 'G3: the torque box is not at 0 %% shrink');
assert(s.template.max_sqp_iterations==4,'deploy:gate','G3: max_sqp_iterations is not 4');
assert(abs(s.template.dt_ctrl-.2)<1e-12,'deploy:gate','G3: the upper period is not 0.2 s');
assert(strcmp(s.inner_mode,'feedback_predictive'),'deploy:gate','G3: inner mode is %s', ...
 s.inner_mode);
assert(abs(s.inner_period-.02)<1e-12,'deploy:gate','G3: the inner period is not 0.02 s');
for name={'native_controller_template','reference_window','disturbance_schedule'}
 assert(hasVariable(mw,name{1}),'deploy:gate', ...
  'G3: the model workspace lost its %s payload',name{1});
end
fprintf(['STRUCTURE ok: inputs from %s; previous command %.3f s; CMG rate -> %s; ' ...
 'servo %g s; physics %s s; mode %s/%s; shrink %g; sqp %g; upper period %g s\n'], ...
 strjoin(s.blocks,', '),s.previous_command_sample_time,s.cmg_command_port, ...
 s.servo_sample_time,s.fixed_step,s.template.authority_certificate_mode,s.inner_mode, ...
 s.shrink,s.template.max_sqp_iterations,s.template.dt_ctrl);
end
% ------------------------------------------------------------------------------
function s=run_and_compare(production,duration,archive,folder,v)
if ~exist(folder,'dir'),mkdir(folder);end
[~,m]=fileparts(production);
load_system(production);
mw=get_param(m,'ModelWorkspace');assignin(mw,'native_results_dir',folder);
si=Simulink.SimulationInput(m);
si=si.setModelParameter('StopTime',num2str(duration));
si=si.setVariable('native_results_dir',folder,'Workspace',m);
wall=tic;r=sim(si);wall_s=toc(wall);
ref=load(archive);
[dcmd,dstate]=compare_prefix(r,ref.r,duration);
s=struct('duration_s',duration,'wall_s',wall_s,'max_command_difference',dcmd, ...
 'max_state_difference',dstate,'samples',numel(r.commands.Time));
assert(dcmd<=1e-9,'deploy:gate','G4: the %g s run differs from F20-B commands by %.3e', ...
 duration,dcmd);
assert(dstate<=1e-9,'deploy:gate','G4: the %g s run differs from F20-B states by %.3e', ...
 duration,dstate);
t=r.commands.Time(:);
assert(numel(t)==round(duration/.001)+1,'deploy:gate','G4: %d samples for %g s',numel(t),duration);
assert(max(abs(t-r.tau_actual.Time(:)))<1e-12,'deploy:gate','G4: the logs are not on one clock');
[~,cmd]=values(r.commands);[~,actual]=values(r.tau_actual);
% r.commands carries [tau_cmd(3); nu_cmd(4)] in the B arm; tau_actual is the
% aerodynamic moment only, so the torque metrics use the first three columns.
cmd=cmd(:,1:3);actual=actual(:,1:3);
[~,delta]=values(r.delta_actual);[~,rate]=values(r.delta_dot);
[~,x]=values(r.states);
assert(all(isfinite(cmd(:))) && all(isfinite(actual(:))) && all(isfinite(x(:))), ...
 'deploy:gate','G4: non-finite log values');
for axis=1:3
 assert(max(abs(delta(:,axis)))<=v.delta_max(axis)+1e-9,'deploy:gate', ...
  'G4: axis %d deflection leaves its own limit',axis);
 assert(max(abs(rate(:,axis)))<=v.delta_dot_max(axis)+1e-9,'deploy:gate', ...
  'G4: axis %d servo rate leaves its own limit',axis);
end
s.limits_ok=true;s.delta_max_abs=max(abs(delta),[],1);s.rate_max_abs=max(abs(rate),[],1);
s.counts=trigger_counts(folder);
assert(s.counts.mpc_cycles==round(duration/.2)+1,'deploy:gate', ...
 'G4: %d upper cycles for %g s',s.counts.mpc_cycles,duration);
assert(s.counts.fallbacks==0,'deploy:gate','G4: %d upper safe fallbacks',s.counts.fallbacks);
if duration>=60
 assert(s.counts.inner_ticks==round(duration/.02)+1,'deploy:gate', ...
  'G4: %d inner ticks for %g s',s.counts.inner_ticks,duration);
 assert(s.counts.inner_fallbacks==0,'deploy:gate','G4: %d inner safe fallbacks', ...
  s.counts.inner_fallbacks);
end
s.torque=struct('cmd_rms_Nm',sqrt(mean(cmd.^2,1)),'err_rms_Nm',sqrt(mean((cmd-actual).^2,1)), ...
 'composite_err_rms_Nm',sqrt(mean((cmd(:)-actual(:)).^2)));
if duration>=60
 s.attitude=attitude_metrics(t,x);
% Independent recomputation must land on the archived F20-B figures 0.6627 / 2.2713.
 assert(abs(s.attitude.rms_deg-0.6627)<5e-3,'deploy:gate', ...
  'G4: attitude RMS is %.4f deg, expected about 0.6627',s.attitude.rms_deg);
 assert(abs(s.attitude.peak_deg-2.2713)<5e-3,'deploy:gate', ...
  'G4: attitude peak is %.4f deg, expected about 2.2713',s.attitude.peak_deg);
end
save(fullfile(folder,sprintf('production_%gs.mat',duration)),'s');
fprintf(['RUN %gs ok: cmd diff %.3e state diff %.3e samples %d cycles %d inner %d ' ...
 'fallbacks %d/%d defl_max %s rate_max %s'],duration,dcmd,dstate,s.samples, ...
 s.counts.mpc_cycles,s.counts.inner_ticks,s.counts.fallbacks,s.counts.inner_fallbacks, ...
 mat2str(s.delta_max_abs,6),mat2str(s.rate_max_abs,6));
if isfield(s,'attitude')
 fprintf(' att RMS %.4f peak %.4f deg',s.attitude.rms_deg,s.attitude.peak_deg);
end
fprintf(' wall %.1fs\n',wall_s);
safe_close(m);
end
% ------------------------------------------------------------------------------
function s=trigger_counts(folder)
a=dir(fullfile(folder,'*_authority.mat'));
assert(numel(a)==1,'deploy:gate','expected one MPC diagnostic file in %s (found %d)', ...
 folder,numel(a));
d=load(fullfile(folder,a(1).name));
s.mpc_cycles=numel(d.diagnostics);
s.fallbacks=0;
for k=1:numel(d.diagnostics),s.fallbacks=s.fallbacks+double(d.diagnostics{k}.fallback);end
f=dir(fullfile(folder,'*_torque_feedback.mat'));
if isempty(f)
 s.inner_ticks=NaN;s.inner_fallbacks=NaN;s.inner_max_ms=NaN;
else
 g=load(fullfile(folder,f(1).name));
 s.inner_ticks=numel(g.diagnostics.time);s.inner_fallbacks=g.summary.fallbacks;
 s.inner_max_ms=g.summary.compute_ms_max;s.inner_mean_ms=g.summary.compute_ms_mean;
 s.inner_over_20ms=g.summary.over_20ms;
end
end
% ------------------------------------------------------------------------------
function [dcmd,dstate]=compare_prefix(fresh,reference,duration)
[ta,a]=values(fresh.commands);[tb,b]=values(reference.commands);
[sx,xa]=values(fresh.states);[sy,xb]=values(reference.states);
if duration>0
 ka=ta<=duration+1e-12;ta=ta(ka);a=a(ka,:);
 kb=tb<=duration+1e-12;tb=tb(kb);b=b(kb,:);
 ka=sx<=duration+1e-12;xa=xa(ka,:);
 kb=sy<=duration+1e-12;xb=xb(kb,:);
end
assert(numel(ta)==numel(tb),'deploy:gate','command logs have %d/%d samples',numel(ta),numel(tb));
assert(max(abs(ta(:)-tb(:)))<1e-12,'deploy:gate','command logs are on different time grids');
assert(isequal(size(xa),size(xb)),'deploy:gate','state logs have different shapes');
dcmd=max(abs(a(:)-b(:)));
dstate=max(abs(xa(:)-xb(:)));
end
% ------------------------------------------------------------------------------
function s=attitude_metrics(t,x)
% Same convention as report_torque_margin_feedback and docs/simulink/09..19.
root=repo_root;bank=load(fullfile(root,'simulink','data','online_plan_bank.mat'));
q=eul2quat(x(:,[3 2 1]));
ref=interp1(bank.time(:),squeeze(bank.q_ref(:,1,:))',t,'linear');
ref=ref./vecnorm(ref,2,2);
err=rad2deg(2*acos(min(1,abs(sum(q.*ref,2)))));
s=struct('rms_deg',sqrt(mean(err.^2)),'peak_deg',max(err), ...
 'definition','quaternion minimum rotation angle against online_plan_bank q_ref');
end
% ------------------------------------------------------------------------------
function safe_close(m)
try
 if bdIsLoaded(m),close_system(m,0);end
catch err
 fprintf('CLOSE_SKIPPED %s (%s)\n',m,err.message);
end
end
function [t,data]=values(ts)
data=squeeze(ts.Data);
if size(data,1)~=numel(ts.Time),data=data.';end
t=ts.Time(:);
end
function value=field_or(s,name,default)
value=default;if isfield(s,name),value=s.(name);end
end
function value=checksum_of(file)
value='';
try
 value=Simulink.getFileChecksum(file);
catch
 d=dir(file);value=sprintf('%d_%s',d.bytes,datestr(d.datenum,30));
end
end
function value=git_sha()
[status,value]=system('git rev-parse HEAD');if status~=0,value='unknown';end
value=strtrim(value);
end
function value=current_branch()
[status,value]=system('git rev-parse --abbrev-ref HEAD');if status~=0,value='unknown';end
value=strtrim(value);
end
