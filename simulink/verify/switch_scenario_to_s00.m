function report=switch_scenario_to_s00(model_tag,stamp)
% Rebuild ONE production model with the S00_pitch30 (+30 deg pitch hold) reference bank,
% keeping that model's own controller configuration otherwise untouched.
%   model_tag 'A' -> NearSpace_CMG_FlightControl_SurfaceOnly.slx  (legacy certificate,
%                    0.2 s open-loop allocation, 7 states)
%   model_tag 'B' -> NearSpace_CMG_FlightControl.slx              (analytic_full, F20 0.02 s
%                    torque feedback, 11 states, 0 % torque-box shrink)
root=repo_root;init_paths;p=vehicle_params;v=p.vehicle;
out=fullfile(root,'simulink','results','scenario_switch',stamp,model_tag);
if ~exist(out,'dir'),mkdir(out);end
switch upper(model_tag)
 case 'A'
  enabled=false;bank_name='S00_pitch30_A_bank.mat';cert='legacy';inner='legacy';
  base='NearSpace_CMG_FlightControl_SurfaceOnly';exp_prev=0.2;exp_inner=0.2;
 case 'B'
  enabled=true;bank_name='S00_pitch30_B_bank.mat';cert='analytic_full';inner='feedback_predictive';
  base='NearSpace_CMG_FlightControl';exp_prev=0.02;exp_inner=0.02;
 otherwise,error('model_tag must be A or B');
end
bank=fullfile(root,'tmp','banks',bank_name);
assert(isfile(bank),'switch:bank','missing bank %s',bank);
target=fullfile(root,'simulink','data','online_plan_bank.mat');
for attempt=1:8
 try,copyfile(bank,target);break;catch err,if attempt==8,rethrow(err);end,pause(2);end
end
% reference of the bank that is about to be baked in
bb=load(bank);pt=pitch_of(bb.q_ref);
expected=struct('samples',numel(bb.time),'t_end_s',bb.time(end), ...
 'pitch_start_deg',pt(1),'pitch_peak_deg',max(pt),'pitch_peak_t_s',bb.time(find(pt==max(pt),1)));
fprintf('SWITCH %s: bank %s (%d samples, %.1f s, pitch %.2f -> %.2f @ %.1f s)\n', ...
 model_tag,bank_name,expected.samples,expected.t_end_s,expected.pitch_start_deg, ...
 expected.pitch_peak_deg,expected.pitch_peak_t_s);
build_closedloop(60,true,enabled,'','online_predicted',false,4,.2,cert,0,inner);
file=fullfile(root,'simulink','models',[base '.slx']);
assert(isfile(file),'switch:build','model was not written: %s',file);
% ---- gates on the file that was just written ---------------------------------
load_system(file);m=base;mw=get_param(m,'ModelWorkspace');
t=getVariable(mw,'native_controller_template');
cert_now=fielddef(t,'authority_certificate_mode','<absent>');
shrink_now=fielddef(t,'authority_torque_shrink_ratio',-1);
b=getVariable(mw,'native_reference_bank');
pt2=pitch_of(b.q_ref);
nref=min(size(b.q_ref,3),size(bb.q_ref,3));
qdiff=max(abs(squeeze(b.q_ref(:,:,1:nref))-squeeze(bb.q_ref(:,:,1:nref))),[],'all');
checks=struct('model',m,'bytes',dir(file).bytes,'cert',cert_now,'shrink',shrink_now, ...
 'dt_ctrl',double(t.dt_ctrl),'sqp',double(t.max_sqp_iterations),'nx',double(t.nx),'N',double(t.N), ...
 'inner_mode',var_or(mw,'native_inner_mode','<absent>'), ...
 'inner_period',num_or(mw,'native_inner_period',-1), ...
 'previous_command_dt',str2double(get_param([m '/Previous_Allocation_Command'],'SampleTime')), ...
 'legacy_block',~isempty(find_system(m,'SearchDepth',1,'Name','Surface_control_allocation')), ...
 'feedback_block',~isempty(find_system(m,'SearchDepth',1,'Name','Torque_feedback_allocation')), ...
 'cmg_present',has_cmg(m), ...
 'fixed_step',str2double(get_param(m,'FixedStep')),'stop_time',get_param(m,'StopTime'), ...
 'bank_samples',numel(b.time),'bank_t_end_s',b.time(end), ...
 'bank_pitch_start_deg',pt2(1),'bank_pitch_peak_deg',max(pt2), ...
 'bank_pitch_peak_t_s',b.time(find(pt2==max(pt2),1)), ...
 'bank_qref_maxdiff',qdiff);
assert(checks.bank_samples==expected.samples,'switch:gate','bank was not baked in');
assert(checks.bank_qref_maxdiff<1e-12,'switch:gate', ...
 'baked reference differs from the exported bank by %.3e',checks.bank_qref_maxdiff);
assert(abs(checks.bank_t_end_s-expected.t_end_s)<1e-9,'switch:gate','baked bank horizon changed');
assert(strcmp(checks.cert,cert),'switch:gate','certificate mode is %s',checks.cert);
assert(abs(checks.dt_ctrl-.2)<1e-12 && checks.sqp==4,'switch:gate','MPC settings changed');
assert(abs(checks.previous_command_dt-exp_prev)<1e-12,'switch:gate', ...
 'Previous_Allocation_Command is %g s',checks.previous_command_dt);
assert(abs(checks.fixed_step-.001)<1e-9,'switch:gate','physics step changed');
assert(checks.cmg_present==enabled,'switch:gate','CMG presence does not match the arm');
fprintf(['SWITCH %s gates ok: %d bytes, cert %s, shrink %g, inner %s/%.3f s, ' ...
 'prev %.3f s, legacy_block %d, feedback_block %d, CMG %d, bank pitch %.2f->%.2f@%.1f\n'], ...
 model_tag,checks.bytes,checks.cert,checks.shrink,checks.inner_mode,checks.inner_period, ...
 checks.previous_command_dt,checks.legacy_block,checks.feedback_block,checks.cmg_present, ...
 checks.bank_pitch_start_deg,checks.bank_pitch_peak_deg,checks.bank_pitch_peak_t_s);
close_system(m,0);
% ---- run it (2 s smoke + 60 s) ----------------------------------------------
runs={};
for duration=[2 60]
 folder=fullfile(out,sprintf('run_%gs',duration));if ~exist(folder,'dir'),mkdir(folder);end
 load_system(file);
 mw=get_param(m,'ModelWorkspace');assignin(mw,'native_results_dir',folder);
 si=Simulink.SimulationInput(m);
 si=si.setModelParameter('StopTime',num2str(duration));
 si=si.setVariable('native_results_dir',folder,'Workspace',m);
 si=si.setModelParameter('CaptureErrors','on');
 wall=tic;r=sim(si);wall_s=toc(wall);
 em='';try,em=r.ErrorMessage;catch,end
 save(fullfile(folder,'closedloop.mat'),'r','wall_s');
 t=tvec(r);x=vals(r.states);e=err_series(t,x,bb);
 cmd=vals(r.commands);cmd=cmd(:,1:3);act=vals(r.tau_actual);act=act(:,1:3);
 delta=vals(r.delta_actual);rate=vals(r.delta_dot);
 win=t<=50+1e-12;
 s=struct('duration_s',duration,'wall_s',wall_s,'samples',numel(t),'t_end_s',t(end), ...
  'aborted',~isempty(em),'error_message',em, ...
  'att_window_rms_deg',rms_(e(win)),'att_window_peak_deg',max(e(win)), ...
  'att_full_rms_deg',rms_(e),'att_full_peak_deg',max(e), ...
  'torque_pitch_err_rms_Nm',rms_(cmd(:,2)-act(:,2)), ...
  'torque_composite_err_rms_Nm',rms_(reshape(cmd-act,[],1)), ...
  'defl_sat_pct',100*mean(abs(delta)>=v.delta_max(:)'-1e-5,1), ...
  'rate_sat_pct',100*mean(abs(rate)>=v.delta_dot_max(:)'-1e-5,1), ...
  'counts',run_counts(folder));
 runs{end+1}=s; %#ok<AGROW>
 fprintf(['SWITCH %s %gs: t_end %.3f s aborted %d, att window RMS %.4f deg (peak %.3f), ' ...
  'pitch tau err %.4f N.m, defl sat %.1f%%, cycles %d, fallbacks %d\n'], ...
  model_tag,duration,s.t_end_s,s.aborted,s.att_window_rms_deg,s.att_window_peak_deg, ...
  s.torque_pitch_err_rms_Nm,max(s.defl_sat_pct),s.counts.mpc_cycles, ...
  s.counts.fallbacks+s.counts.inner_fallbacks);
 close_system(m,0);
end
report=struct('model_tag',model_tag,'stamp',stamp,'output',out,'model_file',file, ...
 'config',checks,'expected_reference',expected,'runs',cell2mat(runs),'git_sha',git_sha());
fid=fopen(fullfile(out,'switch_report.json'),'w');
fprintf(fid,'%s',jsonencode(report,'PrettyPrint',true));fclose(fid);
fprintf('SWITCH_%s_DONE\n',model_tag);
end
function v=fielddef(s,n,d),v=d;if isfield(s,n),v=s.(n);end,end
function v=has_cmg(m)
v=false;
try,v=~isempty(find_system([m '/Native_Plant/CMG'],'SearchDepth',0));catch,end
end
function v=var_or(mw,n,d),v=d;if hasVariable(mw,n),v=getVariable(mw,n);end,end
function v=num_or(mw,n,d),v=d;if hasVariable(mw,n),v=double(getVariable(mw,n));end,end
function p=pitch_of(qref)
nk=size(qref,3);p=zeros(1,nk);
for k=1:nk,q=qref(:,1,k);q=q/norm(q);p(k)=asin(max(-1,min(1,2*(q(1)*q(3)-q(4)*q(2)))));end
p=rad2deg(p);
end
function t=tvec(r),t=r.commands.Time(:);end
function d=vals(ts)
d=squeeze(ts.Data);if size(d,1)~=numel(ts.Time),d=d.';end
end
function e=err_series(t,x,bank)
q=eul2quat(x(:,[3 2 1]));
ref=interp1(bank.time(:),squeeze(bank.q_ref(:,1,:))',t,'linear');ref=ref./vecnorm(ref,2,2);
e=rad2deg(2*acos(min(1,abs(sum(q.*ref,2)))));
end
function s=run_counts(folder)
a=dir(fullfile(folder,'*_authority.mat'));
s=struct('mpc_cycles',NaN,'fallbacks',NaN,'inner_ticks',NaN,'inner_fallbacks',0,'cycle_ms_mean',NaN,'cycle_ms_max',NaN);
if numel(a)==1
 d=load(fullfile(folder,a(1).name));c=zeros(1,numel(d.diagnostics));fb=0;
 for k=1:numel(d.diagnostics),c(k)=d.diagnostics{k}.elapsed_s;fb=fb+double(d.diagnostics{k}.fallback);end
 s.mpc_cycles=numel(d.diagnostics);s.fallbacks=fb;s.cycle_ms_mean=1000*mean(c);s.cycle_ms_max=1000*max(c);
end
f=dir(fullfile(folder,'*_torque_feedback.mat'));
if numel(f)==1,g=load(fullfile(folder,f(1).name));s.inner_ticks=numel(g.diagnostics.time);s.inner_fallbacks=g.summary.fallbacks;end
end
function v=rms_(x),v=sqrt(mean(x(:).^2));end
function v=git_sha()
[st,v]=system('git rev-parse HEAD');if st~=0,v='unknown';end
v=strtrim(v);
end
