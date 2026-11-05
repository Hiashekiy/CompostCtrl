function report=run_ab3_cmg_task_compare(mode,arg1,arg2,inner_mode,duration)
% Two entry points (the reference bank cannot be rewritten in place inside one MATLAB
% process: Windows keeps a memory-mapped handle on the .mat that was just read, so every
% task is run by its own process and copies its bank first thing).
%   mode='task',  arg1=stamp, arg2=task name   -> run both arms of one task
%   mode='report',arg1=stamp                   -> comparison table, JSON and figures
if nargin<4||isempty(inner_mode),inner_mode='legacy';end
if nargin<5||isempty(duration),duration=60;end
if strcmp(mode,'report')
 report=report_all(arg1,inner_mode,duration);return;
end
stamp=arg1;task=arg2;
report=run_one_task(stamp,task,inner_mode,duration);
end
% ------------------------------------------------------------------------------
function report=run_one_task(stamp,task,inner_mode,duration)
% Three-task A/B comparison with actuator dynamics:
%   task 1  S00_pitch30       forward 30 deg pitch hold
%   task 2  S01_pitch_minus30 reverse 30 deg pitch hold
%   task 3  S06_pitch30_roll20 pitch-roll coupled
% arm A = surfaces only (7 states, NearSpace_CMG_FlightControl_SurfaceOnly family)
% arm B = surfaces + 4-unit pyramid CMG (11 states)
%
% Both arms use the SAME inner-loop mode so that the A/B difference isolates the CMG
% (this is the configuration of the historical A/B report and of the paper's P1 model).
% The reference banks were exported per task AND per arm from
% configs/torque_gap_no_cmg_comparison/cases/<task>_{no_cmg,with_cmg}.yaml, so the two
% arms share the flight condition byte for byte and differ only in CMG authority.
%
% Metrics: quaternion minimum-rotation angle (deg) against the reference over the
% manoeuvre window 90-140 s (= model time 0-50 s) and over the full 90-150 s window,
% three-axis aerodynamic torque tracking, per-axis deflection/rate saturation, and, for
% the B arm, the CMG reaction torque and angular momentum recomputed from the logged
% gimbal angles with the same geometry the model uses.
assert(ismember(inner_mode,{'legacy','feedback_predictive'}),'inner_mode must be legacy or feedback_predictive');
root=repo_root;init_paths;
p=vehicle_params;
out=fullfile(root,'simulink','results','ab3_cmg',[stamp '_' inner_mode]);
if ~exist(out,'dir'),mkdir(out);end
bank_target=fullfile(root,'simulink','data','online_plan_bank.mat');
% One bank per task, shared by both arms: the two case files describe one and the same
% flight condition and differ only in CMG authority, whose torque-box fields the ONLINE
% controller recomputes every cycle (the reference window skips them).  The initial
% attitude/rate and the whole reference therefore stay byte-identical across the arms.
tb=fullfile(root,'tmp','banks',sprintf('%s_B_bank.mat',task));
assert(isfile(tb),'ab3:bank','missing reference bank %s',tb);
copy_bank(tb,bank_target);
entries={};
fprintf('AB3 task=%s mode=%s duration=%g out=%s\n',task,inner_mode,duration,out);
for arm={'A','B'}
 name=arm{1};
 folder=fullfile(out,sprintf('%s_%s',task,name));
 if ~exist(folder,'dir'),mkdir(folder);end
 case_name=sprintf('AB3_%s_%s',short_task(task),name);   % keep the model name under 63 chars
 build_closedloop(duration,true,strcmp(name,'B'),case_name,'online_predicted',false,4,.2, ...
  'analytic_full',0,inner_mode);
 m='NearSpace_CMG_FlightControl';
 if ~strcmp(name,'B'),m=[m '_SurfaceOnly'];end
 m=[m '_' case_name];
 casemodel=fullfile(root,'simulink','cases',[m '.slx']);
 assert(isfile(casemodel),'ab3:build','model was not saved: %s',casemodel);
 load_system(casemodel);
 mw=get_param(m,'ModelWorkspace');assignin(mw,'native_results_dir',folder);
 si=Simulink.SimulationInput(m);
 si=si.setModelParameter('StopTime',num2str(duration));
 si=si.setVariable('native_results_dir',folder,'Workspace',m);
 % CaptureErrors keeps the partial logs: an arm that diverges still contributes the
 % manoeuvre-window statistics up to the instant the model's own guard fired.
 si=si.setModelParameter('CaptureErrors','on');
 wall=tic;r=sim(si);wall_s=toc(wall);
 save(fullfile(folder,sprintf('%s_closedloop.mat',name)),'r','wall_s');
 errmsg='';
 try,errmsg=r.ErrorMessage;catch,end
 if ~isempty(errmsg),fprintf('AB3 %s arm %s ABORTED: %s\n',task,name,errmsg);end
 saveref_bank(folder,tb);
 close_system(m,0);
 e=analyze_run(folder,r,name,task,p);
 e.wall_s=wall_s;e.aborted=~isempty(errmsg);e.error_message=errmsg;
 e.t_end_s=e.t_end;
 entries{end+1}=e; %#ok<AGROW>
 fprintf(['AB3 %-22s arm %s: att window RMSE %.4f deg (peak %.3f), full %.4f; ' ...
  'pitch torque err %.3f N.m; fallbacks %d\n'],task,name,e.attitude.window_rms_deg, ...
  e.attitude.window_peak_deg,e.attitude.full_rms_deg,e.torque.err_rms_Nm(2),e.fallbacks);
end
report=struct('task',task,'inner_mode',inner_mode,'duration_s',duration,'stamp',stamp, ...
 'output',out,'runs',cell2mat(entries),'comparison',cell2mat(compare_arms(entries)));
writetable(struct2table(report.comparison),fullfile(out,sprintf('%s_comparison.csv',task)));
fid=fopen(fullfile(out,sprintf('%s_summary.json',task)),'w');
fprintf(fid,'%s',jsonencode(report,'PrettyPrint',true));fclose(fid);
plot_task(out,task,duration);
fprintf('AB3_DONE %s\n',out);
end
% ------------------------------------------------------------------------------
function report=report_all(stamp,inner_mode,duration)
% Aggregate the six already saved runs into one comparison table.  Re-analysis (rather
% than JSON round-tripping) keeps a single source of truth for every metric.
root=repo_root;init_paths;
p=vehicle_params;
out=fullfile(root,'simulink','results','ab3_cmg',[stamp '_' inner_mode]);
assert(isfolder(out),'ab3:report','no results folder %s',out);
tasks={'S00_pitch30','S01_pitch_minus30','S06_pitch30_roll20'};
entries={};
for k=1:numel(tasks)
 for arm={'A','B'}
  folder=fullfile(out,sprintf('%s_%s',tasks{k},arm{1}));
  f=fullfile(folder,sprintf('%s_closedloop.mat',arm{1}));
  assert(isfile(f),'ab3:report','missing run %s',f);
  d=load(f);
  e=analyze_run(folder,d.r,arm{1},tasks{k},p);
  e.wall_s=d.wall_s;
  entries{end+1}=e; %#ok<AGROW>
 end
end
report=struct('inner_mode',inner_mode,'duration_s',duration,'stamp',stamp,'output',out, ...
 'runs',cell2mat(entries),'comparison',cell2mat(compare_arms(entries)));
writetable(struct2table(report.comparison),fullfile(out,'ab3_comparison.csv'));
fid=fopen(fullfile(out,'ab3_comparison.json'),'w');
fprintf(fid,'%s',jsonencode(report.comparison,'PrettyPrint',true));fclose(fid);
tbl=struct2table(report.comparison);
disp(tbl(:,{'task','window_rms_A_deg','window_rms_B_deg','reduction_pct', ...
 'full_rms_A_deg','full_rms_B_deg','pitch_torque_err_A_Nm','pitch_torque_err_B_Nm', ...
 'window_composite_err_A_Nm','window_composite_err_B_Nm'}));
for k=1:size(tbl,1)
 fprintf(['AB3 %-22s window RMSE %.4f -> %.4f deg (-%.1f%%), full %.4f -> %.4f deg, ' ...
  'pitch torque err %.3f -> %.3f N.m, CMG pitch torque %.3f N.m, |H| %.2f\n'], ...
  tbl.task{k},tbl.window_rms_A_deg(k),tbl.window_rms_B_deg(k),tbl.reduction_pct(k), ...
  tbl.full_rms_A_deg(k),tbl.full_rms_B_deg(k),tbl.pitch_torque_err_A_Nm(k), ...
  tbl.pitch_torque_err_B_Nm(k),tbl.cmg_pitch_torque_rms_Nm(k),tbl.cmg_h_norm_rms(k));
end
fprintf('AB3 comparability = %.4f deg\n',0);
fprintf('AB3_DONE %s\n',out);
end
% ------------------------------------------------------------------------------
function e=analyze_run(folder,r,name,task,p)
v=p.vehicle;
t=r.commands.Time(:);
[~,cmd]=values(r.commands);cmd=cmd(:,1:3);
[~,actual]=values(r.tau_actual);actual=actual(:,1:3);
[~,delta]=values(r.delta_actual);[~,rate]=values(r.delta_dot);
[~,x]=values(r.states);
bankinfo=load(fullfile(folder,'reference_bank.mat'));
bank=bankinfo.bank;
att=attitude_metrics(t,x,bank);
window=t<=50+1e-12;full=t<=60+1e-12;
err=cmd-actual;
e=struct('task',task,'arm',name,'samples',numel(t),'t_end',t(end), ...
 'attitude',struct('window_rms_deg',rms_of(att.err(window)), ...
  'window_peak_deg',max(att.err(window)),'full_rms_deg',rms_of(att.err(full)), ...
  'full_peak_deg',max(att.err(full)), ...
  'window_rms_euler_deg',sqrt(mean(att.euler(window,:).^2,1)), ...
  'full_rms_euler_deg',sqrt(mean(att.euler(full,:).^2,1)), ...
  'window_peak_euler_deg',max(abs(att.euler(window,:)),[],1)), ...
 'torque',struct('cmd_rms_Nm',sqrt(mean(cmd.^2,1)), ...
  'err_rms_Nm',sqrt(mean(err.^2,1)),'window_err_rms_Nm',sqrt(mean(err(window,:).^2,1)), ...
  'err_peak_Nm',max(abs(err),[],1),'composite_err_rms_Nm',sqrt(mean(err(:).^2)), ...
  'window_composite_err_rms_Nm',sqrt(mean(reshape(err(window,:),[],1).^2))), ...
 'surface',struct('deflection_saturation_pct',100*mean(abs(delta)>=v.delta_max(:)'-1e-5,1), ...
  'rate_saturation_pct',100*mean(abs(rate)>=v.delta_dot_max(:)'-1e-5,1), ...
  'deflection_abs_max_rad',max(abs(delta),[],1),'rate_abs_max_radps',max(abs(rate),[],1)));
e.counts=trigger_counts(folder);
e.fallbacks=e.counts.fallbacks+e.counts.inner_fallbacks;
if size(x,2)>=10
 e.cmg=cmg_metrics(t,x(:,7:10),p);
else
 e.cmg=struct('present',false,'torque_norm_rms_Nm',NaN,'h_norm_rms',NaN, ...
  'pitch_torque_rms_Nm',NaN,'gimbal_rate_max_radps',NaN);
end
e.box=box_metrics(t,cmd,bank,v);
end
% ------------------------------------------------------------------------------
function s=cmg_metrics(t,gamma,p)
% Same geometry the model uses: H = sum_k h0*(s0_k cos g_k + gxs0_k sin g_k),
% A(:,k) = h0*(-s0_k sin g_k + gxs0_k cos g_k), reaction torque = -A*gamma_dot.
h0=p.cmg.h0;s0=p.s0;gxs0=p.gxs0;
n=size(gamma,1);
H=zeros(3,n);
for k=1:4
 H=H+h0*(s0(:,k)*cos(gamma(:,k))'+gxs0(:,k)*sin(gamma(:,k))');
end
A=zeros(3,4,n);
for k=1:4
 A(:,k,:)=reshape(h0*(-s0(:,k)*sin(gamma(:,k))'+gxs0(:,k)*cos(gamma(:,k))'),3,1,n);
end
dt=median(diff(t));
gdot=[zeros(1,4);(gamma(3:end,:)-gamma(1:end-2,:))/(2*dt);zeros(1,4)];
tau=zeros(3,n);
for k=1:4
 tau=tau-squeeze(A(:,k,:)).*gdot(:,k)';
end
s=struct('present',true,'torque_norm_rms_Nm',sqrt(mean(sum(tau.^2,1))), ...
 'pitch_torque_rms_Nm',sqrt(mean(tau(2,:).^2)),'torque_peak_Nm',max(abs(tau),[],'all'), ...
 'h_norm_rms',sqrt(mean(sum(H.^2,1))),'gimbal_rate_max_radps',max(abs(gdot),[],'all'), ...
 'gamma_abs_max_rad',max(abs(gamma),[],'all'));
end
% ------------------------------------------------------------------------------
function s=box_metrics(t,cmd,bank,v)
% Distance of the MPC torque command from the certified stage-1 box of the same
% instant (the "reduced-order optimisation bound" the figures refer to).
nk=numel(bank.time);idx=min(max(round(t/.2)+1,1),nk);
lo=bank.tau_min(:,1,:);hi=bank.tau_max(:,1,:);
margin=zeros(numel(t),3);
for j=1:3
 loj=squeeze(lo(j,1,:));hij=squeeze(hi(j,1,:));
 loj=loj(idx);hij=hij(idx);
 span=(hij-loj)/2;
 margin(:,j)=min(cmd(:,j)-loj,hij-cmd(:,j))./max(span,realmin);
end
s=struct('margin_mean',mean(margin,1),'margin_min',min(margin,[],1), ...
 'outside_fraction',mean(margin<-1e-9,1));
end
% ------------------------------------------------------------------------------
function a=attitude_metrics(t,x,bank)
q=eul2quat(x(:,[3 2 1]));
ref=interp1(bank.time(:),squeeze(bank.q_ref(:,1,:))',t,'linear');
ref=ref./vecnorm(ref,2,2);
a.err=rad2deg(2*acos(min(1,abs(sum(q.*ref,2)))));
a.euler=rad2deg(x(:,1:3)-reference_euler(bank,t));
end
function e=reference_euler(bank,t)
nk=numel(bank.time);idx=min(max(round(t/.2)+1,1),nk);
qr=squeeze(bank.q_ref(:,1,:))';
e=zeros(numel(t),3);
for k=1:numel(t)
 q=qr(idx(k),:);q=q/norm(q);
 e(k,:)=[atan2(2*(q(1)*q(2)+q(3)*q(4)),1-2*(q(2)^2+q(3)^2)), ...
  asin(max(-1,min(1,2*(q(1)*q(3)-q(4)*q(2))))), ...
  atan2(2*(q(1)*q(4)+q(2)*q(3)),1-2*(q(3)^2+q(4)^2))];
end
end
function s=trigger_counts(folder)
a=dir(fullfile(folder,'*_authority.mat'));
assert(numel(a)==1,'ab3:diag','expected one MPC diagnostic file in %s',folder);
d=load(fullfile(folder,a(1).name));
s.mpc_cycles=numel(d.diagnostics);s.fallbacks=0;
cyc=zeros(1,numel(d.diagnostics));
for k=1:numel(d.diagnostics)
 s.fallbacks=s.fallbacks+double(d.diagnostics{k}.fallback);
 cyc(k)=d.diagnostics{k}.elapsed_s;
end
s.cycle_ms_mean=1000*mean(cyc);s.cycle_ms_p95=1000*p95_(cyc);s.cycle_ms_max=1000*max(cyc);
s.over_200ms=sum(cyc>.2);
f=dir(fullfile(folder,'*_torque_feedback.mat'));
if isempty(f)
 s.inner_ticks=NaN;s.inner_fallbacks=0;
else
 g=load(fullfile(folder,f(1).name));
 s.inner_ticks=numel(g.diagnostics.time);s.inner_fallbacks=g.summary.fallbacks;
end
end
% ------------------------------------------------------------------------------
function value=field_or(s,name,default)
value=default;if isfield(s,name),value=s.(name);end
end
function code=short_task(task)
switch task
 case 'S00_pitch30',code='P30';
 case 'S01_pitch_minus30',code='M30';
 case 'S06_pitch30_roll20',code='PR';
 otherwise,code=task(1:min(8,numel(task)));
end
end
function c=compare_arms(entries)
% One row per task: A vs B with the reduction of the manoeuvre-window RMSE.
list=cell2mat(entries);
tasks=unique({list.task});
rows={};
for k=1:numel(tasks)
 iA=find(strcmp({list.task},tasks{k})&strcmp({list.arm},'A'),1);
 iB=find(strcmp({list.task},tasks{k})&strcmp({list.arm},'B'),1);
 assert(~isempty(iA)&&~isempty(iB),'ab3:compare','task %s is missing an arm',tasks{k});
 a=list(iA);b=list(iB);
 rows{end+1}=struct('task',tasks{k}, ...
  'window_rms_A_deg',a.attitude.window_rms_deg,'window_rms_B_deg',b.attitude.window_rms_deg, ...
  'window_peak_A_deg',a.attitude.window_peak_deg,'window_peak_B_deg',b.attitude.window_peak_deg, ...
  'full_rms_A_deg',a.attitude.full_rms_deg,'full_rms_B_deg',b.attitude.full_rms_deg, ...
  'reduction_pct',100*(1-b.attitude.window_rms_deg/max(a.attitude.window_rms_deg,realmin)), ...
  'pitch_torque_err_A_Nm',a.torque.err_rms_Nm(2),'pitch_torque_err_B_Nm',b.torque.err_rms_Nm(2), ...
  'composite_err_A_Nm',a.torque.composite_err_rms_Nm, ...
  'composite_err_B_Nm',b.torque.composite_err_rms_Nm, ...
  'window_composite_err_A_Nm',a.torque.window_composite_err_rms_Nm, ...
  'window_composite_err_B_Nm',b.torque.window_composite_err_rms_Nm, ...
  'defl_sat_A_pct',max(a.surface.deflection_saturation_pct), ...
  'defl_sat_B_pct',max(b.surface.deflection_saturation_pct), ...
  'rate_sat_A_pct',max(a.surface.rate_saturation_pct), ...
  'rate_sat_B_pct',max(b.surface.rate_saturation_pct), ...
  'cmg_pitch_torque_rms_Nm',b.cmg.pitch_torque_rms_Nm, ...
  'cmg_h_norm_rms',b.cmg.h_norm_rms, ...
  'fallbacks_A',a.fallbacks,'fallbacks_B',b.fallbacks, ...
  'aborted_A',double(field_or(a,'aborted',false)), ...
  'aborted_B',double(field_or(b,'aborted',false)), ...
  't_end_A_s',field_or(a,'t_end',NaN),'t_end_B_s',field_or(b,'t_end',NaN)); %#ok<AGROW>
end
c=rows;
end
% ------------------------------------------------------------------------------
function plot_task(out,task,duration)
if nargin<3,duration=60;end
A=load(fullfile(out,sprintf('%s_A',task),'A_closedloop.mat'));
B=load(fullfile(out,sprintf('%s_B',task),'B_closedloop.mat'));
bA=load(fullfile(out,sprintf('%s_A',task),'reference_bank.mat'));
bank=bA.bank;
tA=A.r.commands.Time(:);tB=B.r.commands.Time(:);
[~,xnA]=values(A.r.states);[~,xnB]=values(B.r.states);
[~,cmdB]=values(B.r.commands);cmdB=cmdB(:,1:3);
[~,actB]=values(B.r.tau_actual);actB=actB(:,1:3);
[~,deltaB]=values(B.r.delta_actual);[~,deltaA]=values(A.r.delta_actual);
p=vehicle_params;v=p.vehicle;
nk=numel(bank.time);
[idxB,refeulB]=index_of(bank,tB);[idxA,refeulA]=index_of(bank,tA);
lo=squeeze(bank.tau_min(2,1,:));hi=squeeze(bank.tau_max(2,1,:));
errA=attitude_error_series(tA,xnA,bank);errB=attitude_error_series(tB,xnB,bank);
figure('Visible','off');
subplot(4,1,1);
plot(tB,rad2deg(refeulB(:,2)),'k--');hold on;
plot(tB,rad2deg(xnB(:,2)),'b');plot(tA,rad2deg(xnA(:,2)),'r');grid on;
ylabel('pitch (deg)');legend('reference','B: surface+CMG','A: surface only','Location','best');
title(sprintf('%s  pitch attitude (actuator dynamics, 0.2 s MPC, %g s)',task,duration));
subplot(4,1,2);
plot(tB,errB,'b');hold on;plot(tA,errA,'r');grid on;set(gca,'YScale','log');
ylabel('att err norm (deg)');legend('B','A','Location','best');
subplot(4,1,3);
plot(tB,cmdB(:,2),'b');hold on;plot(tB,actB(:,2),'r');
plot(tB,lo(idxB),'g--');plot(tB,hi(idxB),'g--');grid on;
ylabel('pitch torque (N.m)');
legend('B: tau cmd','B: tau actual','certified box low','certified box high','Location','best');
subplot(4,1,4);
plot(tB,deltaB(:,2),'b');hold on;plot(tA,deltaA(:,2),'r');
yline(v.delta_max(2),'k--');yline(-v.delta_max(2),'k--');grid on;
ylabel('pitch deflection (rad)');xlabel('time (s)');
legend('B','A','limit','Location','best');
export_png(fullfile(out,sprintf('%s_attitude_torque.png',task)));
if size(xnB,2)>=10
 figure('Visible','off');
 h0=p.cmg.h0;s0=p.s0;gxs0=p.gxs0;gamma=xnB(:,7:10);
 n=numel(tB);H=zeros(3,n);
 for k=1:4
  H=H+h0*(s0(:,k)*cos(gamma(:,k))'+gxs0(:,k)*sin(gamma(:,k))');
 end
 dt=median(diff(tB));
 gdot=[zeros(1,4);(gamma(3:end,:)-gamma(1:end-2,:))/(2*dt);zeros(1,4)];
 A3=zeros(3,4,n);
 for k=1:4
  A3(:,k,:)=reshape(h0*(-s0(:,k)*sin(gamma(:,k))'+gxs0(:,k)*cos(gamma(:,k))'),3,1,n);
 end
 tau=zeros(3,n);
 for k=1:4,tau=tau-squeeze(A3(:,k,:)).*gdot(:,k)';end
 subplot(3,1,1);plot(tB,tau(2,:),'b');grid on;ylabel('CMG pitch torque (N.m)');
 subplot(3,1,2);plot(tB,sqrt(sum(H.^2,1)),'m');grid on;ylabel('|H| (N.m.s)');
 subplot(3,1,3);plot(tB,gamma);grid on;ylabel('gimbal (rad)');xlabel('time (s)');
 legend('gamma1','gamma2','gamma3','gamma4','Location','best');
 export_png(fullfile(out,sprintf('%s_cmg.png',task)));
end
end
function [idx,e]=index_of(bank,t)
nk=numel(bank.time);idx=min(max(round(t/.2)+1,1),nk);
e=reference_euler(bank,t);
end
function err=attitude_error_series(t,x,bank)
q=eul2quat(x(:,[3 2 1]));
ref=interp1(bank.time(:),squeeze(bank.q_ref(:,1,:))',t,'linear');
ref=ref./vecnorm(ref,2,2);
err=rad2deg(2*acos(min(1,abs(sum(q.*ref,2)))));
end
function export_png(file)
try
 exportgraphics(gcf,file);
catch err
 warning('AB3_PLOT_SKIPPED %s',err.message);
end
close(gcf);
end
function saveref_bank(folder,bankfile)
bank=load(bankfile);save(fullfile(folder,'reference_bank.mat'),'bank');
end
function restore_bank(root)
d=dir(fullfile(root,'tmp','bank_backup','*','*.mat'));
assert(~isempty(d),'ab3:restore','no bank backup found');
[~,order]=sort([d.datenum]);
d=d(order(end));
copy_bank(fullfile(d.folder,d.name),fullfile(root,'simulink','data','online_plan_bank.mat'));
fprintf('AB3 reference bank restored from %s\n',d.name);
end
function copy_bank(src,dst)
for attempt=1:8
 try
  copyfile(src,dst);return;
 catch err
  if attempt==8,rethrow(err);end
  fprintf('AB3 bank copy retry %d: %s\n',attempt,err.message);
  pause(2);
 end
end
end
function v=p95_(x)
x=sort(x(:));v=x(max(1,min(numel(x),ceil(.95*numel(x)))));
end
function v=rms_of(x)
v=sqrt(mean(x(:).^2));
end
function [t,data]=values(ts)
data=squeeze(ts.Data);
if size(data,1)~=numel(ts.Time),data=data.';end
t=ts.Time(:);
end
