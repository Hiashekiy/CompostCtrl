function report_torque_tracking
% Torque tracking error of the MPC command against the surface-produced aerodynamic moment,
% for the analytic_full certificate runs and for the legacy runs of the same commit.
% e(t) = tau_cmd(t) - tau_actual(t), no delay compensation, no interpolation of the
% discrete command (zero-order hold by construction; sample-by-sample comparison).
root=repo_root;init_paths;
base=fullfile(root,'simulink','results','online_authority','fast_certificate');
out=fullfile(base,'torque_tracking');if ~exist(out,'dir'),mkdir(out);end
p=vehicle_params;v=p.vehicle;
sets={'analytic_full',fullfile(out,'analytic_full_%s_60s'); ...
      'legacy',fullfile(base,'ablation','legacy_%s_60s')};
rows={};report=struct;alignment=struct;
for s=1:size(sets,1)
 mode=sets{s,1};
 for arm={'A','B'}
  folder=strrep(sets{s,2},'%s',arm{1});
  file=fullfile(folder,[arm{1} '_closedloop.mat']);
  assert(isfile(file),'torque:report','missing run %s',file);
  d=load(file);r=d.r;
  [t,cmd,actual]=aligned(r);
  alignment.(mode).(arm{1})=struct('samples',numel(t),'dt_median',median(diff(t)), ...
   'time_identical',isequal(r.commands.Time,r.tau_actual.Time), ...
   'cmd_hold_steps',nnz(abs(diff(cmd(:,2)))>1e-12), ...
   'first_time',t(1),'last_time',t(end));
  err=cmd-actual;
  rms=@(z)sqrt(mean(z.^2,1));
  cmd_rms=rms(cmd);err_rms=rms(err);err_max=max(abs(err),[],1);
  rel=100*err_rms./max(cmd_rms,realmin);
  note='';
  if any(cmd_rms<1e-3),note='部分轴目标力矩 RMS 接近 0，相对误差不可靠';end
  for axis=1:3
   rows{end+1}=struct('mode',mode,'arm',arm{1},'axis',axis_label(axis), ...
    'cmd_rms_Nm',cmd_rms(axis),'err_rms_Nm',err_rms(axis),'rel_rms_pct',rel(axis), ...
    'err_max_Nm',err_max(axis),'cmd_abs_max_Nm',max(abs(cmd(:,axis))), ...
    'note',note); %#ok<AGROW>
  end
  dz=surface_usage(v,r);
  composition=composition_check(v,r);
  report.(mode).(arm{1})=struct('cmd_rms_Nm',cmd_rms,'err_rms_Nm',err_rms, ...
   'rel_rms_pct',rel,'err_max_Nm',err_max,'total_err_rms_Nm',rms(err(:)), ...
   'total_rel_pct',100*rms(err(:))/max(rms(cmd(:)),realmin), ...
   'segments',segment_rms(t,err),'surface',dz,'composition',composition,'note',note);
  plot_axes(t,cmd,actual,mode,arm{1},out);
 end
end
comparison=struct;
for arm={'A','B'}
 comparison.(arm{1})=struct('analytic_rel_pct',report.analytic_full.(arm{1}).rel_rms_pct, ...
  'legacy_rel_pct',report.legacy.(arm{1}).rel_rms_pct, ...
  'analytic_err_rms_Nm',report.analytic_full.(arm{1}).err_rms_Nm, ...
  'legacy_err_rms_Nm',report.legacy.(arm{1}).err_rms_Nm, ...
  'max_abs_difference_Nm',max(abs(report.analytic_full.(arm{1}).err_rms_Nm- ...
   report.legacy.(arm{1}).err_rms_Nm)));
end
t=struct2table(cell2mat(rows));
writetable(t,fullfile(out,'torque_tracking_summary.csv'));
fid=fopen(fullfile(out,'torque_tracking_summary.json'),'w');
fprintf(fid,'%s',jsonencode(struct('rows',cell2mat(rows),'alignment',alignment, ...
 'per_mode',report,'analytic_vs_legacy',comparison),'PrettyPrint',true));fclose(fid);
disp(t(:,{'mode','arm','axis','cmd_rms_Nm','err_rms_Nm','rel_rms_pct','err_max_Nm'}));
for arm={'A','B'}
 fprintf('%-2s analytic total %.4g N.m (%.2f%%) vs legacy %.4g N.m (%.2f%%)\n',arm{1}, ...
  report.analytic_full.(arm{1}).total_err_rms_Nm,report.analytic_full.(arm{1}).total_rel_pct, ...
  report.legacy.(arm{1}).total_err_rms_Nm,report.legacy.(arm{1}).total_rel_pct);
end
fprintf('TORQUE_TRACKING_REPORT_DONE %s\n',out);
end
function [t,cmd,actual]=aligned(r)
% Both logs come from the same 1 ms solver clock; the MPC command is held between control
% instants by the S-function output, so a sample-by-sample difference is the ZOH comparison.
tc=r.commands.Time(:);ta=r.tau_actual.Time(:);
cmd=values(r.commands);actual=values(r.tau_actual);
cmd=cmd(:,1:3);actual=actual(:,1:3);
assert(numel(tc)==numel(ta)&&max(abs(tc-ta))<1e-12,'torque:align','logs are not on one clock');
t=tc;
end
function data=values(ts)
data=squeeze(ts.Data);if size(data,1)~=numel(ts.Time),data=data';end
end
function label=axis_label(axis)
names={'roll','pitch','yaw'};label=names{axis};
end
function seg=segment_rms(t,err)
edges=[0 15 30 45 61];seg=zeros(4,3);
for k=1:4
 mask=t>=edges(k)&t<edges(k+1);
 if any(mask),seg(k,:)=sqrt(mean(err(mask,:).^2,1));end
end
end
function dz=surface_usage(v,r)
delta=values(r.delta_actual);rate=values(r.delta_dot);
limit=v.delta_max(:)';rate_limit=v.delta_dot_max(:)';
dz=struct('deflection_abs_max_rad',max(abs(delta),[],1), ...
 'deflection_p999_rad',prctile_(abs(delta),99.9), ...
 'deflection_saturation_pct',100*mean(abs(delta)>=limit-1e-5,1), ...
 'rate_abs_max_radps',max(abs(rate),[],1), ...
 'rate_saturation_pct',100*mean(abs(rate)>=rate_limit-1e-5,1));
end
function value=prctile_(data,q)
data=sort(abs(data),1);index=max(1,min(size(data,1),ceil(q/100*size(data,1))));value=data(index,:);
end
function plot_axes(t,cmd,actual,mode,arm,out)
names={'roll','pitch','yaw'};
figure('Visible','off');
for axis=1:3
 subplot(3,1,axis);
 plot(t,cmd(:,axis),'b','LineWidth',1);hold on;plot(t,actual(:,axis),'r','LineWidth',1);grid on;
 ylabel([names{axis} ' (N.m)']);legend('command','actual','Location','best');
 title(sprintf('%s mode, arm %s',mode,arm));
end
xlabel('time (s)');
file=fullfile(out,sprintf('%s_%s_torque.png',mode,arm));
try
 exportgraphics(gcf,file);
catch err
 warning('TORQUE_PLOT_SKIPPED %s',err.message);
end
close(gcf);
figure('Visible','off');
for axis=1:3
 subplot(3,1,axis);plot(t,cmd(:,axis)-actual(:,axis),'k');grid on;
 ylabel([names{axis} ' err (N.m)']);
end
xlabel('time (s)');sgtitle(sprintf('torque tracking error, %s, arm %s',mode,arm));
try
 exportgraphics(gcf,fullfile(out,sprintf('%s_%s_error.png',mode,arm)));
catch err
 warning('TORQUE_PLOT_SKIPPED %s',err.message);
end
close(gcf);
end
function out=composition_check(v,r)
% Verify what tau_actual actually contains: recompute tau_nat + B*delta from the logged
% air data, body rate and measured deflection. If it matches, tau_actual is purely the
% aerodynamic moment from the surfaces (no CMG, no external disturbance).
t=r.states.Time;state=values(r.states);air=values(r.flight_conditions);
delta=values(r.delta_actual);actual=values(r.tau_actual);
if size(air,2)<4,air(:,4)=0;end
stride=max(1,floor(numel(t)/3000));
worst=0;count=0;
for j=1:stride:numel(t)
 if ~all(isfinite([air(j,1:4),state(j,4:6),delta(j,:),actual(j,1:3)])),continue;end
 [natural,B]=online_aero_moment(v,air(j,1:4)',state(j,4:6)',delta(j,:)');
 worst=max(worst,max(abs(natural+B*delta(j,:)'-actual(j,1:3)')));
 count=count+1;
end
out=struct('samples',count,'max_abs_residual_Nm',worst, ...
 'definition','tau_actual versus tau_nat+B*delta from logged air data, body rate and measured deflection');
end
