function report_pitch_scale_repair
root=repo_root;base=fullfile(root,'simulink','results','online_authority');
out=fullfile(base,'pitch_scale_repair');if ~exist(out,'dir'),mkdir(out);end
previous=readtable(fullfile(base,'final_comparison.csv'));
folders=dir(fullfile(base,'L2_60s_*'));result=struct;
for arm={'A','B'}
 name=arm{1};found=false;
 m='NearSpace_CMG_FlightControl';if strcmp(name,'A'),m=[m '_SurfaceOnly'];end
 for j=numel(folders):-1:1
  folder=fullfile(folders(j).folder,folders(j).name);file=fullfile(folder,[name '_surrogate_flight_60s.mat']);
  evidence=fullfile(folder,[m '_authority.mat']);if ~isfile(file) || ~isfile(evidence),continue;end
  logs=load(evidence);valid=find(cellfun(@(i)~isempty(i.authority),logs.diagnostics),1);
  if isempty(valid) || ~isfield(logs.diagnostics{valid}.authority,'axis_scale'),continue;end
  found=true;break;
 end
 assert(found,'No completed axis-specific 60 s run for %s',name);
 d=load(file);r=d.r;u=series(r.commands);u=u(:,1:3);y=series(r.tau_actual);t=r.commands.Time;
 assert(isequal(t,r.tau_actual.Time));e=u-y;
 row=struct('folder',folders(j).name,'command_rms',sqrt(mean(u.^2)), ...
  'error_rms',sqrt(mean(e.^2)),'error_peak',max(abs(e)), ...
  'relative_percent',100*sqrt(sum(e.^2)./sum(u.^2)), ...
  'combined_relative_percent',100*norm(e,'fro')/norm(u,'fro'));
 row.fallbacks=sum(cellfun(@(i)i.fallback,logs.diagnostics));
 row.worst_control_s=max(cellfun(@(i)i.elapsed_s,logs.diagnostics));
 row.mean_control_s=mean(cellfun(@(i)i.elapsed_s,logs.diagnostics));
 row.pitch_scale_range=[Inf -Inf];
 for k=1:numel(logs.diagnostics)
  a=logs.diagnostics{k}.authority;if isempty(a),continue;end
  assert(all(abs(a.axis_scale(2,:)-1)<1e-10),'Pitch still inherits coupled-axis shrink');
  row.pitch_scale_range=[min(row.pitch_scale_range(1),min(a.axis_scale(2,:))) max(row.pitch_scale_range(2),max(a.axis_scale(2,:)))];
  assert(max(a.vertex_residual,[],'all')<1e-9 && max(a.servo_vertex_residual)<1e-9);
 end
 % Diagnostic only: compare all tested delays on the SAME cropped window.
 dt=median(diff(t));maxlag=round(.2/dt);count=size(u,1)-maxlag;
 relative=zeros(maxlag+1,3);command=u(1:count,:);
 for lag=0:maxlag
  residual=command-y(1+lag:count+lag,:);
  relative(lag+1,:)=100*sqrt(sum(residual.^2)./sum(command.^2));
 end
 [row.best_lag_relative_percent,index]=min(relative,[],1);
 row.best_lag_ms=(index-1)*dt*1000;row.lag_zero_percent=relative(1,:);
 oldrow=find(startsWith(previous.folder,'L2_') & previous.duration_s==60 & strcmp(previous.arm,name),1,'last');
 row.previous_folder=previous.folder{oldrow};
 old=load(fullfile(base,row.previous_folder,[name '_surrogate_flight_60s.mat']));
 old_u=series(old.r.commands);old_u=old_u(:,1:3);old_y=series(old.r.tau_actual);
 row.previous_relative_percent=100*sqrt(sum((old_u-old_y).^2)./sum(old_u.^2));
 bank=load(fullfile(root,'simulink','data','online_plan_bank.mat'));
 x=series(r.states);q=eul2quat(x(:,[3 2 1]));qref=interp1(bank.time(:),squeeze(bank.q_ref(:,1,:))',r.states.Time);qref=qref./vecnorm(qref,2,2);
 attitude=rad2deg(2*acos(min(1,abs(sum(q.*qref,2)))));row.attitude_rms_deg=sqrt(mean(attitude.^2));row.attitude_peak_deg=max(attitude);
 result.(name)=row;save(fullfile(out,[name '_metrics.mat']),'row','t','e','relative','attitude');
end
fid=fopen(fullfile(out,'closedloop_report.json'),'w');fprintf(fid,'%s',jsonencode(result));fclose(fid);disp(jsonencode(result));
end
function data=series(ts)
data=squeeze(ts.Data);if size(data,1)~=numel(ts.Time),data=data';end
end
