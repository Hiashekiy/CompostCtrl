function m=evaluate_paper_runs(r,s,folder)
t=r.states.Time(:);x=squeeze(r.states.Data);if size(x,1)~=numel(t),x=x';end
bank=load(fullfile(folder,'reference_bank.mat'));[qr,ref]=paper_reference_series(bank,t);q=paper_euler_quaternions(x(:,1:3));err=rad2deg(2*acos(min(1,abs(sum(q.*qr,2)))));
m=struct('id',s.id,'experiment',s.experiment,'method',s.method,'cmg',s.cmg,'amplitude',s.amplitude,'transition',s.transition,'frequency',s.frequency,'horizon',sum(s.dts),'masked',s.masked,'t_end',t(end),'rmse_deg',sqrt(mean(err.^2)),'peak_deg',max(err),'terminal_deg',err(end),'status','TRACKING_FAIL');
msg=r.ErrorMessage;m.error_message=msg;
if ~isempty(msg)
 if contains(lower(msg),{'alpha_domain','alpharange','atmosphererange','height outside','force polar'}),m.status='MODEL_DOMAIN_FAIL';else,m.status='CONTROLLER_FAIL';end
elseif all(isfinite(x),'all') && t(end)>=s.duration-.001
 ends=s.onset+s.transition;if strcmp(s.profile,'sequence'),ends=[ends 30 45];end;good=true;for finish=ends,hold=t>=finish+2 & t<=finish+10;good=good && any(hold) && all(err(hold)<=1);end;if strcmp(s.profile,'sine'),good=sqrt(mean(err(t>=s.duration-2/s.frequency).^2))<=1;end;if good,m.status='SUCCESS';end
end
m.reference_metric='Normalized linear quaternion interpolation, identical to run_ab3_cmg_task_compare; current-node bank truth';m.profile=s.profile;m.criteria_version='v2:1deg_after_2s_for_8s';if strcmp(s.profile,'sine'),m.criteria_version='v2:last_two_cycles_rmse_le_1deg';end;m.settling_time_s=NaN;finish=s.onset+s.transition;if ~strcmp(s.profile,'sine'),for k=find(t>=finish)',if all(err(k:end)<=1),m.settling_time_s=t(k)-finish;break;end;end;end
m.frequency_tracking_pass=false;m.steady_rmse_deg=NaN;m.gain_db=NaN;m.phase_deg=NaN;if strcmp(s.profile,'sine') && isempty(msg),mask=t>=s.duration-2/s.frequency;m.steady_rmse_deg=sqrt(mean(err(mask).^2));T=t(mask);D=[sin(2*pi*s.frequency*(T-s.onset)),cos(2*pi*s.frequency*(T-s.onset)),ones(size(T)),T-mean(T)];a=D\ref(mask,2);b=D\rad2deg(x(mask,2));m.gain_db=20*log10(norm(b(1:2))/norm(a(1:2)));m.phase_deg=rad2deg(angle(complex(b(1),b(2))/complex(a(1),a(2))));end
if strcmp(s.profile,'sine') && isempty(msg),m.frequency_tracking_pass=m.gain_db>=-3 && abs(m.phase_deg)<=45 && m.steady_rmse_deg<=.5*s.amplitude;m.criteria_version='v3:sine_gain_ge_minus3dB_phase_le45_rmse_halfamp';if ~m.frequency_tracking_pass,m.status='TRACKING_FAIL';end;end
p=vehicle_params;delta=squeeze(r.delta_actual.Data);rate=squeeze(r.delta_dot.Data);m.surface_saturation_pct=100*mean(abs(delta)>=p.vehicle.delta_max(:)'-1e-5,1);m.surface_rate_saturation_pct=100*mean(abs(rate)>=p.vehicle.delta_dot_max(:)'-1e-5,1);
m.physical_surface_bounds_ok=all(abs(delta)<=p.vehicle.delta_max(:)'+1e-6,'all') && all(abs(rate)<=p.vehicle.delta_dot_max(:)'+1e-6,'all');if ~m.physical_surface_bounds_ok && isempty(msg),m.status='CONTROLLER_FAIL';end
m.cmg_h_initial=NaN;m.cmg_h_final=NaN;if size(x,2)==10,H=zeros(numel(t),3);for k=1:numel(t),g=x(k,7:10);H(k,:)=sum(p.cmg.h0*(p.s0.*cos(g)+p.gxs0.*sin(g)),2)';end;m.cmg_h_initial=norm(H(1,:));m.cmg_h_final=norm(H(end,:));end
m.air_drift_fraction=[];m.air_min=[];m.air_max=[];try,air=squeeze(r.flight_conditions.Data);m.air_min=min(air,[],1);m.air_max=max(air,[],1);m.air_drift_fraction=(m.air_max-m.air_min)./max(abs(mean(air,1)),1e-9);catch,end
m.compute_mean_s=NaN;m.compute_p95_s=NaN;m.compute_max_s=NaN;m.over_200ms_pct=NaN;m.fallbacks=0;
files=dir(fullfile(folder,'*_authority.mat'));if ~isempty(files),d=load(fullfile(folder,files(1).name));elapsed=[];for j=1:numel(d.diagnostics),z=d.diagnostics{j};if isfield(z,'elapsed_s'),elapsed(end+1)=z.elapsed_s;end;if isfield(z,'fallback'),m.fallbacks=m.fallbacks+z.fallback;end;end;if ~isempty(elapsed),m.compute_mean_s=mean(elapsed);m.compute_max_s=max(elapsed);m.compute_p95_s=prctile(elapsed,95);m.over_200ms_pct=100*mean(elapsed>.2);end;end
if strcmp(m.status,'SUCCESS') && m.fallbacks>max(3,.1*ceil(t(end)/.2)),m.status='CONTROLLER_FAIL';end
names=[{'time_s'},arrayfun(@(k)sprintf('state_%d',k),1:size(x,2),'UniformOutput',false),{'reference_roll_deg','reference_pitch_deg','reference_yaw_deg','attitude_error_deg'}];writetable(array2table([t,x,ref,err],'VariableNames',names),fullfile(folder,'timeseries.csv'));paper_json(fullfile(folder,'metrics.json'),m);
f=figure('Visible','off');idx=unique([1:ceil(numel(t)/2000):numel(t),numel(t)]);plot(t(idx),ref(idx,2),'k--',t(idx),rad2deg(x(idx,2)));xlabel('Time (s)');ylabel('Pitch (deg)');legend('Reference','Actual');grid on;exportgraphics(f,fullfile(folder,'tracking.png'));exportgraphics(f,fullfile(folder,'tracking.pdf'),'ContentType','vector');close(f);
end










