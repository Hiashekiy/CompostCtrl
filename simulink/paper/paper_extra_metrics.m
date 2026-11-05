function e=paper_extra_metrics(r,s,folder)
p=vehicle_params;t=r.states.Time(:);x=squeeze(r.states.Data);e=struct('id',s.id,'experiment',s.experiment,'cmg',s.cmg,'settled',false);cmd=squeeze(r.commands.Data);actual=squeeze(r.tau_actual.Data);e.aero_torque_rmse=sqrt(mean((cmd(:,1:3)-actual).^2,1));
if size(x,2)==10
 g=x(:,7:10);nu=zeros(size(g));for j=1:4,nu(:,j)=gradient(g(:,j),t);end
 H=zeros(numel(t),3);tau=H;authority=H;for k=1:numel(t),A=p.cmg.h0*(-p.s0.*sin(g(k,:))+p.gxs0.*cos(g(k,:)));H(k,:)=sum(p.cmg.h0*(p.s0.*cos(g(k,:))+p.gxs0.*sin(g(k,:))),2)';tau(k,:)=(-A*nu(k,:)')';authority(k,:)=sum(abs(A),2)'*p.cmg.gdot_max;end
 e.cmg_torque_source='Central/one-sided difference of true unwrapped gamma; endpoint derivatives uncertain';e.cmg_torque_rms=sqrt(mean(tau.^2,1));e.cmg_rate_saturation_pct=100*mean(abs(nu)>=p.cmg.gdot_max-1e-4,1);e.H_initial=norm(H(1,:));e.H_final=norm(H(end,:));e.rate_max=max(abs(nu),[],1);
 baseline=t>=0 & t<=min(2,s.onset/2);g0=mean(g(baseline,:),1);motion=max(abs(g-g0),[],2)>deg2rad(.5);e.preconfiguration_onset=NaN;e.quiet_baseline_observed=max(abs(g(baseline,:)-g(1,:)),[],'all')<=deg2rad(.5);e.onset_note='Time-zero preview may cause left-censored preconfiguration; only identify onset if quiet baseline observed';window=max(1,round(.6/median(diff(t))));for k=find(t>=2 & t<s.onset & e.quiet_baseline_observed)',if k+window<=numel(t) && all(motion(k:k+window)),e.preconfiguration_onset=t(k);break;end;end
 [~,k]=min(abs(t-s.onset));e.H_at_onset=H(k,:);e.axis_rate_box_bound_at_onset=authority(k,:);e.bound_note='instantaneous gdot box; ignores acceleration/transients, not a singularity/momentum-margin measure';
 writetable(array2table([t,g,nu,H,tau,authority],'VariableNames',{'time_s','gamma1_rad','gamma2_rad','gamma3_rad','gamma4_rad','nu1_radps','nu2_radps','nu3_radps','nu4_radps','Hx_Nms','Hy_Nms','Hz_Nms','tau_cmg_x_Nm','tau_cmg_y_Nm','tau_cmg_z_Nm','axis_x_ratebox_Nm','axis_y_ratebox_Nm','axis_z_ratebox_Nm'}),fullfile(folder,'cmg_timeseries.csv'));
 idx=unique([1:ceil(numel(t)/2000):numel(t),numel(t)]);f=figure('Visible','off');tiledlayout(3,1);nexttile;plot(t(idx),actual(idx,2),t(idx),tau(idx,2));ylabel('Pitch torque (N m)');legend('Actual aero','CMG (gamma derivative)');nexttile;plot(t(idx),rad2deg(g(idx,:)));ylabel('Gimbal angle (deg)');nexttile;plot(t(idx),vecnorm(H(idx,:),2,2));ylabel('|H| (N m s)');xlabel('Time (s)');exportgraphics(f,fullfile(folder,'cmg_diagnostics.png'));exportgraphics(f,fullfile(folder,'cmg_diagnostics.pdf'),'ContentType','vector');close(f);
end
if strcmp(s.profile,'sine')
 a=squeeze(r.flight_conditions.Data);ta=r.flight_conditions.Time;mask=ta>=s.duration-2/s.frequency;steady=a(mask,:);e.steady_air_min=min(steady,[],1);e.steady_air_max=max(steady,[],1);e.steady_air_relative_range=range(steady,1)./max(abs(mean(steady,1)),1e-8);e.frequency_response_scope='Time-varying flight segment; no strict LTI bandwidth claimed';
end
file=fullfile(folder,'pid_diagnostics.mat');if isfile(file),d=load(file);e.unload_active_ticks=sum(cellfun(@(z)z.eta>0,d.history));e.low_qbar_ticks=sum(cellfun(@(z)z.low_qbar,d.history));e.unload_compensation_residual_max=max(cellfun(@(z)norm(z.unload_a+z.unload_c),d.history));e.allocation_residual_rms=sqrt(mean(cellfun(@(z)norm(z.residual)^2,d.history)));end
paper_json(fullfile(folder,'extra_metrics.json'),e);
end


