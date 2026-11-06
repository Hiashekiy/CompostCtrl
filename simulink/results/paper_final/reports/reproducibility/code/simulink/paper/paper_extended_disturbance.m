function [tau,validation]=paper_extended_disturbance(old,t)
% MATLAB port of the archived analytic disturbance; native rho table interpolation.
persistent p env c d v offset source_validation
if isempty(source_validation)
p=vehicle_params;env=load('simulink/data/native_environment.mat');resolved=jsondecode(fileread('simulink/data/native_baseline/A4_descent_replay_S00_with_cmg/config_resolved.json'));c=resolved.trajectory;d=resolved.trajectory3;v=p.vehicle;offset=90;source=squeeze(old.tau_d(:,1,:))';model=zeros(size(source));for k=1:numel(old.time),model(k,:)=value(old.time(k));end
relative=max(abs(model-source),[],'all')/max(1,max(abs(source),[],'all'));assert(relative<1e-5,'paper:disturbance','Analytic/native-table source mismatch exceeds 1e-5: %g',relative);source_validation=struct('source','Archived trajectory3 analytic source, absolute offset 90s, native 250m rho table','maximum_source_relative_error',relative,'tolerance',1e-5,'native_interpolation_approximation',true);
end
tau=value(t)';validation=source_validation;
function y=value(t)
a=t+offset;h=c.h0+.5*c.amp*(1-cos(2*pi*a/c.period));speed=max(c.V0-c.dVdt*a,500);rho=interp1(env.height_m,env.rho,h,'linear');rho0=interp1(env.height_m,env.rho,c.h0,'linear');qratio=rho*speed^2/(rho0*c.V0^2);y=v.Md0(:)+v.Md_amp(:).*sin(2*pi*v.Md_freq(:)*a);if d.dist_scale_qinf,y=v.Md0(:)+(y-v.Md0(:))*qratio;end
if a>=d.dist_pulse_start && a<=d.dist_pulse_start+d.dist_pulse_duration,y=y+d.dist_pulse_amp(:)*.5*(1-cos(2*pi*(a-d.dist_pulse_start)/d.dist_pulse_duration));end;y=y';
end
end



