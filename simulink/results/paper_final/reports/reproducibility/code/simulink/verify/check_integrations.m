function check_integrations
root=repo_root; addpath(fullfile(root,'simulink','models'));
out=fullfile(root,'simulink','results','native_integrations');
ref=load(fullfile(root,'simulink','data','native_actuators','reference.mat'));
a=load(fullfile(out,'cmg_integrated_run.mat')); r=a.r;
errors.cmg_nu=max(abs(samples(r.yout{1}.Values)-ref.nu),[],'all');
errors.cmg_gamma=max(abs(samples(r.yout{3}.Values)-ref.gamma_zero),[],'all');
errors.cmg_H=max(abs(samples(r.yout{4}.Values)-ref.H_integrated),[],'all');
errors.cmg_torque=max(abs(samples(r.yout{6}.Values)-ref.torque_integrated),[],'all');
a=load(fullfile(out,'aero_surface_integrated_run.mat'));r=a.r;
errors.surface_delta=max(abs(samples(r.yout{1}.Values)-ref.delta),[],'all');
errors.aero_total=max(abs(samples(r.yout{5}.Values)-ref.aero_integrated),[],'all');
assert(all(structfun(@(x)x<1e-7,errors)));
% Independent continuous first-order solutions in the unsaturated region.
for name={'CMG_Drive','Surface_Actuator'}
 m=name{1};load_system(m);if strcmp(m,'CMG_Drive'),n=4;tau=.03;else,n=3;tau=.05;end
 si=Simulink.SimulationInput(m);si=si.setModelParameter('StopTime','.2');
 si=si.setExternalInput([0 .01*ones(1,n);.2 .01*ones(1,n)]);r=sim(si);
 expected=.01*(1-exp(-r.state.Time/tau));
 err=max(abs(samples(r.state)-expected),[],'all');assert(err<7e-5);
 errors.([m '_analytic'])=err;close_system(m,0);
end
fid=fopen(fullfile(out,'verification.json'),'w');fprintf(fid,'%s',jsonencode(errors,'PrettyPrint',true));fclose(fid);
disp(errors);fprintf('NATIVE_INTEGRATIONS_NUMERICAL_PASS\n');
end
function y=samples(ts)
y=squeeze(ts.Data);if size(y,1)~=numel(ts.Time),y=y';end
end
