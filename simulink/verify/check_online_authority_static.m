function check_online_authority_static
% Stage A: independent cross-check against the saved native physical models.
root=repo_root;p=vehicle_params;v=p.vehicle;
out=fullfile(root,'simulink','results','online_authority','stage_A');
if ~exist(out,'dir'),mkdir(out);end
old=rng;cleanup=onCleanup(@()rng(old));rng(641);
samples=zeros(120,10);errors=zeros(120,3);certificates=cell(120,1);
load_system('Aerodynamic_Model');
for k=1:120
 air=[10^(5*rand-3);1+3500*rand;(2*rand-1)*.6;(2*rand-1)*.2];
 omega=(2*rand(3,1)-1)*.2;delta=(2*rand(3,1)-1).*v.delta_max(:);
 if k==1,air(1)=0;end
 if k==2,delta=v.delta_max(:);end
 samples(k,:)=[air;omega;delta]';
 [natural,B,total]=online_aero_moment(v,air,omega,delta);
 si=Simulink.SimulationInput('Aerodynamic_Model');
 si=si.setModelParameter('StopTime','0');si=si.setExternalInput([0 samples(k,:)]);
 r=sim(si);
 expected={natural,B*delta,total};
 for j=1:3
  value=r.yout{j}.Values.Data;
  errors(k,j)=max(abs(value(:)-expected{j}));
 end
 [lo,hi,nominal]=online_surface_reachability(v,delta,.2,zeros(3,1));
 box=online_certified_torque_box(natural,B,lo,hi,nominal);
 assert(all(box.tau_min<=box.tau_max));
 assert(all(box.vertex_residual<=1e-9));certificates{k}=box;
end
assert(max(errors,[],'all')<1e-8,'Native aero mismatch');
% Cross-check reachability endpoint recurrence using the actual saved servo.
load_system('Surface_Actuator');servo_error=zeros(2,1);
for side=1:2
 actual=[.99;-.99;.5].*v.delta_max(:);target=(2*side-3)*v.delta_max(:);
 [lo,hi]=online_surface_reachability(v,actual,.2,target);
 si=Simulink.SimulationInput('Surface_Actuator');
 si=si.setBlockParameter('Surface_Actuator/Actual_state','InitialCondition',mat2str(actual,17));
 si=si.setModelParameter('StopTime','.2');
 si=si.setExternalInput([0 target';.2 target']);r=sim(si);
 data=squeeze(r.state.Data);if size(data,1)==3,data=data';end
 if side==1,expected=lo;else,expected=hi;end
 servo_error(side)=max(abs(data(end,:)'-expected));
end
assert(max(servo_error)<1e-10,'Native servo mismatch');
% Verify the newly generated visible physical module, including its 3x3 map.
load_system('Online_Surface_Authority');module_errors=zeros(120,2);
env=load(fullfile(root,'simulink','data','native_environment.mat'));
for k=1:120
 h=1000+99000*rand;speed=50+3400*rand;alpha=(2*rand-1)*.6;beta=(2*rand-1)*.2;
 vb=speed*[cos(beta)*cos(alpha);sin(beta);cos(beta)*sin(alpha)];
 omega=samples(k,5:7)';delta=samples(k,8:10)';
 flight=[[0;0;-h];vb;delta;delta];
 rho=interp1(env.height_m,env.rho,h);air=[.5*rho*speed^2;speed;alpha;beta];
 [natural,B]=online_aero_moment(v,air,omega,delta);
 ds=Simulink.SimulationData.Dataset;
 ds=ds.addElement(timeseries(flight',0));ds=ds.addElement(timeseries(omega',0));
 ds=ds.addElement(timeseries(eye(3),0));ds=ds.addElement(timeseries(zeros(1,8),0));
 si=Simulink.SimulationInput('Online_Surface_Authority');si=si.setExternalInput(ds);
 si=si.setModelParameter('StopTime','0','ReturnWorkspaceOutputs','on');r=sim(si);
 natural_native=r.yout{2}.Values.Data;diagnostics=r.yout{4}.Values.Data;diagnostics=diagnostics(:);
 module_errors(k,:)=[max(abs(natural_native(:)-natural)) max(abs(diagnostics(8:16)-B(:)))];
end
assert(max(module_errors,[],'all')<1e-8,'Visible authority module mismatch');
% Rank-deficient coupled mapping must not admit unreachable roll/yaw corners.
box=online_certified_torque_box(zeros(3,1),[1 0 0;0 1 0;1 0 0],-ones(3,1),ones(3,1),zeros(3,1));
assert(max(box.tau_max)<1e-8);
save(fullfile(out,'static_validation.mat'),'samples','errors','servo_error','certificates','module_errors');
writematrix(errors,fullfile(out,'moment_errors.csv'));
fprintf('ONLINE_AUTHORITY_STAGE_A_PASS samples=120 moment_error=%g servo_error=%g\n',max(errors,[],'all'),max(servo_error));
end
