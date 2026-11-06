function benchmark_fast_authority(iterations)
% Micro-benchmark of the two certification stages plus a full 22-stage rollout.
% Same operating points for every mode; the legacy path is the frozen reference.
if nargin<1,iterations=200;end
root=repo_root;init_paths;
out=fullfile(root,'simulink','results','online_authority','fast_certificate');
if ~exist(out,'dir'),mkdir(out);end
p=vehicle_params;v=p.vehicle;
bank=load(fullfile(root,'simulink','data','online_plan_bank.mat'));
env=load(fullfile(root,'simulink','data','native_environment.mat'));
rng(3,'twister');
% --- operating points: a spread of real near-space stages ---------------------
samples=iterations;
air=makedata(@() [10^(3+2*rand());600+900*rand();(2*rand-1)*.12;(2*rand-1)*.03],4,samples);
delta=(2*rand(3,samples)-1).*v.delta_max(:)*0.9;
box_time=zeros(2,1);cert_time=zeros(2,1);box_error=0;
for k=1:samples
 previous=delta(:,k);
 lower_cmd=max(-v.delta_max(:),previous-v.delta_dot_max(:)*.2);
 upper_cmd=min(v.delta_max(:),previous+v.delta_dot_max(:)*.2);
 [lower,upper]=online_surface_reachability(v,delta(:,k),.2,previous,.001,lower_cmd,upper_cmd);
 lower=max(lower,lower_cmd);upper=min(upper,upper_cmd);
 if any(lower>upper),continue;end
 [natural,B]=online_aero_moment(v,air(:,k),zeros(3,1),delta(:,k));
 center=(lower+upper)/2;
 t=tic;legacy=online_certified_torque_box(natural,B,lower,upper,center);box_time(1)=box_time(1)+toc(t);
 t=tic;fast=online_fast_torque_box(natural,B,lower,upper,center);box_time(2)=box_time(2)+toc(t);
 box_error=max(box_error,max(abs([fast.tau_min-legacy.tau_min;fast.tau_max-legacy.tau_max])));
 t=tic;online_certify_servo_vertices(v,delta(:,k),.2,legacy.vertex_delta,lower_cmd,upper_cmd);cert_time(1)=cert_time(1)+toc(t);
 t=tic;online_fast_servo_certificate(v,delta(:,k),.2,legacy.vertex_delta,lower_cmd,upper_cmd);cert_time(2)=cert_time(2)+toc(t);
end
% --- full rollout: one prediction over all 22 stages -------------------------
template=load(fullfile(root,'simulink','tests','05_python_matlab_qp_equivalence','production_online_qp.mat'));
s=template;s.authority_mode='online_predicted';s.authority_max_step=.1;
x=bank.x0(:);if numel(x)==7,x=[x;s.X_nom(8:11,1)];end;x=x(:);
s.flight_state=[[0;0;-env.h0];online_quaternion_rotation(x(1:4))'*env.V_ned0(:);zeros(6,1)];
s.tau_min=nan(3,double(s.N));s.tau_max=nan(3,double(s.N));s.nu_previous=zeros(4,1);
U=zeros(double(s.nu),double(s.N));
rollout_ms=zeros(1,4);modes={'legacy','analytic_box','analytic_full','shadow'};
for m=1:numel(modes)
 s.authority_certificate_mode=modes{m};
 online_authority_rollout(s,x,U,s.flight_state,true);       % warm-up (persistent caches)
 reps=max(3,round(iterations/20));t=tic;
 for j=1:reps
  [X,~,a,ok]=online_authority_rollout(s,x,U,s.flight_state,true);
  assert(ok);
 end
 rollout_ms(m)=1000*toc(t)/reps;
 if m==1,reference=X;else,assert(max(abs(X(:)-reference(:)))<1e-12,'Rollout output changed in %s',modes{m});end
 if isfield(a,'analytic_fallback_count'),fprintf('  %s analytic fallbacks=%d\n',modes{m},a.analytic_fallback_count);end
end
report=struct('iterations',iterations,'box_ms_per_call',struct('legacy',1000*box_time(1)/samples, ...
 'fast',1000*box_time(2)/samples),'certificate_ms_per_call',struct('legacy',1000*cert_time(1)/samples, ...
 'fast',1000*cert_time(2)/samples),'box_rel_error',box_error, ...
 'rollout_ms_22stages',struct(modes{1},rollout_ms(1),modes{2},rollout_ms(2),modes{3},rollout_ms(3),modes{4},rollout_ms(4)), ...
 'rollout_speedup_analytic_full',rollout_ms(1)/rollout_ms(3));
save(fullfile(out,'fast_authority_benchmark.mat'),'report');
fid=fopen(fullfile(out,'fast_authority_benchmark.json'),'w');fprintf(fid,'%s',jsonencode(report,'PrettyPrint',true));fclose(fid);
fprintf('BENCH box %.3f -> %.3f ms | certificate %.3f -> %.3f ms | rollout %.1f -> %.1f ms (x%.2f) | box_err=%.1e\n', ...
 1000*box_time(1)/samples,1000*box_time(2)/samples,1000*cert_time(1)/samples,1000*cert_time(2)/samples, ...
 rollout_ms(1),rollout_ms(3),rollout_ms(1)/rollout_ms(3),box_error);
end
function data=makedata(fn,rows,cols)
data=zeros(rows,cols);
for k=1:cols,data(:,k)=fn();end
end
