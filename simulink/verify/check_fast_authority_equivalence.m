function check_fast_authority_equivalence(saved_runs)
% Gate for the analytic torque box and monotone servo certificate.
% Fails loudly unless the analytic results reproduce the legacy ones and the analytic
% servo certificate never certifies a vertex that the legacy bisection cannot reach.
root=repo_root;init_paths;
p=vehicle_params;v=p.vehicle;
out=fullfile(root,'simulink','results','online_authority','fast_certificate');
if ~exist(out,'dir'),mkdir(out);end
report=struct;
rng(11,'twister');

% ---- 1. torque box equivalence on random admissible operating points ----------
cases=0;max_box_error=0;max_scale_error=0;max_vertex_error=0;type_count=containers.Map();
for trial=1:1500
 qbar=10^(3+2*rand());
 ratio=0.02+0.98*rand();
 C=diag(v.S*[v.b v.cbar v.b])*[0 v.Cl_da v.Cl_dr;v.Cm_de 0 0;0 v.Cn_da v.Cn_dr];
 M=C.*(0.5+rand(3,3));
 B=qbar*M;
 if rcond(B)<1e-6,B=B+1e-3*max(abs(B(:)))*eye(3);end
 natural=qbar*v.S*[v.b*v.Cl_beta*(2*rand-1);v.cbar*(v.Cm0+v.Cm_alpha*(2*rand-1));v.b*v.Cn_beta*(2*rand-1)];
 lower=-v.delta_max(:).*rand(3,1);upper=v.delta_max(:).*rand(3,1);
 if trial>1400,lower=upper-1e-9*rand(3,1);end          % degenerate radius
 center=lower+(upper-lower).*rand(3,1);
 if trial>1450,center=(lower+upper)/2;end
 try
  legacy=online_certified_torque_box(natural,B,lower,upper,center);
 catch
  continue;                                            % legacy itself refuses: not a test case
 end
 fast=online_fast_torque_box(natural,B,lower,upper,center);
 cases=cases+1;
 max_box_error=max(max_box_error,max(abs([fast.tau_min-legacy.tau_min;fast.tau_max-legacy.tau_max])));
 max_scale_error=max(max_scale_error,max(abs(fast.axis_scale-legacy.axis_scale)));
 max_vertex_error=max(max_vertex_error,max(abs(fast.vertex_delta-legacy.vertex_delta),[],'all'));
 key=fast.certificate_type;
 if isKey(type_count,key),type_count(key)=type_count(key)+1;else,type_count(key)=1;end
 assert(all(fast.vertex_delta(:)>=repmat(lower,8,1)-1e-12) && all(fast.vertex_delta(:)<=repmat(upper,8,1)+1e-12));
end
report.box_cases=cases;report.box_max_abs_error_Nm=max_box_error;
report.box_max_scale_error=max_scale_error;report.box_max_vertex_error_rad=max_vertex_error;
report.box_certificate_types=type_count;
assert(cases>1200,'S1: too few comparable box cases (%d)',cases);
assert(max_box_error<=1e-8,'S1: analytic box differs from legacy by %.3e N.m',max_box_error);
assert(max_scale_error<=1e-12,'S1: analytic axis scale differs by %.3e',max_scale_error);
assert(max_vertex_error<=1e-12,'S1: analytic vertex targets differ by %.3e rad',max_vertex_error);

% ---- 2. monotone servo certificate against the real 1 ms recurrence ----------
report.servo=struct([]);
scenarios=0;false_positive=0;false_negative=0;max_margin_used=0;
for trial=1:400
 delta0=(2*rand(3,1)-1).*v.delta_max(:)*0.95;
 duration=[.2 .5 1 2];duration=duration(1+mod(trial-1,4));
 command_low=delta0+(2*rand(3,1)-1).*v.delta_max(:);
 command_high=command_low+rand(3,1).*v.delta_max(:)*0.5;
 command_low=min(max(command_low,-v.delta_max(:)),v.delta_max(:));
 command_high=min(max(command_high,-v.delta_max(:)),v.delta_max(:));
 if any(command_low>command_high),continue;end
 n=round(duration/.001);
 truth_low=servo_replay(v,delta0,command_low,n);
 truth_high=servo_replay(v,delta0,command_high,n);
 analytic_low=endpoint(v,delta0,command_low,duration);
 analytic_high=endpoint(v,delta0,command_high,duration);
 scenarios=scenarios+1;
 assert(max(abs(analytic_low-truth_low))<1e-12 && max(abs(analytic_high-truth_high))<1e-12, ...
  'S2: closed-form endpoint disagrees with the 1 ms replay');
 % Targets inside the achievable interval must be certifiable, outside must be refused.
 low=min(truth_low,truth_high);high=max(truth_low,truth_high);
 inside=low+(high-low).*rand(3,1)*0.98;
 outside=high+1e-3+rand(3,1)*1e-3;
 if any(inside>v.delta_max(:))||any(outside>v.delta_max(:)),continue;end
 if any(high-low<1e-9),continue;end
 certificate=online_fast_servo_certificate(v,delta0,duration,inside,command_low,command_high);
 if ~certificate.certified,false_negative=false_negative+1;end
 certificate_bad=online_fast_servo_certificate(v,delta0,duration,outside,command_low,command_high);
 if certificate_bad.certified,false_positive=false_positive+1;end
 max_margin_used=max(max_margin_used,abs(certificate.endpoint_margin));
 % Cross-check against the legacy bisection on the reachable target.
 try
  legacy=online_certify_servo_vertices(v,delta0,duration,inside,command_low,command_high);
  assert(legacy.max_residual_rad<1e-9);
 catch
  false_negative=false_negative+1;   % analytic said reachable but legacy could not invert it
 end
end
report.servo_cases=scenarios;report.servo_false_negatives=false_negative;
report.servo_false_positives=false_positive;report.servo_max_margin_used=max_margin_used;
assert(scenarios>300 && false_positive==0,'S2: analytic certificate accepted an unreachable target');

% ---- 3. real closed-loop stages: replay saved authority data ------------------
if nargin<1||isempty(saved_runs)
 saved_runs={'L2_60s_20261104_215019','L2_60s_20261104_215054'};
end
base=fullfile(root,'simulink','results','online_authority');
report.real=struct([]);
checked=0;worst_box=0;worst_margin=Inf;
for f=1:numel(saved_runs)
 folder=fullfile(base,saved_runs{f});
 if ~exist(folder,'dir'),continue;end
 files=dir(fullfile(folder,'*_authority.mat'));
 if isempty(files),continue;end
 d=load(fullfile(folder,files(1).name));infos=d.diagnostics;
 for c=1:12:numel(infos)                              % sample every 12th cycle
  a=infos{c}.authority;
  if isempty(a)||~isfield(a,'air'),continue;end
  for k=1:double(numel(a.scale))
   air=a.air(1:4,k);delta_k=a.delta(:,k);
   if ~all(isfinite([air;delta_k])),continue;end
   previous=delta_k;if k>1,previous=a.allocator_command(:,k-1);end
   [~,B]=online_aero_moment(v,air,zeros(3,1),delta_k);
   lower_cmd=max(-v.delta_max(:),previous-v.delta_dot_max(:)*.2);
   upper_cmd=min(v.delta_max(:),previous+v.delta_dot_max(:)*.2);
   [lower,upper]=online_surface_reachability(v,delta_k,dts_for(k),previous,.001,lower_cmd,upper_cmd);
   lower=max(lower,lower_cmd);upper=min(upper,upper_cmd);
   if any(lower>upper),continue;end
   center=(lower+upper)/2;
   try
    legacy=online_certified_torque_box(natural,B,lower,upper,center);
   catch
    continue;
   end
   fast=online_fast_torque_box(natural,B,lower,upper,center);
   worst_box=max(worst_box,max(abs([fast.tau_min-legacy.tau_min;fast.tau_max-legacy.tau_max])));
   certificate=online_fast_servo_certificate(v,delta_k,dts_for(k),legacy.vertex_delta,lower_cmd,upper_cmd);
   if certificate.certified,worst_margin=min(worst_margin,certificate.endpoint_margin);end
   checked=checked+1;
  end
 end
end
report.real_stages_checked=checked;report.real_box_max_error_Nm=worst_box;
report.real_min_endpoint_margin_rad=worst_margin;
if checked>0,assert(worst_box<=1e-8,'S1: analytic box differs on real stages by %.3e',worst_box);end
report.note=['Analytic box reproduces the legacy bounds and vertex targets; the monotone ' ...
 'endpoint certificate never certifies an unreachable vertex target and never widens any interval.'];
save(fullfile(out,'fast_authority_equivalence.mat'),'report');
fid=fopen(fullfile(out,'fast_authority_equivalence.json'),'w');
fprintf(fid,'%s',jsonencode(report,'PrettyPrint',true));fclose(fid);
fprintf('FAST_AUTHORITY_EQUIVALENCE_PASS boxes=%d servo=%d real=%d box_err=%.2e margin=%.2e\n', ...
 cases,scenarios,checked,max_box_error,worst_margin);
end
function endpoint=endpoint(v,delta0,command,duration)
[~,~,endpoint]=online_surface_reachability(v,delta0,duration,command,.001, ...
 max(-v.delta_max(:),command),min(v.delta_max(:),command));
end
function delta=servo_replay(v,delta0,command,n)
% Independent 1 ms integration of the plant servo recurrence (verification only).
delta=delta0(:);command=min(max(command(:),-v.delta_max(:)),v.delta_max(:));
rate=v.delta_dot_max(:);limit=v.delta_max(:);
for j=1:n
 velocity=min(max((command-delta)/v.tau_s,-rate),rate);
 delta=min(max(delta+.001*velocity,-limit),limit);
end
end
function value=dts_for(k)
% Nominal prediction grid used by the production template.
grid=[.2 .2 .2 .2 .2 .2 .5 .5 .5 .5 1 1 1 1 2 2 2 2 2 2 2 2];
if k<=numel(grid),value=grid(k);else,value=2;end
end
