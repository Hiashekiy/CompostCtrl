function report=check_torque_margin_feedback(mode)
% Gate for the torque-margin / 20 ms torque-feedback ablation.
%
%   U1  online_shrink_certified_torque_box: subset, centre, half-width, degenerate
%       boxes, invalid inputs refused, alpha=0 bit-for-bit identity
%   U2  20 ms servo endpoint against an INDEPENDENT 1 ms replay of the plant
%       recurrence and against online_surface_reachability (same physics, other code),
%       plus the analytic slope against finite differences
%   U3  inner torque-feedback controller: command set, angle limit, slew limit,
%       finite output, cost never worse than its own feed-forward start
%   U4  (mode='full') 0 % shrink + legacy inner reproduces the stored analytic_full
%       baseline closed loop bit for bit, and the 20 ms arm triggers exactly
%       2/0.02+1 inner ticks while the MPC keeps 2/0.2+1 cycles
%
% Every threshold is an assert: printing a number is not acceptance.
if nargin<1,mode='unit';end
assert(ismember(mode,{'unit','full'}),'mode must be unit or full');
root=repo_root;init_paths;
p=vehicle_params;v=p.vehicle;
out=fullfile(root,'simulink','results','online_authority','torque_margin_feedback');
if ~exist(out,'dir'),mkdir(out);end
report=struct('mode',mode,'geometry',struct('delta_max_rad',v.delta_max(:).', ...
 'delta_dot_max_radps',v.delta_dot_max(:).','tau_s',v.tau_s,'inner_dt',.02,'servo_dt',.001));

% ---------------- U1: shrink operator -----------------------------------------
rng(23,'twister');
cases=0;worst_halfwidth=0;worst_centre=0;violation=0;
for trial=1:2500
 raw_min=(2*rand(3,1)-1).*10.^(2*rand(3,1));
 raw_max=raw_min+10.^(-1+3*rand(3,1));
 if mod(trial,7)==0,raw_max=raw_min;end                % every axis zero width
 if mod(trial,11)==0,raw_min(2)=raw_max(2);end         % one zero-width axis
 pool=[0 .05 .1 .25 .5 .9];alpha=pool(1+mod(trial,numel(pool)));
 [lo,hi,c,h]=online_shrink_certified_torque_box(raw_min,raw_max,alpha);
 cases=cases+1;
 violation=violation+sum(raw_min>lo+1e-12|lo>hi+1e-12|hi>raw_max+1e-12);
 assert(all(isfinite([lo;hi;c;h])));
 if alpha==0
  assert(isequal(lo,raw_min) && isequal(hi,raw_max),'U1: alpha=0 is not bitwise identity');
 else
  worst_halfwidth=max(worst_halfwidth,max(abs(h-(1-alpha)*(raw_max-raw_min)/2)));
  worst_centre=max(worst_centre,max(abs(c-(raw_max+raw_min)/2)));
 end
end
report.shrink=struct('cases',cases,'violations',violation, ...
 'max_halfwidth_error_Nm',worst_halfwidth,'max_centre_error_Nm',worst_centre);
assert(violation==0,'U1: %d shrink results left the certified box',violation);
assert(worst_halfwidth<=1e-12 && worst_centre<=1e-12,'U1: shrink arithmetic drifted');
for bad=[-0.1 1 1.5 NaN]
 refused=false;try,online_shrink_certified_torque_box([-1;-1;-1],[1;1;1],bad);catch,refused=true;end
 assert(refused,'U1: shrink ratio %g was not refused',bad);
end
refused=false;try,online_shrink_certified_torque_box([1;-1;-1],[-1;1;1],.1);catch,refused=true;end
assert(refused,'U1: a reversed box was not refused');

% ---------------- U2: servo endpoint ------------------------------------------
rng(29,'twister');
scenarios=0;worst_endpoint=0;worst_slope=0;slope_cases=0;
for trial=1:600
 delta0=(2*rand(3,1)-1).*v.delta_max(:)*.99;
 command=(2*rand(3,1)-1).*v.delta_max(:);
 if trial<=40,command=delta0;end                                          % settled
 if trial>40&&trial<=80,command=min(max(command,delta0-v.delta_max(:)),delta0+v.delta_max(:));end
 [analytic,~]=online_servo_endpoint(v,delta0,command,.02,.001);
 replay=servo_replay(v,delta0,command,20);
 [~,~,reachable]=online_surface_reachability(v,delta0,.02,command,.001, ...
  max(-v.delta_max(:),command),min(v.delta_max(:),command));
 scenarios=scenarios+1;
 worst_endpoint=max([worst_endpoint,max(abs(analytic-replay)),max(abs(analytic-reachable))]);
 delta=command-delta0;
 if any(abs(delta)<1e-3),continue;end
 step=1e-7*sign(delta);
 [plus,~]=online_servo_endpoint(v,delta0,command+step,.02,.001);
 [minus,~]=online_servo_endpoint(v,delta0,command-step,.02,.001);
 [~,gain_plus]=online_servo_endpoint(v,delta0,command+step,.02,.001);
 [~,gain_minus]=online_servo_endpoint(v,delta0,command-step,.02,.001);
 if any(abs(gain_plus-gain_minus)>1e-12),continue;end                 % across a knee
 [~,gain]=online_servo_endpoint(v,delta0,command,.02,.001);
 slope=(plus-minus)./(2*step);
 slope_cases=slope_cases+1;
 worst_slope=max(worst_slope,max(abs(slope-gain)));
end
report.servo_endpoint=struct('scenarios',scenarios,'max_abs_error_rad',worst_endpoint, ...
 'slope_cases',slope_cases,'max_slope_error',worst_slope);
assert(scenarios>=600 && worst_endpoint<1e-12,'U2: the 20 ms endpoint disagrees with the 1 ms truth');
assert(slope_cases>200 && worst_slope<1e-6,'U2: analytic servo slope disagrees with finite differences (%.3e)',worst_slope);

% ---------------- U3: inner torque-feedback controller ------------------------
rng(31,'twister');
scenarios=0;improved=0;worst_bound=0;fallbacks=0;max_iterations=0;saturated=0;
mean_static=0;mean_final=0;
for trial=1:400
 qbar=10^(2.5+2*rand());V=200+400*rand();alpha=(-3+6*rand)*pi/180;beta=(-2+2*rand)*pi/180;
 air=[qbar;V;alpha;beta];omega=(2*rand(3,1)-1)*.2;
 delta_actual=(2*rand(3,1)-1).*v.delta_max(:);
 u_previous=(2*rand(3,1)-1).*v.delta_max(:);
 [~,B]=online_aero_moment(v,air,omega,delta_actual);
 tau_cmd=(2*rand(3,1)-1).*sqrt(abs(B)*v.delta_max(:))*10^(-1.5+2.5*rand());
 options=struct('dt',.02,'lambda',1e-3,'max_iterations',1+mod(trial,4));
 [u,info]=online_torque_feedback_step(v,tau_cmd,delta_actual,air,omega,u_previous,options);
 scenarios=scenarios+1;fallbacks=fallbacks+double(info.fallback);
 max_iterations=max(max_iterations,info.iterations);
 assert(all(isfinite(u)),'U3: non-finite inner command');
 worst_bound=max([worst_bound,max(u-info.upper),max(info.lower-u)]);
 assert(all(u>=-v.delta_max(:)-1e-12 & u<=v.delta_max(:)+1e-12),'U3: command outside the angle limit');
 assert(all(abs(u-u_previous)<=v.delta_dot_max(:)*.02+1e-12),'U3: command slew limit violated');
 assert(info.cost_final<=info.cost_initial+1e-9,'U3: inner cost increased');
 mean_static=mean_static+cost_of(v,tau_cmd,delta_actual,air,omega,u_previous, ...
  info.static_command,options);
 mean_final=mean_final+info.cost_final;
 if info.cost_final<info.cost_initial-1e-12,improved=improved+1;end
 saturated=saturated+double(info.slew_saturated);
end
report.inner=struct('scenarios',scenarios,'fallbacks',fallbacks,'improved_ticks',improved, ...
 'max_iterations_used',max_iterations,'max_bound_violation',worst_bound, ...
 'mean_cost_static',mean_static/scenarios,'mean_cost_final',mean_final/scenarios, ...
 'slew_saturated_ticks',saturated);
assert(scenarios==400 && worst_bound<=1e-12,'U3: inner loop left its command set');
assert(mean_final<=mean_static+1e-9,'U3: the inner predictor is worse than its own feed-forward');
% A settled-but-lagging operating point: the inner loop must beat the static
% allocation.  Note that this is only possible where the static command leaves room
% inside the 20 ms command set; at the slew corner both laws are equally aggressive,
% which is why the ablation has to be measured rather than promised.
settled=struct('delta_actual',zeros(3,1),'u_previous',zeros(3,1));
air=[50;300;2*pi/180;0];omega=zeros(3,1);
[~,B0]=online_aero_moment(v,air,omega,settled.delta_actual);
wanted=.02*B0*v.delta_max(:);
options=struct('dt',.02);
[u,info]=online_torque_feedback_step(v,wanted,settled.delta_actual,air,omega,settled.u_previous,options);
static_torque=info.natural+B0*online_servo_endpoint(v,settled.delta_actual,info.static_command,.02,.001);
report.inner.lag_example=struct('command',u(:).','static_command',info.static_command(:).', ...
 'iterations',info.iterations,'cost_initial',info.cost_initial,'cost_final',info.cost_final, ...
 'predicted_torque_error_Nm',norm(wanted-info.achievable_torque), ...
 'static_torque_error_Nm',norm(wanted-static_torque));
assert(info.cost_final<info.cost_initial,'U3: no improvement on a lagging reachable target');
assert(norm(wanted-info.achievable_torque)<norm(wanted-static_torque), ...
 'U3: the predicted torque did not improve on a lagging reachable target');

% ---------------- U4: baseline reproduction and sample times ------------------
report.baseline=struct('checked',false);
if strcmp(mode,'full')
 report.baseline=baseline_reproduction(root);
 assert(report.baseline.max_abs_command_difference<1e-12,'U4: 0 %% shrink does not reproduce the baseline commands');
 assert(report.baseline.max_abs_state_difference<1e-12,'U4: 0 %% shrink changes the state trajectory');
 assert(report.baseline.inner_ticks==report.baseline.expected_inner_ticks, ...
  'U4: inner ticks %d, expected %d',report.baseline.inner_ticks,report.baseline.expected_inner_ticks);
 assert(report.baseline.mpc_cycles==report.baseline.expected_mpc_cycles, ...
  'U4: MPC cycles %d, expected %d',report.baseline.mpc_cycles,report.baseline.expected_mpc_cycles);
 assert(report.baseline.inner_fallbacks==0,'U4: the inner loop needed %d safe fallbacks', ...
  report.baseline.inner_fallbacks);
end
report.note=['U1/U2/U3 are independent of the closed loop; U4 rebuilds the 0 %/legacy arm and ' ...
 'compares it with the stored analytic_full baseline, then counts the 20 ms arm triggers.'];
fid=fopen(fullfile(out,'unit_checks.json'),'w');
fprintf(fid,'%s',jsonencode(report,'PrettyPrint',true));fclose(fid);
fprintf('TORQUE_MARGIN_FEEDBACK_CHECKS_PASS shrink=%d servo=%d inner=%d baseline=%d\n', ...
 cases,report.servo_endpoint.scenarios,report.inner.scenarios,double(report.baseline.checked));
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
function value=cost_of(v,tau_cmd,delta_actual,air,omega,u_previous,candidate,options)
[natural,B]=online_aero_moment(v,air,omega,delta_actual);
endpoint=online_servo_endpoint(v,delta_actual,candidate,options.dt,.001);
r=(tau_cmd-natural)-B*endpoint;
value=sum(r.^2)+options.lambda*sum(((candidate-u_previous)./v.delta_max(:)).^2);
end
function out=baseline_reproduction(root)
out=struct('checked',true,'max_abs_command_difference',0,'max_abs_state_difference',0, ...
 'inner_ticks',0,'expected_inner_ticks',round(2/.02)+1,'mpc_cycles',0, ...
 'expected_mpc_cycles',round(2/.2)+1,'inner_fallbacks',0,'per_arm',struct([]));
base=fullfile(root,'simulink','results','online_authority','fast_certificate','ablation');
folders={};
for arm={'A','B'}
 reference=fullfile(base,sprintf('analytic_full_%s_2s',arm{1}),sprintf('%s_closedloop.mat',arm{1}));
 if ~isfile(reference),error('U4: reference baseline missing: %s',reference);end
 folder=run_torque_margin_feedback_ablation('B0',arm{1},2,'repro');
 folders{end+1}=folder;                                                    %#ok<AGROW>
 fresh=load(fullfile(folder,sprintf('%s_closedloop.mat',arm{1})));ref=load(reference);
 cmd_new=series(fresh.r.commands);cmd_ref=series(ref.r.commands);
 x_new=series(fresh.r.states);x_ref=series(ref.r.states);
 assert(isequal(size(cmd_new),size(cmd_ref)) && isequal(size(x_new),size(x_ref)), ...
  'U4: %s log shapes differ from the baseline',arm{1});
 dc=max(abs(cmd_new(:)-cmd_ref(:)));dx=max(abs(x_new(:)-x_ref(:)));
 out.max_abs_command_difference=max(out.max_abs_command_difference,dc);
 out.max_abs_state_difference=max(out.max_abs_state_difference,dx);
 k=numel(out.per_arm)+1;
 out.per_arm(k).arm=arm{1};out.per_arm(k).max_command_difference=dc;
 out.per_arm(k).max_state_difference=dx;
 fprintf('U4 %s: command difference %.3e, state difference %.3e\n',arm{1},dc,dx);
end
% 20 ms arm: trigger counts, sample-time alignment and safe-fallback count.
folder=run_torque_margin_feedback_ablation('F20','A',2,'repro');
files=dir(fullfile(folder,'*_torque_feedback.mat'));
assert(numel(files)==1,'U4: the inner loop did not write its tick log');
d=load(fullfile(folder,files(1).name));
out.inner_ticks=numel(d.diagnostics.time);
out.inner_fallbacks=d.summary.fallbacks;
authority=dir(fullfile(folder,'*_authority.mat'));
assert(numel(authority)==1,'U4: the MPC did not write its diagnostics');
a=load(fullfile(folder,authority(1).name));
out.mpc_cycles=numel(a.diagnostics);
out.expected_mpc_cycles=round(2/.2)+1;
assert(abs(d.summary.inner_period_s-.02)<1e-12,'U4: inner sample time is not 0.02 s');
assert(all(diff(d.diagnostics.time)>0) && max(abs(diff(d.diagnostics.time)-.02))<1e-9, ...
 'U4: inner ticks are not on a uniform 0.02 s grid');
per_axis=max(abs(d.diagnostics.delta_cmd),[],1);
assert(all(per_axis<=v_delta_max()+1e-12),'U4: an inner command left the deflection limit');
end
function limit=v_delta_max()
p=vehicle_params;limit=p.vehicle.delta_max(:).';
end
function data=series(ts)
data=squeeze(ts.Data);
if size(data,1)~=numel(ts.Time),data=data.';end
end
