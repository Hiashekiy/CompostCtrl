function [X,U,authority,feasible]=online_authority_rollout(s,x,U,flight,restore)
% Reset at measured r/v/delta on EVERY call, including line-search candidates.
% flight=[r_NED;v_body;delta_actual;allocator_previous_command].
% Only attitude/CMG states remain HPIPM decision variables.
%
% s.authority_certificate_mode selects the torque-box / servo certificate algorithm only:
%   legacy        : eight vertex checks + 45-step bisection per vertex (unchanged default)
%   analytic_box  : analytic box, bisection still used for the servo certificate
%   analytic_full : analytic box + monotone-interval servo certificate (no bisection)
%   shadow        : the control path keeps legacy, the analytic results are computed and
%                   compared for verification only (never fed to the controller)
persistent v
if isempty(v),p=vehicle_params;v=p.vehicle;end
assert(numel(flight)==12 && all(isfinite(flight)),'online:feedback','Missing measured flight/servo feedback');
mode='legacy';if isfield(s,'authority_certificate_mode'),mode=s.authority_certificate_mode;end
assert(ismember(mode,{'legacy','analytic_box','analytic_full','shadow'}),'online:certificate', ...
 'unknown authority_certificate_mode %s',mode);
timing=true;if isfield(s,'authority_phase_timing'),timing=logical(s.authority_phase_timing);end
N=double(s.N);X=zeros(double(s.nx),N+1);X(:,1)=x;
if double(s.nx)==11,F=@native_F_B;else,F=@native_F_A;end
z=[flight(1:3);online_quaternion_rotation(x(1:4))*flight(4:6)];delta=flight(7:9);previous=flight(10:12);
authority=struct('tau_min',zeros(3,N),'tau_max',zeros(3,N),'air',zeros(6,N), ...
 'natural',zeros(3,N),'delta',zeros(3,N+1),'allocation_residual',zeros(3,N), ...
 'tracking_residual',zeros(3,N),'vertex_residual',zeros(8,N),'scale',zeros(1,N),'position',zeros(3,N+1));
authority.servo_vertex_residual=zeros(1,N);authority.vertex_commands=zeros(3,8,N);
authority.mode=s.authority_mode;authority.max_step_s=s.authority_max_step;
authority.nominal_tau=zeros(3,N);authority.allocator_command=zeros(3,N);
authority.axis_scale=zeros(3,N);
authority.certificate_mode=mode;
authority.certificate_type=cell(1,N);authority.certified=true(1,N);
authority.endpoint_margin=nan(1,N);authority.box_margin=nan(1,N);
authority.commands_evaluated=false(1,N);authority.analytic_fallback_count=0;
authority.phase_time_s=struct('air',0,'aero',0,'reachability',0,'box',0, ...
 'servo_certificate',0,'allocation',0,'attitude',0,'translation',0,'total',0);
rollout_started=tic;
authority.delta(:,1)=delta;authority.position(:,1)=z(1:3);feasible=true;
nu_previous=zeros(4,1);if isfield(s,'nu_previous'),nu_previous=s.nu_previous;end
for k=1:N
 t_phase=tic;
 R=online_quaternion_rotation(X(1:4,k));[air,detail]=online_flight_condition(z(1:3),R'*z(4:6));
 if strcmp(s.authority_mode,'online_current')
  % L1 diagnostic ablation ONLY: freeze measured air data over the horizon.
  [air,detail]=online_flight_condition(s.flight_state(1:3),s.flight_state(4:6));
 end
 authority.phase_time_s.air=authority.phase_time_s.air+toc(t_phase);
 t_phase=tic;[natural,B]=online_aero_moment(v,air,X(5:7,k),delta);
 authority.phase_time_s.aero=authority.phase_time_s.aero+toc(t_phase);
 lower_cmd=max(-v.delta_max(:),previous-v.delta_dot_max(:)*s.dt_ctrl);
 upper_cmd=min(v.delta_max(:),previous+v.delta_dot_max(:)*s.dt_ctrl);
 t_phase=tic;
 [lower,upper]=online_surface_reachability(v,delta,s.dts(k),previous,.001,lower_cmd,upper_cmd);
 authority.phase_time_s.reachability=authority.phase_time_s.reachability+toc(t_phase);
 lower=max(lower,lower_cmd);upper=min(upper,upper_cmd);
 if any(lower>upper),feasible=false;return;end
 center=(lower+upper)/2;
 t_phase=tic;
 analytic=strcmp(mode,'analytic_box')||strcmp(mode,'analytic_full');
 if analytic,box=online_fast_torque_box(natural,B,lower,upper,center);
 else,box=online_certified_torque_box(natural,B,lower,upper,center);end
 authority.phase_time_s.box=authority.phase_time_s.box+toc(t_phase);
 if isfield(box,'certificate_type'),authority.certificate_type{k}=box.certificate_type;
 else,authority.certificate_type{k}='legacy_vertices';end
 if isfield(box,'margin_delta'),authority.box_margin(k)=box.margin_delta;end
 t_phase=tic;
 if strcmp(mode,'analytic_full')
  certificate=online_fast_servo_certificate(v,delta,s.dts(k),box.vertex_delta,lower_cmd,upper_cmd);
  if ~certificate.certified
   authority.analytic_fallback_count=authority.analytic_fallback_count+1;
   certificate=online_certify_servo_vertices(v,delta,s.dts(k),box.vertex_delta,lower_cmd,upper_cmd);
   authority.certificate_type{k}='legacy_bisection';
  end
 else
  certificate=online_certify_servo_vertices(v,delta,s.dts(k),box.vertex_delta,lower_cmd,upper_cmd);
  if strcmp(mode,'shadow')
   % Verification only: never used for control or for the stored bounds.
   try
    fast_box=online_fast_torque_box(natural,B,lower,upper,center);
    fast_certificate=online_fast_servo_certificate(v,delta,s.dts(k),box.vertex_delta,lower_cmd,upper_cmd);
    authority.shadow_box_error(k)=max(abs([fast_box.tau_min-box.tau_min;fast_box.tau_max-box.tau_max]));
    authority.shadow_scale_error(k)=max(abs(fast_box.axis_scale-box.axis_scale));
    authority.shadow_certified(k)=fast_certificate.certified;
    authority.shadow_margin(k)=fast_certificate.endpoint_margin;
   catch
    authority.shadow_certified(k)=false;
   end
  end
 end
 authority.phase_time_s.servo_certificate=authority.phase_time_s.servo_certificate+toc(t_phase);
 outside=any(U(1:3,k)<box.tau_min-1e-8 | U(1:3,k)>box.tau_max+1e-8);
 if restore,U(1:3,k)=min(max(U(1:3,k),box.tau_min),box.tau_max);
 elseif outside,feasible=false;return;end
 if double(s.nx)==11
  lo=max(-s.gdot,nu_previous-s.gddot*s.dts(k));hi=min(s.gdot,nu_previous+s.gddot*s.dts(k));
  if k==1,lo=max(-s.gdot,nu_previous-s.gddot*s.dt_ctrl);hi=min(s.gdot,nu_previous+s.gddot*s.dt_ctrl);end
  if restore,U(4:7,k)=min(max(U(4:7,k),lo),hi);
  elseif any(U(4:7,k)<lo-1e-8 | U(4:7,k)>hi+1e-8),feasible=false;return;end
  nu_previous=U(4:7,k);
 end
 target=U(1:3,k)-natural;
 t_phase=tic;
 if rcond(B)>1e-10
  command=B\target;
  if all(command>=lower_cmd-1e-10 & command<=upper_cmd+1e-10)
   command=min(max(command,lower_cmd),upper_cmd);residual=B*command-target;
  else,[command,residual]=bounded_control_allocation(B,target,lower_cmd,upper_cmd);end
 else,[command,residual]=bounded_control_allocation(B,target,lower_cmd,upper_cmd);end
 authority.phase_time_s.allocation=authority.phase_time_s.allocation+toc(t_phase);
 if norm(residual,inf)>1e-7,feasible=false;return;end
 t_phase=tic;
 [~,~,next_delta]=online_surface_reachability(v,delta,s.dts(k),command,.001,lower_cmd,upper_cmd);
 X(:,k+1)=F(X(:,k),U(:,k),s.tau_d(:,k),s.dts(k));
 authority.phase_time_s.attitude=authority.phase_time_s.attitude+toc(t_phase);
 t_phase=tic;
 prediction=online_translation_prediction(z,[0;s.dts(k)],X(1:4,k:k+1),s.authority_max_step);
 z=prediction.state(:,end);
 authority.phase_time_s.translation=authority.phase_time_s.translation+toc(t_phase);
 authority.tau_min(:,k)=box.tau_min;authority.tau_max(:,k)=box.tau_max;
 authority.air(:,k)=detail;authority.natural(:,k)=natural;
 authority.vertex_residual(:,k)=box.vertex_residual(:);authority.scale(k)=box.scale;
 authority.axis_scale(:,k)=box.axis_scale;
 % Servo certificate: numeric residual/commands exist only where they were evaluated.
 if isfield(certificate,'commands_evaluated')&&~certificate.commands_evaluated
  authority.servo_vertex_residual(k)=NaN;
  if k==1,authority.vertex_commands=nan(3,8,N);end
  authority.commands_evaluated(k)=false;
  authority.endpoint_margin(k)=certificate.endpoint_margin;
  authority.certified(k)=certificate.certified;
 else
  authority.servo_vertex_residual(k)=certificate.max_residual_rad;
  authority.vertex_commands(:,:,k)=certificate.command;
  authority.commands_evaluated(k)=true;
  authority.certified(k)=true;
 end
 authority.nominal_tau(:,k)=U(1:3,k);authority.allocator_command(:,k)=command;
 authority.allocation_residual(:,k)=residual;authority.tracking_residual(:,k)=natural+B*next_delta-U(1:3,k);
 delta=next_delta;previous=command;authority.delta(:,k+1)=delta;authority.position(:,k+1)=z(1:3);
end
authority.phase_time_s.total=toc(rollout_started);
if ~timing
 for name=fieldnames(authority.phase_time_s)'
  authority.phase_time_s.(name{1})=NaN;
 end
end
end
