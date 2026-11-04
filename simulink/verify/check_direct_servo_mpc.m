function check_direct_servo_mpc
% S2 unit gate: closed-form servo inside the horizon, stage map versus a 1 ms reference,
% Jacobian versus finite differences of the same nonlinear map, one full MPC call.
root=repo_root;init_paths;
bank=load(fullfile(root,'simulink','data','online_plan_bank.mat'));
env=load(fullfile(root,'simulink','data','native_environment.mat'));
out=fullfile(root,'simulink','results','online_authority','direct_servo');if ~exist(out,'dir'),mkdir(out);end
p=vehicle_params;v=p.vehicle;
report={};
for enabled=[false true]
 if enabled,file='production_online_qp.mat';arm='B';x=bank.x0(:);
 else,file='production_online_A_qp.mat';arm='A';x=bank.x0(1:7);end
 s=load(fullfile(root,'simulink','tests','05_python_matlab_qp_equivalence',file));
 if enabled && numel(x)==7,x=[x;s.X_nom(8:11,1)];end
 x=x(:);
 if enabled,s.nx_direct=14;s.nu_direct=7;else,s.nx_direct=10;s.nu_direct=3;end
 s.authority_mode='direct_servo';s.authority_max_step=.1;
 s.direct_weights=struct('cmd',s.weights.r_dtau,'dcmd',s.weights.r_dtau);
 s.max_sqp_iterations=4;s.nu_previous=zeros(4,1);
 s.flight_state=[[0;0;-env.h0];online_quaternion_rotation(x(1:4))'*env.V_ned0(:);zeros(6,1)];
 s.direct_previous_command=s.flight_state(10:12);
 N=double(s.N);
 row=struct('arm',arm,'nx',s.nx_direct,'nu',s.nu_direct,'N',N);
 % 1. closed-form servo state inside a stage
 cmd0=[.05;-.04;.02];probe=servo_predict(v,s.flight_state(7:9),cmd0,.2,[],[],.001);
 row.servo_closed_form_error=0;
 for t=[.05 .11 .175 .2]
  now=direct_servo_state_at(v,s.flight_state(7:9),cmd0,t);
  row.servo_closed_form_error=max(row.servo_closed_form_error,max(abs(now-probe.delta(:,round(t/.001)+1))));
 end
 assert(row.servo_closed_form_error<1e-14,'S2: closed-form servo disagrees with the 1 ms recurrence');
 % 2. stage map versus a 1 ms reference, short and long stage
 air=online_flight_condition([0;0;-env.h0],online_quaternion_rotation(x(1:4))'*env.V_ned0(:));
 nu_cmd=[.3;-.2;.1;-.15];delta_cmd=[.06;-.05;.03];
 xk=x;xk(s.nx_direct-2:s.nx_direct)=s.flight_state(7:9);
 for dt=[.2 2]
  U=zeros(s.nu_direct,1);U(1:3)=delta_cmd;if s.nu_direct==7,U(4:7)=nu_cmd;end
  fast=direct_servo_stage(s,xk,U,dt,air,s.tau_d(:,1));
  slow=xk;
  for j=1:round(dt/.001)
   dm=direct_servo_state_at(v,slow(s.nx_direct-2:s.nx_direct),delta_cmd,.0005);
   de=direct_servo_state_at(v,slow(s.nx_direct-2:s.nx_direct),delta_cmd,.001);
   torque=online_aero_moment(v,air,slow(5:7),dm);
   if s.nx_direct==14
    att=native_F_B(slow(1:11),[torque;U(4:7)],s.tau_d(:,1),.001);
   else
    att=native_F_A(slow(1:7),torque,s.tau_d(:,1),.001);
   end
   slow=[att;de];
  end
  if dt==.2,row.stage_error_dt_0p2=norm(fast-slow,inf);else,row.stage_error_dt_2=norm(fast-slow,inf);end
 end
 % 3. Jacobian versus finite differences of the same map
 [~,Bd]=online_aero_moment(v,air,x(5:7),s.flight_state(7:9));
 a=struct('B_delta',repmat(Bd,[1 1 N]));
 U=zeros(s.nu_direct,N);U(1:3,:)=repmat(delta_cmd,1,N);
 if s.nu_direct==7,U(4:7,:)=repmat(nu_cmd,1,N);end
 X=repmat(x,1,N+1);X(s.nx_direct-2:s.nx_direct,:)=repmat(s.flight_state(7:9),1,N+1);
 [A,B]=direct_servo_linearize(s,X,U,a,s.tau_d);
 worst=0;
 for stage=[1 N]
  dt=s.dts(stage);y0=X(:,stage);u0=U(:,stage);
  base=direct_servo_stage(s,y0,u0,dt,air,s.tau_d(:,stage));
  for j=[5 12 14]
   if j>s.nx_direct,continue;end
   yp=y0;yp(j)=yp(j)+1e-6;
   delta_out=direct_servo_stage(s,yp,u0,dt,air,s.tau_d(:,stage))-base;
   analytic=A(:,j,stage)*1e-6;
   worst=max(worst,norm(delta_out-analytic,inf)/max(norm(analytic,inf),1e-14));
  end
  for j=[1 4 7]
   if j>s.nu_direct,continue;end
   up=u0;up(j)=up(j)+1e-6;
   delta_out=direct_servo_stage(s,y0,up,dt,air,s.tau_d(:,stage))-base;
   analytic=B(:,j,stage)*1e-6;
   worst=max(worst,norm(delta_out-analytic,inf)/max(norm(analytic,inf),1e-14));
  end
 end
 row.jacobian_worst_relative_error=worst;
 % 4. propagation, verification and one MPC call
 x0=[x(1:s.nx_direct-3);s.flight_state(7:9)];
 [~,Up,ap,ok,reason]=direct_servo_propagate(s,x0,zeros(s.nu_direct,N),s.flight_state,true);
 assert(ok,'S2: cold-start propagation failed: %s',reason);
 [valid,detail]=direct_servo_verify(s,x0,Up,ap,s.flight_state);
 assert(valid,'S2: restored plan failed verification: %s',detail);
 state=struct('U',[],'last',[],'nu_previous',zeros(4,1),'delta_command',s.flight_state(10:12));
 [command,state,info]=direct_servo_mpc_step(s,x,state);
 assert(~info.fallback,'S2: direct-servo MPC fell back: %s',reason_of(info));
 assert(info.max_sqp_iterations==4&&info.iterations<=4&&info.qp_solved<=4,'S2: round budget broken');
 assert(all(isfinite(command))&&all(abs(command(1:3))<=v.delta_max(:)+1e-12),'S2: command outside limits');
 [valid,detail]=direct_servo_verify(s,x0,state.U,info.authority,s.flight_state);
 assert(valid,'S2: emitted plan failed verification: %s',detail);
 assert(norm(info.authority.command(:,1)-command(1:3),inf)<1e-12,'S2: emitted command is not the verified one');
 row.qp_solved=info.qp_solved;row.accepted_steps=info.accepted_steps;row.rejected_steps=info.rejected_steps;
 row.early_stop=info.early_stop_reason;row.cycle_s=info.cycle_time_s;
 row.qp_total_s=sum(info.qp_total_time_s);row.rollout_s=sum(info.rollout_time_s);
 row.first_command=command(:)';
 [command2,~,info2]=direct_servo_mpc_step(s,x,state);
 assert(~info2.fallback,'S2: warm-start MPC fell back: %s',reason_of(info2));
 row.warm_command_delta=norm(command2-command,inf);
 report{end+1}=row; %#ok<AGROW>
 fprintf('DIRECT_MPC %s nx=%d nu=%d jac_rel=%.2e stage0.2=%.2e stage2=%.2e QP=%d acc=%d stop=%s cycle=%.3fs\n', ...
  arm,s.nx_direct,s.nu_direct,row.jacobian_worst_relative_error,row.stage_error_dt_0p2, ...
  row.stage_error_dt_2,info.qp_solved,info.accepted_steps,info.early_stop_reason,info.cycle_time_s);
 save(fullfile(out,sprintf('%s_direct_mpc_unit.mat',arm)),'s','command','state','info','row');
end
report=cell2mat(report);
fid=fopen(fullfile(out,'direct_mpc_unit.json'),'w');fprintf(fid,'%s',jsonencode(report,'PrettyPrint',true));fclose(fid);
fprintf('DIRECT_SERVO_MPC_UNIT_PASS\n');
end
function value=reason_of(info)
value='';if isfield(info,'failure'),value=info.failure;end
end
