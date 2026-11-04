function check_online_five_sqp
% Repeatable checks for the five-round full-step SQP (no alpha line search).
root=repo_root;init_paths;
bank=load(fullfile(root,'simulink','data','online_plan_bank.mat'));
env=load(fullfile(root,'simulink','data','native_environment.mat'));
out=fullfile(root,'simulink','results','online_authority','five_sqp');if ~exist(out,'dir'),mkdir(out);end
for enabled=[false true]
 if enabled,file='production_online_qp.mat';arm='B';x=bank.x0(:);
 else,file='production_online_A_qp.mat';arm='A';x=bank.x0(1:7);end
 s=load(fullfile(root,'simulink','tests','05_python_matlab_qp_equivalence',file));
 if enabled && numel(x)==7,x=[x;s.X_nom(8:11,1)];end
 x=x(:);N=double(s.N);nu=double(s.nu);
 s.authority_mode='online_predicted';s.authority_max_step=.1;
 s.flight_state=[[0;0;-env.h0];online_quaternion_rotation(x(1:4))'*env.V_ned0(:);zeros(6,1)];
 s.tau_min=nan(3,N);s.tau_max=nan(3,N);
 % -- default five rounds, cold start --------------------------------------
 [command,state,info]=native_rti_step(s,x,[]);
 assert(~info.fallback,'five_sqp %s fell back on the first cycle: %s',arm,reason(info));
 assert(info.max_sqp_iterations==5,'Default round budget is not 5');
 assert(info.iterations<=5 && info.qp_solved<=5 && numel(info.status_by_qp)==info.iterations, ...
  'Round budget or per-round QP status log is broken');
 assert(info.accepted_steps<=info.qp_solved,'More accepted steps than solved QPs');
 assert(numel(info.iteration_authority)==info.accepted_steps+1,'Accepted nominal points were not logged');
 assert(info.full_step_checks<=info.qp_solved,'More full-step checks than solved QPs');
 assert(all(isfinite(command)) && norm(info.authority.nominal_tau(:,1)-command(1:3),inf)<1e-12, ...
  'First command is not the certified stage-one torque');
 row=struct('arm',arm,'rounds',5,'qp_solved',info.qp_solved,'iterations',info.iterations, ...
  'accepted_steps',info.accepted_steps,'rejected_steps',info.rejected_steps, ...
  'restoration_calls',info.restoration_calls,'full_step_checks',info.full_step_checks, ...
  'certificate_count',info.certificate_count,'early_stop',info.early_stop_reason, ...
  'qp_total_s',sum(info.qp_total_time_s),'qp_solver_s',sum(info.qp_solver_time_s), ...
  'rollout_s',sum(info.rollout_time_s),'cycle_s',info.cycle_time_s);
 % -- warm start: the plan must be shifted in TIME, not by column index -----
 [~,warmstate,warminfo]=native_rti_step(s,x,state);
 assert(~warminfo.fallback,'Warm-start cycle fell back: %s',reason(warminfo));
 shifted=online_shift_controls(state.U,s.dts,s.dt_ctrl);
 for j=1:N
  tau=sum(s.dts(1:j-1))+s.dt_ctrl;index=N;
  for k=1:N
   if tau>=sum(s.dts(1:k-1))-1e-12 && tau<sum(s.dts(1:k-1))+s.dts(k),index=k;break;end
  end
  assert(max(abs(shifted(:,j)-state.U(:,index)))<1e-12,'Warm start misaligned at stage %d',j);
  if abs(s.dts(j)-s.dt_ctrl)<1e-12,assert(max(abs(shifted(:,j)-state.U(:,j+1)))<1e-12);end
 end
 naive=[state.U(:,2:end) state.U(:,end)];
 displaced=find(max(abs(shifted(1:3,:)-naive(1:3,:)),[],1)>1e-12);
 assert(~isempty(displaced),'Non-uniform grid still shifted by index only');
 row.warm_start_misaligned_stages=numel(displaced);
 % -- four-round configuration --------------------------------------------
 s4=s;s4.max_sqp_iterations=4;
 [c4,~,i4]=native_rti_step(s4,x,[],false);
 assert(~i4.fallback,'Four-round configuration fell back: %s',reason(i4));
 assert(i4.max_sqp_iterations==4 && i4.iterations<=4 && i4.qp_solved<=4,'Four-round budget broken');
 row.rounds4_qp_solved=i4.qp_solved;row.rounds4_accepted=i4.accepted_steps;row.rounds4_command_delta=norm(c4-command,inf);
 for bad=[0 3 6 12]
  sb=s;sb.max_sqp_iterations=bad;rejected=false;
  try,native_rti_step(sb,x,[]);catch,rejected=true;end
  assert(rejected,'max_sqp_iterations=%d was accepted',bad);
 end
 % -- injected QP failure on rounds 1..5 -----------------------------------
 warm=online_shift_controls(state.U,s.dts,s.dt_ctrl);
 [Xr,Ur,ar,okr]=online_authority_rollout(s,x,warm,s.flight_state,true);
 assert(okr && verify_authority_candidate(s,x,Xr,Ur,ar,s.flight_state));
 failed=state;failed.inject_failure_at=1;
 [c1,st1,i1]=native_rti_step(s,x,failed);
 assert(~i1.fallback && i1.accepted_steps==0,'Round-1 failure did not stop the cycle cleanly');
 assert(strcmp(i1.early_stop_reason,'injected_qp_failure'));
 assert(max(abs(c1-Ur(:,1)))<1e-12,'Round-1 failure did not return the certified nominal first control');
 % Rounds the solver never reaches cannot be injected. A cold start at a slightly
 % perturbed measured attitude needs more rounds, so the mid-iteration cases are
 % exercised on the hardest of a fixed set of perturbations.
 probes=[0 0 0;2 0 0;0 2 0;0 0 2;3 3 0;-3 2 0;0 3 -4];
 chosen=x;chosen_rounds=0;
 for k=1:size(probes,1)
  xk=x;xk(1:4)=quat_mul(x(1:4),euler_quat(probes(k,:)/180*pi));xk(1:4)=xk(1:4)/norm(xk(1:4));
  try,[~,~,ik]=native_rti_step(s,xk,[]);catch,ik.iterations=0;end
  if ik.iterations>chosen_rounds,chosen=xk;chosen_rounds=ik.iterations;end
 end
 accepted_by_round=zeros(1,5);
 for iteration=1:5
  injected=state;injected.inject_failure_at=iteration;
  [ci,sti,ii]=native_rti_step(s,x,injected);
  accepted_by_round(iteration)=ii.accepted_steps;
  assert(~ii.fallback,'Injected failure at round %d forced a fallback',iteration);
  if iteration<=warminfo.iterations
   assert(strcmp(ii.early_stop_reason,'injected_qp_failure'),'Injected failure at round %d was not reported',iteration);
   assert(ii.accepted_steps<iteration);
   assert(all(isfinite(ci)) && max(ii.authority.vertex_residual,[],'all')<=1e-9);
   assert(all(ci(1:3)>=ii.authority.tau_min(:,1)-1e-8) && all(ci(1:3)<=ii.authority.tau_max(:,1)+1e-8), ...
    'Injected-failure output is outside its own certified box');
   assert(norm(ii.authority.nominal_tau(:,1)-ci(1:3),inf)<1e-12,'Output torque is not the certified stage-one torque');
   save(fullfile(out,sprintf('%s_injected_round%d.mat',arm,iteration)),'ii','ci','sti','iteration');
  end
 end
 for iteration=[3 5]
  injected=struct('U',[],'last',[],'nu_previous',zeros(4,1),'inject_failure_at',iteration);
  [ci,sti,ii]=native_rti_step(s,chosen,injected);
  assert(~ii.fallback,'Cold-start injected failure at round %d forced a fallback',iteration);
  if iteration<=chosen_rounds
   assert(strcmp(ii.early_stop_reason,'injected_qp_failure'),'Cold-start injected failure at round %d was not reported',iteration);
   assert(ii.accepted_steps<iteration);
   assert(norm(ii.authority.nominal_tau(:,1)-ci(1:3),inf)<1e-12,'Cold-start output is not the certified stage-one torque');
   save(fullfile(out,sprintf('%s_cold_injected_round%d.mat',arm,iteration)),'ii','ci','sti','iteration');
  end
 end
 row.rounds_reached_without_injection=warminfo.iterations;
 row.rounds_reached_cold_perturbed=chosen_rounds;
 row.accepted_steps_by_injected_round=accepted_by_round;
 % -- the first command gate rejects tampered plans ------------------------
 [valid,detail]=online_verify_first_command(s,st1.U,i1.authority,s.flight_state);
 assert(valid,'Certified first command was rejected: %s',detail);
 tampered=st1.U;tampered(1:3,1)=tampered(1:3,1)+1;
 [valid,~]=online_verify_first_command(s,tampered,i1.authority,s.flight_state);
 assert(~valid,'First-command gate accepted a torque outside the certified box');
 tampered=st1.U;tampered(1:3,1)=i1.authority.tau_max(:,1)+1e-3;
 [valid,~]=online_verify_first_command(s,tampered,i1.authority,s.flight_state);
 assert(~valid,'First-command gate accepted an over-range command');
 assert(nu==size(st1.U,1));
 save(fullfile(out,[arm '_five_sqp.mat']),'s','command','state','info','i4','c4','warminfo','row');
 fid=fopen(fullfile(out,[arm '_five_sqp.json']),'w');fprintf(fid,'%s',jsonencode(row,'PrettyPrint',true));fclose(fid);
 fprintf('ONLINE_FIVE_SQP_PASS %s QP=%d accepted=%d stop=%s cycle=%gs\n',arm,info.qp_solved, ...
  info.accepted_steps,info.early_stop_reason,info.cycle_time_s);
end
fprintf('ONLINE_FIVE_SQP_TEST_PASS\n');
end
function value=reason(info)
value='';if isfield(info,'failure'),value=info.failure;end
end
function q=quat_mul(p,q)
% Scalar-first quaternion product, so the perturbation stays inside the 4-vector.
p=p(:)';q=q(:)';
q=[p(1)*q(1)-p(2)*q(2)-p(3)*q(3)-p(4)*q(4); ...
   p(1)*q(2)+p(2)*q(1)+p(3)*q(4)-p(4)*q(3); ...
   p(1)*q(3)-p(2)*q(4)+p(3)*q(1)+p(4)*q(2); ...
   p(1)*q(4)+p(2)*q(3)-p(3)*q(2)+p(4)*q(1)];
end
function q=euler_quat(d)
% Roll/pitch/yaw offsets (rad) as one scalar-first quaternion.
q=[1 0 0 0];axes=eye(3);
for axis=1:3
 q=quat_mul(q,[cos(d(axis)/2) axes(axis,:)*sin(d(axis)/2)]);
end
end
