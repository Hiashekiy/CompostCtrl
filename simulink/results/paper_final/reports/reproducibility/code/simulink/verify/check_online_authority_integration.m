function check_online_authority_integration
root=repo_root;init_paths;
bank=load(fullfile(root,'simulink','data','online_plan_bank.mat'));
env=load(fullfile(root,'simulink','data','native_environment.mat'));
out=fullfile(root,'simulink','results','online_authority','stage_C');if ~exist(out,'dir'),mkdir(out);end
for enabled=[false true]
 if enabled,file='production_online_qp.mat';arm='B';x=bank.x0(:);
 else,file='production_online_A_qp.mat';arm='A';x=bank.x0(1:7);end
 s=load(fullfile(root,'simulink','tests','05_python_matlab_qp_equivalence',file));
 if enabled && numel(x)==7,x=[x;s.X_nom(8:11,1)];end
 x=x(:);
 s.authority_mode='online_predicted';s.authority_max_step=.1;
 s.flight_state=[[0;0;-env.h0];online_quaternion_rotation(x(1:4))'*env.V_ned0(:);zeros(6,1)];
 s.tau_min=nan(3,double(s.N));s.tau_max=nan(3,double(s.N));
 [command,state,info]=native_rti_step(s,x,[]);
 assert(~info.fallback,'Single MPC call fell back: %s',getfailure(info));
 assert(isequal(size(info.authority.tau_min),[3 double(s.N)]));
 assert(all(isfinite(command)) && all(info.authority.tau_min<=info.authority.tau_max,'all'));
 assert(max(info.authority.vertex_residual,[],'all')<=1e-9);
 assert(max(abs(diff(info.authority.air(4,:))))>1e-8,'Frozen future dynamic pressure');
 % An old unsafe command must never survive an online solver failure.
 state.last=1e6*ones(double(s.nu),1);
 [failed,~,failure]=native_rti_step(s,x,state,true);
 assert(failure.fallback && all(abs(failed)<1e6));
 changed=state.U;changed(1,:)=changed(1,:)+.01;
 [~,~,~,valid]=online_authority_rollout(s,x,changed,s.flight_state,false);
 % Direct per-node rollout is sensitive to the nominal attitude/control history.
 altered=state.U;altered(2,:)=altered(2,:)+.1;
 [~,~,perturbed,~]=online_authority_rollout(s,x,altered,s.flight_state,true);
 sensitivity=max(abs(perturbed.air-info.authority.air),[],'all');assert(sensitivity>1e-10);
 save(fullfile(out,[arm '_single_call.mat']),'s','command','state','info','failed','failure','valid','sensitivity');
 fprintf('ONLINE_INTEGRATION_PASS %s time=%g rejected=%d sensitivity=%g\n',arm,info.elapsed_s,info.rejected,sensitivity);
end
end
function value=getfailure(info)
value='';if isfield(info,'failure'),value=info.failure;end
end
