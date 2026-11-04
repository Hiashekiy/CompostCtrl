function [command,state,info]=native_online_rti_step(s,x,state,force_failure)
% Online authority branch. No access to offline tau_min/max, including fallback.
started=tic;N=double(s.N);nx=double(s.nx);s.nu_previous=state.nu_previous;
if nx==11,E=@native_E_B;else,E=@native_E_A;end
if isempty(state.U),U=zeros(double(s.nu),N);
else,U=[state.U(:,2:end) state.U(:,end)];end
info=struct('fallback',false,'status',0,'objective',inf,'iterations',0,'rejected',0,'authority',[]);
info.iteration_authority={};
info.repaired_candidates=0;
try
 [X,U,a,ok]=online_authority_rollout(s,x,U,s.flight_state,true);
 assert(ok,'online:restoration','Unable to restore feasible nominal control');
 obj=objective(X);info.authority=a;
 for iteration=1:12
  info.iterations=iteration;
  if force_failure,error('native:forcedFailure','Injected failure');end
  % Recompute bounds for the accepted nominal trajectory before EVERY QP.
  [X,U,a,ok]=online_authority_rollout(s,x,U,s.flight_state,false);
  assert(ok,'online:nominal','Nominal point outside recomputed authority');
  info.iteration_authority{end+1}=a;
  work=s;work.tau_min=a.tau_min;work.tau_max=a.tau_max;
  qp=assemble_native_qp(work,X,U);qp.validate_readback=false;r=solve_hpipm_snapshot(qp);
  info.status=r.status;
  assert(r.status==0 && all(isfinite(r.U),'all'),'online:qp','HPIPM failed');
  best=obj;bestX=[];bestU=[];bestA=[];
  for alpha=[1 .5 .25 .125]
   candidate=U+alpha*r.U;
   try
    [trajectory,candidate,ca,valid]=online_authority_rollout(s,x,candidate,s.flight_state,false);
    if ~valid
     % Causal feasibility restoration: recompute each node after repairing its
     % control, then propagate BOTH attitude and translation with that control.
     % This is never clipping against a box belonging to the old trajectory.
     [trajectory,candidate,ca,valid]=online_authority_rollout(s,x,candidate,s.flight_state,true);
     if valid
      valid=verify_authority_candidate(s,x,trajectory,candidate,ca,s.flight_state);
      if valid,info.repaired_candidates=info.repaired_candidates+1;end
     end
    end
   catch
    valid=false;
   end
   if ~valid,info.rejected=info.rejected+1;continue;end
   cost=objective(trajectory);
   if cost<best,best=cost;bestX=trajectory;bestU=candidate;bestA=ca;end
  end
  if isempty(bestX),break;end
  X=bestX;U=bestU;a=bestA;obj=best;
  if max(abs(r.U),[],'all')<1e-4,break;end
 end
 command=U(:,1);state.U=U;state.last=command;info.objective=obj;info.authority=a;
catch err
 % A failed future prediction cannot reuse an old command at a new air state.
 p=vehicle_params;v=p.vehicle;flight=s.flight_state;
 air=online_flight_condition(flight(1:3),flight(4:6));
 [natural,B]=online_aero_moment(v,air,x(5:7),flight(7:9));
 lo=max(-v.delta_max(:),flight(10:12)-v.delta_dot_max(:)*s.dt_ctrl);
 hi=min(v.delta_max(:),flight(10:12)+v.delta_dot_max(:)*s.dt_ctrl);
 held=min(max(flight(7:9),lo),hi);tau=natural+B*held;
 command=tau;
 if nx==11
  nu=min(max(zeros(4,1),max(-s.gdot,state.nu_previous-s.gddot*s.dt_ctrl)),min(s.gdot,state.nu_previous+s.gddot*s.dt_ctrl));
  command=[tau;nu];
 end
 state.U=[];info.fallback=true;info.failure=err.message;info.status=-1;info.authority=[];
end
if nx==11,state.nu_previous=command(4:7);end
info.elapsed_s=toc(started);
 function cost=objective(trajectory)
  W=s.weights;weight=s.dts(:)/sum(s.dts);cost=0;
  for j=1:N+1
   [e,~]=E(s.q_ref(:,j),trajectory(:,j));
   if j<=N
    dw=trajectory(5:7,j)-s.omega_ref(:,j);
    cost=cost+weight(j)*(W.w_att/W.e_scale^2*(e'*e)+W.w_rate/W.omega_scale^2*(dw'*dw));
   else,cost=cost+W.q_terminal/W.e_scale^2*(e'*e);end
  end
 end
end
