function [command,state,info]=native_rti_step(s,x,state,force_failure)
% Online B-arm RTI, shifted trajectory warm start, bounded previous/PD fallback.
% s supplies current and future reference/authority; it must be updated every tick.
if nargin<3 || isempty(state),state=struct('U',[],'last',[],'nu_previous',zeros(4,1));end
if nargin<4,force_failure=false;end
if isfield(s,'authority_mode') && startsWith(s.authority_mode,'online_')
 [command,state,info]=native_online_rti_step(s,x,state,force_failure);return;
end
N=double(s.N);dt=s.dts(:);nx=double(s.nx);
if nx==11,F=@native_F_B;E=@native_E_B;else,F=@native_F_A;E=@native_E_A;end
if isempty(state.U)
 U=s.tau_nom(:,1:N);if nx==11,U=[U;zeros(4,N)];end
else,U=[state.U(:,2:end) state.U(:,end)];end
X=rollout(U);U=clip(U);obj=objective(X);status=0;
try
 for iteration=1:12
  if force_failure,error('native:forcedFailure','Injected failure');end
  qp=assemble_native_qp(s,X,U);qp.validate_readback=false;r=solve_hpipm_snapshot(qp);status=r.status;
  if status~=0 || any(~isfinite(r.U),'all'),error('native:qpFailed','HPIPM status %d',status);end
  best=obj;bestX=[];bestU=[];
  for alpha=[1 .5 .25 .125]
   candidate=clip(U+alpha*r.U);trajectory=rollout(candidate);cost=objective(trajectory);
   if cost<best,best=cost;bestX=trajectory;bestU=candidate;end
  end
  if isempty(bestX),break;end
  X=bestX;U=bestU;obj=best;
  if max(abs(r.U),[],'all')<1e-4,break;end
 end
 command=U(:,1);state.U=U;state.last=command;info.fallback=false;
catch err
 if isempty(state.last)
  [e,~]=E(s.q_ref(:,1),x);
  acceleration=2/s.dt_ctrl*(.05*e-(x(5:7)-s.omega_ref(:,1)));
  tau=min(max(s.tau_nom(:,1)+s.J*acceleration,s.tau_min(:,1)),s.tau_max(:,1));
  lo=max(-s.gdot,state.nu_previous-s.gddot*s.dt_ctrl);hi=min(s.gdot,state.nu_previous+s.gddot*s.dt_ctrl);
  if nx==11,[nu,~]=bounded_control_allocation(cmg_A(x(8:11)),-s.gap(:,1),lo,hi);command=[tau;nu];else,command=tau;end
 else,command=state.last;end
 state.U=[];info.fallback=true;info.failure=err.message;status=-1;
end
command(1:3)=min(max(command(1:3),s.tau_min(:,1)),s.tau_max(:,1));
if nx==11
 command(4:7)=min(max(command(4:7),max(-s.gdot,state.nu_previous-s.gddot*s.dt_ctrl)),min(s.gdot,state.nu_previous+s.gddot*s.dt_ctrl));
 state.nu_previous=command(4:7);
end
info.status=status;info.objective=obj;
if exist('iteration','var'),info.iterations=iteration;else,info.iterations=0;end
 function value=clip(value)
  value(1:3,:)=min(max(value(1:3,:),s.tau_min(:,1:N)),s.tau_max(:,1:N));
  if nx==11,value(4:7,:)=min(max(value(4:7,:),-s.gdot),s.gdot);end
 end
 function trajectory=rollout(value)
  trajectory=zeros(nx,N+1);trajectory(:,1)=x;
  for j=1:N,trajectory(:,j+1)=F(trajectory(:,j),value(:,j),s.tau_d(:,j),dt(j));end
 end
 function cost=objective(trajectory)
  W=s.weights;weight=dt/sum(dt);cost=0;
  for j=1:N+1
   [e,~]=E(s.q_ref(:,j),trajectory(:,j));
   if j<=N
    dw=trajectory(5:7,j)-s.omega_ref(:,j);
    cost=cost+weight(j)*(W.w_att/W.e_scale^2*(e'*e)+W.w_rate/W.omega_scale^2*(dw'*dw));
   else,cost=cost+W.q_terminal/W.e_scale^2*(e'*e);end
  end
 end
 function A=cmg_A(gamma)
  A=zeros(3,4);
  for j=1:4,A(:,j)=s.h0*(-s.s0(:,j)*sin(gamma(j))+s.gxs0(:,j)*cos(gamma(j)));end
 end
end
