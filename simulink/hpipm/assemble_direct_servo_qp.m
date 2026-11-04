function s=assemble_direct_servo_qp(template,X,U,a)
% Gauss-Newton QP for the direct-servo model. The tracking cost is unchanged; the control
% cost is the physically normalised servo/CMG command cost of direct_servo_objective, and
% its gradient carries BOTH couplings of the adjacent-command penalty:
%   d/du_k  |u_k-u_{k-1}|^2 + |u_{k+1}-u_k|^2
% = 2(u_k-u_{k-1}) - 2(u_{k+1}-u_k)                       (linear term, both neighbours)
% and the block-diagonal Hessian 2*lambda*(2) per stage. HPIPM's stage-wise R cannot hold
% the off-diagonal u_k/u_{k+1} block, so this stays an explicit BLOCK-DIAGONAL second-order
% approximation of an exact first-order model; it is never described as an exact SQP Hessian.
% model_mode='legacy' reproduces the bcd96fd cost (no CMG terms, first-neighbour only).
persistent v
if isempty(v),p=vehicle_params;v=p.vehicle;end
s=template;N=double(s.N);nx=double(s.nx_direct);nu=double(s.nu_direct);natt=nx-3;W=s.weights;Wc=s.direct_weights;
legacy=isfield(s,'model_mode')&&strcmp(s.model_mode,'legacy');
assert(isscalar(Wc.cmd)&&isscalar(Wc.dcmd),'direct:cost','direct_servo command weights must be scalars');
s.nx=nx;s.nu=nu;s.lu=zeros(nu,N);s.uu=zeros(nu,N);
if natt==11,E=@native_E_B;else,E=@native_E_A;end
Ew=zeros(3,nx);Ew(:,5:7)=eye(3);
dt=s.dts(:);wk=dt/sum(dt);qa=wk*2*W.w_att/W.e_scale^2;qr=wk*2*W.w_rate/W.omega_scale^2;
qt=2*W.q_terminal/W.e_scale^2;
s.Q=zeros(nx,nx,N+1);s.q=zeros(nx,N+1);
for k=1:N+1
 [e,Je]=E(s.q_ref(:,k),X(1:natt,k));
 Je_full=zeros(3,nx);Je_full(:,1:size(Je,2))=Je;
 weight=qa(min(k,N));if k>N,weight=qt;end
 s.Q(:,:,k)=weight*(Je_full'*Je_full);s.q(:,k)=weight*(Je_full'*e);
 if k<=N
  s.Q(:,:,k)=s.Q(:,:,k)+qr(k)*(Ew'*Ew);
  s.q(:,k)=s.q(:,k)+qr(k)*Ew'*(X(5:7,k)-s.omega_ref(:,k));
 end
end
[A,B]=direct_servo_linearize(s,X,U,a,s.tau_d);
s.A=A;s.B=B;s.R=zeros(nu,nu,N);s.r=zeros(nu,N);
D=v.delta_max(:);
for k=1:N
 rc=2*wk(k)*Wc.cmd./D.^2;rd=2*wk(k)*Wc.dcmd./D.^2;
 if k==1,previous=s.direct_previous_command(:);else,previous=U(1:3,k-1);end
 next_command=U(1:3,k);
 neighbour=0;
 if k<N,next_command=U(1:3,k+1);neighbour=2*wk(k+1)*Wc.dcmd./D.^2;end
 s.R(1:3,1:3,k)=diag(rc+rd+neighbour);
 s.r(1:3,k)=rc.*U(1:3,k)+rd.*(U(1:3,k)-previous);
 if k<N,s.r(1:3,k)=s.r(1:3,k)-neighbour.*(U(1:3,k+1)-U(1:3,k));end
 if nu==7
  rn=2*wk(k)*W.r_nu/W.nu_scale^2;rdn=2*wk(k)*W.r_dnu/W.nu_scale^2;
  if legacy
   s.R(4:7,4:7,k)=(rn+rdn)*eye(4);s.r(4:7,k)=rn*U(4:7,k);
   if k>1,s.r(4:7,k)=s.r(4:7,k)+rdn*(U(4:7,k)-U(4:7,k-1));end
  else
   if k==1,previous_nu=s.nu_previous(:);else,previous_nu=U(4:7,k-1);end
   nb=0;if k<N,nb=2*wk(k+1)*W.r_dnu/W.nu_scale^2;end
   s.R(4:7,4:7,k)=(rn+rdn+nb)*eye(4);
   s.r(4:7,k)=rn*U(4:7,k)+rdn*(U(4:7,k)-previous_nu);
   if k<N,s.r(4:7,k)=s.r(4:7,k)-nb*(U(4:7,k+1)-U(4:7,k));end
  end
 end
 span=v.delta_dot_max(:)*s.dt_ctrl;
 if k==1,previous_command=s.direct_previous_command(:);else,previous_command=U(1:3,k-1);end
 lower=max(-v.delta_max(:),previous_command-span);upper=min(v.delta_max(:),previous_command+span);
 s.lu(1:3,k)=lower-U(1:3,k);s.uu(1:3,k)=upper-U(1:3,k);
 if nu==7
  cmg_span=s.gddot*dt(k);
  s.lu(4:7,k)=max(-s.gdot-U(4:7,k),-cmg_span);s.uu(4:7,k)=min(s.gdot-U(4:7,k),cmg_span);
 end
end
s.b=zeros(nx,N);
end
