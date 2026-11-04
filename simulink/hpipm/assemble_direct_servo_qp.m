function s=assemble_direct_servo_qp(template,X,U,a)
% Gauss-Newton QP for the direct-servo model. Tracking cost is unchanged (same attitude,
% rate and terminal weights); the control cost is a NEW set of physically normalised
% servo-command terms, because the control is now radians rather than N.m.
persistent v
if isempty(v),p=vehicle_params;v=p.vehicle;end
s=template;N=double(s.N);nx=double(s.nx_direct);nu=double(s.nu_direct);natt=nx-3;W=s.weights;Wc=s.direct_weights;
assert(isscalar(Wc.cmd)&&isscalar(Wc.dcmd),'direct:cost','direct_servo command weights must be scalars, got %s and %s', ...
 mat2str(size(Wc.cmd)),mat2str(size(Wc.dcmd)));
s.nx=nx;s.nu=nu;s.lu=zeros(nu,N);s.uu=zeros(nu,N);
if natt==11,E=@native_E_B;Ew=zeros(3,nx);Ew(:,5:7)=eye(3);
else,E=@native_E_A;Ew=zeros(3,nx);Ew(:,5:7)=eye(3);end
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
delta_scale=v.delta_max(:);
for k=1:N
 rt=(wk(k)*2*Wc.dcmd)./delta_scale.^2;rs=(wk(k)*2*Wc.cmd)./delta_scale.^2;
 s.R(1:3,1:3,k)=diag(rt)+diag(rs);s.r(1:3,k)=rs.*U(1:3,k);
 if k>1,s.r(1:3,k)=s.r(1:3,k)+rt.*(U(1:3,k)-U(1:3,k-1));end
 if nu==7
  rn=wk(k)*2*W.r_nu/W.nu_scale^2;rdn=wk(k)*2*W.r_dnu/W.nu_scale^2;
  s.R(4:7,4:7,k)=(rn+rdn)*eye(4);s.r(4:7,k)=rn*U(4:7,k);
  if k>1,s.r(4:7,k)=s.r(4:7,k)+rdn*(U(4:7,k)-U(4:7,k-1));end
 end
 % Increment bounds stay inside the physical actuator envelope at THIS nominal point.
 span=v.delta_dot_max(:)*s.dt_ctrl;
 if k==1,previous=s.direct_previous_command(:);else,previous=U(1:3,k-1);end
 lower=max(-v.delta_max(:),previous-span);upper=min(v.delta_max(:),previous+span);
 s.lu(1:3,k)=lower-U(1:3,k);s.uu(1:3,k)=upper-U(1:3,k);
 if nu==7
  cmg_span=s.gddot*dt(k);
  s.lu(4:7,k)=max(-s.gdot-U(4:7,k),-cmg_span);s.uu(4:7,k)=min(s.gdot-U(4:7,k),cmg_span);
 end
end
s.b=zeros(nx,N);
end
