function s=assemble_native_qp(template,X,U)
% MATLAB assembly of the production RTI Gauss-Newton QP (B configuration).
% Generated functions contain only controller prediction, never physical truth.
s=template;N=double(s.N);nx=double(s.nx);nu=double(s.nu);W=s.weights;
if nx==11,E=@native_E_B;Af=@native_A_B;Bf=@native_B_B;else,E=@native_E_A;Af=@native_A_A;Bf=@native_B_A;end
dt=s.dts(:);wk=dt/sum(dt);qa=wk*2*W.w_att/W.e_scale^2;
qr=wk*2*W.w_rate/W.omega_scale^2;qt=2*W.q_terminal/W.e_scale^2;
Ew=zeros(3,nx);Ew(:,5:7)=eye(3);
for k=1:N+1
 [e,Je]=E(s.q_ref(:,k),X(:,k));
 if k<=N,weight=qa(k);else,weight=qt;end
 s.Q(:,:,k)=weight*(Je'*Je);s.q(:,k)=weight*(Je'*e);
 if k<=N
  s.Q(:,:,k)=s.Q(:,:,k)+qr(k)*(Ew'*Ew);
  s.q(:,k)=s.q(:,k)+qr(k)*Ew'*(X(5:7,k)-s.omega_ref(:,k));
 end
end
for k=1:N
 s.A(:,:,k)=Af(X(:,k),U(:,k),s.tau_d(:,k),dt(k));
 s.B(:,:,k)=Bf(X(:,k),U(:,k),s.tau_d(:,k),dt(k));
 s.R(:,:,k)=zeros(nu);s.r(:,k)=zeros(nu,1);
 rn=wk(k)*2*W.r_nu/W.nu_scale^2;rdn=wk(k)*2*W.r_dnu/W.nu_scale^2;
 rt=wk(k)*2*W.r_dtau/W.d_scale^2;
 if nx==11,s.R(4:7,4:7,k)=(rn+rdn)*eye(4);s.r(4:7,k)=rn*U(4:7,k);end
 s.R(1:3,1:3,k)=rt*eye(3);
 if k>1
  if nx==11,s.r(4:7,k)=s.r(4:7,k)+rdn*(U(4:7,k)-U(4:7,k-1));end
  s.r(1:3,k)=rt*(U(1:3,k)-U(1:3,k-1));
 end
 span=s.gddot*dt(k);
 if nx==11
  s.lu(:,k)=[s.tau_min(:,k)-U(1:3,k);max(-s.gdot-U(4:7,k),-span)];
  s.uu(:,k)=[s.tau_max(:,k)-U(1:3,k);min(s.gdot-U(4:7,k),span)];
 else,s.lu(:,k)=s.tau_min(:,k)-U(:,k);s.uu(:,k)=s.tau_max(:,k)-U(:,k);end
end
s.b=zeros(nx,N);
end
