function [A,B]=direct_servo_linearize(s,X,U,a,tau_d)
% Discrete Jacobians of the direct-servo model at one nominal trajectory.
% The attitude/CMG block is the existing validated native_A_*/native_B_* discretisation,
% linearised with the AERODYNAMIC moment (natural + B_delta*delta) as its input, and the
% servo block is appended analytically, so the servo state really is an optimised state:
%   delta+ = (1-g)*delta + g*delta_cmd,  g = 1-exp(-dt/tau_s)
%   tau_a  = tau_nat + B_delta*delta
%   A_new  = [[A_old, B_old(:,tau)*B_delta*(1-g)]; [0, (1-g)]]
%   B_new  = [[B_old(:,tau)*B_delta*g, B_old(:,nu)]; [g, 0]]
% The chain rule through B_delta is what makes this a 14/10-dim model rather than the old
% 11/7-dim matrices reused with a padded state; check_direct_servo_mpc.m verifies the result
% against finite differences of the nonlinear map.
persistent v
if isempty(v),p=vehicle_params;v=p.vehicle;end
N=double(s.N);nx=double(s.nx_direct);nu=double(s.nu_direct);natt=nx-3;
legacy=isfield(s,'model_mode')&&strcmp(s.model_mode,'legacy');
A=zeros(nx,nx,N);B=zeros(nx,nu,N);
for k=1:N
 dt=s.dts(k);
 if legacy
  % bcd96fd behaviour: the servo COMMAND is passed where the torque-level MEX expects a
  % moment. Kept only to reproduce the reviewed experiment, never as the fixed model.
  working=U(:,k);
 else
  % Working point in the units the attitude MEX actually linearises about: the aerodynamic
  % moment produced by the nominal servo angle, plus the nominal CMG rate command.
  if natt==11
   working=[a.torque(:,k);U(4:7,k)];
  else
   working=a.torque(:,k);
  end
 end
 if natt==11
  A0=native_A_B(X(1:11,k),working,tau_d(:,k),dt);
  B0=native_B_B(X(1:11,k),working,tau_d(:,k),dt);
  B0_tau=B0(:,1:3);B0_nu=B0(:,4:7);
 else
  A0=native_A_A(X(1:7,k),working,tau_d(:,k),dt);
  B0=native_B_A(X(1:7,k),working,tau_d(:,k),dt);
  B0_tau=B0(:,1:3);B0_nu=zeros(7,0);
 end
 gain=1-exp(-dt/v.tau_s);g=diag(min(max(gain,0),1)*ones(3,1));
 Bd=a.B_delta(:,:,k);
 A(1:natt,1:natt,k)=A0;
 A(1:natt,natt+1:nx,k)=B0_tau*Bd*(eye(3)-g);
 A(natt+1:nx,1:natt,k)=0;
 A(natt+1:nx,natt+1:nx,k)=eye(3)-g;
 B(1:natt,1:3,k)=B0_tau*Bd*g;
 B(natt+1:nx,1:3,k)=g;
 if natt==11,B(1:11,4:7,k)=B0_nu;end
end
end
