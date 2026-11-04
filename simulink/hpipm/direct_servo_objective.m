function [J,terms]=direct_servo_objective(s,X,U,previous_command,nu_previous)
% The single complete nonlinear objective of the direct-servo MPC. The controller's
% acceptance test, the QP's linear term and every diagnostic call this one function, so
% "the QP decreased its objective" and "the accepted trajectory decreased the cost" can
% no longer refer to different expressions.
%
%           J = sum_k w_k [ w_att/e_s^2 |e_k|^2 + w_rate/omega_s^2 |omega_k-w_ref|^2
%                           + lambda_c |D^-1 u_k|^2 + lambda_dc |D^-1 (u_k-u_{k-1})|^2 ]
%               + q_T/e_s^2 |e_N|^2
%               + (B arm only) sum_k w_k [ r_nu/nu_s^2 |nu_k|^2 + r_dnu/nu_s^2 |nu_k-nu_{k-1}|^2 ]
% with u_0 = the command actually executed before this control period and nu_0 the last
% executed CMG rate command. D = diag(delta_max).
persistent v
if isempty(v),p=vehicle_params;v=p.vehicle;end
N=double(s.N);nx=double(s.nx_direct);nu=double(s.nu_direct);natt=nx-3;W=s.weights;Wc=s.direct_weights;
weight=s.dts(:)/sum(s.dts);D=v.delta_max(:);
terms=struct('att',0,'rate',0,'terminal',0,'cmd',0,'dcmd',0,'nu',0,'dnu',0);
for j=1:N+1
 if natt==11,[e,~]=native_E_B(s.q_ref(:,j),X(1:11,j));else,[e,~]=native_E_A(s.q_ref(:,j),X(1:7,j));end
 if j<=N
  dw=X(5:7,j)-s.omega_ref(:,j);
  terms.att=terms.att+weight(j)*(W.w_att/W.e_scale^2*(e'*e));
  terms.rate=terms.rate+weight(j)*(W.w_rate/W.omega_scale^2*(dw'*dw));
 else
  terms.terminal=terms.terminal+W.q_terminal/W.e_scale^2*(e'*e);
 end
end
for k=1:N
 terms.cmd=terms.cmd+weight(k)*sum(Wc.cmd*(U(1:3,k)').^2./D'.^2);
 if k==1,previous=previous_command(:);else,previous=U(1:3,k-1);end
 terms.dcmd=terms.dcmd+weight(k)*sum(Wc.dcmd*((U(1:3,k)-previous)').^2./D'.^2);
 if nu==7
  terms.nu=terms.nu+weight(k)*sum(W.r_nu*(U(4:7,k)').^2/W.nu_scale^2);
  if k==1,previous_nu=nu_previous(:);else,previous_nu=U(4:7,k-1);end
  terms.dnu=terms.dnu+weight(k)*sum(W.r_dnu*((U(4:7,k)-previous_nu)').^2/W.nu_scale^2);
 end
end
J=terms.att+terms.rate+terms.terminal+terms.cmd+terms.dcmd+terms.nu+terms.dnu;
end
