function bank=make_paper_reference_bank(base,s,dts)
bank=load(base);old=bank;N=numel(dts);bank.time=(0:.2:s.duration)';K=numel(bank.time);q0=old.q_ref(:,1,1);nodes=[0;cumsum(dts(:))];
for f={'q_ref','omega_ref','tau_d','tau_min','tau_max','tau_nom'},bank.(f{1})=zeros(size(old.(f{1}),1),N+1,K);end
bank.gap=zeros(3,1,K);
extend=isfield(s,'extend_disturbance') && s.extend_disturbance;source=squeeze(old.tau_d(:,1,:))';assert(max(abs(source-source(1,:)),[],'all')<1e-10 || s.duration<=old.time(end) || extend,'paper:disturbance','Cannot extend nonconstant disturbance beyond source support');
for k=1:K
 for j=1:N+1
 t=bank.time(k)+nodes(j);bank.q_ref(:,j,k)=att(t);h=1e-4;qa=att(t-h);qb=att(t+h);qd=(qb-qa)/(2*h);q=bank.q_ref(:,j,k);bank.omega_ref(:,j,k)=2*(q(1)*qd(2:4)-qd(1)*q(2:4)-cross(q(2:4),qd(2:4)));
 if extend && t>old.time(end),[bank.tau_d(:,j,k),bank.disturbance_validation]=paper_extended_disturbance(old,t);else
 bank.tau_d(:,j,k)=interp1(old.time,source,min(max(t,old.time(1)),old.time(end)),'linear')';end
 end
end
bank.paper_spec=s;bank.paper_dts=dts;bank.paper_note='tau_nom/gap zero; online authority recomputes bounds; disturbance inherited from common S00 source';assert(max(abs(vecnorm(bank.q_ref,2,1)-1),[],'all')<1e-10);
function q=att(t)
a=zeros(3,1);if strcmp(s.profile,'sine'),a(2)=s.amplitude*sin(2*pi*s.frequency*max(0,t-s.onset));if t<s.onset,a(2)=0;end
else
 z=min(1,max(0,(t-s.onset)/s.transition));a(2)=s.amplitude*(10*z^3-15*z^4+6*z^5);
 if strcmp(s.profile,'coupled'),a(1)=20*(10*z^3-15*z^4+6*z^5);end
 if strcmp(s.profile,'sequence'),z=min(1,max(0,(t-25)/5));a(2)=a(2)-s.amplitude*(10*z^3-15*z^4+6*z^5);z=min(1,max(0,(t-40)/5));a(1)=15*(10*z^3-15*z^4+6*z^5);end
end
r=paper_quat(deg2rad(a));q=[q0(1)*r(1)-dot(q0(2:4),r(2:4));q0(1)*r(2:4)+r(1)*q0(2:4)+cross(q0(2:4),r(2:4))];q=q/norm(q);
end
end


