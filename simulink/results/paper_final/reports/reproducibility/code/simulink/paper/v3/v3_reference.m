function bank=v3_reference(s,dts)
base=fullfile(repo_root,'simulink','data','paper_banks','archive','S00_pitch30_B_bank.mat');old=load(base);times=(0:.2:s.duration)';nodes=[0;cumsum(dts(:))];T=nodes+times';tt=T(:);q0=old.q_ref(:,1,1);[~,e0]=paper_reference_series(old,0);[e,edot]=profile(tt);e=e+deg2rad(e0);Q=paper_euler_quaternions(e);omega=[edot(:,1)-edot(:,3).*sin(e(:,2)),edot(:,2).*cos(e(:,1))+edot(:,3).*sin(e(:,1)).*cos(e(:,2)),-edot(:,2).*sin(e(:,1))+edot(:,3).*cos(e(:,1)).*cos(e(:,2))];
source=squeeze(old.tau_d(:,1,:))';td=interp1(old.time,source,min(tt,old.time(end)),'linear');for k=find(tt>old.time(end))',[tau,validation]=paper_extended_disturbance(old,tt(k));td(k,:)=tau';end
N=numel(dts);K=numel(times);bank=old;bank.time=times;bank.q_ref=reshape(Q',4,N+1,K);bank.omega_ref=reshape(omega',3,N+1,K);bank.tau_d=reshape(td',3,N+1,K);for f={'tau_min','tau_max','tau_nom'},bank.(f{1})=zeros(3,N+1,K);end;bank.gap=zeros(3,1,K);if isfield(s,'gamma0') && ~isempty(s.gamma0),bank.x0(8:11)=s.gamma0(:);end
bank.metadata=struct('case_id',s.id,'physical_base','archived_S00','prediction_dts',dts,'task_onset',s.onset,'seed',0,'reference_profile',s.profile,'disturbance_rule','native validated analytic continuation after60s for all cases/horizons');if exist('validation','var'),bank.metadata.disturbance_validation=validation;end
% Independent quaternion derivative/body-rate identity, evaluated away from start discontinuities.
sel=1:max(1,floor(numel(tt)/100)):numel(tt);h=1e-5;[ep,~]=profile(tt(sel)+h);[em,~]=profile(tt(sel)-h);qp=paper_euler_quaternions(ep+deg2rad(e0));qm=paper_euler_quaternions(em+deg2rad(e0));qd=(qp-qm)/(2*h);q=Q(sel,:);w=2*(q(:,1).*qd(:,2:4)-qd(:,1).*q(:,2:4)-cross(q(:,2:4),qd(:,2:4),2));valid=abs(tt(sel)-s.onset)>.001;assert(max(abs(w(valid,:)-omega(sel(valid),:)),[],'all')<1e-4);assert(max(abs(vecnorm(Q,2,2)-1))<1e-12);
function [e,de]=profile(t)
e=zeros(numel(t),3);de=e;
if strcmp(s.profile,'sine'),active=t>=s.onset;e(active,2)=s.amplitude*sin(2*pi*s.frequency*(t(active)-s.onset));de(active,2)=s.amplitude*2*pi*s.frequency*cos(2*pi*s.frequency*(t(active)-s.onset));
else
if isfield(s,'events') && ~isempty(s.events),events=s.events;else,events=[s.onset,s.transition,0,s.amplitude,0];end
for j=1:size(events,1),a=min(1,max(0,(t-events(j,1))/events(j,2)));z=10*a.^3-15*a.^4+6*a.^5;dz=(30*a.^2-60*a.^3+30*a.^4)/events(j,2);e=e+z*events(j,3:5);de=de+dz*events(j,3:5);end
end;e=deg2rad(e);de=deg2rad(de);
end
end
