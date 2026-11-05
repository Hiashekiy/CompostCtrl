function [u,z,d]=pid_cmg_control_step(x,qref,wref,flight,c,z)
p=vehicle_params;if isempty(z),z=struct('I',zeros(3,1),'nu',zeros(4,1),'unload',false);end
q=paper_quat(x(1:3));sg=sign(qref'*q);if sg==0,sg=1;end;e=2*sg*(q(1)*qref(2:4)-qref(1)*q(2:4)-cross(q(2:4),qref(2:4)));
air=online_flight_condition(flight(1:3),flight(4:6));[nat,B]=online_aero_moment(p.vehicle,air,x(4:6),flight(7:9));[dl,du]=online_surface_reachability(p.vehicle,flight(7:9),.2,flight(10:12));lo=nat+max(B,0)*dl+min(B,0)*du;hi=nat+max(B,0)*du+min(B,0)*dl;
g=x(7:10);A=p.cmg.h0*(-p.s0.*sin(g')+p.gxs0.*cos(g'));H=sum(p.cmg.h0*(p.s0.*cos(g')+p.gxs0.*sin(g')),2);
target=c.kp.*e+c.ki.*z.I-c.kd.*(x(4:6)-wref);ta0=min(max(.7*target,lo),hi);req=target-ta0;D=-A'/(A*A'+c.damping*eye(3));lim=p.cmg.gdot_max;step=p.cmg.gddot_max*.2;lower=max(-lim,z.nu-step);upper=min(lim,z.nu+step);nu0=min(max(D*req,lower),upper);nu=nu0;ta=ta0;eta=0;unload_a=zeros(3,1);unload_c=zeros(3,1);
if norm(H)>c.hon,z.unload=true;elseif norm(H)<c.hoff,z.unload=false;end
if c.unload && air(1)>=c.qmin && z.unload
 tu=c.kh*max(0,norm(H)-c.hoff)*H/max(norm(H),eps);
 for scale=[1 .5 .25 .125 .0625 .03125]
 candidate=min(max(nu0+scale*D*tu,lower),upper);external=A*(candidate-nu0);newta=ta0+external;
 if all(newta>=lo-1e-10 & newta<=hi+1e-10) && H'*external<0
 nu=candidate;ta=newta;eta=scale;unload_a=external;unload_c=-external;break;
 end
 end
end
delivered=ta-A*nu;active=abs(target-delivered)<.01 | e.*(target-delivered)<=0;z.I=min(max(z.I+.2*e.*active,-c.imax),c.imax);z.nu=nu;z.H=H;z.eta=eta;z.unload_a=unload_a;z.unload_c=unload_c;z.residual=target-delivered;z.qbar=air(1);z.low_qbar=air(1)<c.qmin;z.target=target;u=[ta;nu];d=[lo;hi;double(c.unload && z.unload);norm(target-delivered)];
end
