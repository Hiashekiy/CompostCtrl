function [natural,B,total]=online_aero_moment(v,air,omega,delta)
% Same SI convention as build_actuators_aero: air=[qbar;V;alpha;beta].
% delta=[elevator;aileron;rudder], moments=[roll;pitch;yaw].
air=air(:);omega=omega(:);delta=delta(:);
assert(numel(air)==4 && numel(omega)==3 && numel(delta)==3);
assert(all(isfinite([air;omega;delta])) && air(1)>=0 && air(2)>=0);
D=[v.Cl_p*v.b/2 0 v.Cl_r*v.b/2;0 v.Cm_q*v.cbar/2 0;v.Cn_p*v.b/2 0 v.Cn_r*v.b/2];
C=[0 v.Cl_da v.Cl_dr;v.Cm_de 0 0;0 v.Cn_da v.Cn_dr];
G=diag(v.S*[v.b v.cbar v.b]);
natural=air(1)*G*([v.Cl_beta*air(4);v.Cm0+v.Cm_alpha*air(3);v.Cn_beta*air(4)]+D*omega/max(air(2),1));
B=air(1)*G*C;
total=natural+B*delta;
end
