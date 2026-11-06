function test_paper_core
p=vehicle_params;f=[0;0;-20000;100;0;0;zeros(6,1)];c=struct('kp',[200;200;200],'ki',[1;1;1],'kd',[120;120;120],'imax',.2,'damping',.01,'unload',false,'hon',80,'hoff',60,'kh',.1,'qmin',1);x=zeros(10,1);
for signum=[-1 1]
 [u,z,d]=pid_cmg_control_step(x,paper_quat([0;signum*.01;0]),zeros(3,1),f,c,[]);A=p.cmg.h0*p.gxs0;assert(signum*(u(2)-A(2,:)*u(4:7))>0);assert(all(abs(u(4:7))<=p.cmg.gddot_max*.2+eps));assert(all(isfinite([u;d;z.I])));
end
sat=c;sat.kp=ones(3,1)*1e9;z=[];for k=1:20,[u,z]=pid_cmg_control_step(x,paper_quat([0;.4;0]),zeros(3,1),f,sat,z);end;assert(abs(z.I(2))<1e-12,'Conditional integration must stop under same-direction saturation');
x(7:10)=[pi/2;pi/2;0;0];gate=c;gate.unload=true;gate.qmin=inf;[~,z]=pid_cmg_control_step(x,paper_quat(x(1:3)),zeros(3,1),f,gate,[]);assert(z.eta==0 && norm(z.unload_a)==0);
template=load('simulink/tests/05_python_matlab_qp_equivalence/production_online_qp.mat');for H=[5 8 12 16 sum(template.dts)],dts=make_paper_grid(template.dts,H);assert(numel(dts)==double(template.N));end
fprintf('PAPER_CORE_TEST_PASS\n');
end


