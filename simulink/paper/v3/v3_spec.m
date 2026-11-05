function s=v3_spec
p=load('simulink/tests/05_python_matlab_qp_equivalence/production_online_qp.mat');s=struct('id','','group','','pair_id','','method','MPC','cmg',true,'profile','step','amplitude',10,'onset',8,'transition',5,'duration',60,'frequency',0,'dts',p.dts,'events',zeros(0,5),'gamma0',[],'pid',v3_pid_config(.7,1),'masked',false,'phase','formal');
end
