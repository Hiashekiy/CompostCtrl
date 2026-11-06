function run_dt01_sqp4
% Sequential A/B runs avoid introducing concurrent-controller timing load.
root=repo_root;init_paths;
bank=load(fullfile(root,'simulink','data','online_plan_bank.mat'));
s=load(fullfile(root,'simulink','tests','05_python_matlab_qp_equivalence','production_online_qp.mat'));
dts=s.dts;dts(1)=.1;new=online_resample_reference_bank(bank,s.dts,dts,.1);
assert(all(abs(diff(new.time)-.1)<1e-10));
assert(max(abs(new.q_ref(:,1,2)-(bank.q_ref(:,1,1)+bank.q_ref(:,1,2))/norm(bank.q_ref(:,1,1)+bank.q_ref(:,1,2))))<1e-12);
folders={'L2_60s_20261104_203114','L2_60s_20261104_203147'};
for enabled=[false true]
 folder=build_closedloop(60,true,enabled,'','online_predicted',true,4,.1);
 [~,name]=fileparts(folder);folders{end+1}=name;
end
report_dt01_sqp4(folders);
fprintf('DT01_SQP4_PASS\n');
end
