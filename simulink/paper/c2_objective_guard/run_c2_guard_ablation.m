function run_c2_guard_ablation
% Exactly one new C2 simulation. The reference result is read from paper_final.
init_paths;root=repo_root;addpath(fullfile(root,'simulink/paper'),fullfile(root,'simulink/paper/v3'));
out=fullfile(root,'simulink/results/paper_experiments/C2_objective_guard_ablation');
if ~isfolder(out),mkdir(out);end
assert(~isfile(fullfile(out,'simulation_invocation.json')),'c2guard:once','Simulation already invoked; analysis may be rerun, simulation may not');
for name={'raw','config','models','implementation','checks','tables','figures','reports'},if ~isfolder(fullfile(out,name{1})),mkdir(fullfile(out,name{1}));end;end
base=fullfile(root,'simulink/results/paper_final/E2_Controller_Comparison');sc=fullfile(base,'scenarios/V2_C2_3');raw=fullfile(base,'raw/V2_C2_3');
cfg=jsondecode(fileread(fullfile(sc,'case_config.json')));build=jsondecode(fileread(fullfile(sc,'build_config.json')));
before=struct('model_sha256',paper_hash(fullfile(sc,'model.slx')),'bank_sha256',paper_hash(fullfile(sc,'reference_bank.mat')),'raw_sha256',paper_hash(fullfile(raw,'sim_output.mat')));
assert(strcmp(before.model_sha256,build.model_sha256)&&strcmp(before.bank_sha256,build.bank_sha256));
assert(cfg.duration==60&&isequal(cfg.events(:,1)',[8 20 32])&&all(cfg.events(:,2)==4));
copyfile(fullfile(sc,'reference_bank.mat'),fullfile(out,'config/reference_bank.mat'));
copyfile(fullfile(sc,'case_config.json'),fullfile(out,'config/baseline_case_config.json'));
copyfile(fullfile(sc,'model.slx'),fullfile(out,'implementation/baseline_model.slx'));
original=fullfile(root,'simulink/cases/paper_v3/NearSpace_CMG_FlightControl_V2_C2_3.slx');assert(strcmp(paper_hash(original),before.model_sha256));
m='NearSpace_CMG_FlightControl_V2_C2_3';load_system(original);mw=get_param(m,'ModelWorkspace');template_before=getVariable(mw,'native_controller_template');
assert(template_before.max_sqp_iterations==4&&template_before.dt_ctrl==.2&&isequal(template_before.dts(:),cfg.dts(:)));
assert(~isfield(template_before,'enforce_objective_decrease')||template_before.enforce_objective_decrease);
bank=load(fullfile(out,'config/reference_bank.mat'));embedded_bank=getVariable(mw,'native_reference_bank');
for f={'q_ref','omega_ref','tau_d'},assert(max(abs(embedded_bank.(f{1})-bank.(f{1})),[],'all')<1e-10);end
save(fullfile(out,'config/embedded_reference_bank.mat'),'-struct','embedded_bank');
source=fullfile(root,'simulink/results/paper_final/reports/reproducibility/code/simulink/hpipm/native_online_rti_step.m');
text=fileread(source);text=strrep(text,'function [command,state,info]=native_online_rti_step','function [command,state,info]=baseline_native_online_rti_step');
fid=fopen(fullfile(out,'checks/baseline_native_online_rti_step.m'),'w','n','UTF-8');fwrite(fid,text,'char');fclose(fid);addpath(fullfile(out,'checks'));
% C2-only cold-start function checks, not Simulink runs or new trajectories.
d=load(fullfile(raw,'sim_output.mat'),'r');r=d.r;xx=squeeze(r.states.Data);qq=paper_euler_quaternions(xx(1,1:3));x=[qq';xx(1,4:10)'];
s=template_before;for f={'q_ref','omega_ref','tau_d','tau_min','tau_max','tau_nom','gap'},value=embedded_bank.(f{1});s.(f{1})=value(:,:,1);end
ff=squeeze(r.flight_servo_measured.Data);s.flight_state=ff(1,:)';state=struct('U',[],'last',[],'nu_previous',zeros(4,1));
[c0,st0,i0]=baseline_native_online_rti_step(s,x,state,false);[c1,st1,i1]=native_online_rti_step(s,x,state,false);
s.enforce_objective_decrease=true;[c2,st2,i2]=native_online_rti_step(s,x,state,false);
assert(~i0.fallback&&~i1.fallback&&~i2.fallback);assert(isequaln(c0,c1)&&isequaln(c1,c2)&&isequaln(st0,st1)&&isequaln(st1,st2));
for field={'accepted_steps','rejected_steps','status','iterations','early_stop_reason','objective'},assert(isequaln(i0.(field{1}),i1.(field{1}))&&isequaln(i1.(field{1}),i2.(field{1})));end
assert(i1.enforce_objective_decrease&&i2.enforce_objective_decrease);
paper_json(fullfile(out,'checks/default_guard_regression.json'),struct('passed',true,'scenario','C2 initial control tick','default_and_explicit_true_match_archived_controller',true,'new_simulations',0));
clear d r xx ff;
template_after=template_before;template_after.enforce_objective_decrease=false;
assignin(mw,'native_controller_template',template_after);assignin(mw,'native_results_dir',fullfile(out,'raw'));
assert(isequaln(rmfield(template_after,'enforce_objective_decrease'),template_before));
save(fullfile(out,'config/controller_templates.mat'),'template_before','template_after');
cfg.id='C2_ObjectiveGuard_OFF';cfg.enforce_objective_decrease=false;cfg.max_sqp_iterations=4;paper_json(fullfile(out,'config/case_config.json'),cfg);
target=fullfile(out,'models/NearSpace_CMG_FlightControl_C2_OG_OFF.slx');save_system(m,target);close_system('all',0);m='NearSpace_CMG_FlightControl_C2_OG_OFF';load_system(target);
mw=get_param(m,'ModelWorkspace');assert(isequaln(getVariable(mw,'native_controller_template'),template_after));assert(isequaln(getVariable(mw,'native_reference_bank'),embedded_bank));
assert(strcmp(get_param(m,'FixedStep'),'.001')&&str2double(get_param(m,'StopTime'))==60);
Simulink.fileGenControl('set','CacheFolder',fullfile(out,'cache'),'CodeGenFolder',fullfile(out,'codegen'),'createDir',true);
set_param(m,'SimulationCommand','update');
manifest=struct('baseline',before,'baseline_case','V2_C2_3','model_sha256',paper_hash(target),'bank_sha256',paper_hash(fullfile(out,'config/reference_bank.mat')),'native_online_rti_step_sha256',paper_hash(which('native_online_rti_step')),'template_only_change','enforce_objective_decrease=false','max_sqp_iterations',4,'stagnation_stop_unchanged',true,'baseline_simulations_added',0);
paper_json(fullfile(out,'config/provenance.json'),manifest);copyfile(which('native_online_rti_step'),fullfile(out,'implementation/native_online_rti_step.m'));
paper_json(fullfile(out,'simulation_invocation.json'),struct('invocations',1,'requested_end_s',60,'model',m,'enforce_objective_decrease',false));
fprintf('C2_GUARD_SINGLE_SIM_BEGIN\n');si=Simulink.SimulationInput(m);si=si.setModelParameter('CaptureErrors','on');clock=tic;
try
 r=sim(si);wall_s=toc(clock);save(fullfile(out,'raw/sim_output.mat'),'r','wall_s','-v7.3');
 paper_json(fullfile(out,'simulation_receipt.json'),struct('simulations',1,'wall_s',wall_s,'error_message',r.ErrorMessage,'actual_end_s',r.tout(end),'complete',r.tout(end)>=59.999));
catch err
 paper_json(fullfile(out,'simulation_receipt.json'),struct('simulations',1,'wall_s',toc(clock),'complete',false,'error_message',getReport(err,'extended','hyperlinks','off')));close_system(m,0);rethrow(err);
end
close_system(m,0);
assert(strcmp(paper_hash(original),before.model_sha256));assert(strcmp(paper_hash(fullfile(sc,'model.slx')),before.model_sha256));assert(strcmp(paper_hash(fullfile(raw,'sim_output.mat')),before.raw_sha256));
fprintf('C2_GUARD_SINGLE_SIM_COMPLETE end=%g wall=%g\n',r.tout(end),wall_s);
end
