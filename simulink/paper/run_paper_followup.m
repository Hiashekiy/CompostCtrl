function run_paper_followup(out,group)
if nargin<2,group='all';end
% A predeclared common valid-amplitude follow-up; retain every original 30-deg run.
t=load('simulink/tests/05_python_matlab_qp_equivalence/production_online_qp.mat');base=fullfile(repo_root,'simulink','data','paper_banks','archive','S00_pitch30_B_bank.mat');
s=struct('id','','experiment','E2_followup','method','MPC','cmg',true,'profile','step','amplitude',10,'onset',8,'transition',5,'duration',60,'frequency',0,'dts',t.dts,'masked',false);s.pid=struct('kp',[200;200;200],'ki',[1;1;1],'kd',[120;120;120],'imax',.2,'damping',.01,'unload',false,'hon',80,'hoff',60,'kh',.1,'qmin',1);
plan={};for profile={'step','sequence'},for method={'PID_no_unload','PID_reactive_unload','MPC'},v=s;v.profile=profile{1};v.method=method{1};v.pid.unload=contains(v.method,'reactive');plan{end+1}=v;end,end
for H=[5 8 12 16 sum(t.dts)],v=s;v.experiment='E3_followup';v.dts=make_paper_grid(t.dts,H);plan{end+1}=v;end
v=s;v.experiment='E3_followup';v.masked=true;plan{end+1}=v;
for hz=[.01 .02 .04 .06 .08 .1 .15 .2],for cmg=[false true],v=s;v.experiment='E4_followup';v.profile='sine';v.amplitude=1;v.frequency=hz;v.duration=8+4/hz;v.cmg=cmg;v.extend_disturbance=true;plan{end+1}=v;end,end
for k=1:numel(plan),plan{k}.id=sprintf('F%03d%s',k,char('A'+plan{k}.cmg));end
paper_json(fullfile(out,'matrix_followup.json'),struct('reason','Original E1 30deg CMG cases hit alpha domain. Original shortened horizon grids were not aligned to the 1ms servo clock. Add 10deg E2/E3 tasks with 1ms-quantized horizons keeping six initial .2s stages; preserve original failures.','runs',{plan}));
for k=1:numel(plan)
v=plan{k};if ~strcmp(group,'all') && ~startsWith(v.experiment,group),continue;end;folder=fullfile(out,v.experiment,v.id);if ~isfolder(folder),mkdir(folder);end;if isfile(fullfile(folder,'run_status.json')),continue;end
try
if startsWith(v.method,'PID'),cfg=tune_pid(base,s,out);if v.pid.unload,assert(verify_pid_unload(base,s,cfg,out),'paper:unload','Physical unloading acceptance failed');end;cfg.unload=v.pid.unload;v.pid=cfg;end
paper_json(fullfile(folder,'case_config.json'),v);bank=make_paper_reference_bank(base,v,v.dts);path=fullfile(folder,'reference_bank.mat');save(path,'-struct','bank');m=build_paper_case(v,path,folder);si=Simulink.SimulationInput(m);si=si.setModelParameter('CaptureErrors','on');wall=tic;r=sim(si);wall_s=toc(wall);save(fullfile(folder,'sim_output.mat'),'r','wall_s','-v7.3');e=evaluate_paper_runs(r,v,folder);st=struct('status',e.status,'error',e.error_message,'wall_s',wall_s);close_system(m,0);
catch err,st=struct('status','INFRA_FAIL','error',getReport(err));end
paper_json(fullfile(folder,'run_status.json'),st);
end
end





