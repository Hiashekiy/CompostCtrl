function run_paper_baselines(out)
t=load('simulink/tests/05_python_matlab_qp_equivalence/production_online_qp.mat');tasks={'S00_pitch30','S06_pitch30_roll20'};
for j=1:2,for cmg=[false true]
s=struct('id',sprintf('BASE%d%s',j,char('A'+cmg)),'experiment','E1_baseline','method','MPC','cmg',cmg,'profile','archive','amplitude',30,'onset',20,'transition',20,'duration',60,'frequency',0,'dts',t.dts,'masked',false,'archived_task',tasks{j});folder=fullfile(out,s.experiment,s.id);if ~isfolder(folder),mkdir(folder);end;if isfile(fullfile(folder,'run_status.json')),continue;end
paper_json(fullfile(folder,'case_config.json'),s);
try
bank=load(fullfile(repo_root,'simulink','data','paper_banks','archive',[tasks{j} '_B_bank.mat']));path=fullfile(folder,'reference_bank.mat');save(path,'-struct','bank');m=build_paper_case(s,path,folder);si=Simulink.SimulationInput(m);si=si.setModelParameter('CaptureErrors','on');wall=tic;r=sim(si);wall_s=toc(wall);save(fullfile(folder,'sim_output.mat'),'r','wall_s','-v7.3');e=evaluate_paper_runs(r,s,folder);st=struct('status',e.status,'error',e.error_message,'wall_s',wall_s);close_system(m,0);
catch err,st=struct('status','INFRA_FAIL','error',getReport(err));end
paper_json(fullfile(folder,'run_status.json'),st);
end,end
end

