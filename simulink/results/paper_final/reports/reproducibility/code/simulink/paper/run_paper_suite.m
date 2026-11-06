function result=run_paper_suite(mode,stamp)
if nargin<1,mode='all';end
if nargin<2,stamp='20261105_paper_v1';end
init_paths;addpath(fileparts(mfilename('fullpath')));out=fullfile(repo_root,'simulink','results','paper_experiments',stamp);if ~isfolder(out),mkdir(out);end
if strcmp(mode,'report'),result=paper_report(out);paper_final_report(out);return;end
frozen=fullfile(out,'preflight.json');if isfile(frozen),latest=paper_preflight(fullfile(out,'preflight_checks',datestr(now,'yyyymmdd_HHMMSS')));p=jsondecode(fileread(frozen));for j=1:3,assert(strcmp(p.protected{j}.sha256,latest.protected{j}.sha256),'paper:provenance','Protected input differs from frozen experiment');end;else,p=paper_preflight(out);end;result=p;if strcmp(mode,'preflight'),return;end
assert(isempty(p.blockers),'paper:preflight','Model preflight blocked');
t=load('simulink/tests/05_python_matlab_qp_equivalence/production_online_qp.mat');base=fullfile(repo_root,'simulink','data','paper_banks','archive','S00_pitch30_B_bank.mat');
% Freeze the entire matrix before any run. PID settings are provisional until calibration.
s=struct('id','','experiment','E1','method','MPC','cmg',true,'profile','step','amplitude',30,'onset',8,'transition',5,'duration',60,'frequency',0,'dts',t.dts,'masked',false);
s.pid=struct('kp',[200;200;200],'ki',[1;1;1],'kd',[120;120;120],'imax',.2,'damping',.01,'unload',false,'hon',80,'hoff',60,'kh',.1,'qmin',1);
plan={};
for a=[10 20 30 40],for d=[2 5 10 20],for cmg=[false true],v=s;v.amplitude=a;v.transition=d;v.cmg=cmg;plan{end+1}=v;end,end,end
for profile={'step','sequence'},for method={'PID_no_unload','PID_reactive_unload','MPC'},v=s;v.experiment='E2';v.profile=profile{1};v.method=method{1};v.pid.unload=contains(v.method,'reactive');plan{end+1}=v;end,end
for H=[5 8 12 16 sum(t.dts)],v=s;v.experiment='E3';v.dts(2:end)=v.dts(2:end)*(H-.2)/sum(v.dts(2:end));plan{end+1}=v;end
v=s;v.experiment='E3';v.masked=true;plan{end+1}=v;
for hz=[.01 .02 .04 .06 .08 .1 .15 .2],for cmg=[false true],v=s;v.experiment='E4';v.profile='sine';v.amplitude=1;v.frequency=hz;v.duration=8+4/hz;v.cmg=cmg;plan{end+1}=v;end,end
for cmg=[false true],v=s;v.profile='coupled';v.cmg=cmg;plan{end+1}=v;end
if strcmp(mode,'smoke'),v=s;v.duration=2;v.amplitude=0;plan={v};v.cmg=false;plan{2}=v;end
for k=1:numel(plan),plan{k}.id=sprintf('P%03d%s',k,char('A'+plan{k}.cmg));end
paper_json(fullfile(out,['matrix_' mode '.json']),plan);
for k=1:numel(plan)
 v=plan{k};if ~ismember(mode,{'all','smoke'}) && ~strcmp(mode,v.experiment),continue;end
 folder=fullfile(out,v.experiment,v.id);if ~isfolder(folder),mkdir(folder);end
 if isfile(fullfile(folder,'run_status.json')),continue;end
 paper_json(fullfile(folder,'case_config.json'),v);fprintf('PAPER_CASE %d/%d %s\n',k,numel(plan),v.id);
 try
 if startsWith(v.method,'PID'),cfg=tune_pid(base,s,out);if v.pid.unload,assert(verify_pid_unload(base,s,cfg,out),'paper:unload','Physical unloading acceptance failed; method excluded');end;cfg.unload=v.pid.unload;v.pid=cfg;paper_json(fullfile(folder,'case_config.json'),v);end
 bank=make_paper_reference_bank(base,v,t.dts);path=fullfile(folder,'reference_bank.mat');save(path,'-struct','bank');m=build_paper_case(v,path,folder);
 % Re-evaluate analytic references on changed horizon, preserving true node zero.
 if max(abs(v.dts-t.dts))>1e-10
 bank=make_paper_reference_bank(base,v,v.dts);mw=get_param(m,'ModelWorkspace');w=getVariable(mw,'reference_window');for j=1:numel(bank.time),q=bank.q_ref(:,:,j);om=bank.omega_ref(:,:,j);w(j,2:1+numel(q))=q(:)';w(j,2+numel(q):1+numel(q)+numel(om))=om(:)';end;assignin(mw,'reference_window',w);save_system(m);
 end
 si=Simulink.SimulationInput(m);si=si.setModelParameter('CaptureErrors','on');wall=tic;r=sim(si);wall_s=toc(wall);save(fullfile(folder,'sim_output.mat'),'r','wall_s','-v7.3');metrics=evaluate_paper_runs(r,v,folder);st=struct('status',metrics.status,'error',metrics.error_message,'wall_s',wall_s);close_system(m,0);
 catch err,st=struct('status','INFRA_FAIL','error',getReport(err,'extended','hyperlinks','off'));fprintf(2,'%s\n',st.error);end
 paper_json(fullfile(folder,'run_status.json'),st);try,paper_report(out);catch report_error,paper_json(fullfile(out,'report_error.json'),struct('error',getReport(report_error)));end
end
for j=1:3,assert(strcmp(p.protected{j}.sha256,paper_hash(p.protected{j}.path)),'Protected input changed');end
if strcmp(mode,'all'),paper_finish(stamp);elseif ismember(mode,{'E2','E3','E4'}),run_paper_followup(out,mode);end
result=paper_report(out);if strcmp(mode,'all'),paper_final_report(out);end
end
function result=paper_report(out)
files=dir(fullfile(out,'E*','*','run_status.json'));rows={};for k=1:numel(files),st=jsondecode(fileread(fullfile(files(k).folder,files(k).name)));rows{end+1}=struct('path',files(k).folder,'status',st.status,'error',st.error);end
result=rows;paper_json(fullfile(out,'manifest.json'),rows);
f=fopen(fullfile(out,'四组论文实验结果.md'),'w','n','UTF-8');c=onCleanup(@()fclose(f));fprintf(f,'# 四组实验运行记录\n\n仅记录本次真实运行。PID 标定见 pid_calibration；卸载验收见 unload_acceptance。未通过验收的方法会标记失败，不得用于有效对照。全部论文验收尚未完成，不能预设 CMG 或 MPC 必然更优。\n\n');for k=1:numel(rows),fprintf(f,'- %s: %s\n',rows{k}.path,rows{k}.status);end
metrics=dir(fullfile(out,'E*','*','metrics.json'));entries={};for k=1:numel(metrics),e=jsondecode(fileread(fullfile(metrics(k).folder,metrics(k).name)));keep={'id','experiment','method','profile','cmg','amplitude','transition','frequency','horizon','masked','t_end','rmse_deg','peak_deg','terminal_deg','status','gain_db','phase_deg','steady_rmse_deg','compute_mean_s','compute_p95_s','compute_max_s','over_200ms_pct','fallbacks'};z=struct;for j=1:numel(keep),if isfield(e,keep{j}),value=e.(keep{j});elseif strcmp(keep{j},'profile'),value='unknown';else,value=NaN;end;if isempty(value) && ~ischar(value),value=NaN;end;z.(keep{j})=value;end;entries{end+1}=z;end
if ~isempty(entries),T=struct2table([entries{:}],'AsArray',true);writetable(T,fullfile(out,'metrics.csv'));plot_paper_figures(T,out);end
end
