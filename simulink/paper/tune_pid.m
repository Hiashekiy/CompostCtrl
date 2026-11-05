function cfg=tune_pid(base,s,rootout)
folder=fullfile(rootout,'pid_calibration');if ~isfolder(folder),mkdir(folder);end
frozen=fullfile(folder,'pid_tuned.mat');if isfile(frozen),d=load(frozen);cfg=d.cfg;return;end
p=vehicle_params;J=diag(p.J);best=inf;cfg=[];records={};
for k=1:4
 c=s.pid;wn=[.2 .35 .5 .75];c.kp=J*wn(k)^2;c.kd=2*.85*wn(k)*J;c.ki=c.kp*.02;
 scores=[];
 for a=[5 12]
 v=s;v.id=sprintf('TC%d_%d',k,a);v.amplitude=a;v.transition=8;v.duration=35;v.method='PID_no_unload';v.experiment='calibration';v.pid=c;
 sub=fullfile(folder,v.id);if ~isfolder(sub),mkdir(sub);end
 try
 bank=make_paper_reference_bank(base,v,v.dts);path=fullfile(sub,'reference_bank.mat');save(path,'-struct','bank');m=build_paper_case(v,path,sub);si=Simulink.SimulationInput(m);si=si.setModelParameter('CaptureErrors','on');r=sim(si);save(fullfile(sub,'sim_output.mat'),'r');e=evaluate_paper_runs(r,v,sub);close_system(m,0);score=e.rmse_deg;if ~isempty(e.error_message),score=inf;end
 catch err,score=inf;paper_json(fullfile(sub,'failure.json'),struct('error',getReport(err)));end
 scores(end+1)=score;
 end
 value=mean(scores);records{end+1}=struct('candidate',k,'score',value,'config',c);if value<best,best=value;cfg=c;end
end
paper_json(fullfile(folder,'tuning.json'),records);assert(~isempty(cfg),'paper:tuning','No valid independent PID calibration');save(frozen,'cfg','best');paper_json(fullfile(folder,'pid_tuned.json'),cfg);
end
