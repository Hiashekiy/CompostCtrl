function ok=verify_pid_unload(base,s,cfg,out)
folder=fullfile(out,'unload_acceptance');if ~isfolder(folder),mkdir(folder);end
marker=fullfile(folder,'acceptance_v2.json');if isfile(marker),d=jsondecode(fileread(marker));ok=d.passed;return;end
p=vehicle_params;g=[pi/2;pi/2;0;0];records={};
for trial=1:3
 sub=fullfile(folder,sprintf('trial_%d',trial));if ~isfolder(sub),mkdir(sub);end
 c=cfg;c.unload=trial~=1;if trial==3,c.qmin=inf;end
 v=s;v.id=sprintf('UNLOAD%d',trial);v.duration=15;v.amplitude=0;v.method='PID_reactive_unload';v.pid=c;
 try
 bank=make_paper_reference_bank(base,v,v.dts);bank.x0(8:11)=g;path=fullfile(sub,'reference_bank.mat');save(path,'-struct','bank');m=build_paper_case(v,path,sub);si=Simulink.SimulationInput(m);si=si.setModelParameter('CaptureErrors','on');r=sim(si);save(fullfile(sub,'sim_output.mat'),'r');x=squeeze(r.states.Data);H=zeros(size(x,1),3);for k=1:size(x,1),H(k,:)=sum(p.cmg.h0*(p.s0.*cos(x(k,7:10))+p.gxs0.*sin(x(k,7:10))),2)';end
 d=load(fullfile(sub,'pid_diagnostics.mat'));eta=cellfun(@(z)z.eta,d.history);comp=cellfun(@(z)norm(z.unload_a+z.unload_c),d.history);rec=struct('initial_H',norm(H(1,:)),'final_H',norm(H(end,:)),'error',r.ErrorMessage,'active_ticks',sum(eta>0),'compensation_residual',max(comp));close_system(m,0);
 catch err,rec=struct('error',getReport(err),'initial_H',NaN,'final_H',NaN,'active_ticks',0,'compensation_residual',inf);end
 records{trial}=rec;
end
a=records{2};ok=all(cellfun(@(z)isempty(z.error),records)) && a.final_H<.95*a.initial_H && a.final_H<records{1}.final_H && a.active_ticks>0 && a.compensation_residual<1e-8 && records{1}.active_ticks==0 && records{3}.active_ticks==0;
paper_json(marker,struct('passed',ok,'trials',{records},'gate_test','trial3 raises controller qbar gate to infinity; physical model unchanged'));
end
