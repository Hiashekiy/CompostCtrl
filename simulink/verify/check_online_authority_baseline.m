function check_online_authority_baseline(duration)
% Run saved L0 models without rebuilding or overwriting historical results.
if nargin<1,duration=60;end
root=repo_root;init_paths;
out=fullfile(root,'simulink','results','online_authority',['baseline_' datestr(now,'yyyymmdd_HHMMSS')]);
mkdir(out);bank=load(fullfile(root,'simulink','data','online_plan_bank.mat'));
report=struct;
for enabled=[false true]
 m='NearSpace_CMG_FlightControl';arm='B';
 if ~enabled,m=[m '_SurfaceOnly'];arm='A';end
 load_system(m);
 mw=get_param(m,'ModelWorkspace');assignin(mw,'native_results_dir',out);
 % Redirect S-function statistics as well as raw output to the new directory.
 si=Simulink.SimulationInput(m);si=si.setModelParameter('StopTime',num2str(duration));
 si=si.setVariable('native_results_dir',out,'Workspace',m);
 t0=tic;r=sim(si);elapsed=toc(t0);
 save(fullfile(out,[arm '_L0.mat']),'r','elapsed');
 data=squeeze(r.states.Data);if size(data,1)~=numel(r.states.Time),data=data';end
 q=eul2quat(data(:,[3 2 1]));
 ref=interp1(bank.time(:),squeeze(bank.q_ref(:,1,:))',r.states.Time);
 ref=ref./vecnorm(ref,2,2);error_deg=rad2deg(2*acos(min(1,abs(sum(q.*ref,2)))));
 report.(arm)=struct('simulation_s',duration,'elapsed_s',elapsed,'rms_deg',sqrt(mean(error_deg.^2)),'peak_deg',max(error_deg));
 stats=jsondecode(fileread(fullfile(out,[m '_runtime.json'])));
 report.(arm).fallbacks=stats.fallbacks;
 mw=get_param(m,'ModelWorkspace');template=getVariable(mw,'native_controller_template');
 lower=reshape(bank.tau_min,3,[]);upper=reshape(bank.tau_max,3,[]);
 report.(arm).tau_min_range=[min(lower,[],2) max(lower,[],2)];
 report.(arm).tau_max_range=[min(upper,[],2) max(upper,[],2)];
 historical=fullfile(root,'simulink','results','native_closedloop','FlightControl_AB_metrics_60s.csv');
 if isfile(historical),copyfile(historical,fullfile(out,'historical_metrics_60s.csv'));end
 close_system(m,0);
end
save(fullfile(out,'report.mat'),'report');
fid=fopen(fullfile(out,'report.json'),'w');fprintf(fid,'%s',jsonencode(report));fclose(fid);
disp(report);fprintf('BASELINE_REPORT %s\n',out);
end
