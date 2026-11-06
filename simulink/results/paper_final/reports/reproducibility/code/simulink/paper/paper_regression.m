function paper_regression(out)
try,run_paper_baselines(out);catch err,paper_json(fullfile(out,'baseline_error.json'),struct('error',getReport(err)));end
folder=fullfile(out,'regression');if ~isfolder(folder),mkdir(folder);end
path='simulink/models/NearSpace_CMG_FlightControl.slx';before=paper_hash(path);load_system(path);m='NearSpace_CMG_FlightControl';mw=get_param(m,'ModelWorkspace');bank=getVariable(mw,'native_reference_bank');
for duration=[2 60]
sub=fullfile(folder,sprintf('%gs',duration));if ~isfolder(sub),mkdir(sub);end
si=Simulink.SimulationInput(m);si=si.setModelParameter('StopTime',num2str(duration),'CaptureErrors','on');si=si.setVariable('native_results_dir',sub,'Workspace',m);if isfile(fullfile(sub,'sim_output.mat')),saved=load(fullfile(sub,'sim_output.mat'));r=saved.r;else,r=sim(si);save(fullfile(sub,'sim_output.mat'),'r','-v7.3');end;times=r.states.Time(:);x=squeeze(r.states.Data);[qr,~]=paper_reference_series(bank,times);q=paper_euler_quaternions(x(:,1:3));angles=rad2deg(2*acos(min(1,abs(sum(q.*qr,2)))));measured=sqrt(mean(angles(times<=50).^2));paper_json(fullfile(sub,'status.json'),struct('error',r.ErrorMessage,'duration',duration,'window_rmse_deg',measured,'historical_60s_window_rmse_deg',.3092,'within_005deg',duration==60 && abs(measured-.3092)<.005));
end
close_system(m,0);assert(strcmp(before,paper_hash(path)));
end




