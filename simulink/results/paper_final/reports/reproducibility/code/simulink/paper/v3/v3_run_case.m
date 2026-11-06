function result=v3_run_case(s,out)
folder=fullfile(out,s.group,'cases',s.id);if ~isfolder(folder),mkdir(folder);end;guard=v3_file_lock(fullfile(folder,'.run.lock'));
signature=paper_hash(which('v3_pid_step'));configtext=jsonencode(s);hash=java.security.MessageDigest.getInstance('SHA-256');hash.update(unicode2native([configtext signature],'UTF-8'));confighash=lower(reshape(dec2hex(typecast(hash.digest(),'uint8'),2)',1,[]));statusfile=fullfile(folder,'run_status.json');
if isfile(statusfile) && isfile(fullfile(folder,'sim_output.mat'))
oldstatus=jsondecode(fileread(statusfile));if strcmp(oldstatus.status,'ANALYSIS_FAIL') && strcmp(oldstatus.config_hash,confighash)
paper_json(fullfile(folder,'analysis_failure_history.json'),oldstatus);d=load(fullfile(folder,'sim_output.mat'));result=v3_analyze(d.r,s,folder);oldstatus.status=result.status;oldstatus.error=result.error_message;oldstatus.actual_end_s=result.actual_end_s;paper_json(statusfile,oldstatus);return;
end;end
if isfile(statusfile) && isfile(fullfile(folder,'sim_output.mat')) && isfile(fullfile(folder,'metrics.json')),st=jsondecode(fileread(statusfile));if isfield(st,'config_hash') && strcmp(st.config_hash,confighash) && ~ismember(st.status,{'INFRA_FAIL','ANALYSIS_FAIL'}),result=jsondecode(fileread(fullfile(folder,'metrics.json')));return;end;error('v3:resume','Configuration changed or failed analysis requires explicit retry revision: %s',folder);end
paper_json(fullfile(folder,'case_config.json'),s);paper_json(statusfile,struct('status','RUNNING','config_hash',confighash));fprintf('V3_RUN %s %s\n',s.group,s.id);m='';
try
path=fullfile(folder,'reference_bank.mat');if ~isfile(path),bank=v3_reference(s,s.dts);save(path,'-struct','bank');end
m=v3_build(s,path,folder,out);si=Simulink.SimulationInput(m);si=si.setModelParameter('CaptureErrors','on');wall=tic;r=sim(si);wall_s=toc(wall);save(fullfile(folder,'sim_output.mat'),'r','wall_s','-v7.3');close_system(m,0);m='';
try,result=v3_analyze(r,s,folder);st=struct('status',result.status,'config_hash',confighash,'wall_time_s',wall_s,'error',result.error_message,'actual_end_s',result.actual_end_s);catch err,st=struct('status','ANALYSIS_FAIL','config_hash',confighash,'wall_time_s',wall_s,'error',getReport(err));result=[];end
catch err,st=struct('status','INFRA_FAIL','config_hash',confighash,'error',getReport(err,'extended','hyperlinks','off'));result=[];if ~isempty(m) && bdIsLoaded(m),close_system(m,0);end
end
paper_json(statusfile,st);if ~isempty(result),fprintf('V3_RESULT %s %s rmse=%g end=%g\n',s.id,result.status,result.rmse_partial_deg,result.actual_end_s);else,fprintf(2,'V3_FAILURE %s %s\n',s.id,st.error);end
end
