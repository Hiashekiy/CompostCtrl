function result=e2r_run_case(s,out)
folder=fullfile(out,s.group,'cases',s.id);if ~isfolder(folder),mkdir(folder);end;guard=v3_file_lock(fullfile(folder,'.run.lock'));
source=struct();for name={'e2r_pid_step','e2r_pid_sfun','e2r_select_unload','e2r_drive_endpoint','v3_build','v3_reference','v3_analyze'},source.(name{1})=paper_hash(which(name{1}));end
txt=jsonencode(struct('spec',s,'sources',source));md=java.security.MessageDigest.getInstance('SHA-256');md.update(unicode2native(txt,'UTF-8'));signature=lower(reshape(dec2hex(typecast(md.digest(),'uint8'),2)',1,[]));statusfile=fullfile(folder,'run_status.json');
if isfile(statusfile)
 st=jsondecode(fileread(statusfile));assert(strcmp(st.config_hash,signature),'e2r:resume','Configuration or implementation changed; use a new unique ID');
 if isfile(fullfile(folder,'metrics.json')) && ~ismember(st.status,{'INFRA_FAIL','ANALYSIS_FAIL','RUNNING'}),result=jsondecode(fileread(fullfile(folder,'metrics.json')));return;end
 if strcmp(st.status,'ANALYSIS_FAIL') && isfile(fullfile(folder,'sim_output.mat')),d=load(fullfile(folder,'sim_output.mat'));result=v3_analyze(d.r,s,folder);st.status=result.status;paper_json(statusfile,st);return;end
 error('e2r:retry','Failed/incomplete infrastructure requires an explicit revision ID');
end
paper_json(fullfile(folder,'case_config.json'),s);paper_json(fullfile(folder,'source_manifest.json'),source);paper_json(statusfile,struct('status','RUNNING','config_hash',signature));fprintf('E2R_RUN %s %s\n',s.group,s.id);m='';
try
 path=fullfile(folder,'reference_bank.mat');bank=v3_reference(s,s.dts);save(path,'-struct','bank');m=v3_build(s,path,folder,out);
 if startsWith(s.method,'PID')
  set_param([m '/Online_RTI_HPIPM'],'FunctionName','e2r_pid_sfun');
  add_block('built-in/Outport',[m '/Native_Plant/E2r_measured_rate'],'Port','9','PortDimensions','4');add_line([m '/Native_Plant'],'CMG/1','E2r_measured_rate/1');add_line(m,'Native_Plant/9','Online_RTI_HPIPM/4');
  set_param(m,'InitFcn','init_paths; addpath(fullfile(repo_root,''simulink'',''paper'')); addpath(fullfile(repo_root,''simulink'',''paper'',''v3'')); addpath(fullfile(repo_root,''simulink'',''paper'',''v3'',''e2_revision''));');save_system(m);
 end
 bc=jsondecode(fileread(fullfile(folder,'build_config.json')));bc.model_sha256=paper_hash(bc.model_path);bc.upper_function=get_param([m '/Online_RTI_HPIPM'],'FunctionName');bc.revision_sources=source;paper_json(fullfile(folder,'build_config.json'),bc);
 si=Simulink.SimulationInput(m);si=si.setModelParameter('CaptureErrors','on');wall=tic;r=sim(si);wall_s=toc(wall);save(fullfile(folder,'sim_output.mat'),'r','wall_s','-v7.3');close_system(m,0);m='';
 try,result=v3_analyze(r,s,folder);st=struct('status',result.status,'config_hash',signature,'wall_time_s',wall_s,'error',result.error_message,'actual_end_s',result.actual_end_s);catch err,result=[];st=struct('status','ANALYSIS_FAIL','config_hash',signature,'wall_time_s',wall_s,'error',getReport(err));end
catch err
 result=[];st=struct('status','INFRA_FAIL','config_hash',signature,'error',getReport(err,'extended','hyperlinks','off'));if ~isempty(m)&&bdIsLoaded(m),close_system(m,0);end
end
paper_json(statusfile,st);if isempty(result),fprintf(2,'E2R_FAILURE %s %s\n',s.id,st.error);else,fprintf('E2R_RESULT %s %s RMSE=%g end=%g\n',s.id,result.status,result.rmse_partial_deg,result.actual_end_s);end
end
