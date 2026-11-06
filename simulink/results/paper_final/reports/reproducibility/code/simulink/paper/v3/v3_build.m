function m=v3_build(s,bankpath,folder,batch)
models=fullfile(repo_root,'simulink','cases','paper_v3');build_closedloop(s.duration,true,s.cmg,s.id,'online_predicted',false,4,.2,'analytic_full',0,'feedback_predictive',bankpath,folder,models);
m='NearSpace_CMG_FlightControl';if ~s.cmg,m=[m '_SurfaceOnly'];end;m=[m '_' s.id];assert(numel(m)<=63);load_system(fullfile(models,[m '.slx']));mw=get_param(m,'ModelWorkspace');t=getVariable(mw,'native_controller_template');t.dts=s.dts;assignin(mw,'native_controller_template',t);assignin(mw,'native_results_dir',folder);
if startsWith(s.method,'PID'),assignin(mw,'paper_pid_config',s.pid);set_param([m '/Online_RTI_HPIPM'],'FunctionName','v3_pid_sfun');end
set_param(m,'InitFcn','init_paths; addpath(fullfile(repo_root,''simulink'',''paper'')); addpath(fullfile(repo_root,''simulink'',''paper'',''v3''));');
if s.cmg
 for item={'1','2','4','6'}
 port=str2double(item{1});names={'nu_actual','cmg_acceleration','','cmg_H','','cmg_torque'};name=names{port};parent=[m '/Native_Plant'];add_block('built-in/ToWorkspace',[parent '/v3_' name],'VariableName',name,'SaveFormat','Timeseries','MaxDataPoints','inf');add_line(parent,['CMG/' item{1}],['v3_' name '/1']);
 end
end
assert(strcmp(get_param(m,'FixedStep'),'.001'));assert(getVariable(mw,'native_inner_period')==.02 && t.dt_ctrl==.2 && t.max_sqp_iterations==4);assert(~isempty(find_system([m '/Native_Plant'],'LookUnderMasks','all','Name','CMG'))==logical(s.cmg));
ph=get_param([m '/Online_RTI_HPIPM'],'PortHandles');for j=1:3,assert(get_param(ph.Inport(j),'Line')>0);end;for j=1:2,assert(get_param(ph.Outport(j),'Line')>0);end
save_system(m);close_system(m,0);load_system(fullfile(models,[m '.slx']));
config=struct('model',m,'model_path',fullfile(models,[m '.slx']),'model_sha256',paper_hash(fullfile(models,[m '.slx'])),'bank_sha256',paper_hash(bankpath),'dts',s.dts,'controller',s.method,'upper_function',get_param([m '/Online_RTI_HPIPM'],'FunctionName'),'dt_ctrl',.2,'inner_period',.02,'fixed_step',.001,'sqp_limit',4,'cmg',s.cmg,'physical_logger','CMG cluster output1 nu_actual/output2 acceleration/output4 H/output6 reaction torque');config.pid_source_sha256=paper_hash(which('v3_pid_step'));pidpath=fullfile(batch,'calibration','pid_frozen.json');if startsWith(s.method,'PID') && isfile(pidpath),config.frozen_pid_sha256=paper_hash(pidpath);else,config.frozen_pid_sha256='not_applicable_or_precheck';end;paper_json(fullfile(folder,'build_config.json'),config);
end
