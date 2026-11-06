function m=build_paper_case(s,path,folder)
assert(~isempty(s.id));build_closedloop(s.duration,true,s.cmg,s.id,'online_predicted',false,4,.2,'analytic_full',0,'feedback_predictive',path);
m='NearSpace_CMG_FlightControl';if ~s.cmg,m=[m '_SurfaceOnly'];end;m=[m '_' s.id];load_system(fullfile('simulink','cases',[m '.slx']));mw=get_param(m,'ModelWorkspace');assignin(mw,'native_results_dir',folder);
t=getVariable(mw,'native_controller_template');if isfield(s,'dts'),t.dts=s.dts;assignin(mw,'native_controller_template',t);end
if isfield(s,'masked') && s.masked
 w=getVariable(mw,'reference_window');N=double(t.N)+1;
 for k=1:size(w,1),if w(k,1)<s.onset,w(k,2:1+4*N)=repmat(w(k,2:5),1,N);w(k,2+4*N:1+7*N)=0;end,end
 assignin(mw,'reference_window',w);
end
if startsWith(s.method,'PID'),assignin(mw,'paper_pid_config',s.pid);set_param([m '/Online_RTI_HPIPM'],'FunctionName','native_pid_cmg_sfun');end
save_system(m);close_system(m,0);load_system(fullfile('simulink','cases',[m '.slx']));
mw=get_param(m,'ModelWorkspace');tt=getVariable(mw,'native_controller_template');ph=get_param([m '/Online_RTI_HPIPM'],'PortHandles');for j=1:3,assert(get_param(ph.Inport(j),'Line')>0);end;for j=1:2,assert(get_param(ph.Outport(j),'Line')>0);end
config=struct('model',m,'reference_bank',path,'bank_sha256',paper_hash(path),'method',s.method,'upper_function',get_param([m '/Online_RTI_HPIPM'],'FunctionName'),'upper_period',tt.dt_ctrl,'sqp_limit',tt.max_sqp_iterations,'dts',tt.dts,'horizon_s',sum(tt.dts),'inner_period',getVariable(mw,'native_inner_period'),'inner_mode',getVariable(mw,'native_inner_mode'),'cmg_blocks',{find_system([m '/Native_Plant'],'LookUnderMasks','all','Name','CMG')});assert((~isempty(config.cmg_blocks))==logical(s.cmg),'paper:hardware','CMG hardware presence mismatch');assert(config.upper_period==.2 && config.sqp_limit==4 && config.inner_period==.02);paper_json(fullfile(folder,'build_config.json'),config);
end


