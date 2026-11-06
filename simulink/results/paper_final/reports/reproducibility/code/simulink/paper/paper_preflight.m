function p=paper_preflight(out)
if ~isfolder(out),mkdir(out);end
[~,sha]=system('git rev-parse HEAD');p=struct('git_sha',strtrim(sha),'matlab',version,'toolboxes',ver,'blockers',{{}});
p.parameters=vehicle_params;p.hpipm=isfolder('D:/Tools/hpipm');p.mex=which('hpipm_mex');
files={'simulink/models/NearSpace_CMG_FlightControl.slx','simulink/models/NearSpace_CMG_FlightControl_SurfaceOnly.slx','simulink/data/online_plan_bank.mat'};
p.protected=cell(1,3);
for k=1:3
 e=struct('path',files{k},'sha256',paper_hash(files{k}));
 if k<3
 try
 load_system(files{k});[~,m]=fileparts(files{k});mw=get_param(m,'ModelWorkspace');e.template=getVariable(mw,'native_controller_template');e.horizon=sum(e.template.dts);e.inner_mode=getVariable(mw,'native_inner_mode');e.inner_period=getVariable(mw,'native_inner_period');b=getVariable(mw,'native_reference_bank');for f={'time','x0','q_ref','omega_ref','tau_d','tau_nom','gap'},e.bank_sizes.(f{1})=size(b.(f{1}));end
e.blocks=find_system(m,'SearchDepth',1,'Type','Block');e.block_types=get_param(e.blocks,'BlockType');e.cmg=find_system([m '/Native_Plant'],'LookUnderMasks','all','Name','CMG');e.previous_period=get_param([m '/Previous_Allocation_Command'],'SampleTime');close_system(m,0);
 catch err,e.error=getReport(err);p.blockers{end+1}=e.error;end
 end
 p.protected{k}=e;
end
p.air_order='qbar,V,alpha,beta (online_flight_condition and online_aero_moment source)';paper_json(fullfile(out,'preflight.json'),p);
end
