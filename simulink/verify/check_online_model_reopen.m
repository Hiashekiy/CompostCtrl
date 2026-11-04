function check_online_model_reopen
root=repo_root;init_paths;
folder=fullfile(root,'simulink','results','online_authority','reopen');if ~exist(folder,'dir'),mkdir(folder);end
for enabled=[false true]
 m='NearSpace_CMG_FlightControl';arm='B';if ~enabled,m=[m '_SurfaceOnly'];arm='A';end
 file=fullfile(root,'simulink','models',[m '.slx']);load_system(file);
 mw=get_param(m,'ModelWorkspace');s=getVariable(mw,'native_controller_template');
 assert(strcmp(s.authority_mode,'online_predicted'),'Production model was left in an ablation mode');
 assert(all(isnan(s.tau_min),'all') && all(isnan(s.tau_max),'all'),'Offline bounds entered online template');
 ports=get_param([m '/Online_RTI_HPIPM'],'PortHandles');assert(numel(ports.Inport)==3);
 for j=1:3,assert(get_param(ports.Inport(j),'Line')>0);end
 assert(get_param(ports.Outport(1),'Line')>0 && get_param(ports.Outport(2),'Line')>0);
 if enabled
  path=[m '/Native_Plant/CMG/Drive'];ph=get_param(path,'PortHandles');
  for j=1:numel(ph.Inport),assert(get_param(ph.Inport(j),'Line')>0);end
  for j=1:numel(ph.Outport),assert(get_param(ph.Outport(j),'Line')>0);end
 end
 assignin(mw,'native_results_dir',folder);
 si=Simulink.SimulationInput(m);si=si.setModelParameter('StopTime','2');r=sim(si);
 assert(all(isfinite(r.states.Data),'all') && all(isfinite(r.commands.Data),'all'));
 if enabled
  data=squeeze(r.states.Data);if size(data,1)~=numel(r.states.Time),data=data';end
  assert(all(max(abs(data(:,7:10)),[],1)>1e-3),'CMG gimbals failed to move after reopening');
 end
 save(fullfile(folder,[arm '_reopened.mat']),'r');
 blocks=find_system(m,'LookUnderMasks','all','Type','Block');
 refs=cellfun(@(b)get_param(b,'ReferenceBlock'),blocks,'UniformOutput',false);
 types=cellfun(@(b)get_param(b,'BlockType'),blocks,'UniformOutput',false);
 assert(any(contains(refs,'6DoF (Quaternion)')),'Native Aerospace 6DoF missing');
 writetable(table(blocks,types,refs),fullfile(folder,[arm '_model_inventory.csv']));
 close_system(m,0);fprintf('ONLINE_MODEL_REOPEN_PASS %s\n',arm);
end
check_actuation('NearSpace_CMG_AttitudeValidation_S00_pitch30',fullfile(folder,'P0_regression'));
end
