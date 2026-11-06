function run_online_frozen_baseline
% Execute preserved L0 models, with extra read-only logs and original solver path.
root=repo_root;init_paths;base=fullfile(root,'simulink','results','online_authority');
for duration=[2 10 60]
 for enabled=[false true]
  m='NearSpace_CMG_FlightControl';arm='B';if ~enabled,m=[m '_SurfaceOnly'];arm='A';end
  folder=fullfile(base,sprintf('L0_%gs_%s',duration,datestr(now,'yyyymmdd_HHMMSS')));mkdir(folder);
  load_system(fullfile(base,'frozen_models',[m '.slx']));
  assert(strcmpi(get_param(m,'FileName'),fullfile(base,'frozen_models',[m '.slx'])),'Wrong frozen model resolved');
  mw=get_param(m,'ModelWorkspace');assignin(mw,'native_results_dir',folder);assignin(mw,'native_capture_diagnostics',true);
  template=getVariable(mw,'native_controller_template');
  assert(~isfield(template,'authority_mode') || strcmp(template.authority_mode,'legacy_frozen'),'Frozen model is not L0');
  sources={'Aerodynamic_Surface/1','Aerodynamic_Surface/2','Aerodynamic_Surface/5'};names={'delta_actual','delta_dot','tau_actual'};
  for k=1:3
   add_block('built-in/ToWorkspace',[m '/' names{k}],'VariableName',names{k},'SaveFormat','Timeseries','MaxDataPoints','inf');
   add_line(m,sources{k},[names{k} '/1']);
  end
  started=tic;si=Simulink.SimulationInput(m);si=si.setModelParameter('StopTime',num2str(duration));r=sim(si);elapsed=toc(started);
  save(fullfile(folder,sprintf('%s_surrogate_flight_%gs.mat',arm,duration)),'r','elapsed');close_system(m,0);
  fprintf('FROZEN_L0_PASS %s %gs elapsed=%g\n',arm,duration,elapsed);
 end
end
end
