function check_dt01_configuration
% Check the saved production models, not merely the model generator defaults.
root=repo_root;init_paths;
original=load(fullfile(root,'simulink','data','online_plan_bank.mat'));
for enabled=[false true]
 m='NearSpace_CMG_FlightControl';if ~enabled,m=[m '_SurfaceOnly'];end
 load_system(fullfile(root,'simulink','models',[m '.slx']));
 mw=get_param(m,'ModelWorkspace');s=getVariable(mw,'native_controller_template');
 assert(abs(s.dt_ctrl-.1)<1e-12 && s.max_sqp_iterations==4);
 assert(abs(s.dts(1)-.1)<1e-12 && double(s.N)==22);
 assert(abs(str2double(get_param(m,'FixedStep'))-.001)<1e-12);
 assert(abs(str2double(get_param([m '/Previous_Allocation_Command'],'SampleTime'))-.1)<1e-12);
 bank=getVariable(mw,'native_reference_bank');window=getVariable(mw,'reference_window');
 assert(isequal(getVariable(mw,'disturbance_schedule'),[original.time(:) squeeze(original.tau_d(:,1,:))']));
 assert(all(abs(diff(bank.time)-.1)<1e-10));
 assert(max(abs(window(:,1)-(bank.time(:)-1e-6)))<1e-12);
 assert(max(abs(vecnorm(reshape(bank.q_ref,4,[]))-1))<1e-12);
 % Compile invokes S-function lifecycle hooks: isolate their output from raw runs.
 scratch=fullfile(root,'simulink','results','online_authority','dt01_sqp4','configuration');
 if ~exist(scratch,'dir'),mkdir(scratch);end
 assignin(mw,'native_results_dir',scratch);
 feval(m,[],[],[],'compile');
 try
  for name={'Online_RTI_HPIPM','Surface_control_allocation'}
   sample=get_param([m '/' name{1}],'CompiledSampleTime');
   if iscell(sample),sample=sample{1};end
   assert(abs(sample(1)-.1)<1e-12,'Block is not running at 100 ms');
  end
 catch err
  feval(m,[],[],[],'term');close_system(m,0);rethrow(err);
 end
 feval(m,[],[],[],'term');close_system(m,0);
 fprintf('DT01_CONFIGURATION_PASS %s\n',m);
end
end
