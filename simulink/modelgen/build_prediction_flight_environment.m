function build_prediction_flight_environment
% Independent native 6DOF replay validation fixture. Attitude remains dynamic.
% Never called from an S-function callback or used to force a target attitude.
root=repo_root;p=vehicle_params;env=load(fullfile(root,'simulink','data','native_environment.mat'));
m='Prediction_FlightEnvironment';load_system('aerolib6dof');load_system('Environment_And_Surrogate_Forces');
if bdIsLoaded(m),close_system(m,0);end
new_system(m);set_param(m,'SolverType','Fixed-step','Solver','ode4','FixedStep','.001','StopTime','23.2','ReturnWorkspaceOutputs','on');
add_block('built-in/Inport',[m '/Moment_B'],'PortDimensions','3');
add_block('aerolib6dof/6DoF (Quaternion)',[m '/Aircraft_6DOF'],'Mass',num2str(p.vehicle.mass,17),'Inertia',mat2str(p.J,17), ...
 'xme_0',mat2str([0 0 -env.h0],17),'Vm_0',mat2str(env.V_ned0(:)',17),'eul_0','[0 0 0]','pm_0','[0 0 0]','Position',[400 50 600 300]);
add_block('built-in/SubSystem',[m '/Environment'],'Position',[80 350 320 550]);
Simulink.BlockDiagram.copyContentsToSubsystem('Environment_And_Surrogate_Forces',[m '/Environment']);
add_line(m,'Moment_B/1','Aircraft_6DOF/2');add_line(m,'Environment/1','Aircraft_6DOF/1');
add_line(m,'Aircraft_6DOF/2','Environment/1');add_line(m,'Aircraft_6DOF/5','Environment/2');add_line(m,'Aircraft_6DOF/4','Environment/3');
sources={'Aircraft_6DOF/1','Aircraft_6DOF/2','Aircraft_6DOF/3','Environment/2'};
names={'velocity_NED','position_NED','euler','air'};
for k=1:4
 add_block('built-in/ToWorkspace',[m '/' names{k}],'VariableName',names{k},'SaveFormat','Timeseries','MaxDataPoints','inf','Position',[700 60*k 840 60*k+30]);
 add_line(m,sources{k},[names{k} '/1']);
end
save_system(m,fullfile(root,'simulink','models',[m '.slx']));
blocks=find_system(m,'Type','Block');refs=cellfun(@(b)get_param(b,'ReferenceBlock'),blocks,'UniformOutput',false);
out=fullfile(root,'simulink','results','online_authority','stage_B');if ~exist(out,'dir'),mkdir(out);end
writetable(table(blocks,refs),fullfile(out,'native_model_inventory.csv'));close_system(m,0);
end
