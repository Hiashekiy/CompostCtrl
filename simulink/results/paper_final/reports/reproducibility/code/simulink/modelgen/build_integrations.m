function build_integrations
root=repo_root; models=fullfile(root,'simulink','models');
ref=load(fullfile(root,'simulink','data','native_actuators','reference.mat'));
out=fullfile(root,'simulink','results','native_integrations'); if ~exist(out,'dir'),mkdir(out);end
addpath(models); load_system('CMG_Drive'); load_system('CMG_Geometry');
m='CMG_Cluster'; if bdIsLoaded(m),close_system(m,0);end
new_system(m); settings(m);
add_block('built-in/SubSystem',[m '/Drive'],'Position',[200 80 345 230]);
Simulink.BlockDiagram.copyContentsToSubsystem('CMG_Drive',[m '/Drive']);
add_block('built-in/SubSystem',[m '/Geometry'],'Position',[540 80 720 300]);
Simulink.BlockDiagram.copyContentsToSubsystem('CMG_Geometry',[m '/Geometry']);
g=[m '/Geometry'];
for k=1:4
 replace_source(g,sprintf('gamma%d',k),k);
 replace_source(g,sprintf('nu%d',k),k+4);
end
names={'H','A','torque'};
for k=1:3,replace_sink(g,names{k},k);end
add_block('built-in/Inport',[m '/nu_command'],'Position',[40 110 70 130],'PortDimensions','4');
add_line(m,'nu_command/1','Drive/1');
for j=1:2
 name=sprintf('Split_%d',j); add_block('built-in/Demux',[m '/' name], ...
  'Position',[400 60+130*j 405 150+130*j],'Outputs','4');
 if j==1,src='Drive/3';else,src='Drive/1';end
 add_line(m,src,[name '/1']);
 for k=1:4,add_line(m,sprintf('%s/%d',name,k),sprintf('Geometry/%d',k+4*(j-1)),'autorouting','on');end
end
src={'Drive/1','Drive/2','Drive/3','Geometry/1','Geometry/2','Geometry/3'};
names={'nu_actual','acceleration','gamma','H','A','torque'};
for k=1:6
 add_block('built-in/Outport',[m '/' names{k}],'Port',num2str(k),'Position',[850 40+65*k 880 60+65*k]);
 add_line(m,src{k},[names{k} '/1'],'autorouting','on');
end
ref=load(fullfile(root,'simulink','data','native_actuators','reference.mat'));
si=Simulink.SimulationInput(m);si=si.setExternalInput([ref.t(:) ref.cmg_cmd]);r=sim(si);
assert(r.yout.numElements==6);
save(fullfile(out,'cmg_integrated_run.mat'),'r'); export(m);
close_system(m,0);

load_system('Surface_Actuator');load_system('Aerodynamic_Model');
m='Aerodynamic_Surface';if bdIsLoaded(m),close_system(m,0);end
new_system(m);settings(m);
for item={'Surface_Actuator','Aerodynamic_Model'}
 name=item{1}; add_block('built-in/SubSystem',[m '/' name]);
 Simulink.BlockDiagram.copyContentsToSubsystem(name,[m '/' name]);
end
set_param([m '/Surface_Actuator'],'Position',[210 60 390 160]);
set_param([m '/Aerodynamic_Model'],'Position',[600 80 780 220]);
add_block('built-in/Inport',[m '/delta_command'],'Port','1','PortDimensions','3','Position',[30 80 60 100]);
add_block('built-in/Inport',[m '/flight_qbar_V_alpha_beta_omega'],'Port','2','PortDimensions','7','Position',[30 230 60 250]);
add_block('built-in/Mux',[m '/Flight_and_actual_surface'],'Inputs','2','Position',[490 160 495 220]);
add_line(m,'delta_command/1','Surface_Actuator/1');
add_line(m,'flight_qbar_V_alpha_beta_omega/1','Flight_and_actual_surface/1');
add_line(m,'Surface_Actuator/1','Flight_and_actual_surface/2');
add_line(m,'Flight_and_actual_surface/1','Aerodynamic_Model/1');
src={'Surface_Actuator/1','Surface_Actuator/2','Aerodynamic_Model/1','Aerodynamic_Model/2','Aerodynamic_Model/3'};
names={'delta_actual','delta_dot','natural_moment','surface_increment','total_moment'};
for k=1:5
 add_block('built-in/Outport',[m '/' names{k}],'Port',num2str(k),'Position',[900 50+65*k 930 70+65*k]);
 add_line(m,src{k},[names{k} '/1'],'autorouting','on');
end
% Two root inputs, respectively command and measured flight condition.
ds=Simulink.SimulationData.Dataset;
ds=ds.addElement(timeseries(ref.surface_cmd,ref.t(:)),'delta_command');
flight=repmat([1000 1000 .1 .02 .01 .02 .03],numel(ref.t),1);
ds=ds.addElement(timeseries(flight,ref.t(:)),'flight');
si=Simulink.SimulationInput(m);si=si.setExternalInput(ds);r=sim(si);
assert(r.yout.numElements==5);save(fullfile(out,'aero_surface_integrated_run.mat'),'r');
export(m);close_system(m,0);fprintf('NATIVE_INTEGRATIONS_PASS\n');
 function settings(name)
  set_param(name,'SolverType','Fixed-step','Solver','FixedStepDiscrete','FixedStep',num2str(ref.dt,17), ...
   'StopTime','8','ReturnWorkspaceOutputs','on','SaveFormat','Dataset');
 end
 function replace_source(parent,name,port)
  path=[parent '/' name]; pos=get_param(path,'Position'); handles=get_param(path,'LineHandles');
  dest=get_param(handles.Outport,'DstPortHandle');delete_line(handles.Outport);delete_block(path);
  add_block('built-in/Inport',path,'Position',pos,'Port',num2str(port));
  ph=get_param(path,'PortHandles');for d=dest(:)',add_line(parent,ph.Outport,d,'autorouting','on');end
 end
 function replace_sink(parent,name,port)
  path=[parent '/' name];pos=get_param(path,'Position');handles=get_param(path,'LineHandles');
  srcport=get_param(handles.Inport,'SrcPortHandle');delete_line(handles.Inport);delete_block(path);
  add_block('built-in/Outport',path,'Position',pos,'Port',num2str(port));
  ph=get_param(path,'PortHandles');add_line(parent,srcport,ph.Inport,'autorouting','on');
 end
 function export(name)
  save_system(name,fullfile(models,[name '.slx']));print(['-s' name],'-dpng',fullfile(out,[name '.png']));
  blocks=find_system(name,'LookUnderMasks','all','Type','Block');
  types=cellfun(@(b)get_param(b,'BlockType'),blocks,'UniformOutput',false);
  assert(~any(ismember(types,{'Fcn','MATLABFcn','S-Function'})));
  writetable(table(blocks,types),fullfile(out,[name '_inventory.csv']));
 end
end
