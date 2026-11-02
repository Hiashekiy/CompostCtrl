function build_native_closedloop(duration,flight,enabled,case_name)
if nargin<1,duration=2;end
if nargin<2,flight=false;end
if nargin<3,enabled=true;end
if nargin<4,case_name='';end
root=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(root,'simulink','models'));addpath(fullfile(root,'simulink','hpipm'));hpipm_startup;
bank=load(fullfile(root,'simulink','data','online_plan_bank.mat'));
if enabled,plant='Plant_6DOF_CMG';else,plant='Plant_6DOF_SurfaceOnly';end
load_system(plant);
if flight,m='NearSpace_CMG_FlightControl';else,m='NearSpace_CMG_AttitudeValidation';end
if ~enabled,m=[m '_SurfaceOnly'];end
if ~isempty(case_name),m=[m '_' case_name];end
if bdIsLoaded(m),close_system(m,0);end
new_system(m);set_param(m,'SolverType','Fixed-step','Solver','ode4','FixedStep','.001','StopTime',num2str(duration), ...
 'ReturnWorkspaceOutputs','on','SaveFormat','Dataset');
mw=get_param(m,'ModelWorkspace');
if enabled,template_file='production_online_qp.mat';else,template_file='production_online_A_qp.mat';end
template=load(fullfile(root,'simulink','tests','05_python_matlab_qp_equivalence',template_file));
assignin(mw,'native_controller_template',template);
assignin(mw,'native_reference_bank',bank);
add_block('built-in/SubSystem',[m '/Native_Plant'],'Position',[500 60 770 330]);
Simulink.BlockDiagram.copyContentsToSubsystem(plant,[m '/Native_Plant']);
q=bank.x0(1:4);w=q(1);x=q(2);y=q(3);z=q(4);
eul=[atan2(2*(w*x+y*z),1-2*(x*x+y*y));asin(2*(w*y-z*x));atan2(2*(w*z+x*y),1-2*(y*y+z*z))];
set_param([m '/Native_Plant/Aircraft_6DOF'],'pm_0',mat2str(bank.x0(5:7),17),'eul_0',mat2str(eul,17));
if enabled,set_param([m '/Native_Plant/CMG/Drive/Unwrapped_gamma'],'InitialCondition',mat2str(bank.x0(8:11),17));end
if enabled && ~flight,configure_ideal_p0_drive([m '/Native_Plant/CMG/Drive'],bank.x0(8:11),template.gdot);end
if enabled,assert_connected([m '/Native_Plant/CMG']);end
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function',[m '/Online_RTI_HPIPM'],'FunctionName','native_mpc_sfun','Position',[90 60 280 145]);
if enabled,outputs='[3 4]';n='3';else,outputs='3';n='2';end
if enabled
 add_block('built-in/Demux',[m '/Control_split'],'Outputs',outputs,'Position',[340 65 345 145]);
else
 add_block('built-in/Gain',[m '/Control_split'],'Gain','1','Position',[330 65 370 105]);
end
add_line(m,'Online_RTI_HPIPM/1','Control_split/1');add_line(m,'Control_split/1','Native_Plant/1');if enabled,add_line(m,'Control_split/2','Native_Plant/4');end
add_block('built-in/Mux',[m '/State_feedback'],'Inputs',n,'Position',[840 150 845 275]);
add_line(m,'Native_Plant/3','State_feedback/1');add_line(m,'Native_Plant/6','State_feedback/2');if enabled,add_line(m,'Native_Plant/8','State_feedback/3');end;add_line(m,'State_feedback/1','Online_RTI_HPIPM/1','autorouting','on');
add_block('built-in/Constant',[m '/Zero_force_attitude_test'],'Value','[0;0;0]','Position',[310 260 420 300]);add_line(m,'Zero_force_attitude_test/1','Native_Plant/3');
if flight
 load_system('Environment_And_Surrogate_Forces');load_system('Aerodynamic_Surface');
 env=load(fullfile(root,'simulink','data','native_environment.mat'));
 R=[1-2*(y*y+z*z) 2*(x*y-w*z) 2*(x*z+w*y);2*(x*y+w*z) 1-2*(x*x+z*z) 2*(y*z-w*x);2*(x*z-w*y) 2*(y*z+w*x) 1-2*(x*x+y*y)];
 set_param([m '/Native_Plant/Aircraft_6DOF'],'xme_0',mat2str([0 0 -env.h0],17),'Vm_0',mat2str(R'*env.V_ned0(:),17));
 ph=get_param([m '/Zero_force_attitude_test'],'LineHandles');delete_line(ph.Outport);delete_block([m '/Zero_force_attitude_test']);
 add_block('built-in/SubSystem',[m '/Environment_And_Forces'],'Position',[1040 440 1280 640]);
 Simulink.BlockDiagram.copyContentsToSubsystem('Environment_And_Surrogate_Forces',[m '/Environment_And_Forces']);
 add_line(m,'Native_Plant/2','Environment_And_Forces/1');add_line(m,'Native_Plant/5','Environment_And_Forces/2');add_line(m,'Native_Plant/4','Environment_And_Forces/3');
 add_line(m,'Environment_And_Forces/1','Native_Plant/3');
 add_block('built-in/SubSystem',[m '/Aerodynamic_Surface'],'Position',[500 470 730 650]);
 Simulink.BlockDiagram.copyContentsToSubsystem('Aerodynamic_Surface',[m '/Aerodynamic_Surface']);
 add_block('built-in/Mux',[m '/Actual_flight_condition'],'Inputs','2','Position',[860 480 865 610]);
 add_line(m,'Environment_And_Forces/2','Actual_flight_condition/1');add_line(m,'Native_Plant/6','Actual_flight_condition/2');add_line(m,'Actual_flight_condition/1','Aerodynamic_Surface/2');
 % MPC total torque -> bounded surface allocation -> servo -> actual total torque.
 lh=get_param([m '/Control_split'],'LineHandles');dest=get_param(lh.Outport(1),'DstPortHandle');delete_line(lh.Outport(1));
 add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function',[m '/Surface_control_allocation'],'FunctionName','native_surface_allocation_sfun','Position',[280 480 430 560]);
 add_block('built-in/Mux',[m '/Allocation_inputs'],'Inputs','3','Position',[160 470 165 610]);
 add_line(m,'Control_split/1','Allocation_inputs/1');add_line(m,'Aerodynamic_Surface/3','Allocation_inputs/2');
 add_block('built-in/Demux',[m '/Air_data_split'],'Outputs','4','Position',[1330 500 1335 620]);add_line(m,'Environment_And_Forces/2','Air_data_split/1');add_line(m,'Air_data_split/1','Allocation_inputs/3');
 add_line(m,'Allocation_inputs/1','Surface_control_allocation/1');add_line(m,'Surface_control_allocation/1','Aerodynamic_Surface/1');
 add_line(m,'Aerodynamic_Surface/5','Native_Plant/1');
 add_block('built-in/ToWorkspace',[m '/flight_conditions'],'VariableName','flight_conditions','SaveFormat','Timeseries','MaxDataPoints','inf');
 add_line(m,'Environment_And_Forces/2','flight_conditions/1');
end
mw=get_param(m,'ModelWorkspace');dist=[bank.time(:) squeeze(bank.tau_d(:,1,:))'];assignin(mw,'disturbance_schedule',dist);
add_block('simulink/Sources/From Workspace',[m '/Disturbance'],'VariableName','disturbance_schedule','Interpolate','off','OutputAfterFinalValue','Holding final value','Position',[290 180 430 220]);add_line(m,'Disturbance/1','Native_Plant/2');
for pair={'State_feedback/1','Online_RTI_HPIPM/1'}
 src=pair{1};if startsWith(src,'State'),name='states';else,name='commands';end
 add_block('built-in/ToWorkspace',[m '/' name],'VariableName',name,'SaveFormat','Timeseries','MaxDataPoints','inf');add_line(m,src,[name '/1'],'autorouting','on');
end
out=fullfile(root,'simulink','results','native_closedloop');if ~exist(out,'dir'),mkdir(out);end
logs=find_system(m,'LookUnderMasks','all','BlockType','ToWorkspace');
for i=1:numel(logs)
 if ~strcmp(get_param(logs{i},'Parent'),m)
  set_param(logs{i},'VariableName',sprintf('native_subsystem_log_%04d',i));
 end
end
r=sim(m);assert(all(isfinite(r.states.Data),'all') && all(isfinite(r.commands.Data),'all'));
if flight,tag='surrogate_flight';else,tag='attitude';end
if enabled,arm='B';else,arm='A';end
if isempty(case_name),suffix='';else,suffix=['_' case_name];end
save(fullfile(out,sprintf('%s_%s_%gs%s.mat',arm,tag,duration,suffix)),'r');save_system(m,fullfile(root,'simulink',[m '.slx']));
Simulink.BlockDiagram.arrangeSystem(m);
set_param(m,'InitFcn','native_startup;');
save_system(m);
try,print(['-s' m],'-dpng',fullfile(out,'model.png'));catch,warning('model screenshot skipped');end
close_system(m,0);fprintf('NATIVE_CLOSEDLOOP_PASS duration=%g\n',duration);
end

function configure_ideal_p0_drive(path,gamma0,gdot)
% P0 executes the commanded gimbal rate directly; P1/flight keeps the tested drive.
% Simulink.SubSystem.deleteContents also deletes the parent-level lines attached to
% this subsystem's ports.  They are captured first and restored afterwards, because
% an unconnected drive leaves the gimbals frozen at gamma0: the CMG then produces no
% reaction torque while the MPC still optimises the four rate commands.
parent=get_param(path,'Parent');
ph=get_param(path,'PortHandles');
out_targets=cell(1,numel(ph.Outport));in_sources=cell(1,numel(ph.Inport));
for k=1:numel(ph.Outport)
 line=get_param(ph.Outport(k),'Line');
 if line>0,out_targets{k}=get_param(line,'DstPortHandle');end
end
for k=1:numel(ph.Inport)
 line=get_param(ph.Inport(k),'Line');
 if line>0,in_sources{k}=get_param(line,'SrcPortHandle');end
end
Simulink.SubSystem.deleteContents(path);
add_block('built-in/Inport',[path '/command'],'PortDimensions','4','Position',[20 60 50 80]);
add_block('built-in/Saturate',[path '/Rate_limit'],'UpperLimit',num2str(gdot,17), ...
 'LowerLimit',num2str(-gdot,17),'Position',[100 50 175 90]);
add_block('built-in/UnitDelay',[path '/Unwrapped_gamma'],'InitialCondition',mat2str(gamma0,17),'SampleTime','.001','Position',[300 180 390 220]);
add_block('built-in/Gain',[path '/Angle_increment'],'Gain','.001','Position',[220 80 280 115]);
add_block('built-in/Sum',[path '/Next_gamma'],'Inputs','++','Position',[340 80 370 115]);
add_block('built-in/Constant',[path '/Ideal_acceleration_diagnostic'],'Value','zeros(4,1)','Position',[100 280 230 315]);
for k=1:3
 names={'actual','derivative','gamma'};add_block('built-in/Outport',[path '/' names{k}],'Port',num2str(k),'Position',[470 70*k 500 70*k+20]);
end
add_line(path,'command/1','Rate_limit/1');add_line(path,'Rate_limit/1','actual/1');add_line(path,'Rate_limit/1','Angle_increment/1');
add_line(path,'Angle_increment/1','Next_gamma/1');add_line(path,'Unwrapped_gamma/1','Next_gamma/2');add_line(path,'Next_gamma/1','Unwrapped_gamma/1');
add_line(path,'Unwrapped_gamma/1','gamma/1');add_line(path,'Ideal_acceleration_diagnostic/1','derivative/1');
ph=get_param(path,'PortHandles');
for k=1:numel(out_targets)
 for h=reshape(out_targets{k},1,[])
  if h<=0,continue;end
  % deleteContents leaves the destination segments dangling; remove them first.
  line=get_param(h,'Line');
  if line>0,delete_line(line);end
  add_line(parent,ph.Outport(k),h,'autorouting','on');
 end
end
for k=1:numel(in_sources)
 if isempty(in_sources{k}) || in_sources{k}(1)<=0,continue;end
 line=get_param(ph.Inport(k),'Line');
 if line>0,delete_line(line);end
 add_line(parent,in_sources{k}(1),ph.Inport(k),'autorouting','on');
end
for k=1:numel(ph.Outport)
 assert(get_param(ph.Outport(k),'Line')>0,'%s: output port %d is unconnected',path,k);
end
for k=1:numel(ph.Inport)
 assert(get_param(ph.Inport(k),'Line')>0,'%s: input port %d is unconnected',path,k);
end
end
function assert_connected(path)
% Fail at build time instead of silently running an inert actuator.
for type={'Inport','Outport'}
 blocks=find_system(path,'SearchDepth',1,'BlockType',type{1});
 for i=1:numel(blocks)
  lh=get_param(blocks{i},'LineHandles');
  if strcmp(type{1},'Inport'),h=lh.Outport;else,h=lh.Inport;end
  assert(h>0,'%s: %s %s is unconnected',path,get_param(blocks{i},'Name'),type{1});
 end
end
end
