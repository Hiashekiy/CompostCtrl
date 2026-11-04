function build_closedloop(duration,flight,enabled,case_name,authority_mode)
if nargin<1,duration=2;end
if nargin<2,flight=false;end
if nargin<3,enabled=true;end
if nargin<4,case_name='';end
if nargin<5,if flight,authority_mode='online_predicted';else,authority_mode='legacy_frozen';end;end
root=repo_root;addpath(fullfile(root,'simulink','models'));addpath(fullfile(root,'simulink','hpipm'));hpipm_startup;
bank=load(fullfile(root,'simulink','data','online_plan_bank.mat'));
if enabled,plant='Plant_6DOF_CMG';else,plant='Plant_6DOF_SurfaceOnly';end
load_system(plant);
if flight,m='NearSpace_CMG_FlightControl';else,m='NearSpace_CMG_AttitudeValidation';end
if ~enabled,m=[m '_SurfaceOnly'];end
if ~isempty(case_name),m=[m '_' case_name];end
if bdIsLoaded(m),close_system(m,0);end
if isempty(case_name),modeldir=fullfile(root,'simulink','models');else,modeldir=fullfile(root,'simulink','cases');end
assert(ismember(authority_mode,{'online_predicted','online_current','legacy_frozen'}),'Unknown authority mode');
online=flight && startsWith(authority_mode,'online_');
out=fullfile(root,'simulink','results','native_closedloop');
if online
 label='L2';if strcmp(authority_mode,'online_current'),label='L1';end
 out=fullfile(root,'simulink','results','online_authority',sprintf('%s_%gs_%s',label,duration,datestr(now,'yyyymmdd_HHMMSS')));
 backup=fullfile(root,'simulink','results','online_authority','frozen_models');if ~exist(backup,'dir'),mkdir(backup);end
 oldfile=fullfile(modeldir,[m '.slx']);
 if isfile(oldfile) && ~isfile(fullfile(backup,[m '.slx'])),copyfile(oldfile,fullfile(backup,[m '.slx']));end
end
if ~exist(out,'dir'),mkdir(out);end
new_system(m);set_param(m,'SolverType','Fixed-step','Solver','ode4','FixedStep','.001','StopTime',num2str(duration), ...
 'ReturnWorkspaceOutputs','on','SaveFormat','Dataset');
mw=get_param(m,'ModelWorkspace');
if enabled,template_file='production_online_qp.mat';else,template_file='production_online_A_qp.mat';end
template=load(fullfile(root,'simulink','tests','05_python_matlab_qp_equivalence',template_file));
% Some reference banks export only the common seven attitude states.
% The B template already carries the baseline CMG configuration in X_nom.
if enabled && numel(bank.x0)==7,bank.x0=[bank.x0(:);template.X_nom(8:11,1)];end
template.authority_mode=authority_mode;
if online
 template.authority_max_step=.1;
 template.tau_min=nan(3,double(template.N));template.tau_max=nan(3,double(template.N));
 assignin(mw,'native_results_dir',out);
end
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
 if online
  if ~isfile(fullfile(root,'simulink','models','Online_Surface_Authority.slx')),build_online_authority;end
  load_system('Online_Surface_Authority');
  add_block('built-in/Mux',[m '/Flight_And_Servo_Feedback'],'Inputs','4','Position',[840 700 845 850]);
  add_block('built-in/UnitDelay',[m '/Previous_Allocation_Command'],'SampleTime','.2','InitialCondition','zeros(3,1)','Position',[490 760 650 800]);
  add_line(m,'Surface_control_allocation/1','Previous_Allocation_Command/1');
  add_line(m,'Native_Plant/2','Flight_And_Servo_Feedback/1');
  add_line(m,'Native_Plant/5','Flight_And_Servo_Feedback/2');
  add_line(m,'Aerodynamic_Surface/1','Flight_And_Servo_Feedback/3');
  add_line(m,'Previous_Allocation_Command/1','Flight_And_Servo_Feedback/4');
  add_line(m,'Flight_And_Servo_Feedback/1','Online_RTI_HPIPM/3');
  add_block('built-in/SubSystem',[m '/Online_Surface_Authority'],'Position',[1040 700 1330 920]);
  Simulink.BlockDiagram.copyContentsToSubsystem('Online_Surface_Authority',[m '/Online_Surface_Authority']);
  add_line(m,'Flight_And_Servo_Feedback/1','Online_Surface_Authority/1');
  add_line(m,'Native_Plant/6','Online_Surface_Authority/2');add_line(m,'Native_Plant/4','Online_Surface_Authority/3');
  add_line(m,'Online_RTI_HPIPM/2','Online_Surface_Authority/4');
  for j=1:4
   add_block('built-in/Terminator',[m sprintf('/Authority_monitor_%d',j)]);
   add_line(m,sprintf('Online_Surface_Authority/%d',j),sprintf('Authority_monitor_%d/1',j));
  end
  sources={'Aerodynamic_Surface/1','Aerodynamic_Surface/2','Aerodynamic_Surface/5','Flight_And_Servo_Feedback/1'};
  names={'delta_actual','delta_dot','tau_actual','flight_servo_measured'};
  for j=1:numel(names)
   add_block('built-in/ToWorkspace',[m '/' names{j}],'VariableName',names{j},'SaveFormat','Timeseries','MaxDataPoints','inf');
   add_line(m,sources{j},[names{j} '/1']);
  end
 end
end
mw=get_param(m,'ModelWorkspace');dist=[bank.time(:) squeeze(bank.tau_d(:,1,:))'];assignin(mw,'disturbance_schedule',dist);
add_block('simulink/Sources/From Workspace',[m '/Disturbance'],'VariableName','disturbance_schedule','Interpolate','off','OutputAfterFinalValue','Holding final value','Position',[290 180 430 220]);add_line(m,'Disturbance/1','Native_Plant/2');
% The reference the controller consumes lives in the model workspace (it has to:
 % the MPC needs the FUTURE sequence, which is not a causal signal).  This
 % subsystem exposes that same schedule as ordinary signals so the diagram shows
 % where the attitude/rate/torque reference comes from, and so it can be logged
 % and watched live.  Both copies are derived from the same bank below.
 ref_att=[bank.time(:) euldeg(squeeze(bank.q_ref(:,1,:))')];   % euldeg already returns degrees
 ref_rate=[bank.time(:) squeeze(bank.omega_ref(:,1,:))'];
 ref_tau=[bank.time(:) squeeze(bank.tau_nom(:,1,:))'];
 % Flattened per-instant window: this is what the controller consumes through its
 % second input port, so the reference is carried by a real signal line.
 nk=numel(bank.time);nN=size(bank.tau_min,2)-1;win=[];
 for k=1:nk
  qk=squeeze(bank.q_ref(:,:,k));wk=squeeze(bank.omega_ref(:,:,k));tk=squeeze(bank.tau_d(:,:,k));
  if online,lo=zeros(3,nN);hi=zeros(3,nN);
  else,lo=bank.tau_min(:,1:nN,k);hi=bank.tau_max(:,1:nN,k);end
  nm=bank.tau_nom(:,1:nN,k);
  v=[qk(:);wk(:);tk(:);lo(:);hi(:);nm(:);bank.gap(:,1,k)];
  if isempty(win),win=zeros(nk,numel(v));end
  win(k,:)=v.';
 end
 % Shift the data instants slightly earlier: with Interpolate=off the block can still
 % hold the previous row exactly on the 0.2 s boundary (floating-point comparison),
 % which makes the controller lag one control period.
 assignin(mw,'reference_window',[bank.time(:)-1e-6 win]);
 assignin(mw,'reference_attitude_deg',ref_att);
 assignin(mw,'reference_body_rate',ref_rate);
 assignin(mw,'reference_torque_nom',ref_tau);
 rm=[m '/Reference_Manager'];
 add_block('built-in/SubSystem',rm,'Position',[1290 30 1620 300]);
 srcs={'window','reference_window'; ...
       'attitude_deg','reference_attitude_deg'; ...
       'body_rate','reference_body_rate'; ...
       'torque_nom','reference_torque_nom'};
 for k=1:size(srcs,1)
  add_block('simulink/Sources/From Workspace',[rm '/' srcs{k,1}],'VariableName',srcs{k,2}, ...
   'Interpolate','off','OutputAfterFinalValue','Holding final value','Position',[25 25+70*(k-1) 185 60+70*(k-1)]);
  add_block('built-in/Outport',[rm sprintf('/ref%d',k)],'Port',num2str(k), ...
   'Position',[230 30+70*(k-1) 260 55+70*(k-1)]);
  add_line(rm,[srcs{k,1} '/1'],sprintf('ref%d/1',k));
 end
 add_line(m,'Reference_Manager/1','Online_RTI_HPIPM/2');   % reference window -> controller
 add_block('built-in/ToWorkspace',[m '/reference_log'],'VariableName','reference_log_att_deg', ...
  'SaveFormat','Timeseries','MaxDataPoints','inf','Position',[1310 270 1450 310]);
 add_line(m,'Reference_Manager/2','reference_log/1');
 add_block('built-in/Gain',[m '/Euler_to_deg'],'Gain',num2str(180/pi,17),'Position',[1150 300 1190 340]);
 add_line(m,'Native_Plant/3','Euler_to_deg/1');
 add_block('built-in/Mux',[m '/Attitude_compare'],'Inputs','2','Position',[1230 300 1235 400]);
 add_line(m,'Euler_to_deg/1','Attitude_compare/1');add_line(m,'Reference_Manager/2','Attitude_compare/2');
 add_block('built-in/Scope',[m '/Attitude_vs_Reference'],'Position',[1290 330 1360 380]);
 add_line(m,'Attitude_compare/1','Attitude_vs_Reference/1');
for pair={'State_feedback/1','Online_RTI_HPIPM/1'}
 src=pair{1};if startsWith(src,'State'),name='states';else,name='commands';end
 add_block('built-in/ToWorkspace',[m '/' name],'VariableName',name,'SaveFormat','Timeseries','MaxDataPoints','inf');add_line(m,src,[name '/1'],'autorouting','on');
end
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
save_with_retry(fullfile(out,sprintf('%s_%s_%gs%s.mat',arm,tag,duration,suffix)),r);
save_system(m,fullfile(modeldir,[m '.slx']));
Simulink.BlockDiagram.arrangeSystem(m);
set_param(m,'InitFcn','init_paths;');
save_system(m,fullfile(modeldir,[m '.slx']));
try,print(['-s' m],'-dpng',fullfile(out,'model.png'));catch,warning('model screenshot skipped');end
close_system(m,0);fprintf('NATIVE_CLOSEDLOOP_PASS duration=%g\n',duration);
end
function save_with_retry(file,result)
% On Windows a just-closed MATLAB process can still hold the target file, which
% makes save fail with "Invalid argument"; retry before giving up.
r=result;                                   % saved under the same variable name as before
for attempt=1:6
 try
  save(file,'r');return;
 catch err
  if attempt==6,rethrow(err);end
  fprintf('SAVE_RETRY %d %s (%s)\n',attempt,file,err.identifier);pause(15);
 end
end
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
function e=euldeg(q)
% Quaternion (scalar first) to roll/pitch/yaw in degrees, row-wise.
q=q./vecnorm(q,2,2);
w=q(:,1);x=q(:,2);y=q(:,3);z=q(:,4);
e=rad2deg([atan2(2*(w.*x+y.*z),1-2*(x.^2+y.^2)), ...
           asin(max(-1,min(1,2*(w.*y-z.*x)))), ...
           atan2(2*(w.*z+x.*y),1-2*(y.^2+z.^2))]);
end
