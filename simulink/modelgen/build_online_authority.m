function build_online_authority
% Visible physical diagnostics plus the solver's certified stage-zero bounds.
% Horizon geometry/servo certification remains in the iterative MPC algorithm.
root=repo_root;p=vehicle_params;v=p.vehicle;
load_system('Environment_And_Surrogate_Forces');load_system('Aerodynamic_Model');
m='Online_Surface_Authority';if bdIsLoaded(m),close_system(m,0);end
new_system(m);set_param(m,'SolverType','Fixed-step','Solver','ode4','FixedStep','.001','StopTime','0');
b('Inport','Flight_State_In',[20 40 50 60],'Port','1','PortDimensions','12');
b('Inport','omega',[20 180 50 200],'Port','2','PortDimensions','3');
b('Inport','DCM',[20 250 50 270],'Port','3','PortDimensions','[3 3]');
b('Inport','Certified_Bounds_From_RTI',[20 330 50 350],'Port','4','PortDimensions','8');
b('Demux','Measured_State',[100 20 105 150],'Outputs','[3 3 3 3]');w('Flight_State_In/1','Measured_State/1');
b('SubSystem','Flight_Condition',[200 30 430 170]);Simulink.BlockDiagram.copyContentsToSubsystem('Environment_And_Surrogate_Forces',[m '/Flight_Condition']);
w('Measured_State/1','Flight_Condition/1');w('Measured_State/2','Flight_Condition/2');w('DCM/1','Flight_Condition/3');
b('Terminator','Unused_force',[490 30 510 50]);w('Flight_Condition/1','Unused_force/1');
b('Constant','Zero_surface',[210 220 300 250],'Value','zeros(3,1)');
b('Mux','Natural_inputs',[470 170 475 270],'Inputs','3');w('Flight_Condition/2','Natural_inputs/1');w('omega/1','Natural_inputs/2');w('Zero_surface/1','Natural_inputs/3');
b('SubSystem','Natural_Moment',[540 170 760 300]);Simulink.BlockDiagram.copyContentsToSubsystem('Aerodynamic_Model',[m '/Natural_Moment']);w('Natural_inputs/1','Natural_Moment/1');
for k=2:3,b('Terminator',sprintf('Unused_moment%d',k),[800 200+30*k 820 220+30*k]);w(sprintf('Natural_Moment/%d',k),sprintf('Unused_moment%d/1',k));end
b('Demux','Air_components',[480 60 485 130],'Outputs','4');w('Flight_Condition/2','Air_components/1');
for k=2:4,b('Terminator',sprintf('Unused_air%d',k),[540 10+30*k 560 30+30*k]);w(sprintf('Air_components/%d',k),sprintf('Unused_air%d/1',k));end
C=v.S*[0 v.b*v.Cl_da v.b*v.Cl_dr;v.cbar*v.Cm_de 0 0;0 v.b*v.Cn_da v.b*v.Cn_dr];
b('Gain','Surface_Effectiveness',[570 60 750 110],'Gain',mat2str(C(:),17),'Multiplication','Matrix(K*u)');w('Air_components/1','Surface_Effectiveness/1');
b('SubSystem','Actuator_Reachability',[200 420 500 620]);
a=[m '/Actuator_Reachability'];Simulink.SubSystem.deleteContents(a);
add_block('built-in/Inport',[a '/actual'],'Port','1','PortDimensions','3');
add_block('built-in/Inport',[a '/previous_command'],'Port','2','PortDimensions','3');
for k=1:2
 tag=sprintf('side%d',k);signum=2*k-3;
 add_block('built-in/Constant',[a '/' tag '_slew'],'Value',mat2str(signum*v.delta_dot_max(:)*.2,17));
 add_block('built-in/Sum',[a '/' tag '_target'],'Inputs','++');
 add_line(a,'previous_command/1',[tag '_target/1']);add_line(a,[tag '_slew/1'],[tag '_target/2']);
 add_block('built-in/Saturate',[a '/' tag '_command'],'LowerLimit',mat2str(-v.delta_max(:),17),'UpperLimit',mat2str(v.delta_max(:),17));
 add_line(a,[tag '_target/1'],[tag '_command/1']);
 add_block('built-in/Sum',[a '/' tag '_error'],'Inputs','+-');add_line(a,[tag '_command/1'],[tag '_error/1']);add_line(a,'actual/1',[tag '_error/2']);
 add_block('built-in/Gain',[a '/' tag '_inverse_tau'],'Gain',num2str(1/v.tau_s,17));add_line(a,[tag '_error/1'],[tag '_inverse_tau/1']);
 add_block('built-in/Saturate',[a '/' tag '_rate'],'LowerLimit',mat2str(-v.delta_dot_max(:),17),'UpperLimit',mat2str(v.delta_dot_max(:),17));add_line(a,[tag '_inverse_tau/1'],[tag '_rate/1']);
 add_block('built-in/Gain',[a '/' tag '_dt'],'Gain','.001');add_line(a,[tag '_rate/1'],[tag '_dt/1']);
 add_block('built-in/Sum',[a '/' tag '_next'],'Inputs','++');add_line(a,'actual/1',[tag '_next/1']);add_line(a,[tag '_dt/1'],[tag '_next/2']);
 add_block('built-in/Outport',[a '/' tag '_endpoint'],'Port',num2str(k));add_line(a,[tag '_next/1'],[tag '_endpoint/1']);
end
Simulink.Annotation(a,'One physical 1 ms step. Full-stage reachability and all eight vertex command paths are certified inside online_authority_rollout.');
Simulink.BlockDiagram.arrangeSystem(a);
w('Measured_State/3','Actuator_Reachability/1');w('Measured_State/4','Actuator_Reachability/2');
b('Demux','Authority_Bounds',[200 320 205 390],'Outputs','[6 2]');w('Certified_Bounds_From_RTI/1','Authority_Bounds/1');
b('Mux','Diagnostics',[850 100 855 500],'Inputs','7');
sources={'Flight_Condition/2','Natural_Moment/1','Surface_Effectiveness/1','Authority_Bounds/2','Measured_State/3','Actuator_Reachability/1','Actuator_Reachability/2'};
for k=1:7,w(sources{k},sprintf('Diagnostics/%d',k));end
sources={'Flight_Condition/2','Natural_Moment/1','Authority_Bounds/1','Diagnostics/1'};
names={'air_measured','natural_measured','bounds_online','diagnostics'};
for k=1:4,b('Outport',names{k},[950 80*k 980 80*k+20],'Port',num2str(k));w(sources{k},[names{k} '/1']);end
Simulink.BlockDiagram.arrangeSystem(m);save_system(m,fullfile(root,'simulink','models',[m '.slx']));
out=fullfile(root,'simulink','results','online_authority','module');if ~exist(out,'dir'),mkdir(out);end
blocks=find_system(m,'Type','Block');types=cellfun(@(x)get_param(x,'BlockType'),blocks,'UniformOutput',false);refs=cellfun(@(x)get_param(x,'ReferenceBlock'),blocks,'UniformOutput',false);
assert(~any(ismember(types,{'S-Function','MATLABFcn','Fcn'})));
writetable(table(blocks,types,refs),fullfile(out,'inventory.csv'));close_system(m,0);
 function b(type,name,pos,varargin),add_block(['built-in/' type],[m '/' name],'Position',pos,varargin{:});end
 function w(src,dst),add_line(m,src,dst,'autorouting','on');end
end
