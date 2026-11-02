function build_surrogate_flight_forces
% Native environment and force feedback adapter. Public surrogate, not identified airframe.
root=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(root,'simulink','models'));addpath(fullfile(root,'simulink','data'));
env=load(fullfile(root,'simulink','data','native_environment.mat'));p=vehicle_params;load_system('Surrogate_Aero_Forces');
m='Environment_And_Surrogate_Forces';if bdIsLoaded(m),close_system(m,0);end
new_system(m);set_param(m,'SolverType','Fixed-step','Solver','ode4','FixedStep','.001','StopTime','0','ReturnWorkspaceOutputs','on');
b('Inport','position_NED',[20 45 50 65],'Port','1','PortDimensions','3');
b('Inport','velocity_body',[20 155 50 175],'Port','2','PortDimensions','3');
b('Inport','DCM_earth_to_body',[20 650 50 670],'Port','3','PortDimensions','[3 3]');
b('Demux','Position',[100 20 105 100],'Outputs','3');w('position_NED/1','Position/1');
b('Gain','Height',[150 70 200 100],'Gain','-1');w('Position/3','Height/1');
b('Lookup_n-D','MSIS2_density',[250 55 450 100],'NumberOfTableDimensions','1', ...
 'BreakpointsForDimension1',mat2str(env.height_m,17),'Table',mat2str(env.rho,17));w('Height/1','MSIS2_density/1');
b('DotProduct','Speed_squared',[150 160 210 200]);w('velocity_body/1','Speed_squared/1');w('velocity_body/1','Speed_squared/2');
b('Sqrt','Speed',[250 180 300 210]);w('Speed_squared/1','Speed/1');
b('Product','Density_speed_squared',[500 80 530 110]);w('MSIS2_density/1','Density_speed_squared/1');w('Speed_squared/1','Density_speed_squared/2');
b('Gain','Dynamic_pressure',[580 80 650 110],'Gain','.5');w('Density_speed_squared/1','Dynamic_pressure/1');
b('Demux','Velocity',[100 280 105 370],'Outputs','3');w('velocity_body/1','Velocity/1');
b('Trigonometry','Alpha',[250 290 310 330],'Operator','atan2');w('Velocity/3','Alpha/1');w('Velocity/1','Alpha/2');
b('Saturate','Speed_floor',[350 190 410 220],'LowerLimit','1','UpperLimit','inf');w('Speed/1','Speed_floor/1');
b('Product','Sideslip_sine',[450 300 485 335],'Inputs','*/');w('Velocity/2','Sideslip_sine/1');w('Speed_floor/1','Sideslip_sine/2');
b('Saturate','Asin_domain',[530 300 600 335],'LowerLimit','-1','UpperLimit','1');w('Sideslip_sine/1','Asin_domain/1');
b('Trigonometry','Beta',[650 300 710 335],'Operator','asin');w('Asin_domain/1','Beta/1');
b('Mux','Air_data',[750 100 755 330],'Inputs','3');w('Dynamic_pressure/1','Air_data/1');w('Alpha/1','Air_data/2');w('Beta/1','Air_data/3');
b('SubSystem','Public_surrogate',[830 130 1010 270]);Simulink.BlockDiagram.copyContentsToSubsystem('Surrogate_Aero_Forces',[m '/Public_surrogate']);w('Air_data/1','Public_surrogate/1');
b('Constant','Gravity_NED',[220 600 360 635],'Value',mat2str([0;0;p.vehicle.mass*9.80665],17));
b('Product','Gravity_body',[500 620 550 660],'Multiplication','Matrix(*)');w('DCM_earth_to_body/1','Gravity_body/1');w('Gravity_NED/1','Gravity_body/2');
b('Sum','Total_body_force',[1090 310 1120 350],'Inputs','++');w('Public_surrogate/1','Total_body_force/1');w('Gravity_body/1','Total_body_force/2');
b('Outport','Force_B',[1230 320 1260 340]);w('Total_body_force/1','Force_B/1');
b('Mux','Flight_condition',[850 430 855 570],'Inputs','4');w('Dynamic_pressure/1','Flight_condition/1');w('Speed/1','Flight_condition/2');w('Alpha/1','Flight_condition/3');w('Beta/1','Flight_condition/4');
b('Outport','qbar_V_alpha_beta',[1230 470 1260 490],'Port','2');w('Flight_condition/1','qbar_V_alpha_beta/1');
% Each table range is guarded; there is no atmosphere extrapolation at 120 km.
for j=1:2
 b('Constant',sprintf('Hbound%d',j),[100 750+50*j 180 775+50*j],'Value',num2str((j-1)*120000));
 op={'>=','<='};b('RelationalOperator',sprintf('ValidH%d',j),[260 750+50*j 320 775+50*j],'Operator',op{j});w('Height/1',sprintf('ValidH%d/1',j));w(sprintf('Hbound%d/1',j),sprintf('ValidH%d/2',j));
end
b('Logic','Height_valid',[420 800 470 845],'Operator','AND','Inputs','2');w('ValidH1/1','Height_valid/1');w('ValidH2/1','Height_valid/2');
add_block('simulink/Model Verification/Assertion',[m '/Atmosphere_range'],'Position',[530 800 680 840]);w('Height_valid/1','Atmosphere_range/1');
ds=Simulink.SimulationData.Dataset;ds=ds.addElement(timeseries([0 0 -50000],0));ds=ds.addElement(timeseries([3000 0 500],0));ds=ds.addElement(timeseries(eye(3),0));
si=Simulink.SimulationInput(m);si=si.setExternalInput(ds);r=sim(si);assert(all(isfinite(r.yout{1}.Values.Data),'all'));
save_system(m,fullfile(root,'simulink','models',[m '.slx']));close_system(m,0);fprintf('SURROGATE_FLIGHT_FORCE_ADAPTER_PASS\n');
 function b(type,name,pos,varargin),add_block(['built-in/' type],[m '/' name],'Position',pos,varargin{:});end
 function w(src,dst),add_line(m,src,dst,'autorouting','on');end
end
