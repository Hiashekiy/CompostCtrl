function build_native_actuators_aero
% Explicit discrete servo/drive graphs and linear-derivative aerodynamic moments.
root=native_root; addpath(fullfile(root,'simulink','data'));
p=vehicle_params; v=p.vehicle; c=p.cmg;
out=fullfile(root,'simulink','results','native_actuators_aero');
if ~exist(out,'dir'), mkdir(out); end
models=fullfile(root,'simulink','models');
ref=load(fullfile(root,'simulink','data','native_actuators','reference.mat'));
dt=double(ref.dt);
load_system('simulink');
make_drive('CMG_Drive',4,c.gdot_max*ones(4,1),c.gddot_max*ones(4,1),c.tau_g,true);
make_drive('Surface_Actuator',3,v.delta_max(:),v.delta_dot_max(:),v.tau_s,false);
make_aero;
fprintf('NATIVE_ACTUATORS_AERO_PASS\n');

 function begin(name)
  if bdIsLoaded(name), close_system(name,0); end
  new_system(name); set_param(name,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
   'FixedStep',num2str(dt,17),'StopTime','8','ReturnWorkspaceOutputs','on');
 end
 function b(m,type,name,pos,varargin)
  add_block(['built-in/' type],[m '/' name],'Position',pos,varargin{:});
 end
 function w(m,src,dst), add_line(m,src,dst,'autorouting','on'); end
 function finish(m)
  save_system(m,fullfile(models,[m '.slx']));
  blocks=find_system(m,'LookUnderMasks','all','Type','Block');
  types=cellfun(@(x)get_param(x,'BlockType'),blocks,'UniformOutput',false);
  assert(~any(ismember(types,{'Fcn','S-Function','MATLABFcn'})));
  writetable(table(blocks,types,'VariableNames',{'Block','Type'}),fullfile(out,[m '_inventory.csv']));
  print(['-s' m],'-dpng',fullfile(out,[m '.png']));
 end
 function logsignal(m,src,name,y)
  b(m,'ToWorkspace',['log_' name],[810 y 925 y+30],'VariableName',name,'SaveFormat','Timeseries','MaxDataPoints','inf');
  w(m,src,['log_' name '/1']);
 end
 function make_drive(m,n,limit,deriv,tau,is_cmg)
  begin(m);
  b(m,'Inport','command',[25 70 55 90],'PortDimensions',num2str(n),'SampleTime',num2str(dt));
  b(m,'Saturate','Command_limit',[100 55 175 100],'UpperLimit',mat2str(limit),'LowerLimit',mat2str(-limit));
  b(m,'Sum','Tracking_error',[225 60 250 95],'Inputs','+-');
  b(m,'Gain','Inverse_time_constant',[290 60 375 95],'Gain',num2str(1/tau,17));
  b(m,'Saturate','Derivative_limit',[415 60 495 95],'UpperLimit',mat2str(deriv),'LowerLimit',mat2str(-deriv));
  b(m,'Gain','Euler_increment',[530 60 590 95],'Gain',num2str(dt,17));
  b(m,'Sum','Next_state',[630 60 655 95],'Inputs','++');
  b(m,'Saturate','State_limit',[630 150 705 185],'UpperLimit',mat2str(limit),'LowerLimit',mat2str(-limit));
  b(m,'UnitDelay','Actual_state',[400 230 470 270],'InitialCondition',mat2str(zeros(n,1)),'SampleTime',num2str(dt));
  w(m,'command/1','Command_limit/1'); w(m,'Command_limit/1','Tracking_error/1');
  w(m,'Actual_state/1','Tracking_error/2'); w(m,'Tracking_error/1','Inverse_time_constant/1');
  w(m,'Inverse_time_constant/1','Derivative_limit/1'); w(m,'Derivative_limit/1','Euler_increment/1');
  w(m,'Euler_increment/1','Next_state/1'); w(m,'Actual_state/1','Next_state/2');
  w(m,'Next_state/1','State_limit/1'); w(m,'State_limit/1','Actual_state/1');
  b(m,'Outport','actual',[760 230 790 250],'Port','1'); w(m,'Actual_state/1','actual/1');
  b(m,'Outport','derivative',[760 290 790 310],'Port','2'); w(m,'Derivative_limit/1','derivative/1');
  logsignal(m,'Actual_state/1','state',355); logsignal(m,'Derivative_limit/1','deriv',405);
  if is_cmg
   b(m,'Gain','Angle_increment',[530 460 590 495],'Gain',num2str(dt));
   b(m,'Sum','Next_gamma',[630 460 655 495],'Inputs','++');
   b(m,'UnitDelay','Unwrapped_gamma',[400 530 480 565], ...
    'InitialCondition',mat2str([2*pi-.02;0;0;0],17),'SampleTime',num2str(dt));
   w(m,'Actual_state/1','Angle_increment/1'); w(m,'Angle_increment/1','Next_gamma/1');
   w(m,'Unwrapped_gamma/1','Next_gamma/2'); w(m,'Next_gamma/1','Unwrapped_gamma/1');
   b(m,'Outport','gamma',[760 500 790 520],'Port','3'); w(m,'Unwrapped_gamma/1','gamma/1');
   logsignal(m,'Unwrapped_gamma/1','gamma_state',565);
   cmd=ref.cmg_cmd; expected=ref.nu; expected_deriv=ref.accel;
  else
   cmd=ref.surface_cmd; expected=ref.delta; expected_deriv=ref.delta_dot;
  end
  si=Simulink.SimulationInput(m); si=si.setExternalInput([ref.t(:) cmd]); r=sim(si);
  actual=squeeze(r.state.Data); derivative=squeeze(r.deriv.Data);
  if size(actual,1)==n, actual=actual'; derivative=derivative'; end
  fprintf('%s actual=%s expected=%s derivative=%s time=%s\n',m,mat2str(size(actual)),mat2str(size(expected)),mat2str(size(derivative)),mat2str(size(r.state.Time)));
  err=[max(abs(actual-expected),[],'all'),max(abs(derivative-expected_deriv),[],'all')];
  assert(all(err<1e-10),'Drive discrete recurrence mismatch');
  if is_cmg
   angles=squeeze(r.gamma_state.Data); if size(angles,1)==n, angles=angles'; end
   assert(max(abs(angles-ref.gamma),[],'all')<1e-10);
  end
  save(fullfile(out,[m '_validation.mat']),'r','err');
  % Production initial gamma is zero. Boundary-crossing IC belongs only to the test.
  if is_cmg, set_param([m '/Unwrapped_gamma'],'InitialCondition','zeros(4,1)'); end
  finish(m); close_system(m,0); disp([m ' PASS']);
 end
 function make_aero
  m='Aerodynamic_Model'; begin(m); set_param(m,'StopTime','0');
  b(m,'Inport','flight_and_surface',[20 145 50 165],'PortDimensions','10');
  b(m,'Demux','Inputs',[90 30 95 340],'Outputs','10'); w(m,'flight_and_surface/1','Inputs/1');
  % Packed input: qbar,V,alpha,beta,p,q,r,de,da,dr. Derivative tables are parameterized, not CFD.
  slopes=[v.Cl_beta v.Cm_alpha v.Cn_beta]; inports=[4 3 4];
  for j=1:3
   name=sprintf('Static_coefficient_%d',j);
   b(m,'Lookup_n-D',name,[180 20+80*j 340 55+80*j], ...
    'NumberOfTableDimensions','1','BreakpointsForDimension1','[-pi 0 pi]', ...
    'Table',mat2str(slopes(j)*[-pi 0 pi],17));
   w(m,sprintf('Inputs/%d',inports(j)),[name '/1']);
  end
  b(m,'Mux','Static_coefficients',[380 95 385 290],'Inputs','3');
  for j=1:3, w(m,sprintf('Static_coefficient_%d/1',j),sprintf('Static_coefficients/%d',j)); end
  b(m,'Constant','Cm0',[180 365 230 395],'Value',mat2str([0;v.Cm0;0],17));
  b(m,'Mux','Rates',[180 440 185 510],'Inputs','3');
  b(m,'Mux','Deflections',[180 560 185 630],'Inputs','3');
  for j=1:3, w(m,sprintf('Inputs/%d',4+j),sprintf('Rates/%d',j)); w(m,sprintf('Inputs/%d',7+j),sprintf('Deflections/%d',j)); end
  D=[v.Cl_p*v.b/2 0 v.Cl_r*v.b/2;0 v.Cm_q*v.cbar/2 0;v.Cn_p*v.b/2 0 v.Cn_r*v.b/2];
  C=[0 v.Cl_da v.Cl_dr;v.Cm_de 0 0;0 v.Cn_da v.Cn_dr];
  b(m,'Gain','Damping_derivatives',[250 440 370 480],'Gain',mat2str(D,17),'Multiplication','Matrix(K*u)');
  b(m,'Saturate','Speed_floor',[250 520 340 550],'LowerLimit','1','UpperLimit','inf');
  b(m,'Product','Normalized_damping',[415 435 455 480],'Inputs','*/');
  w(m,'Rates/1','Damping_derivatives/1'); w(m,'Inputs/2','Speed_floor/1');
  w(m,'Damping_derivatives/1','Normalized_damping/1'); w(m,'Speed_floor/1','Normalized_damping/2');
  b(m,'Gain','Surface_derivatives',[250 590 380 625],'Gain',mat2str(C,17),'Multiplication','Matrix(K*u)'); w(m,'Deflections/1','Surface_derivatives/1');
  b(m,'Sum','Natural_coefficients',[500 250 530 300],'Inputs','+++');
  w(m,'Static_coefficients/1','Natural_coefficients/1'); w(m,'Cm0/1','Natural_coefficients/2'); w(m,'Normalized_damping/1','Natural_coefficients/3');
  G=diag(v.S*[v.b v.cbar v.b]);
  src={'Natural_coefficients/1','Surface_derivatives/1'};
  for j=1:2
   b(m,'Gain',sprintf('Geometry_%d',j),[570 230*j 675 230*j+40],'Gain',mat2str(G,17),'Multiplication','Matrix(K*u)');
   b(m,'Product',sprintf('Moment_%d',j),[715 230*j 745 230*j+35]);
   w(m,src{j},sprintf('Geometry_%d/1',j)); w(m,sprintf('Geometry_%d/1',j),sprintf('Moment_%d/1',j));
   w(m,'Inputs/1',sprintf('Moment_%d/2',j));
   b(m,'Outport',sprintf('component_%d',j),[800 230*j 830 230*j+20],'Port',num2str(j));
   w(m,sprintf('Moment_%d/1',j),sprintf('component_%d/1',j));
  end
  b(m,'Sum','Total_moment',[825 365 855 400],'Inputs','++'); w(m,'Moment_1/1','Total_moment/1'); w(m,'Moment_2/1','Total_moment/2');
  b(m,'Outport','total',[970 365 1000 385],'Port','3'); w(m,'Total_moment/1','total/1');
  logsignal(m,'Total_moment/1','total_moment',680);
  data=readmatrix(fullfile(root,'simulink','data','native_baseline','A4_descent_replay_S00_with_cmg','aerodynamic_reference.csv'));
  errors=zeros(size(data,1),1);
  for k=1:size(data,1)
   z=[.5*data(k,1)*data(k,2)^2 data(k,2:10)];
   si=Simulink.SimulationInput(m); si=si.setExternalInput([0 z]); r=sim(si);
   errors(k)=max(abs(squeeze(r.total_moment.Data(:))-data(k,17:19)'));
  end
  assert(all(errors<1e-7)); writematrix(errors,fullfile(out,'aerodynamic_errors.csv'));
  finish(m); close_system(m,0); fprintf('AERO_PASS samples=%d error=%g\n',numel(errors),max(errors));
 end
end
