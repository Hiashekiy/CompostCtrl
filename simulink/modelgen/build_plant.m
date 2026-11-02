function build_plant
root=repo_root;addpath(fullfile(root,'simulink','data'));addpath(fullfile(root,'simulink','models'));
p=vehicle_params;load_system('CMG_Cluster');load_system('aerolib6dof');
out=fullfile(root,'simulink','results','native_plant');if ~exist(out,'dir'),mkdir(out);end
for enabled=[false true]
 if enabled,m='Plant_6DOF_CMG';else,m='Plant_6DOF_SurfaceOnly';end
 if bdIsLoaded(m),close_system(m,0);end
 new_system(m);set_param(m,'SolverType','Fixed-step','Solver','ode4','FixedStep','.001','StopTime','.8', ...
  'ReturnWorkspaceOutputs','on','SaveFormat','Dataset','SaveState','on');
 names={'aero_total_moment','disturbance_moment','body_force'};
 for k=1:3,b('Inport',names{k},[20 40+80*k 50 60+80*k],'Port',num2str(k),'PortDimensions','3');end
 if enabled
  b('Inport','nu_command',[20 410 50 430],'Port','4','PortDimensions','4');
  b('SubSystem','CMG',[170 370 320 530]);Simulink.BlockDiagram.copyContentsToSubsystem('CMG_Cluster',[m '/CMG']);
  w('nu_command/1','CMG/1');
  H='CMG/4';torque='CMG/6';
 else
  b('Constant','Zero_H',[180 390 250 425],'Value','[0;0;0]');H='Zero_H/1';torque=H;
 end
 b('aero','Aircraft_6DOF',[850 90 1060 360], 'Mass',num2str(p.vehicle.mass), ...
  'Inertia',mat2str(p.J,17),'pm_0','[.02 -.01 .015]');
 b('Demux','Omega_components',[420 90 425 180],'Outputs','3');
 b('Demux','H_components',[420 370 425 460],'Outputs','3');
 w('Aircraft_6DOF/6','Omega_components/1');w(H,'H_components/1');
 for i=1:3
  j=mod(i,3)+1;k=mod(i+1,3)+1;
  for side=1:2
   name=sprintf('Cross_%d_%d',i,side);b('Product',name,[490+80*side 90*i 520+80*side 90*i+30]);
   if side==1,a=j;c=k;else,a=k;c=j;end
   w(sprintf('Omega_components/%d',a),[name '/1']);w(sprintf('H_components/%d',c),[name '/2']);
  end
  b('Sum',sprintf('Cross_component_%d',i),[720 90*i 745 90*i+30],'Inputs','+-');
  w(sprintf('Cross_%d_1/1',i),sprintf('Cross_component_%d/1',i));
  w(sprintf('Cross_%d_2/1',i),sprintf('Cross_component_%d/2',i));
 end
 b('Mux','Omega_cross_H',[770 380 775 470],'Inputs','3');
 for i=1:3,w(sprintf('Cross_component_%d/1',i),sprintf('Omega_cross_H/%d',i));end
 b('Sum','Total_equivalent_moment',[800 40 830 85],'Inputs','+++-');
 w('aero_total_moment/1','Total_equivalent_moment/1');w('disturbance_moment/1','Total_equivalent_moment/2');
 w(torque,'Total_equivalent_moment/3');w('Omega_cross_H/1','Total_equivalent_moment/4');
 w('Total_equivalent_moment/1','Aircraft_6DOF/2');w('body_force/1','Aircraft_6DOF/1');
 vars={'Ve','Xe','Euler','DCM','Vb','omega','omega_dot'};
 for i=1:7
  b('Outport',vars{i},[1190 50*i 1220 50*i+20],'Port',num2str(i));w(sprintf('Aircraft_6DOF/%d',i),[vars{i} '/1']);
 end
 if enabled,b('Outport','gamma',[1190 460 1220 480],'Port','8');w('CMG/3','gamma/1');end
 ds=Simulink.SimulationData.Dataset;time=(0:.001:.8)';
 inputs={[2 3 -1],[.1 .2 .1],[0 0 0]};if enabled,inputs{4}=[.1 -.1 .05 -.05];end
 for i=1:numel(inputs),ds=ds.addElement(timeseries(repmat(inputs{i},numel(time),1),time));end
 si=Simulink.SimulationInput(m);si=si.setExternalInput(ds);r=sim(si);
 save(fullfile(out,[m '.mat']),'r');
 save_system(m,fullfile(root,'simulink','models',[m '.slx']));
 print(['-s' m],'-dpng',fullfile(out,[m '.png']));
 blocks=find_system(m,'SearchDepth',1,'Type','Block');
 refs=cellfun(@(x)get_param(x,'ReferenceBlock'),blocks,'UniformOutput',false);
 writetable(table(blocks,refs),fullfile(out,[m '_links.csv']));
 close_system(m,0);fprintf('%s SIM_PASS\n',m);
end
 function b(type,name,pos,varargin)
  if strcmp(type,'aero'),src='aerolib6dof/6DoF (Quaternion)';else,src=['built-in/' type];end
  add_block(src,[m '/' name],'Position',pos,varargin{:});
 end
 function w(src,dst),add_line(m,src,dst,'autorouting','on');end
end
