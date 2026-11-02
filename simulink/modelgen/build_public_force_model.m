function build_public_force_model
root=repo_root;addpath(fullfile(root,'simulink','data'));p=vehicle_params;
table=load(fullfile(root,'simulink','data','public_surrogate','force_polar.mat'));
m='Surrogate_Aero_Forces';if bdIsLoaded(m),close_system(m,0);end
new_system(m);set_param(m,'SolverType','Fixed-step','Solver','FixedStepDiscrete','FixedStep','.001','StopTime','0','ReturnWorkspaceOutputs','on');
b('Inport','qbar_alpha_beta',[20 90 50 110],'PortDimensions','3');b('Demux','Inputs',[95 40 100 180],'Outputs','3');w('qbar_alpha_beta/1','Inputs/1');
for j=1:2
 names={'CL','CD'};name=names{j};
 b('Lookup_n-D',name,[180 65*j 320 65*j+30],'NumberOfTableDimensions','1', ...
  'BreakpointsForDimension1',mat2str(table.alpha_rad,17),'Table',mat2str(table.(name),17));
 w('Inputs/2',[name '/1']);b('Gain',[name '_S'],[370 65*j 440 65*j+30],'Gain',num2str(p.vehicle.S,17));w([name '/1'],[name '_S/1']);
 b('Product',[name '_force'],[490 65*j 520 65*j+30]);w([name '_S/1'],[name '_force/1']);w('Inputs/1',[name '_force/2']);
end
for item={'sin_alpha','cos_alpha','sin_beta','cos_beta'}
 name=item{1};if startsWith(name,'sin'),op='sin';else,op='cos';end
 b('Trigonometry',name,[180 240+50*find(strcmp(item{1},{'sin_alpha','cos_alpha','sin_beta','cos_beta'})) 260 270+50*find(strcmp(item{1},{'sin_alpha','cos_alpha','sin_beta','cos_beta'}))],'Operator',op);
 if endsWith(name,'alpha'),src='Inputs/2';else,src='Inputs/3';end;w(src,[name '/1']);
end
terms={'Dx',{'CD_force/1','cos_alpha/1','cos_beta/1'};'Lx',{'CL_force/1','sin_alpha/1'}; ...
 'Dy',{'CD_force/1','sin_beta/1'};'Dz',{'CD_force/1','sin_alpha/1','cos_beta/1'};'Lz',{'CL_force/1','cos_alpha/1'}};
for j=1:size(terms,1)
 name=terms{j,1};sources=terms{j,2};b('Product',name,[600 60*j 635 60*j+30],'Inputs',num2str(numel(sources)));
 for k=1:numel(sources),w(sources{k},sprintf('%s/%d',name,k));end
end
b('Sum','Fx',[710 90 740 125],'Inputs','-+');w('Dx/1','Fx/1');w('Lx/1','Fx/2');
b('Gain','Fy',[710 180 760 210],'Gain','-1');w('Dy/1','Fy/1');
b('Sum','Fz',[710 270 740 305],'Inputs','--');w('Dz/1','Fz/1');w('Lz/1','Fz/2');
b('Mux','Force_body',[820 80 825 320],'Inputs','3');for j=1:3,names={'Fx','Fy','Fz'};w([names{j} '/1'],sprintf('Force_body/%d',j));end
b('Outport','F_B',[950 120 980 140]);w('Force_body/1','F_B/1');
b('ToWorkspace','force_log',[920 300 1040 330],'VariableName','force','SaveFormat','Timeseries','MaxDataPoints','inf');w('Force_body/1','force_log/1');
% Explicit guard: lookup data must not silently extrapolate.
for j=1:2
 bounds=[table.alpha_rad(1) table.alpha_rad(end)];
 b('Constant',sprintf('Alpha_bound_%d',j),[20 470+60*j 100 500+60*j],'Value',num2str(bounds(j),17));
 operators={'>=','<='};b('RelationalOperator',sprintf('In_domain_%d',j),[180 470+60*j 240 500+60*j],'Operator',operators{j});
 w('Inputs/2',sprintf('In_domain_%d/1',j));w(sprintf('Alpha_bound_%d/1',j),sprintf('In_domain_%d/2',j));
end
b('Logic','Domain_valid',[330 530 390 570],'Operator','AND','Inputs','2');
w('In_domain_1/1','Domain_valid/1');w('In_domain_2/1','Domain_valid/2');
add_block('simulink/Model Verification/Assertion',[m '/Assert_alpha_domain'],'Position',[450 530 570 565]);w('Domain_valid/1','Assert_alpha_domain/1');
err=0;
for a=deg2rad(-40:2:40)
 for beta=deg2rad([-5 0 5])
  qbar=1000;alpha_deg=rad2deg(a);L=qbar*p.vehicle.S*(-.20704+.029244*alpha_deg);D=qbar*p.vehicle.S*(.07854-.0061592*alpha_deg+.000621408*alpha_deg^2);
  expected=[-D*cos(a)*cos(beta)+L*sin(a);-D*sin(beta);-D*sin(a)*cos(beta)-L*cos(a)];
  si=Simulink.SimulationInput(m);si=si.setExternalInput([0 qbar a beta]);r=sim(si);err=max(err,max(abs(r.force.Data(:)-expected)));
 end
end
assert(err<1e-8);out=fullfile(root,'simulink','results','public_surrogate');if ~exist(out,'dir'),mkdir(out);end
save(fullfile(out,'validation.mat'),'err');save_system(m,fullfile(root,'simulink','models',[m '.slx']));print(['-s' m],'-dpng',fullfile(out,'model.png'));close_system(m,0);fprintf('PUBLIC_FORCE_MODEL_PASS error=%g\n',err);
 function b(type,name,pos,varargin),add_block(['built-in/' type],[m '/' name],'Position',pos,varargin{:});end
 function w(src,dst),add_line(m,src,dst,'autorouting','on');end
end
