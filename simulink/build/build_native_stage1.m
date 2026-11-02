function build_native_stage1
% Rebuild and validate the genuine Aerospace Blockset rigid-body block.
root=native_root;
out=fullfile(root,'simulink','results','native_stage1');
td=fullfile(root,'simulink','tests');
if ~exist(out,'dir'), mkdir(out); end
if ~exist(td,'dir'), mkdir(td); end
m='native_6dof_validation';
if bdIsLoaded(m), close_system(m,0); end
load_system('aerolib6dof'); new_system(m);
set_param(m,'SolverType','Fixed-step','Solver','ode4','FixedStep','0.001', ...
 'StopTime','2','ReturnWorkspaceOutputs','on','SaveState','on', ...
 'StateSaveName','xout','SaveFormat','Dataset');
add_block('simulink/Sources/Constant',[m '/Force_B_N'],'Value','[0;0;0]','Position',[30 80 120 115]);
add_block('simulink/Sources/Constant',[m '/Moment_B_Nm'],'Value','[0;0;0]','Position',[30 230 120 265]);
b=[m '/Aircraft_6DOF'];
add_block('aerolib6dof/6DoF (Quaternion)',b,'Position',[235 60 445 330], ...
 'Mass','10','Inertia','diag([2 3 4])','xme_0','[0 0 0]', ...
 'Vm_0','[0 0 0]','eul_0','[0 0 0]','pm_0','[0 0 0]','k_quat','1');
add_line(m,'Force_B_N/1','Aircraft_6DOF/1');
add_line(m,'Moment_B_Nm/1','Aircraft_6DOF/2');
names={'Ve','Xe','Euler','DCM','Vb','omega','omega_dot'};
for k=1:7
 y=35+55*k;
 add_block('simulink/Sinks/To Workspace',[m '/' names{k}], ...
  'VariableName',names{k},'SaveFormat','Timeseries','MaxDataPoints','inf','Position',[570 y 690 y+25]);
 add_line(m,sprintf('Aircraft_6DOF/%d',k),[names{k} '/1'],'autorouting','on');
end
add_block('simulink/Sinks/Scope',[m '/Attitude_Scope'],'Position',[760 175 805 220]);
add_line(m,'Aircraft_6DOF/3','Attitude_Scope/1','autorouting','on');
set_param(m,'SimulationCommand','update');
actual_reference=get_param(b,'ReferenceBlock');
fprintf('ACTUAL_6DOF_REFERENCE %s\n',actual_reference);
assert(~isempty(actual_reference) && startsWith(actual_reference,'aerolib'),'Expected linked Aerospace block');
save_system(m,fullfile(td,'native_6dof_validation.slx'));
% Record wrapper and toolbox internals, preserving provenance of built-in Fcn blocks.
blocks=find_system(m,'LookUnderMasks','all','FollowLinks','on','Type','Block');
rows=cell(numel(blocks),6);
for k=1:numel(blocks)
 p=blocks{k}; typ=get_param(p,'BlockType');
 rows(k,:)={p,typ,get_param(p,'ReferenceBlock'),get_param(p,'Parent'), ...
 get_param(p,'MaskType'),ismember(typ,{'Fcn','S-Function','MATLABFcn'})};
end
writetable(cell2table(rows,'VariableNames',{'Name','Type','ReferenceBlock','Parent','Mask','CodeBlock'}),fullfile(out,'block_inventory.csv'));
fid=fopen(fullfile(out,'block_parameters.json'),'w');
fprintf(fid,'%s',jsonencode(struct('ReferenceBlock',get_param(b,'ReferenceBlock'), ...
 'Names',{get_param(b,'MaskNames')},'Values',{get_param(b,'MaskValues')},'Ports',get_param(b,'Ports')),'PrettyPrint',true)); fclose(fid);
% Principal-axis tests avoid nonlinear cross-axis terms and have exact solutions.
cases={'rest','coast','constant_force','constant_moment'};
metrics=zeros(4,5);
for c=1:4
 si=Simulink.SimulationInput(m);
 si=si.setBlockParameter([m '/Force_B_N'],'Value','[0;0;0]');
 si=si.setBlockParameter([m '/Moment_B_Nm'],'Value','[0;0;0]');
 si=si.setBlockParameter(b,'Vm_0','[0 0 0]');
 if c==2, si=si.setBlockParameter(b,'Vm_0','[3 -2 1]'); end
 if c==3, si=si.setBlockParameter([m '/Force_B_N'],'Value','[20;0;0]'); end
 if c==4, si=si.setBlockParameter([m '/Moment_B_Nm'],'Value','[0;0;4]'); end
 result=sim(si); t=result.Xe.Time; n=numel(t);
 v=zeros(n,3); x=v; w=v; angle=v;
 if c==2, v=repmat([3 -2 1],n,1); x=t*[3 -2 1]; end
 if c==3, v(:,1)=2*t; x(:,1)=t.^2; end
 if c==4, w(:,3)=t; angle(:,3)=0.5*t.^2; end
 % Extract the actual integrated quaternion, not a normalized reconstruction.
 states=result.xout; q=[];
 for j=1:states.numElements
  el=states.getElement(j); bp=el.BlockPath.convertToCell;
  if size(el.Values.Data,2)==4
   fprintf('Quaternion state candidate: %s\n',strjoin(bp,'/'));
   q=el.Values.Data;
  end
 end
 assert(~isempty(q),'Integrated quaternion state was not found');
 metrics(c,:)=[max(abs(result.Xe.Data-x),[],'all'),max(abs(result.Ve.Data-v),[],'all'), ...
 max(abs(result.omega.Data-w),[],'all'),max(abs(result.Euler.Data-angle),[],'all'),max(abs(vecnorm(q,2,2)-1))];
 assert(all(metrics(c,:)<1e-7),'Analytic validation failed: %s',cases{c});
 save(fullfile(out,[cases{c} '.mat']),'result','x','v','w','angle','q');
 T=table(t,result.Xe.Data,result.Ve.Data,result.omega.Data,result.Euler.Data,q, ...
 'VariableNames',{'t','position','velocity','omega','euler','quaternion'});
 writetable(T,fullfile(out,[cases{c} '.csv']));
end
T=array2table(metrics,'VariableNames',{'position_error_m','velocity_error_mps','omega_error_radps','euler_error_rad','quaternion_norm_error'},'RowNames',cases);
writetable(T,fullfile(out,'validation_metrics.csv'),'WriteRowNames',true); disp(T);
print(['-s' m],'-dpng',fullfile(out,'model.png'));
close_system(m,0);
fprintf('STAGE1_NUMERICAL_PASS\n');
end
