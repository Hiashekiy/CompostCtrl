function build_cmg_geometry
% Native algebraic CMG geometry only; drive dynamics are a separate pending step.
root=repo_root;
addpath(fullfile(root,'simulink','data'));
p=vehicle_params;
d=fullfile(root,'simulink','data','native_baseline','A4_descent_replay_S00_with_cmg');
ref=load(fullfile(d,'cmg_mapping_reference.mat'));
out=fullfile(root,'simulink','results','native_cmg_geometry');
models=fullfile(root,'simulink','models');
if ~exist(out,'dir'), mkdir(out); end
if ~exist(models,'dir'), mkdir(models); end
m='CMG_Geometry'; if bdIsLoaded(m), close_system(m,0); end
load_system('simulink'); new_system(m);
set_param(m,'SolverType','Fixed-step','Solver','FixedStepDiscrete','FixedStep','0.001', ...
 'StopTime','0','ReturnWorkspaceOutputs','on');
for k=1:4
 y=40+180*(k-1);
 block('Constant',sprintf('gamma%d',k),[20 y 65 y+25],'Value','0');
 block('Constant',sprintf('nu%d',k),[610 y+95 655 y+120],'Value','0');
 block('Trigonometry',sprintf('sin%d',k),[110 y 155 y+25],'Operator','sin');
 block('Trigonometry',sprintf('cos%d',k),[110 y+65 155 y+90],'Operator','cos');
 wire(sprintf('gamma%d/1',k),sprintf('sin%d/1',k));
 wire(sprintf('gamma%d/1',k),sprintf('cos%d/1',k));
 % s = s0 cos(gamma) + (g x s0) sin(gamma)
 % Acol = h0*(-s0 sin(gamma) + (g x s0) cos(gamma))
 vectors={p.cmg.h0*p.gxs0(:,k),p.cmg.h0*p.s0(:,k), ...
          -p.cmg.h0*p.s0(:,k),p.cmg.h0*p.gxs0(:,k)};
 for j=1:4
  name=sprintf('K%d_%d',k,j);
  block('Gain',name,[220 y+30*(j-1) 340 y+23+30*(j-1)], ...
   'Gain',mat2str(vectors{j},17),'Multiplication','Matrix(K*u)');
  if ismember(j,[1 3]), src=sprintf('sin%d/1',k); else, src=sprintf('cos%d/1',k); end
  wire(src,[name '/1']);
 end
 block('Sum',sprintf('H%d',k),[400 y+10 425 y+40],'Inputs','++');
 block('Sum',sprintf('A%d',k),[460 y+80 485 y+110],'Inputs','++');
 wire(sprintf('K%d_1/1',k),sprintf('H%d/1',k));
 wire(sprintf('K%d_2/1',k),sprintf('H%d/2',k));
 wire(sprintf('K%d_3/1',k),sprintf('A%d/1',k));
 wire(sprintf('K%d_4/1',k),sprintf('A%d/2',k));
 block('Product',sprintf('Adot%d',k),[700 y+70 730 y+100]);
 wire(sprintf('A%d/1',k),sprintf('Adot%d/1',k));
 wire(sprintf('nu%d/1',k),sprintf('Adot%d/2',k));
end
block('Sum','H_sum',[820 100 850 160],'Inputs','++++');
block('Mux','A_columns',[820 250 825 340],'Inputs','4');
block('Reshape','A_matrix',[890 270 960 310],'OutputDimensionality','Customize','OutputDimensions','[3 4]');
block('Sum','Hdot_sum',[820 460 850 520],'Inputs','++++');
block('Gain','Reaction_sign',[900 470 950 505],'Gain','-1');
for k=1:4
 wire(sprintf('H%d/1',k),sprintf('H_sum/%d',k));
 wire(sprintf('A%d/1',k),sprintf('A_columns/%d',k));
 wire(sprintf('Adot%d/1',k),sprintf('Hdot_sum/%d',k));
end
wire('A_columns/1','A_matrix/1'); wire('Hdot_sum/1','Reaction_sign/1');
sources={'H_sum/1','A_matrix/1','Reaction_sign/1'}; vars={'H','A','torque'};
for j=1:3
 block('ToWorkspace',vars{j},[1040 130*j 1130 130*j+30], ...
  'VariableName',vars{j},'SaveFormat','Timeseries'); wire(sources{j},[vars{j} '/1']);
end
set_param(m,'SimulationCommand','update');
save_system(m,fullfile(models,[m '.slx']));
errors=zeros(size(ref.gamma,1),3);
for i=1:size(ref.gamma,1)
 si=Simulink.SimulationInput(m);
 for k=1:4
  si=si.setBlockParameter([m sprintf('/gamma%d',k)],'Value',num2str(ref.gamma(i,k),17));
  si=si.setBlockParameter([m sprintf('/nu%d',k)],'Value',num2str(ref.nu(i,k),17));
 end
 r=sim(si);
 expected={ref.H(i,:)',squeeze(ref.A(i,:,:)),ref.torque(i,:)'};
 for j=1:3
  value=r.get(vars{j}); actual=squeeze(value.Data);
  errors(i,j)=max(abs(actual(:)-expected{j}(:)));
 end
end
assert(all(errors<1e-10,'all'),'CMG geometry reference mismatch');
writetable(array2table(errors,'VariableNames',{'H_error','A_error','torque_error'}),fullfile(out,'errors.csv'));
blocks=find_system(m,'Type','Block');
types=cellfun(@(b)get_param(b,'BlockType'),blocks,'UniformOutput',false);
assert(~any(ismember(types,{'Fcn','S-Function','MATLABFcn'})));
writetable(table(blocks,types,'VariableNames',{'Block','Type'}),fullfile(out,'block_inventory.csv'));
print(['-s' m],'-dpng',fullfile(out,'model.png'));
save(fullfile(out,'verification.mat'),'errors');
disp(max(errors)); fprintf('CMG_GEOMETRY_PASS samples=%d\n',size(ref.gamma,1));
close_system(m,0);
 function block(type,name,pos,varargin)
  add_block(['built-in/' type],[m '/' name],'Position',pos,varargin{:});
 end
 function wire(src,dst)
  add_line(m,src,dst,'autorouting','on');
 end
end
