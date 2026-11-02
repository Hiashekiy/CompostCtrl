function build_hpipm_interface
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
out=fullfile(root,'simulink','results','hpipm_interface');if ~exist(out,'dir'),mkdir(out);end
file=fullfile(root,'simulink','tests','05_python_matlab_qp_equivalence','production_qp.mat');
s=load(file);packed=hpipm_pack_qp(s);hpipm_startup;
m='HPIPM_Interface';if bdIsLoaded(m),close_system(m,0);end
new_system(m);set_param(m,'SolverType','Fixed-step','Solver','FixedStepDiscrete', ...
 'FixedStep','.2','StopTime','.2','ReturnWorkspaceOutputs','on');
% All stage data arrive through one explicit input. The template supplies dimensions only.
add_block('built-in/Inport',[m '/Packed_OCP_QP'],'PortDimensions',num2str(numel(packed)),'Position',[25 100 55 120]);
add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function',[m '/HPIPM_Solver_Interface'], ...
 'FunctionName','hpipm_interface_sfun', ...
 'Position',[160 65 370 170]);
add_line(m,'Packed_OCP_QP/1','HPIPM_Solver_Interface/1');
vars={'first_control_increment','status','residuals'};
for k=1:3
 add_block('built-in/ToWorkspace',[m '/' vars{k}],'VariableName',vars{k}, ...
  'SaveFormat','Timeseries','MaxDataPoints','inf','Position',[500 50+65*k 660 80+65*k]);
 add_line(m,sprintf('HPIPM_Solver_Interface/%d',k),[vars{k} '/1'],'autorouting','on');
end
si=Simulink.SimulationInput(m);si=si.setExternalInput([0 packed';.2 packed']);r=sim(si);
value=squeeze(r.first_control_increment.Data);
if size(value,1)==double(s.nu),value=value';end
error=max(abs(value-s.dU_python(:,1)'),[],'all');
assert(error<1e-7 && all(r.status.Data==0,'all'));
save(fullfile(out,'simulation.mat'),'r','error');
save_system(m,fullfile(root,'simulink','models',[m '.slx']));
print(['-s' m],'-dpng',fullfile(out,'model.png'));
fid=fopen(fullfile(out,'report.json'),'w');fprintf(fid,'%s',jsonencode(struct('first_control_error',error,'status',0,'sample_time',.2)));fclose(fid);
close_system(m,0);fprintf('SIMULINK_HPIPM_INTERFACE_PASS error=%g\n',error);
end
