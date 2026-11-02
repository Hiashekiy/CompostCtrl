function check_hpipm_snapshot
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
out=fullfile(root,'simulink','tests','05_python_matlab_qp_equivalence');
s=load(fullfile(out,'production_qp.mat'));
r=solve_hpipm_snapshot(s);
r.control_error=max(abs(r.U-s.dU_python),[],'all');
r.state_error=max(abs(r.X-s.dX_python),[],'all');
r.objective_error=abs(r.objective-s.objective_python);
assert(r.status==0 && r.matrix_error<1e-12 && r.control_error<1e-7 && r.state_error<1e-7);
assert(r.objective_error<1e-7*(1+abs(s.objective_python)) && max(r.residuals)<1e-6);
save(fullfile(out,'matlab_solution.mat'),'r');
fid=fopen(fullfile(out,'matlab_report.json'),'w');
summary=rmfield(r,{'X','U'});fprintf(fid,'%s',jsonencode(summary,'PrettyPrint',true));fclose(fid);
disp(summary); fprintf('PRODUCTION_QP_EQUIVALENCE_PASS\n');
end
