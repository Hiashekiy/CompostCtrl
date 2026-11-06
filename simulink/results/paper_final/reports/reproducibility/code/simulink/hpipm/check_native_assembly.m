function check_native_assembly
hpipm_startup;root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
out=fullfile(root,'simulink','tests','05_python_matlab_qp_equivalence');
t=load(fullfile(out,'production_qp.mat'));s=assemble_native_qp(t,t.X_nom,t.U_nom);
fields={'A','B','b','Q','q','R','r','lu','uu'};err=struct;
for k=1:numel(fields),f=fields{k};err.(f)=max(abs(s.(f)-t.(f)),[],'all');end
assert(all(structfun(@(v)v<1e-9,err)),'Online QP assembly differs from Python');
r=solve_hpipm_snapshot(s);err.solution=max(abs(r.U-t.dU_python),[],'all');assert(err.solution<1e-7);
fid=fopen(fullfile(out,'assembly_report.json'),'w');fprintf(fid,'%s',jsonencode(err,'PrettyPrint',true));fclose(fid);
disp(err);fprintf('NATIVE_QP_ASSEMBLY_PASS\n');
end
