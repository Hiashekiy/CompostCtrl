function check_native_allocation
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));s=load(fullfile(root,'simulink','data','allocation_reference.mat'));
error=0;
for k=1:numel(s.cost)
 [u,res]=bounded_control_allocation(s.M(:,:,k),s.target(:,k),s.lower(:,k),s.upper(:,k));
 assert(all(u>=s.lower(:,k)-1e-10) && all(u<=s.upper(:,k)+1e-10));
 error=max(error,abs(norm(res)^2-s.cost(k))/(1+abs(s.cost(k))));
end
assert(error<1e-9);out=fullfile(root,'simulink','results','native_closedloop');
fid=fopen(fullfile(out,'allocation_report.json'),'w');fprintf(fid,'%s',jsonencode(struct('samples',numel(s.cost),'relative_objective_error',error)));fclose(fid);
fprintf('NATIVE_ALLOCATION_PASS error=%g\n',error);
end
