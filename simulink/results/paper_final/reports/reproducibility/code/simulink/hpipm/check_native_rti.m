function check_native_rti
hpipm_startup;root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
out=fullfile(root,'simulink','tests','05_python_matlab_qp_equivalence');s=load(fullfile(out,'production_online_qp.mat'));
[command,state,info]=native_rti_step(s,s.X_nom(:,1),[]);
error=max(abs(command-s.online_command_python(:)));disp(info);disp([command s.online_command_python(:)]);disp(error);assert(error<1e-6 && ~info.fallback);
[fallback,~,failed]=native_rti_step(s,s.X_nom(:,1),state,true);
assert(failed.fallback && max(abs(fallback-command))<1e-10,'Previous feasible fallback differs');
[cold,~,coldinfo]=native_rti_step(s,s.X_nom(:,1),[],true);
assert(coldinfo.fallback && all(isfinite(cold)) && all(cold(1:3)>=s.tau_min(:,1)) && all(cold(1:3)<=s.tau_max(:,1)));
[warm,~,warminfo]=native_rti_step(s,s.X_nom(:,1),state);
assert(all(isfinite(warm)) && ~warminfo.fallback);
report=struct('first_command_error',error,'previous_fallback_pass',true,'cold_fallback_pass',true,'warm_start_pass',true);
fid=fopen(fullfile(out,'online_rti_report.json'),'w');fprintf(fid,'%s',jsonencode(report,'PrettyPrint',true));fclose(fid);
disp(report);fprintf('ONLINE_RTI_TEST_PASS\n');
end
