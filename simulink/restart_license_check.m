function restart_license_check
% Fresh-session license checkout and a real Aerospace 6DOF smoke simulation.
root=fileparts(fileparts(mfilename('fullpath')));
out=fullfile(root,'simulink','results','license_recheck',datestr(now,'yyyymmdd_HHMMSS'));
mkdir(out);diary(fullfile(out,'check.txt'));guard=onCleanup(@()diary('off'));
report=struct('version',version,'time',datestr(now,31),'matlabroot',matlabroot);
features={'MATLAB','Simulink','Aerospace_Toolbox','Aerospace_Blockset'};
for k=1:numel(features)
 f=features{k};[ok,msg]=license('checkout',f);
 report.(f)=struct('checkout',ok,'message',msg);
 fprintf('%s checkout=%d\n%s\n',f,ok,msg);
end
report.block_create=false;report.simulation=false;
m='license_recheck_6dof';
try
 load_system('simulink');load_system('aerolib6dof');
 if bdIsLoaded(m),close_system(m,0);end
 new_system(m);
 add_block('aerolib6dof/6DoF (Quaternion)',[m '/Aircraft_6DOF'],'Position',[220 50 440 300]);
 report.block_create=true;
 for k=1:2
  n=sprintf('Zero_%d',k);
  add_block('built-in/Constant',[m '/' n],'Value','[0;0;0]','Position',[30 70+100*k 90 100+100*k]);
  add_line(m,[n '/1'],sprintf('Aircraft_6DOF/%d',k));
 end
 for k=1:7
  n=sprintf('Output_%d',k);add_block('built-in/Outport',[m '/' n],'Port',num2str(k),'Position',[560 40*k 590 40*k+20]);
  add_line(m,sprintf('Aircraft_6DOF/%d',k),[n '/1']);
 end
 set_param(m,'SolverType','Fixed-step','Solver','ode4','FixedStep','.001','StopTime','.1', ...
  'ReturnWorkspaceOutputs','on','SaveFormat','Dataset');
 r=sim(m);report.simulation=true;
 save(fullfile(out,'smoke_simulation.mat'),'r');save_system(m,fullfile(out,[m '.slx']));
 open_system(m);fprintf('REAL_6DOF_CREATE_AND_SIM_PASS\n');
catch e
 report.error=getReport(e,'extended','hyperlinks','off');fprintf('%s\n',report.error);
 if bdIsLoaded(m),close_system(m,0);end
end
fid=fopen(fullfile(out,'report.json'),'w');fprintf(fid,'%s',jsonencode(report,'PrettyPrint',true));fclose(fid);
fprintf('LICENSE_RECHECK_REPORT %s\n',out);
try,slLibraryBrowser;catch e,fprintf('Library Browser: %s\n',e.message);end
end
