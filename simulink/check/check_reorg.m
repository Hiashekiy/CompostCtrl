function check_reorg
% Verify the role-based simulink/ layout: every moved script must parse and resolve
% from the expected folder, and the generated models must be where scripts look.
root=native_root;
groups={'build',{'build_native_actuators_aero','build_native_closedloop','build_native_cmg_geometry', ...
                 'build_native_integrations','build_native_plant','build_native_stage1', ...
                 'build_public_force_model','build_surrogate_flight_forces'}; ...
        'check',{'check_cmg_actuation','check_env','check_native_integrations','check_p0_case', ...
                 'export_native_plant_logs','finalize_native_delivery','native_stage1_probe', ...
                 'report_native_closedloop','report_native_p0_matrix'}};
bad=0;
for g=1:size(groups,1)
 folder=groups{g,1};names=groups{g,2};
 for k=1:numel(names)
  f=fullfile(root,'simulink',folder,[names{k} '.m']);
  if exist(f,'file')~=2,fprintf('MISSING FILE %s\n',f);bad=bad+1;continue;end
  % nargin(name) parses the file without executing it; a syntax error throws here
  try
   nargin(names{k});
  catch err
   if startsWith(err.identifier,'MATLAB:m_')||contains(err.message,'Parse error')
    fprintf('PARSE FAIL   %-28s %s\n',names{k},err.message);bad=bad+1;continue;
   end
   rethrow(err);
  end
  w=which(names{k});
  if isempty(w)||~contains(w,folder)
   fprintf('RESOLVE FAIL %-28s -> %s\n',names{k},w);bad=bad+1;
  end
 end
end
nmodels=numel(dir(fullfile(root,'simulink','models','*.slx')));
ncases=numel(dir(fullfile(root,'simulink','cases','*.slx')));
fprintf('models=%d cases=%d\n',nmodels,ncases);
if ncases~=16,fprintf('CASE COUNT wrong: %d\n',ncases);bad=bad+1;end
for f={'models/NearSpace_CMG_FlightControl.slx','models/NearSpace_CMG_AttitudeValidation.slx', ...
       'cases/NearSpace_CMG_AttitudeValidation_S00_pitch30.slx', ...
       'cases/NearSpace_CMG_AttitudeValidation_SurfaceOnly_S07_sequential.slx'}
 % exist returns 2 for a plain file but 4 for a Simulink model that is on the path
 p=fullfile(root,'simulink',f{1});
 if ~any(exist(p,'file')==[2 4]),fprintf('MISSING MODEL %s\n',f{1});bad=bad+1;end
end
assert(bad==0,'layout check found %d problems',bad);
fprintf('SIMULINK_LAYOUT_PASS build=%d check=%d models=%d cases=%d\n', ...
 size(groups{1,2},2),size(groups{2,2},2),nmodels,ncases);
end
