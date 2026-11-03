function finalize_delivery
% Refresh default A/B models after case sweeps, and audit their reloadability.
root=repo_root;init_paths;
build_closedloop(2,true,true);build_closedloop(2,true,false);
models={'NearSpace_CMG_FlightControl','NearSpace_CMG_FlightControl_SurfaceOnly'};
out=fullfile(root,'simulink','results','native_delivery');if ~exist(out,'dir'),mkdir(out);end
for k=1:numel(models)
 name=models{k};file=fullfile(root,'simulink','models',[name '.slx']);load_system(file);
 mw=get_param(name,'ModelWorkspace');assert(hasVariable(mw,'native_controller_template') && hasVariable(mw,'native_reference_bank'));
 set_param(name,'StopTime','60');save_system(name);
 si=Simulink.SimulationInput(name);si=si.setModelParameter('StopTime','.4');r=sim(si);assert(all(isfinite(r.states.Data),'all'));
 blocks=find_system(name,'LookUnderMasks','all','FollowLinks','on','Type','Block');rows=cell(numel(blocks),5);
 for j=1:numel(blocks)
  b=blocks{j};rows(j,:)={b,get_param(b,'BlockType'),get_param(b,'ReferenceBlock'),get_param(b,'Parent'),get_param(b,'MaskType')};
 end
 retry_write(@() writetable(cell2table(rows,'VariableNames',{'Name','Type','ReferenceBlock','Parent','Mask'}), ...
  fullfile(out,[name '_inventory.csv'])),[name '_inventory.csv']);
 retry_write(@() print(['-s' name],'-dpng',fullfile(out,[name '.png'])),[name '.png']);close_system(name,0);
end
report_closedloop;fprintf('NATIVE_DELIVERY_RELOAD_PASS\n');
end
function retry_write(fn,label)
% Evidence files can be held for a few seconds by the MATLAB process that just
% closed; retry instead of aborting the delivery audit.
for attempt=1:5
 try
  fn();return;
 catch err
  if attempt==5
   warning('EVIDENCE_WRITE_SKIPPED %s (%s)',label,err.identifier);return;
  end
  fprintf('WRITE_RETRY %d %s\n',attempt,label);pause(10);
 end
end
end
