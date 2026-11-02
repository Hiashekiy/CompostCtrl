function finalize_native_delivery
% Refresh default A/B models after case sweeps, and audit their reloadability.
root=native_root;native_startup;
build_native_closedloop(2,true,true);build_native_closedloop(2,true,false);
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
 writetable(cell2table(rows,'VariableNames',{'Name','Type','ReferenceBlock','Parent','Mask'}),fullfile(out,[name '_inventory.csv']));
 print(['-s' name],'-dpng',fullfile(out,[name '.png']));close_system(name,0);
end
report_native_closedloop;fprintf('NATIVE_DELIVERY_RELOAD_PASS\n');
end
