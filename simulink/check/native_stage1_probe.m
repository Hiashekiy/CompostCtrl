function native_stage1_probe
% Inspect installed libraries; no guessed block paths are used by the builder.
root=native_root;
out = fullfile(root,'simulink','results','native_stage1');
if ~exist(out,'dir'), mkdir(out); end
diary(fullfile(out,'environment.txt')); cleanup=onCleanup(@() diary('off'));
disp(version); products=ver; disp(struct2table(products));
writetable(struct2table(products),fullfile(out,'installed_products.csv'));
ids={'MATLAB','Simulink','Aerospace_Blockset','Aerospace_Toolbox','Simscape','SimMechanics','Optimization_Toolbox','Instrument_Control_Toolbox'};
licenseRows=cell(numel(ids),4);
for k=1:numel(ids)
 available=license('test',ids{k});
 [ok,msg]=license('checkout',ids{k});
 fprintf('LICENSE %s test=%d checkout=%d %s\n',ids{k},available,ok,msg);
 licenseRows(k,:)={ids{k},available,ok,msg};
end
writetable(cell2table(licenseRows,'VariableNames',{'Feature','Test','Checkout','Diagnostic'}),fullfile(out,'licenses.csv'));
libs={'simulink','aerolib6dof','aerolib','aeroblks','aerolibatmos','aerolibtransform','sm_lib'};
for k=1:numel(libs)
 try
  load_system(libs{k});
  b=find_system(libs{k},'SearchDepth',2,'LookUnderMasks','none','Type','Block');
  fprintf('LIBRARY %s\n',libs{k}); disp(b);
 catch e, fprintf('UNAVAILABLE %s: %s\n',libs{k},e.message); end
end
if bdIsLoaded('aerolib6dof')
 b=find_system('aerolib6dof','LookUnderMasks','none','RegExp','on','Name','.*Quaternion.*');
 for k=1:numel(b)
  fprintf('CANDIDATE %s\n',b{k});
  disp(get_param(b{k},'DialogParameters'));
  disp(get_param(b{k},'MaskNames')); disp(get_param(b{k},'MaskValues'));
  disp(get_param(b{k},'Ports'));
  meta=struct('Path',b{k},'LibraryVersion',get_param('aerolib6dof','Version'), ...
   'Names',{get_param(b{k},'MaskNames')},'Values',{get_param(b{k},'MaskValues')}, ...
   'Ports',get_param(b{k},'Ports'));
  fid=fopen(fullfile(out,'library_6dof_metadata.json'),'w');
  fprintf(fid,'%s',jsonencode(meta,'PrettyPrint',true)); fclose(fid);
  ports=find_system(b{k},'SearchDepth',1,'LookUnderMasks','all','RegExp','on','BlockType','Inport|Outport');
  for j=1:numel(ports)
   fprintf('PORT %s %s %s\n',get_param(ports{j},'BlockType'),get_param(ports{j},'Port'),strrep(ports{j},newline,' '));
  end
 end
end
end
