function revision_controller_audit(old,out)
% Compare stored controller templates including weights, independent of dts.
ids={'V3_S','V3_L','V3_Z','V3_H05_Z','V3_H08_M','V3_H08_Z','V3_H12_M','V3_H12_Z','V3_H16_M','V3_H16_Z'};
base=[];rows={};initial=[];current_reference=[];
for j=1:numel(ids)
 root=out;if j<=3,root=old;end;folder=fullfile(root,'E3/cases',ids{j});b=jsondecode(fileread(fullfile(folder,'build_config.json')));
 load_system(b.model_path);cleanup=onCleanup(@()close_system(b.model,0));mw=get_param(b.model,'ModelWorkspace');template=getVariable(mw,'native_controller_template');template=rmfield(template,'dts');
 if isempty(base),base=template;else,assert(isequaln(template,base),'revision:template','Controller template differs beyond prediction dts: %s',ids{j});end
 bank=load(fullfile(folder,'reference_bank.mat'));if isempty(initial),initial=bank.x0;else,assert(isequaln(initial,bank.x0));end
 nowq=squeeze(bank.q_ref(:,1,:));nowomega=squeeze(bank.omega_ref(:,1,:));nowdist=squeeze(bank.tau_d(:,1,:));
 if j==2,current_reference=nowq;end
 if j>=4 && endsWith(ids{j},'_M'),assert(isequaln(nowq,current_reference),'Current physical reference differs across M horizons');end
 baseline=load(fullfile(old,'E3/cases/V3_L/reference_bank.mat'),'tau_d','omega_ref');assert(isequaln(nowdist,squeeze(baseline.tau_d(:,1,:))));
 rows{end+1}=struct('id',ids{j},'same_template_except_dts',true,'model_sha256',paper_hash(b.model_path),'bank_sha256',paper_hash(fullfile(folder,'reference_bank.mat')));
 clear cleanup
end
paper_json(fullfile(out,'reports/E3_controller_audit.json'),struct('passed',true,'weights_initial_nominals_and_constraints_identical',true,'physical_current_reference_M_identical',true,'initial_state_and_current_disturbance_identical',true,'allowed_template_difference','dts only','models',{rows}));
end
