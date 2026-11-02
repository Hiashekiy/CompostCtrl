function report_native_p0_matrix
root=native_root;out=fullfile(root,'simulink','results','native_closedloop');
p=jsondecode(fileread(fullfile(out,'p0_matrix_progress.json')));assert(numel(p)==16,'P0 sweep incomplete');
rows=cell(16,6);
for k=1:numel(p)
 if iscell(p),entry=p{k};else,entry=p(k);end
 name=['NearSpace_CMG_AttitudeValidation'];if strcmp(entry.arm,'A'),name=[name '_SurfaceOnly'];end
 name=[name '_' entry.case];load_system(fullfile(root,'simulink','cases',[name '.slx']));
 mw=get_param(name,'ModelWorkspace');bank=getVariable(mw,'native_reference_bank');
 s=load(fullfile(out,sprintf('%s_attitude_%gs_%s.mat',entry.arm,entry.duration_s,entry.case)));r=s.r;t=r.states.Time;
 x=squeeze(r.states.Data);if size(x,1)~=numel(t),x=x';end
 a=x(:,1)/2;b=x(:,2)/2;c=x(:,3)/2;
 q=[cos(a).*cos(b).*cos(c)+sin(a).*sin(b).*sin(c),sin(a).*cos(b).*cos(c)-cos(a).*sin(b).*sin(c), ...
 cos(a).*sin(b).*cos(c)+sin(a).*cos(b).*sin(c),cos(a).*cos(b).*sin(c)-sin(a).*sin(b).*cos(c)];
 ref=interp1(bank.time(:),squeeze(bank.q_ref(:,1,:))',t);ref=ref./vecnorm(ref,2,2);
 angle=2*acos(min(1,abs(sum(q.*ref,2))))*180/pi;
 runtime=jsondecode(fileread(fullfile(out,[name '_runtime.json'])));
 rows(k,:)={entry.case,entry.arm,t(end),max(angle),sqrt(mean(angle.^2)),runtime.fallbacks};close_system(name,0);
end
table=cell2table(rows,'VariableNames',{'Case','Arm','Duration_s','MaxError_deg','RMSError_deg','Fallbacks'});
writetable(table,fullfile(out,'p0_matrix_summary.csv'));disp(table);fprintf('P0_MATRIX_REPORT_PASS\n');
end
