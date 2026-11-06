function export_e2_gimbal_pairs(out)
% Pointwise paired differences computed before any display subsampling.
T=readtable(fullfile(out,'manifest.csv'),'TextType','string');
for task=1:2
 data=cell(1,2);for method=1:2,id=sprintf('E2R_M%d_%d',task,method);row=T(T.case_id==id,:);d=load(fullfile(out,char(row.raw_path)),'r');data{method}=struct('t',d.r.states.Time(:),'x',squeeze(d.r.states.Data));end
 assert(isequal(data{1}.t,data{2}.t));t=data{1}.t;delta=rad2deg(data{2}.x(:,7:10)-data{1}.x(:,7:10));n=numel(t);m=floor(n/20);idx=[1:20:n,n];for j=1:4,A=reshape(delta(1:m*20,j),20,m);[~,a]=min(A,[],1);[~,b]=max(A,[],1);idx=[idx,(0:m-1)*20+a,(0:m-1)*20+b];end;idx=unique(idx);writetable(array2table([t(idx) delta(idx,:)],'VariableNames',{'time_s','dg1_deg','dg2_deg','dg3_deg','dg4_deg'}),fullfile(out,'.work',sprintf('E2_config_pair_%d.csv',task)));
end
end
