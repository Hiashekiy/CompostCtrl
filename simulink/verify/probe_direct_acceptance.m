function probe_direct_acceptance(folders)
% Q1 of the acceptance study: what did the bcd96fd direct-servo controller ACTUALLY emit?
% Reads only saved per-cycle diagnostics; no simulation is run.
root=repo_root;base=fullfile(root,'simulink','results','online_authority');
if nargin<1||isempty(folders),folders={'L2_60s_20261104_232924','L2_60s_20261104_232950'};end
for f=1:numel(folders)
 folder=fullfile(base,folders{f});
 files=dir(fullfile(folder,'*_direct_authority.mat'));
 if isempty(files),fprintf('%s: no direct diagnostics\n',folders{f});continue;end
 d=load(fullfile(folder,files(1).name));infos=d.diagnostics;t=d.control_times;
 first=zeros(3,numel(infos));accepted=zeros(1,numel(infos));qp=zeros(1,numel(infos));
 stop=cell(1,numel(infos));delta1=zeros(3,numel(infos));
 for k=1:numel(infos)
  a=infos{k}.authority;
  if ~isempty(a),first(:,k)=a.command(:,1);delta1(:,k)=a.delta(:,1);end
  accepted(k)=infos{k}.accepted_steps;qp(k)=infos{k}.qp_solved;
  if isfield(infos{k},'early_stop_reason'),stop{k}=infos{k}.early_stop_reason;end
 end
 moved=zeros(1,numel(infos));
 for k=2:numel(infos),moved(k)=norm(first(:,k)-first(:,k-1),inf)>1e-9;end
 names=unique(stop);counts=zeros(1,numel(names));
 for k=1:numel(names),counts(k)=sum(strcmp(stop,names{k}));end
 fprintf('== %s cycles=%d\n',folders{f},numel(infos));
 fprintf('   accepted_steps: mean=%.3f zero_cycles=%d (%.1f%%) max=%d\n',mean(accepted),sum(accepted==0), ...
  100*mean(accepted==0),max(accepted));
 fprintf('   first-command CHANGED vs previous cycle: %d cycles (%.1f%%)\n',sum(moved),100*mean(moved));
 fprintf('   |first command| max=%.4f rad, mean=%.4f rad; measured delta mean=%.4f rad\n', ...
  max(abs(first(:))),mean(abs(first(:))),mean(abs(delta1(:))));
 fprintf('   QP per cycle: mean=%.3f max=%d\n',mean(qp),max(qp));
 for k=1:numel(names),fprintf('   stop %-24s %d\n',names{k},counts(k));end
 fprintf('   first-command trajectory (every 10 s): ');
 for k=1:50:numel(infos),fprintf('t=%.1f[%.3f %.3f %.3f] ',t(k),first(:,k));end
 fprintf('\n');
end
end
