function plot_paper_figures(T,out)
folder=fullfile(out,'figures');if ~isfolder(folder),mkdir(folder);end
for exp={'E1','E2','E3','E4'}
 S=T(strcmp(T.experiment,exp{1}),:);if isempty(S),continue;end;writetable(S,fullfile(out,[exp{1} '_summary.csv']));
end
S=T(strcmp(T.experiment,'E1') & strcmp(T.profile,'step'),:);if ~isempty(S)
for cmg=[false true]
 M=S(S.cmg==cmg,:);a=unique(M.amplitude);d=unique(M.transition);Z=nan(numel(a),numel(d));for k=1:height(M),i=find(a==M.amplitude(k));j=find(d==M.transition(k));if strcmp(M.status{k},'SUCCESS'),Z(i,j)=1;elseif strcmp(M.status{k},'TRACKING_FAIL'),Z(i,j)=0;else,Z(i,j)=-1;end;end
 f=figure('Visible','off');imagesc(d,a,Z);set(gca,'YDir','normal');xlabel('Transition duration (s)');ylabel('Amplitude (deg)');title(sprintf('CMG=%d; -1=domain/technical, 0=tracking fail, 1=success',cmg));colorbar;exportgraphics(f,fullfile(folder,sprintf('E1_grid_%d.png',cmg)));exportgraphics(f,fullfile(folder,sprintf('E1_grid_%d.pdf',cmg)),'ContentType','vector');close(f);
end
end
if any(strcmp(T.experiment,'E4_followup')),which='E4_followup';else,which='E4';end;S=T(strcmp(T.experiment,which) & isfinite(T.gain_db),:);if ~isempty(S)
for axisname={'gain_db','phase_deg'},f=figure('Visible','off');hold on;for cmg=[false true],M=S(S.cmg==cmg,:);[~,ix]=sort(M.frequency);semilogx(M.frequency(ix),M.(axisname{1})(ix),'-o','DisplayName',sprintf('CMG=%d',cmg));end;xlabel('Frequency (Hz)');ylabel(axisname{1},'Interpreter','none');legend;grid on;exportgraphics(f,fullfile(folder,[axisname{1} '.png']));exportgraphics(f,fullfile(folder,[axisname{1} '.pdf']),'ContentType','vector');close(f);end
end
end




