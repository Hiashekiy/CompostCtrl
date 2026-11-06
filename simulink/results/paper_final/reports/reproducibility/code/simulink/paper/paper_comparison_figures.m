function paper_comparison_figures(out)
folder=fullfile(out,'figures');if ~isfolder(folder),mkdir(folder);end
sets={{'E1_baseline/BASE1A','E1_baseline/BASE1B'}, {'E2_followup/F001B','E2_followup/F002B','E2_followup/F003B'}, {'E2_followup/F004B','E2_followup/F005B','E2_followup/F006B'}, {'E3_followup/F007B','E3_followup/F011B','E3_followup/F012B'}};
labels={{'Surface MPC','CMG MPC'},{'PID','PID unload','MPC'},{'PID','PID unload','MPC'},{'5 s horizon','Full horizon','Masked future'}};
for j=1:numel(sets)
blue=[0 .447 .741];orange=[.85 .325 .098];purple=[.494 .184 .556];if j==1,colors=[blue;orange];elseif j==4,colors=[blue;orange;purple];else,colors=[blue;purple;orange];end
 f=figure('Visible','off');tiledlayout(2,1);ax1=nexttile;hold on;ax2=nexttile;hold on;drawn=false;
 for k=1:numel(sets{j})
 sub=fullfile(out,sets{j}{k});path=fullfile(sub,'sim_output.mat');if ~isfile(path),continue;end
 d=load(path);t=d.r.states.Time;x=squeeze(d.r.states.Data);bank=load(fullfile(sub,'reference_bank.mat'));[qr,ref]=paper_reference_series(bank,t);idx=unique([1:ceil(numel(t)/2000):numel(t),numel(t)]);plot(ax1,t(idx),rad2deg(x(idx,2)),'Color',colors(k,:),'DisplayName',labels{j}{k});
 if ~drawn,plot(ax1,t(idx),ref(idx,2),'k--','DisplayName','Reference');drawn=true;end
 if j==4 && size(x,2)==10,plot(ax2,t(idx),rad2deg(x(idx,7)),'Color',colors(k,:),'DisplayName',labels{j}{k});else,q=paper_euler_quaternions(x(:,1:3));err=rad2deg(2*acos(min(1,abs(sum(q.*qr,2)))));plot(ax2,t(idx),err(idx),'Color',colors(k,:),'DisplayName',labels{j}{k});end
 end
 if drawn,xlabel(ax1,'Time (s)');ylabel(ax1,'Pitch (deg)');legend(ax1,'Location','best');grid(ax1,'on');xlabel(ax2,'Time (s)');if j==4,ylabel(ax2,'Gimbal 1 (deg)');else,ylabel(ax2,'Attitude error (deg)');end;legend(ax2,'Location','best');grid(ax2,'on');exportgraphics(f,fullfile(folder,sprintf('comparison_%d.png',j)));exportgraphics(f,fullfile(folder,sprintf('comparison_%d.pdf',j)),'ContentType','vector');end
 close(f);
end
end



