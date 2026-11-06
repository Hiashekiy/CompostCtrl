function v3_savefig(f,out,name)
lines=findall(f,'Type','line');for k=1:numel(lines),xx=lines(k).XData;yy=lines(k).YData;if numel(xx)>3000 && numel(xx)==numel(yy),[~,lo]=min(yy);[~,hi]=max(yy);idx=unique([1:ceil(numel(xx)/3000):numel(xx),numel(xx),lo,hi]);set(lines(k),'XData',xx(idx),'YData',yy(idx));end;end
layouts=findall(f,'Type','tiledlayout');w=1100;h=650;if ~isempty(layouts),sz=layouts(1).GridSize;w=max(900,500*sz(2));h=max(500,260*sz(1));end;set(f,'Position',[50 50 w h],'Color','w');set(findall(f,'Type','axes'),'FontName','Arial','FontSize',10,'LineWidth',.8);set(findall(f,'Type','line'),'LineWidth',1.2);set(findall(f,'Type','legend'),'FontSize',8);drawnow;
exportgraphics(f,fullfile(out,'figures',[name '.png']),'Resolution',300);exportgraphics(f,fullfile(out,'figures',[name '.pdf']),'ContentType','vector');close(f);
end
