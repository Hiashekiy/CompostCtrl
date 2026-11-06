function label_c2_figure(kind)
% Explicit axes-bound keys also survive headless multi-panel export.
switch kind
case 'pitch'
 labels={'Blue: actual; orange dashed: reference','','Blue: aero; orange: CMG; yellow: aero+CMG; purple: reference-acceleration proxy',''};
case 'state'
 labels={'Blue/orange/yellow/purple: gimbal 1/2/3/4','Blue: Hx; orange: Hy; yellow: Hz; purple: H norm','Blue: positive projection; orange: negative projection',''};
case 'actuator'
 labels={'Blue/orange/yellow/purple: nu1/nu2/nu3/nu4','Blue: CMG rate; orange: CMG acceleration; yellow: surface angle; purple: surface rate','Blue: SQP tried; orange: QP solved; yellow: accepted; purple: rejected','Blue: pitch error; orange: fallback'};
end
for k=1:numel(labels)
 if ~isempty(labels{k}),ax=nexttile(k);text(ax,.015,.97,labels{k},'Units','normalized','VerticalAlignment','top','FontSize',8,'BackgroundColor','white','Interpreter','none');end
end
end
