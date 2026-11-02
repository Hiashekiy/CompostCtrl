function plot_flight_ab(suffix)
% Side-by-side data comparison of the two flight-mode closed loops:
%   A = models/NearSpace_CMG_FlightControl_SurfaceOnly.slx  (surface only)
%   B = models/NearSpace_CMG_FlightControl.slx              (surface + CMG)
% Reads the logs written by build_closedloop(duration,true,enabled) and writes
%   results/native_closedloop/FlightControl_AB_comparison_<suffix>.png
%   results/native_closedloop/FlightControl_AB_metrics_<suffix>.csv
%
% Every curve is drawn on an explicit axes handle (no gca, no yyaxis) so panels
% cannot inherit each other's state.
%
% Usage: plot_flight_ab            % 60 s logs
%        plot_flight_ab('2s')      % any '<dur>s' with both logs present
if nargin<1,suffix='60s';end
root=repo_root;
out=fullfile(root,'simulink','results','native_closedloop');
bank=load(fullfile(root,'simulink','data','online_plan_bank.mat'));

D=struct();
for arm={'A','B'}
 f=fullfile(out,[arm{1} '_surrogate_flight_' suffix '.mat']);
 if exist(f,'file')~=2
  error('missing %s (run modelgen/build_closedloop(...,true,%d) first)',f,strcmp(arm{1},'B'));
 end
 d=load(f);r=d.r;t=r.states.Time;S.t=t;
 x=series(r.states);S.att=x(:,1:3);S.omega=x(:,4:6);S.gamma=[];
 if size(x,2)>=10,S.gamma=x(:,7:10);end
 u=series(r.commands);S.tau=u(:,1:3);S.nu=[];
 if size(u,2)>=7,S.nu=u(:,4:7);end
 S.fc=[];if any(strcmp(r.who,'flight_conditions')),S.fc=series(r.flight_conditions);end
 ref=interp1(bank.time(:),squeeze(bank.q_ref(:,1,:))',t,'linear','extrap');ref=ref./vecnorm(ref,2,2);
 e=2*acos(min(1,abs(sum(eul2q(S.att).*ref,2))))*180/pi;
 S.err=e;S.maxerr=max(e);S.rmserr=sqrt(mean(e.^2));S.finalerr=e(end);
 D.(arm{1})=S;
end

colA=[.35 .35 .35];colB=[0 .45 .74];colA2=[.70 .40 .10];colB2=[.90 .60 .10];
fig=figure('Color','w','Position',[40 20 1600 1150],'Name',['Flight closed loop A/B ' suffix]);

% 1 attitude
a=subplot(5,2,1);line2(a,D,colA,colA2,colB,colB2,'att',[2 3],'deg', ...
 {'A pitch','A yaw','B pitch','B yaw'},'姿态角：A（仅舵面） vs B（舵面+CMG）');
% 2 attitude error
a=subplot(5,2,2);hold(a,'on');grid(a,'on');
plot(a,D.A.t,D.A.err,'--','Color',colA,'DisplayName',sprintf('A max %.2f° / rms %.2f°',D.A.maxerr,D.A.rmserr));
plot(a,D.B.t,D.B.err,'-','Color',colB,'DisplayName',sprintf('B max %.2f° / rms %.2f°',D.B.maxerr,D.B.rmserr));
ylabel(a,'误差 (deg)');title(a,'相对内嵌参考的姿态误差');legend(a,'Location','best');
% 3/4 body rates (deg/s: rad/s would be plotted with a x10^-3 exponent label)
a=subplot(5,2,3);line2(a,D,colA,colA2,colB,colB2,'omega',2,'deg/s',{'A','B'},'俯仰角速度 q',true);
a=subplot(5,2,4);line2(a,D,colA,colA2,colB,colB2,'omega',3,'deg/s',{'A','B'},'偏航角速度 r',true);
% 5 pitch aero moment command
a=subplot(5,2,5);line2(a,D,colA,colA2,colB,colB2,'tau',2,'N·m',{'A \tau_{a,y}','B \tau_{a,y}'},'俯仰总气动力矩命令');
% 6 pitch gimbal rate command (B only)
a=subplot(5,2,6);hold(a,'on');grid(a,'on');
if ~isempty(D.B.nu)
 plot(a,D.B.t,D.B.nu(:,1));plot(a,D.B.t,D.B.nu(:,2));plot(a,D.B.t,D.B.nu(:,3));plot(a,D.B.t,D.B.nu(:,4));
 legend(a,{'\nu_1','\nu_2','\nu_3','\nu_4'},'Location','best','NumColumns',2);
end
ylabel(a,'\nu cmd (rad/s)');title(a,'B 组 CMG 框架速率命令（A 组无此通道）');
% 7 gimbal angles (B only)
a=subplot(5,2,7);hold(a,'on');grid(a,'on');
if ~isempty(D.B.gamma)
 plot(a,D.B.t,D.B.gamma);legend(a,{'\gamma_1','\gamma_2','\gamma_3','\gamma_4'},'Location','best');
end
ylabel(a,'\gamma (rad)');title(a,'B 组框架角（滑环无限行程假设）');
% 8 dynamic pressure
a=subplot(5,2,8);hold(a,'on');grid(a,'on');
if ~isempty(D.B.fc)
 plot(a,D.A.t,D.A.fc(:,1),'--','Color',colA);plot(a,D.B.t,D.B.fc(:,1),'-','Color',colB);
 legend(a,{'A qbar','B qbar'},'Location','best');
end
ylabel(a,'qbar (Pa)');title(a,'动压');
% 9 flow angles
a=subplot(5,2,9);hold(a,'on');grid(a,'on');
if ~isempty(D.B.fc)
 plot(a,D.A.t,rad2deg(D.A.fc(:,3)),'--','Color',colA2);plot(a,D.B.t,rad2deg(D.B.fc(:,3)),'-','Color',colB2);
 plot(a,D.A.t,rad2deg(D.A.fc(:,4)),':','Color',colA);plot(a,D.B.t,rad2deg(D.B.fc(:,4)),':','Color',colB);
 legend(a,{'A \alpha','B \alpha','A \beta','B \beta'},'Location','best');
end
ylabel(a,'deg');title(a,'迎角与侧滑角');
% 10 speed
a=subplot(5,2,10);hold(a,'on');grid(a,'on');
if ~isempty(D.B.fc)
 plot(a,D.A.t,D.A.fc(:,2),'--','Color',colA);plot(a,D.B.t,D.B.fc(:,2),'-','Color',colB);
 legend(a,{'A V','B V'},'Location','best');
end
ylabel(a,'V (m/s)');xlabel(a,'时间 (s)');title(a,'速度');

sgtitle(sprintf('工程闭环 A/B 数据对比（公开升阻力替代模型，%s）：A max %.2f° / rms %.2f°    B max %.2f° / rms %.2f°', ...
 suffix,D.A.maxerr,D.A.rmserr,D.B.maxerr,D.B.rmserr),'FontWeight','bold');

png=fullfile(out,['FlightControl_AB_comparison_' suffix '.png']);
if getenv('PLOT_FLIGHT_AB_DEBUG')
 for k=1:10
  ax=subplot(5,2,k);ch=ax.Children;fprintf('[dbg] panel %2d "%s" YLim=%s\n',k,get(ax,'Title').String,mat2str(get(ax,'YLim'),4));
  for i=1:numel(ch)
   if isprop(ch(i),'YData'),yd=ch(i).YData;fprintf('        line n=%d min=%.4g max=%.4g type=%s\n',numel(yd),min(yd),max(yd),ch(i).Type);end
  end
 end
end
exportgraphics(fig,png,'Resolution',150);close(fig);
metrics=table(["A_surface_only";"B_surface_plus_CMG"],[D.A.maxerr;D.B.maxerr],[D.A.rmserr;D.B.rmserr], ...
 [D.A.finalerr;D.B.finalerr],'VariableNames',{'Arm','MaxError_deg','RMSError_deg','FinalError_deg'});
writetable(metrics,fullfile(out,['FlightControl_AB_metrics_' suffix '.csv']));
disp(metrics);fprintf('PLOT_FLIGHT_AB_PASS %s\n',png);
end

function line2(ax,D,colA,colA2,colB,colB2,field,cols,unit,labels,titl,toDeg)
if nargin<12,toDeg=false;end
hold(ax,'on');grid(ax,'on');
k=0;
for c=cols
 k=k+1;
 ya=D.A.(field);ya=ya(:,c);
 yb=D.B.(field);yb=yb(:,c);
 if strcmp(field,'att')||toDeg,ya=rad2deg(ya);yb=rad2deg(yb);end
 plot(ax,D.A.t,ya,'--','Color',colA,'DisplayName',labels{k});
 plot(ax,D.B.t,yb,'-','Color',colB,'DisplayName',labels{numel(cols)+k});
end
ylabel(ax,unit);title(ax,titl);legend(ax,'Location','best','NumColumns',2);
end

function q=eul2q(e)
a=e(:,1)/2;b=e(:,2)/2;c=e(:,3)/2;
q=[cos(a).*cos(b).*cos(c)+sin(a).*sin(b).*sin(c), ...
   sin(a).*cos(b).*cos(c)-cos(a).*sin(b).*sin(c), ...
   cos(a).*sin(b).*cos(c)+sin(a).*cos(b).*sin(c), ...
   cos(a).*cos(b).*sin(c)-sin(a).*sin(b).*cos(c)];
end
function data=series(ts)
data=squeeze(ts.Data);if size(data,1)~=numel(ts.Time),data=data.';end
end
