function plot_p0_ab(case_name)
% A/B comparison figure for one P0 case, in the style of the paper-side
% figures/torque_gap_no_cmg_comparison/<phase>/<case>_AB.png (2x2 panels):
%   (a) 姿态跟踪（按工况的机动轴）  参考 / 仅舵面 A / 舵面+CMG B
%   (b) 三轴姿态误差范数            A、B 的短弧误差与 RMSE
%   (c) 该轴力矩分工                气动上下限带、气动总力矩(A/B)、CMG 力矩、参考需求
%   (d) CMG 该轴角动量              H(gamma) 与 ±4h0 参考
%
% The axis of panels (a)/(c)/(d) follows the case: roll* -> roll, yaw* -> yaw,
% otherwise pitch. Data: the saved P0 logs in results/native_closedloop plus the
% case model's ModelWorkspace (future-reference bank and controller template).
%
% Usage: plot_p0_ab('S00_pitch30')   % one case
%        plot_p0_ab                 % all eight cases
if nargin<1
 cases={'S00_pitch30','S01_pitch_minus30','S02_pitch10','S03_pitch45', ...
        'S04_roll30','S05_yaw30','S06_pitch30_roll20','S07_sequential'};
 for k=1:numel(cases),plot_p0_ab(cases{k});end
 fprintf('PLOT_P0_AB_ALL_PASS (%d cases)\n',numel(cases));return;
end

root=repo_root;out=fullfile(root,'simulink','results','native_closedloop');
[fileA,dur]=locate(out,'A',case_name);
fileB=locate(out,'B',case_name);

A=loadlog(fileA);B=loadlog(fileB);
tk=loadmodel(case_name,false);tm=loadmodel(case_name,true);
bank=tk.bank;tmpl=tm.tmpl;
t=B.t;tref=bank.time(:);
h0=tmpl.h0;s0=tmpl.s0;gxs0=tmpl.gxs0;
cmgB=cmgmoment(s0,gxs0,h0,B.gamma,B.nu);        % tau_cmg = -A(gamma) nu (already negated inside)
Hy=cmgH(s0,gxs0,h0,B.gamma);

% which body axis this case manoeuvres about
if contains(case_name,'yaw')
 axidx=3;axname='偏航';axsym='z';
elseif contains(case_name,'roll') && ~contains(case_name,'pitch')
 axidx=1;axname='滚转';axsym='x';
else
 axidx=2;axname='俯仰';axsym='y';
end

qref=interp1(tref,squeeze(bank.q_ref(:,1,:))',t,'linear','extrap');
refAngle=rad2deg(eulerxyz(qref(:,1:4)));
refAngle=refAngle(:,axidx);
lo=squeeze(bank.tau_min(axidx,1,:));hi=squeeze(bank.tau_max(axidx,1,:));
nom=squeeze(bank.tau_nom(axidx,1,:));

A.err=errangle(A.q,t,bank);B.err=errangle(B.q,t,bank);
A.maxerr=max(A.err);B.maxerr=max(B.err);

dref=abs(gradient(refAngle,t));                % reference manoeuvre activity
mask=dref>0.2*max(dref);
if ~any(mask),mask=true(size(t));end
idx=find(mask);win=[t(idx(1)) t(idx(end))];
C_REF=[.45 .45 .45];C_A=[.85 .45 .08];C_B=[0 .45 .74];C_CMG=[.10 .60 .55];C_NOM=[.35 .35 .35];

fig=figure('Color','w','Position',[40 40 1560 930],'Name',['P0 A/B ' case_name]);
annotation(fig,'textbox',[0.045 0.962 0.9 0.04],'String', ...
 sprintf('%s：舵面与CMG协同控制（Simulink 原生迁移，%g s 窗口）',strrep(case_name,'_',' '),dur), ...
 'FontSize',15,'FontWeight','bold','EdgeColor','none','Interpreter','none');
annotation(fig,'textbox',[0.045 0.928 0.92 0.03],'String', ...
 sprintf('浅蓝背景：参考机动区间 %.1f–%.1f s；浅绿区域与绿虚线：%s气动力矩可行区间与上下限；RMSE 为全程窗口', ...
 win(1),win(2),axname),'FontSize',9.5,'Color',[.35 .35 .35],'EdgeColor','none');

% (a) attitude tracking on the manoeuvre axis
ax=axes('Position',[.065 .515 .395 .345]);hold(ax,'on');grid(ax,'on');box(ax,'off');
plot(ax,t,refAngle,'--','Color',C_REF,'LineWidth',1.2);
plot(ax,t,A.attdeg(:,axidx),'-','Color',C_A,'LineWidth',1.4);
plot(ax,t,B.attdeg(:,axidx),'-','Color',C_B,'LineWidth',1.6);
legend(ax,{'参考','仅舵面 A','舵面+CMG B'},'Location','northwest','Orientation','horizontal','Box','off');
ylabel(ax,[axname '姿态角 / °']);xlabel(ax,'时间 / s');title(ax,['(a) ' axname '姿态跟踪']);
xlim(ax,[t(1) t(end)]);shade(ax,t,mask);

% (b) attitude error norm
ax=axes('Position',[.560 .515 .395 .345]);hold(ax,'on');grid(ax,'on');box(ax,'off');
plot(ax,t,A.err,'-','Color',C_A,'LineWidth',1.6);
plot(ax,t,B.err,'-','Color',C_B,'LineWidth',1.8);
legend(ax,{sprintf('仅舵面 A  RMSE %.3f°，峰值 %.2f°',rms(A.err),max(A.err)), ...
           sprintf('舵面+CMG B  RMSE %.3f°，峰值 %.2f°',rms(B.err),max(B.err))}, ...
        'Location','northwest','Box','off');
ylabel(ax,'短弧姿态误差 / °');xlabel(ax,'时间 / s');title(ax,'(b) 三轴姿态误差范数');
xlim(ax,[t(1) t(end)]);shade(ax,t,mask);

% (c) torque split on the manoeuvre axis
ax=axes('Position',[.065 .075 .395 .345]);hold(ax,'on');grid(ax,'on');box(ax,'off');
patch(ax,[tref;flipud(tref)],[hi(:);flipud(lo(:))],[.85 .93 .88],'EdgeColor','none','FaceAlpha',.85);
plot(ax,tref,hi,'--','Color',[.25 .60 .40],'LineWidth',1.0);
plot(ax,tref,lo,':','Color',[.25 .60 .40],'LineWidth',1.0);
plot(ax,tref,nom,'--','Color',C_NOM,'LineWidth',1.0);
plot(ax,t,A.tau(:,axidx),'-','Color',C_A,'LineWidth',1.4);
plot(ax,t,B.tau(:,axidx),'-','Color',C_B,'LineWidth',1.2);
plot(ax,t,cmgB(:,axidx),'-','Color',C_CMG,'LineWidth',1.3);
legend(ax,{'可行区间','气动上限','气动下限','参考需求 \tau_{nom}','A 气动总力矩','B 气动总力矩','B CMG 力矩'}, ...
 'Location','southwest','NumColumns',2,'Box','off','FontSize',8);
ylabel(ax,[axname '轴力矩 / N·m']);xlabel(ax,'时间 / s');title(ax,['(c) 联合控制的' axname '轴力矩分工']);
xlim(ax,[t(1) t(end)]);shade(ax,t,mask,0.22);

% (d) CMG momentum on the manoeuvre axis
ax=axes('Position',[.560 .075 .395 .345]);hold(ax,'on');grid(ax,'on');box(ax,'off');
plot(ax,t,Hy(:,axidx),'-','Color',C_B,'LineWidth',1.6);
yl=[-4*h0 4*h0];
for k=1:2,plot(ax,[t(1) t(end)],[yl(k) yl(k)],':','Color',[.55 .55 .55],'LineWidth',.9);end
text(ax,.02,.92,sprintf('虚线 = ±4h_0 = ±%.0f N·m·s；CMG 内部状态，与瞬时输出力矩区分',4*h0), ...
 'Units','normalized','FontSize',8,'Color',[.4 .4 .4]);
ylabel(ax,['角动量 H_' axsym ' / N·m·s']);xlabel(ax,'时间 / s');
title(ax,['(d) CMG ' axname '轴角动量（A 组无 CMG）']);
xlim(ax,[t(1) t(end)]);ylim(ax,yl*1.15);shade(ax,t,mask);

png=fullfile(out,sprintf('P0_%s_AB.png',case_name));
exportgraphics(fig,png,'Resolution',150);close(fig);
m=table(["A_surface_only";"B_surface_plus_CMG"],[A.maxerr;B.maxerr],[rms(A.err);rms(B.err)], ...
 [A.err(end);B.err(end)],[max(abs(A.tau(:,axidx)));max(abs(B.tau(:,axidx)))], ...
 'VariableNames',{'Arm','MaxError_deg','RMSError_deg','FinalError_deg','PeakTauAxis_Nm'});
writetable(m,fullfile(out,sprintf('P0_%s_metrics.csv',case_name)));
disp(m);fprintf('PLOT_P0_AB_PASS %s\n',png);
end

% ---------------------------------------------------------------- helpers
function [f,dur]=locate(out,arm,case_name)
cands=dir(fullfile(out,sprintf('%s_attitude_*s_%s.mat',arm,case_name)));
if isempty(cands),error('missing %s case log for %s',arm,case_name);end
f=fullfile(out,cands(1).name);
tok=regexp(cands(1).name,'attitude_([0-9.]+)s_','tokens');dur=str2double(tok{1}{1});
end
function S=loadlog(f)
d=load(f);r=d.r;S.t=r.states.Time;x=series(r.states);
S.eul=x(:,1:3);S.gamma=[];S.nu=[];
if size(x,2)>=10,S.gamma=x(:,7:10);end
u=series(r.commands);S.tau=u(:,1:3);if size(u,2)>=7,S.nu=u(:,4:7);end
S.attdeg=rad2deg(S.eul);
S.q=eul2q(S.eul);
end
function S=loadmodel(case_name,enabled)
root=repo_root;
name='NearSpace_CMG_AttitudeValidation';if ~enabled,name=[name '_SurfaceOnly'];end
name=[name '_' case_name];
load_system(fullfile(root,'simulink','cases',[name '.slx']));
mw=get_param(name,'ModelWorkspace');
S.bank=getVariable(mw,'native_reference_bank');
S.tmpl=getVariable(mw,'native_controller_template');
close_system(name,0);
end
function A=cmgA(s0,gxs0,h0,gamma)
A=zeros(3,numel(gamma));                      % 3 x 4, column j = dH/dgamma_j
for j=1:4,A(:,j)=h0*(-s0(:,j)*sin(gamma(j))+gxs0(:,j)*cos(gamma(j)));end
end
function tau=cmgmoment(s0,gxs0,h0,gamma,nu)
tau=zeros(size(gamma,1),3);
for k=1:size(gamma,1)
 tau(k,:)=-(cmgA(s0,gxs0,h0,gamma(k,:)')*nu(k,:)')';
end
end
function H=cmgH(s0,gxs0,h0,gamma)
H=zeros(size(gamma,1),3);
for k=1:size(gamma,1)
 Hk=zeros(3,1);
 for j=1:4,Hk=Hk+h0*(s0(:,j)*cos(gamma(k,j))+gxs0(:,j)*sin(gamma(k,j)));end
 H(k,:)=Hk';
end
end
function e=errangle(q,tt,bank)
ref=interp1(bank.time(:),squeeze(bank.q_ref(:,1,:))',tt,'linear','extrap');ref=ref./vecnorm(ref,2,2);
e=2*acos(min(1,abs(sum(q.*ref,2))))*180/pi;
end
function e=eulerxyz(q)
q=q./vecnorm(q,2,2);
w=q(:,1);x=q(:,2);y=q(:,3);z=q(:,4);
e=[atan2(2*(w.*x+y.*z),1-2*(x.^2+y.^2)), ...
   asin(max(-1,min(1,2*(w.*y-z.*x)))), ...
   atan2(2*(w.*z+x.*y),1-2*(y.^2+z.^2))];
end
function q=eul2q(e)
a=e(:,1)/2;b=e(:,2)/2;c=e(:,3)/2;
q=[cos(a).*cos(b).*cos(c)+sin(a).*sin(b).*sin(c), ...
   sin(a).*cos(b).*cos(c)-cos(a).*sin(b).*sin(c), ...
   cos(a).*sin(b).*cos(c)+sin(a).*cos(b).*sin(c), ...
   cos(a).*cos(b).*sin(c)-sin(a).*sin(b).*cos(c)];
end
function shade(ax,t,mask,alpha)
% Fill every contiguous active run, using the current y-limits.
% Must be called AFTER the data is plotted, otherwise the limits are not set yet.
if nargin<4,alpha=0.16;end
yl=ylim(ax);d=diff([0 mask(:)' 0]);starts=find(d==1);ends=find(d==-1)-1;
for k=1:numel(starts)
 patch(ax,[t(starts(k)) t(ends(k)) t(ends(k)) t(starts(k))],[yl(1) yl(1) yl(2) yl(2)], ...
  [.80 .88 .97],'EdgeColor','none','FaceAlpha',alpha,'HandleVisibility','off');
end
ylim(ax,yl);
end
function data=series(ts)
data=squeeze(ts.Data);if size(data,1)~=numel(ts.Time),data=data.';end
end
