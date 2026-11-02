function check_p0_case(case_name,enabled)
% Build, simulate and score one P0 case with the current builder (no progress bookkeeping).
% Usage: check_p0_case('S00_pitch30',true)   % true = B (surface+CMG), false = A
root=fileparts(fileparts(mfilename('fullpath')));
addpath(fullfile(root,'simulink'));addpath(fullfile(root,'simulink','hpipm'));
if ischar(enabled),enabled=logical(str2double(enabled));end
bank=load(fullfile(root,'simulink','data','online_plan_bank.mat'));
duration=double(bank.time(end));
build_native_closedloop(duration,false,enabled,case_name);
m='NearSpace_CMG_AttitudeValidation';if ~enabled,m=[m '_SurfaceOnly'];end
m=[m '_' case_name];
out=fullfile(root,'simulink','results','native_closedloop');
load_system(fullfile(root,'simulink',[m '.slx']));
mw=get_param(m,'ModelWorkspace');bank=getVariable(mw,'native_reference_bank');
arm='A';if enabled,arm='B';end
S=load(fullfile(out,sprintf('%s_attitude_%gs_%s.mat',arm,duration,case_name)));r=S.r;t=r.states.Time;
x=squeeze(r.states.Data);if size(x,1)~=numel(t),x=x';end
a=x(:,1)/2;b=x(:,2)/2;c=x(:,3)/2;
q=[cos(a).*cos(b).*cos(c)+sin(a).*sin(b).*sin(c),sin(a).*cos(b).*cos(c)-cos(a).*sin(b).*sin(c), ...
 cos(a).*sin(b).*cos(c)+sin(a).*cos(b).*sin(c),cos(a).*cos(b).*sin(c)-sin(a).*sin(b).*cos(c)];
ref=interp1(bank.time(:),squeeze(bank.q_ref(:,1,:))',t);ref=ref./vecnorm(ref,2,2);
angle=2*acos(min(1,abs(sum(q.*ref,2))))*180/pi;
fprintf('CASE %s ARM %s max=%.4f rms=%.4f final=%.4f\n',case_name,arm,max(angle),sqrt(mean(angle.^2)),angle(end));
if enabled
 fprintf('gamma_end=%s  gamma_max=%s  nu_max=%s\n',mat2str(x(end,7:10),6), ...
  mat2str(max(abs(x(:,7:10)),[],1),6),mat2str(max(abs(r.commands.Data(:,4:7)),[],1),6));
end
close_system(m,0);
end
