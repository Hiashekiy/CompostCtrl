function direct_servo_mpc_sfun(block)
% Level-2 S-function for the direct-servo MPC.
% Port 1: measured state [Euler(3); omega(3); gamma(4)] (gamma only for the CMG arm).
% Port 2: this control instant's reference window (same flattened layout as the RTI S-function).
% Port 3: measured flight/servo feedback [r_NED(3); v_body(3); delta_actual(3); previous_delta_command(3)].
% Port 1 out: [delta_cmd(3)] (+ [nu_cmd(4)] for the CMG arm) -> straight to the servo and CMG drive.
% Port 2 out: 8 diagnostics (sent to the visible Online_Surface_Authority monitor only).
block.NumDialogPrms=0;
isA=contains(get_param(bdroot(block.BlockHandle),'Name'),'SurfaceOnly');
W=0;
try
 mw=get_param(bdroot(block.BlockHandle),'ModelWorkspace');
 t=getVariable(mw,'native_controller_template');W=10*(double(t.N)+1)+9*double(t.N)+3;
catch
end
block.NumInputPorts=3;block.NumOutputPorts=2;
block.SetPreCompPortInfoToDefaults;
if isA,block.InputPort(1).Dimensions=6;block.OutputPort(1).Dimensions=3;
else,block.InputPort(1).Dimensions=10;block.OutputPort(1).Dimensions=7;end
block.InputPort(1).DirectFeedthrough=true;
block.InputPort(2).Dimensions=W;block.InputPort(2).DirectFeedthrough=true;
block.InputPort(3).Dimensions=12;block.InputPort(3).DirectFeedthrough=true;
block.OutputPort(2).Dimensions=8;
block.SampleTimes=[.2 0];
block.SimStateCompliance='HasNoSimState';
block.RegBlockMethod('Start',@(b)runtime('start',b));
block.RegBlockMethod('Outputs',@(b)runtime('output',b));
block.RegBlockMethod('Terminate',@(b)runtime('terminate',b));
end
function runtime(action,b)
persistent contexts
if isempty(contexts),contexts=containers.Map('KeyType','double','ValueType','any');end
key=double(b.BlockHandle);
if strcmp(action,'start')
 mw=get_param(bdroot(b.BlockHandle),'ModelWorkspace');
 c.s=getVariable(mw,'native_controller_template');
 c.bank=[];c.state=[];c.time=-inf;c.command=zeros(double(c.s.nu_direct),1);c.fallbacks=0;
 c.results_dir='';if hasVariable(mw,'native_results_dir'),c.results_dir=getVariable(mw,'native_results_dir');end
 c.diagnostics={};c.control_times=[];c.preview=zeros(8,1);c.root=get_param(bdroot(b.BlockHandle),'Name');
 contexts(key)=c;return;
end
if strcmp(action,'terminate')
 if isKey(contexts,key)
  c=contexts(key);fprintf('DIRECT_MPC fallbacks=%d\n',c.fallbacks);
  root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
  folder=c.results_dir;if isempty(folder),folder=fullfile(root,'simulink','results','direct_servo');end
  if ~exist(folder,'dir'),mkdir(folder);end
  stats=jsonencode(struct('fallbacks',c.fallbacks,'last_control_time',c.time));
  try
   diagnostics=c.diagnostics;control_times=c.control_times;
   save(fullfile(folder,[c.root '_direct_authority.mat']),'diagnostics','control_times');
   fid=fopen(fullfile(folder,[c.root '_direct_runtime.json']),'w');
   if fid>0,fprintf(fid,'%s',stats);fclose(fid);end
  catch err
   warning('DIRECT_MPC_STATS_SKIPPED %s',err.message);
  end
  remove(contexts,key);
 end
 return;
end
c=contexts(key);
if b.CurrentTime>c.time+1e-9
 c.s=parse_window(b,c.s);
 input=b.InputPort(1).Data;roll=input(1);pitch=input(2);yaw=input(3);
 cr=cos(roll/2);sr=sin(roll/2);cp=cos(pitch/2);sp=sin(pitch/2);cy=cos(yaw/2);sy=sin(yaw/2);
 q=[cr*cp*cy+sr*sp*sy;sr*cp*cy-cr*sp*sy;cr*sp*cy+sr*cp*sy;cr*cp*sy-sr*sp*cy];
 c.s.flight_state=b.InputPort(3).Data(:);
 c.s.direct_previous_command=c.s.flight_state(10:12);
 started=tic;[c.command,c.state,info]=direct_servo_mpc_step(c.s,[q;input(4:end)],c.state);
 if ~isfield(info,'elapsed_s'),info.elapsed_s=toc(started);end
 c.diagnostics{end+1}=info;c.control_times(end+1)=b.CurrentTime;
 if ~isempty(info.authority)
  a=info.authority;
  c.preview=[a.command(:,1);a.delta(:,min(2,size(a.delta,2)));0;0];
 else
  c.preview=[c.command(1:3);c.command(1:3);0;0];
 end
 if mod(round(b.CurrentTime/c.s.dt_ctrl),25)==0
  fprintf(['DIRECT_AUTHORITY t=%g compute=%gs QP=%d accepted=%d rejected=%d stop=%s ' ...
   'fallback=%d\n'],b.CurrentTime,info.elapsed_s,info.iterations,info.accepted_steps, ...
   info.rejected,reason_of(info),info.fallback);
 end
 if info.fallback&&c.fallbacks<3,fprintf('DIRECT_FALLBACK t=%g %s\n',b.CurrentTime,info.failure);end
 c.fallbacks=c.fallbacks+double(info.fallback);c.time=b.CurrentTime;contexts(key)=c;
end
b.OutputPort(1).Data=c.command;
b.OutputPort(2).Data=c.preview;
end
function value=reason_of(info)
value='';if isfield(info,'early_stop_reason'),value=info.early_stop_reason;end
end
function s=parse_window(b,s)
w=b.InputPort(2).Data(:);
n1=double(s.N)+1;nN=double(s.N);p=0;
s.q_ref=reshape(w(p+1:p+4*n1),4,n1);p=p+4*n1;
s.omega_ref=reshape(w(p+1:p+3*n1),3,n1);p=p+3*n1;
s.tau_d=reshape(w(p+1:p+3*n1),3,n1);p=p+3*n1;
p=p+3*nN;p=p+3*nN;                       % legacy torque bounds are not used
s.tau_nom=reshape(w(p+1:p+3*nN),3,nN);p=p+3*nN;
s.gap=reshape(w(p+1:p+3),3,1);
if p+3~=numel(w)
 error('direct:window','reference window has %d values, expected %d',numel(w),p+3);
end
end
