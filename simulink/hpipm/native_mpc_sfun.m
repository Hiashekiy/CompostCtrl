function native_mpc_sfun(block)
% Port 1: measured state [Euler(3); omega(3); gamma(4)] (gamma only for the B arm).
% Port 2 (only when the model workspace carries `reference_window`): this control
% instant's reference window, flattened as
%   q_ref(4xN+1), omega_ref(3xN+1), tau_d(3xN+1), tau_min(3xN), tau_max(3xN),
%   tau_nom(3xN), gap(3)
% so the reference enters the loop through a visible signal.
%
% Models built before Reference_Manager existed have no `reference_window`
% variable; they keep the legacy single input port and read the embedded bank.
% Rebuilding a model with modelgen/build_closedloop switches it to the wired path.
block.NumDialogPrms=0;
isA=contains(get_param(bdroot(block.BlockHandle),'Name'),'SurfaceOnly');
wired=false;W=0;online=false;
try
 mw=get_param(bdroot(block.BlockHandle),'ModelWorkspace');
 if hasVariable(mw,'native_controller_template')
  t=getVariable(mw,'native_controller_template');
  online=isfield(t,'authority_mode') && startsWith(t.authority_mode,'online_');
  wired=hasVariable(mw,'reference_window') || online;
  W=10*(double(t.N)+1)+9*double(t.N)+3;
 end
catch
end
block.NumInputPorts=1+double(wired)+double(online);
% Declare final counts ONCE. Temporarily shrinking to one output deletes the
% saved line on diagnostic port 2 when Simulink reinitializes this S-function.
block.NumOutputPorts=1+double(online);
block.SetPreCompPortInfoToDefaults;
if isA,block.InputPort(1).Dimensions=6;block.OutputPort(1).Dimensions=3;
else,block.InputPort(1).Dimensions=10;block.OutputPort(1).Dimensions=7;end
block.InputPort(1).DirectFeedthrough=true;
if wired,block.InputPort(2).Dimensions=W;block.InputPort(2).DirectFeedthrough=true;end
if online
 block.InputPort(3).Dimensions=12;block.InputPort(3).DirectFeedthrough=true;
 block.OutputPort(2).Dimensions=8;
end
period=.2;
try
 mw=get_param(bdroot(block.BlockHandle),'ModelWorkspace');
 if hasVariable(mw,'native_controller_template')
  t=getVariable(mw,'native_controller_template');period=t.dt_ctrl;
 end
catch
end
block.SampleTimes=[period 0];
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
 c.wired=hasVariable(mw,'reference_window');
 c.s=getVariable(mw,'native_controller_template');
 if c.wired,c.bank=[];else,c.bank=getVariable(mw,'native_reference_bank');end
 c.state=[];c.time=-inf;c.command=zeros(double(c.s.nu),1);c.fallbacks=0;
 c.results_dir='';if hasVariable(mw,'native_results_dir'),c.results_dir=getVariable(mw,'native_results_dir');end
 c.online=isfield(c.s,'authority_mode') && startsWith(c.s.authority_mode,'online_');
 c.sqp_iterations=4;if isfield(c.s,'max_sqp_iterations'),c.sqp_iterations=double(c.s.max_sqp_iterations);end
 assert(~c.online || ismember(c.sqp_iterations,[4 5]),'online:sqp','max_sqp_iterations must be 4 or 5');
 assert(~c.online || c.wired,'online:reference','Online mode requires the wired reference window');
 c.capture=c.online || hasVariable(mw,'native_capture_diagnostics');
 c.diagnostics={};c.control_times=[];
 c.authority_preview=zeros(8,1);
 contexts(key)=c;return;
end
if strcmp(action,'terminate')
 if isKey(contexts,key)
  c=contexts(key);fprintf('ONLINE_MPC fallbacks=%d\n',c.fallbacks);
  root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
  folder=c.results_dir;if isempty(folder),folder=fullfile(root,'simulink','results','native_closedloop');end
  if ~exist(folder,'dir'),mkdir(folder);end
  % Statistics only: a locked/read-only results file must not abort the run.
  stats=jsonencode(struct('fallbacks',c.fallbacks,'last_control_time',c.time));
  try
   if c.capture
    diagnostics=c.diagnostics;control_times=c.control_times;
    save(fullfile(folder,[get_param(bdroot(b.BlockHandle),'Name') '_authority.mat']),'diagnostics','control_times');
   end
   fid=fopen(fullfile(folder,[get_param(bdroot(b.BlockHandle),'Name') '_runtime.json']),'w');
   if fid>0,fprintf(fid,'%s',stats);fclose(fid);else,error('native:locked','cannot open runtime file');end
   fid=fopen(fullfile(folder,sprintf('%s_runtime_%gs.json',get_param(bdroot(b.BlockHandle),'Name'),c.time)),'w');
   if fid>0,fprintf(fid,'%s',stats);fclose(fid);else,error('native:locked','cannot open runtime file');end
  catch err
   warning('ONLINE_MPC_STATS_SKIPPED %s',err.message);
  end
  remove(contexts,key);
 end
 return;
end
c=contexts(key);
if b.CurrentTime>c.time+1e-9
 if c.wired
  if c.time<0,check_window(b,c.s);end
  c.s=parse_window(b,c.s);
 else
  k=min(round(b.CurrentTime/c.s.dt_ctrl)+1,numel(c.bank.time));
  for f={'q_ref','omega_ref','tau_d','tau_min','tau_max','tau_nom','gap'}
   name=f{1};v=c.bank.(name);c.s.(name)=v(:,:,k);
  end
 end
 input=b.InputPort(1).Data;roll=input(1);pitch=input(2);yaw=input(3);
 cr=cos(roll/2);sr=sin(roll/2);cp=cos(pitch/2);sp=sin(pitch/2);cy=cos(yaw/2);sy=sin(yaw/2);
 q=[cr*cp*cy+sr*sp*sy;sr*cp*cy-cr*sp*sy;cr*sp*cy+sr*cp*sy;cr*cp*sy-sr*sp*cy];
 if c.online,c.s.flight_state=b.InputPort(3).Data(:);end
 call_started=tic;[c.command,c.state,info]=native_rti_step(c.s,[q;input(4:end)],c.state);
 if ~isfield(info,'elapsed_s'),info.elapsed_s=toc(call_started);end
 if c.capture,c.diagnostics{end+1}=info;c.control_times(end+1)=b.CurrentTime;end
 if c.online
  if ~isempty(info.authority)
   a=info.authority;c.authority_preview=[a.tau_min(:,1);a.tau_max(:,1);a.scale(1);max(a.vertex_residual(:,1))];
  else,c.authority_preview=[c.command(1:3);c.command(1:3);0;0];end
  if mod(round(b.CurrentTime/c.s.dt_ctrl),25)==0
   fprintf(['ONLINE_AUTHORITY t=%g compute=%gs SQP<=%d QP=%d accepted=%d rejected=%d ' ...
    'stop=%s fallback=%d\n'],b.CurrentTime,info.elapsed_s,c.sqp_iterations,info.iterations, ...
    diagnostics_field(info,'accepted_steps'),info.rejected,diagnostics_field(info,'early_stop_reason'),info.fallback);
  end
 end
 if info.fallback && c.fallbacks<3,fprintf('MPC_FALLBACK t=%g %s\n',b.CurrentTime,info.failure);end
 c.fallbacks=c.fallbacks+double(info.fallback);c.time=b.CurrentTime;contexts(key)=c;
end
b.OutputPort(1).Data=c.command;
if c.online,b.OutputPort(2).Data=c.authority_preview;end
end
function s=parse_window(b,s)
% Unpack the flattened reference window delivered on input port 2.
% Returns s: MATLAB structs are values, so the caller must take the result back.
w=b.InputPort(2).Data(:);
n1=double(s.N)+1;nN=double(s.N);p=0;
s.q_ref=reshape(w(p+1:p+4*n1),4,n1);p=p+4*n1;
s.omega_ref=reshape(w(p+1:p+3*n1),3,n1);p=p+3*n1;
s.tau_d=reshape(w(p+1:p+3*n1),3,n1);p=p+3*n1;
online=isfield(s,'authority_mode') && startsWith(s.authority_mode,'online_');
if ~online,s.tau_min=reshape(w(p+1:p+3*nN),3,nN);end;p=p+3*nN;
if ~online,s.tau_max=reshape(w(p+1:p+3*nN),3,nN);end;p=p+3*nN;
s.tau_nom=reshape(w(p+1:p+3*nN),3,nN);p=p+3*nN;
s.gap=reshape(w(p+1:p+3),3,1);
if p+3~=numel(w),error('native:window','reference window has %d values, expected %d',numel(w),p+3);end
end
function check_window(b,s)
% First control instant: fail loudly if the wiring does not match the template.
w=b.InputPort(2).Data(:);
n1=double(s.N)+1;nN=double(s.N);expect=10*n1+9*nN+3;
if numel(w)~=expect
 error('native:window','reference window has %d values, expected %d (N=%d); check Reference_Manager',numel(w),expect,double(s.N));
end
end
function value=diagnostics_field(info,name)
% Solver diagnostics added by the five-round SQP are optional for older callers.
value=0;if strcmp(name,'early_stop_reason'),value='';end
if isfield(info,name),value=info.(name);end
end
