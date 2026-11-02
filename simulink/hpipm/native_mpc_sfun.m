function native_mpc_sfun(block)
block.NumDialogPrms=0;block.NumInputPorts=1;block.NumOutputPorts=1;
block.SetPreCompPortInfoToDefaults;
isA=contains(get_param(bdroot(block.BlockHandle),'Name'),'SurfaceOnly');
if isA,block.InputPort(1).Dimensions=6;block.OutputPort(1).Dimensions=3;else,block.InputPort(1).Dimensions=10;block.OutputPort(1).Dimensions=7;end
block.InputPort(1).DirectFeedthrough=true;block.SampleTimes=[.2 0];
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
 root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
 if contains(get_param(bdroot(b.BlockHandle),'Name'),'SurfaceOnly'),file='production_online_A_qp.mat';else,file='production_online_qp.mat';end
 mw=get_param(bdroot(b.BlockHandle),'ModelWorkspace');
 c.s=getVariable(mw,'native_controller_template');
 c.bank=getVariable(mw,'native_reference_bank');
 c.state=[];c.time=-inf;c.command=zeros(double(c.s.nu),1);c.fallbacks=0;contexts(key)=c;return;
end
if strcmp(action,'terminate')
 if isKey(contexts,key)
  c=contexts(key);fprintf('ONLINE_MPC fallbacks=%d\n',c.fallbacks);
  root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
  folder=fullfile(root,'simulink','results','native_closedloop');if ~exist(folder,'dir'),mkdir(folder);end
  % Statistics only: a locked/read-only results file must not abort the run
  % (an older MATLAB session or a scanner can hold these files open on Windows).
  stats=jsonencode(struct('fallbacks',c.fallbacks,'last_control_time',c.time));
  try
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
 k=min(round(b.CurrentTime/c.s.dt_ctrl)+1,numel(c.bank.time));
 for f={'q_ref','omega_ref','tau_d','tau_min','tau_max','tau_nom','gap'}
  name=f{1};v=c.bank.(name);c.s.(name)=v(:,:,k);
 end
 input=b.InputPort(1).Data;roll=input(1);pitch=input(2);yaw=input(3);
 cr=cos(roll/2);sr=sin(roll/2);cp=cos(pitch/2);sp=sin(pitch/2);cy=cos(yaw/2);sy=sin(yaw/2);
 q=[cr*cp*cy+sr*sp*sy;sr*cp*cy-cr*sp*sy;cr*sp*cy+sr*cp*sy;cr*cp*sy-sr*sp*cy];
 [c.command,c.state,info]=native_rti_step(c.s,[q;input(4:end)],c.state);
 if info.fallback && c.fallbacks<3,fprintf('MPC_FALLBACK t=%g %s\n',b.CurrentTime,info.failure);end
 c.fallbacks=c.fallbacks+double(info.fallback);c.time=b.CurrentTime;contexts(key)=c;
end
b.OutputPort(1).Data=c.command;
end
