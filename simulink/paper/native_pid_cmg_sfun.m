function native_pid_cmg_sfun(b)
b.NumDialogPrms=0;b.NumInputPorts=3;b.NumOutputPorts=2;b.SetPreCompPortInfoToDefaults;mw=get_param(bdroot(b.BlockHandle),'ModelWorkspace');t=getVariable(mw,'native_controller_template');
for k=1:3,b.InputPort(k).DirectFeedthrough=true;end
b.InputPort(1).Dimensions=10;b.InputPort(2).Dimensions=10*(double(t.N)+1)+9*double(t.N)+3;b.InputPort(3).Dimensions=12;b.OutputPort(1).Dimensions=7;b.OutputPort(2).Dimensions=8;b.SampleTimes=[.2 0];b.SimStateCompliance='HasNoSimState';b.RegBlockMethod('Start',@(b)step(b,'start'));b.RegBlockMethod('Outputs',@(b)step(b,'out'));b.RegBlockMethod('Terminate',@(b)step(b,'end'));
end
function step(b,action)
persistent ctx
if isempty(ctx),ctx=containers.Map('KeyType','double','ValueType','any');end
key=double(b.BlockHandle);
if strcmp(action,'start'),mw=get_param(bdroot(b.BlockHandle),'ModelWorkspace');ctx(key)=struct('config',getVariable(mw,'paper_pid_config'),'z',[],'time',-inf,'u',zeros(7,1),'d',zeros(8,1),'history',{{}},'times',[],'folder',getVariable(mw,'native_results_dir'));return;end
if strcmp(action,'end'),c=ctx(key);history=c.history;times=c.times;save(fullfile(c.folder,'pid_diagnostics.mat'),'history','times');remove(ctx,key);return;end
c=ctx(key);if b.CurrentTime>c.time+1e-9,w=b.InputPort(2).Data;N=(numel(w)-13)/19+1;[c.u,c.z,c.d]=pid_cmg_control_step(b.InputPort(1).Data,w(1:4),w(4*N+1:4*N+3),b.InputPort(3).Data,c.config,c.z);c.time=b.CurrentTime;c.history{end+1}=c.z;c.times(end+1)=c.time;ctx(key)=c;end
b.OutputPort(1).Data=c.u;b.OutputPort(2).Data=c.d;
end

