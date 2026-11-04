function native_surface_allocation_sfun(block)
block.NumDialogPrms=0;block.NumInputPorts=1;block.NumOutputPorts=1;block.SetPreCompPortInfoToDefaults;
block.InputPort(1).Dimensions=7;block.InputPort(1).DirectFeedthrough=true;block.OutputPort(1).Dimensions=3;
period=.2;mw=get_param(bdroot(block.BlockHandle),'ModelWorkspace');
if hasVariable(mw,'native_control_period'),period=getVariable(mw,'native_control_period');end
block.SampleTimes=[period 0];block.SimStateCompliance='HasNoSimState';
block.RegBlockMethod('Start',@(b)step('start',b));block.RegBlockMethod('Outputs',@(b)step('output',b));
block.RegBlockMethod('Terminate',@(b)step('terminate',b));
end
function step(action,b)
persistent state
if isempty(state),state=containers.Map('KeyType','double','ValueType','any');end
key=double(b.BlockHandle);
if strcmp(action,'start')
 root=fileparts(fileparts(mfilename('fullpath')));addpath(fullfile(root,'data'));p=vehicle_params;c.v=p.vehicle;c.delta=zeros(3,1);c.time=-inf;c.period=b.SampleTimes(1);state(key)=c;return;
end
if strcmp(action,'terminate'),if isKey(state,key),remove(state,key);end;return;end
c=state(key);v=c.v;
if b.CurrentTime>c.time+1e-9
 input=b.InputPort(1).Data;Q=input(7);
 M=Q*v.S*[0 v.b*v.Cl_da v.b*v.Cl_dr;v.cbar*v.Cm_de 0 0;0 v.b*v.Cn_da v.b*v.Cn_dr];
 lo=max(-v.delta_max(:),c.delta-v.delta_dot_max(:)*c.period);hi=min(v.delta_max(:),c.delta+v.delta_dot_max(:)*c.period);
 c.delta=bounded_control_allocation(M,input(1:3)-input(4:6),lo,hi);c.time=b.CurrentTime;state(key)=c;
end
b.OutputPort(1).Data=c.delta;
end
