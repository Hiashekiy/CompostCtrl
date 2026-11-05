function plan=v3_plan(group,cfg)
s=v3_spec;plan={};if nargin>1,s.pid=cfg;end
switch group
case 'E1'
for a=[5 7.5 10 12.5 15 17.5 20 22.5 25],for dur=[2 4 6 8 10 12 16 20],for cmg=[false true],v=s;v.amplitude=a;v.transition=dur;v.cmg=cmg;v.pair_id=sprintf('V1_%04d_%04d',round(a*10),round(dur*10));v.id=[v.pair_id '_' char('A'+cmg)];plan{end+1}=v;end,end,end
case 'E3'
for name={'L','S','Z'},v=s;v.id=['V3_' name{1}];v.pair_id='migration';v.onset=35;if strcmp(name{1},'S'),v.dts=make_paper_grid(s.dts,5);end;if strcmp(name{1},'Z'),v.amplitude=0;end;plan{end+1}=v;end
case 'E4'
for a=[1 3 5],for hz=[.02 .04 .06 .08 .1 .12 .15 .18 .2],for method=1:3,v=s;v.amplitude=a;v.frequency=hz;v.profile='sine';v.duration=8+4/hz;v.pid.unload=false;v.cmg=method~=1;if method==2,v.method='PID_CHAIN_CMG';end;v.pair_id=sprintf('V4_%03d_%03d',round(a*10),round(hz*1000));v.id=sprintf('%s_%d',v.pair_id,method);plan{end+1}=v;end,end,end
case 'E2'
for a=[5 10 15],for method=1:3,v=s;v.amplitude=a;v.pair_id=sprintf('V2_S%02d',a);v=select(v,method);plan{end+1}=v;end,end
for task=1:2,for method=1:3,v=s;v.pair_id=sprintf('V2_C%d',task);if task==1,v.events=[8 5 0 8 0;22 5 0 -8 0;36 5 6 0 0;48 4 -6 0 0];else,v.events=[8 4 0 4 0;20 4 0 4 0;32 4 0 4 0];end;v=select(v,method);plan{end+1}=v;end,end
for task=1:2,for method=1:3,v=s;v.pair_id=sprintf('V2_M%d',task);v.onset=35;v.amplitude=5;if task==1,v.gamma0=[pi/2;pi/2;0;0];else,v.gamma0=ones(4,1)*1.5;end;v=select(v,method);plan{end+1}=v;end,end
otherwise,error('Unknown group');end
for k=1:numel(plan),plan{k}.group=group;if strcmp(plan{k}.method,'MPC'),plan{k}.pid=struct;end;end
function v=select(v,method)
v.id=sprintf('%s_%d',v.pair_id,method);if method==1,v.method='PID_surface_first';elseif method==2,v.method='PID_surface_first_unload';v.pid.unload=true;else,v.method='MPC';end
end
end
