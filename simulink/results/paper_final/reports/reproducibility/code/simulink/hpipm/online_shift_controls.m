function U=online_shift_controls(previous,dts,dt_ctrl)
% Re-sample an accepted piecewise-constant torque plan one control period later.
% s.dts is a NON-UNIFORM grid (here 6x0.2, 4x0.5, 4x1, 8x2 s), so shifting the
% columns by one index is not a shift in time. Stage j of the new plan starts at
% new_time+sum(dts(1:j-1)) with new_time=dt_ctrl; the value there is the old stage
% that contains that instant (half-open intervals), holding the old final stage
% beyond the end of the old horizon. Only the control plan is reused: the state
% trajectory must always be re-propagated from the new measured state.
previous=double(previous);dt_ctrl=double(dt_ctrl);dts=double(dts(:));
N=numel(dts);
assert(size(previous,2)==N && ~isempty(previous),'online:shift', ...
 'Stored control plan has %d stages, the new grid has %d',size(previous,2),N);
starts=[0;cumsum(dts)];                 % old stage boundaries, starts(1:N) are stage starts
target=starts(1:N)+dt_ctrl;             % new stage starts expressed in old time base
index=zeros(1,N);
for j=1:N
 index(j)=find(starts(1:N)<=target(j)+1e-12,1,'last');
 if isempty(index(j)),index(j)=1;end
end
U=previous(:,index);
end
