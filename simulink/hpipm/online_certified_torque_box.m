function box=online_certified_torque_box(natural,B,lower,upper,center,tolerance)
% Certifies all eight vertices by bounded equality feasibility, including rank loss.
% This geometric certificate applies to the supplied deflection set only.
if nargin<6,tolerance=1e-9;end
lower=lower(:);upper=upper(:);center=center(:);natural=natural(:);
assert(isequal(size(B),[3 3]) && numel(natural)==3 && numel(lower)==3 && numel(upper)==3 && numel(center)==3);
assert(all(isfinite([natural;B(:);lower;upper;center])) && all(lower<=upper));
assert(isscalar(tolerance) && isfinite(tolerance) && tolerance>0);
center=min(max(center,lower),upper);radius=min(center-lower,upper-center);
half=abs(B)*radius;tau_center=natural+B*center;
signs=2*(dec2bin(0:7,3)-'0')-1;
inverse=[];if rcond(B)>1e-10,inverse=B\eye(3);end
% Connected output rows share actuators; independent pitch must retain its
% complete interval even when the roll/yaw group requires a smaller inner box.
support=B~=0;connected=(double(support)*double(support)')>0;
connected=connected | logical(eye(3));
for j=1:3,connected=connected | (double(connected(:,j))*double(connected(j,:))>0);end
remaining=true(3,1);groups={};
while any(remaining)
 first=find(remaining,1);rows=find(connected(first,:));groups{end+1}=rows;remaining(rows)=false; %#ok<AGROW>
end
scales=zeros(3,1);
if ~isempty(inverse)
 demand=abs(inverse)*half;
 for g=1:numel(groups)
  rows=groups{g};actuators=any(support(rows,:),1)' & demand>0;
  if numel(rows)==1 && nnz(support(rows,:))==1,scales(rows)=1;continue;end
  value=1;if any(actuators),value=min([1;radius(actuators)./demand(actuators)]);end
  if value<1,value=value*(1-1e-12);end
  scales(rows)=value;
 end
else
 for g=1:numel(groups)
  rows=groups{g};trial=scales;trial(rows)=1;
  if feasible(trial),scales=trial;continue;end
  lo=0;hi=1;
  for j=1:40
   mid=(lo+hi)/2;trial=scales;trial(rows)=mid;
   if feasible(trial),lo=mid;else,hi=mid;end
  end
  scales(rows)=lo;
 end
end
[ok,residual,vertices]=feasible(scales);assert(ok);
box=struct('tau_min',tau_center-scales.*half,'tau_max',tau_center+scales.*half, ...
 'center_delta',center,'scale',min(scales),'axis_scale',scales,'vertex_residual',residual,'vertex_delta',vertices);
 function [ok,residual,vertices]=feasible(scale)
  residual=zeros(1,8);vertices=zeros(3,8);
  for k=1:8
   target=B*center+scale.*half.*signs(k,:)';
   if ~isempty(inverse)
    trial=inverse*target;vertices(:,k)=min(max(trial,lower),upper);err=B*vertices(:,k)-target;
   else
    [vertices(:,k),err]=bounded_control_allocation(B,target,lower,upper);
   end
   residual(k)=norm(err,inf);
  end
  ok=all(residual<=tolerance);
 end
end
