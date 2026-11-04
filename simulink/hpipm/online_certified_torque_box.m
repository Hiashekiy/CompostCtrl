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
lo=0;hi=1;
if ~isempty(inverse)
 demand=abs(inverse)*half;
 active=demand>0;
 if any(active),lo=min([1;radius(active)./demand(active)])*(1-1e-12);else,lo=1;end
elseif feasible(1),lo=1;else
 for j=1:40
  mid=(lo+hi)/2;
  if feasible(mid),lo=mid;else,hi=mid;end
 end
end
[ok,residual,vertices]=feasible(lo);assert(ok);
box=struct('tau_min',tau_center-lo*half,'tau_max',tau_center+lo*half, ...
 'center_delta',center,'scale',lo,'vertex_residual',residual,'vertex_delta',vertices);
 function [ok,residual,vertices]=feasible(scale)
  residual=zeros(1,8);vertices=zeros(3,8);
  for k=1:8
   target=B*center+scale*half.*signs(k,:)';
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
