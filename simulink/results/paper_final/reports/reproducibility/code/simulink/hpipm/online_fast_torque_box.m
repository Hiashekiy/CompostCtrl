function box=online_fast_torque_box(natural,B,lower,upper,center,tolerance)
% Analytic replacement for the eight-vertex inner torque box.
%
% The legacy function builds the box and then verifies all eight vertices one by one
% through the allocator. With tau = natural + B*delta the same certificate is a single
% linear condition: every point of the axis-aligned box {tau_c + e, |e| <= h} maps to
% delta = c + B^-1 e, whose worst-case magnitude per servo is exactly |B^-1|*h. Hence
%        all eight vertices (and the whole box) are admissible  <=>  |B^-1| h <= r
% element-wise, with r the symmetric servo radius. The vertex ANGLE targets are then one
% matrix product instead of eight inverse solves, and the vertex torque residual is the
% same quantity evaluated vectorised.
%
% centre, radius, h0 = |B| r, the connected-axis grouping, the per-group scaling and the
% (1-1e-12) margin are taken over unchanged from online_certified_torque_box, so the box
% bounds stay identical; only the eight-vertex verification loop is removed.
%
% Any condition that cannot be proven analytically (singular B, angle tolerance, vertex
% torque residual) falls back to the legacy function and says so in certificate_type.
persistent v signs
if isempty(v)
 p=vehicle_params;v=p.vehicle;
 signs=2*(dec2bin(0:7,3)-'0')-1;   % cached: constant bit pattern, not physics
end
if nargin<6,tolerance=1e-9;end
lower=lower(:);upper=upper(:);center=center(:);natural=natural(:);
assert(isequal(size(B),[3 3]) && numel(natural)==3 && numel(lower)==3 && numel(upper)==3 && numel(center)==3);
assert(all(isfinite([natural;B(:);lower;upper;center])) && all(lower<=upper));
assert(isscalar(tolerance) && isfinite(tolerance) && tolerance>0);
% Angle tolerance: radians. eps(delta_max) is ~5.5e-17 here, so 1e-12 rad is a
% conservative machine-level bound that never widens the admissible interval.
tol_delta=1e-12*max(1,max(abs(v.delta_max(:))));
center=min(max(center,lower),upper);radius=min(center-lower,upper-center);
half=abs(B)*radius;tau_center=natural+B*center;
if rcond(B)<=1e-10
 box=online_certified_torque_box(natural,B,lower,upper,center,tolerance);
 box.certified=true;box.certificate_type='legacy_singular';box.margin_delta=NaN;box.fallback=true;return;
end
inverse=B\eye(3);
support=B~=0;connected=(double(support)*double(support)')>0;
connected=connected | logical(eye(3));
for j=1:3,connected=connected | (double(connected(:,j))*double(connected(j,:))>0);end
remaining=true(3,1);groups={};
while any(remaining)
 first=find(remaining,1);rows=find(connected(first,:));groups{end+1}=rows;remaining(rows)=false; %#ok<AGROW>
end
scales=zeros(3,1);demand=abs(inverse)*half;
for g=1:numel(groups)
 rows=groups{g};actuators=any(support(rows,:),1)' & demand>0;
 if numel(rows)==1 && nnz(support(rows,:))==1,scales(rows)=1;continue;end
 value=1;if any(actuators),value=min([1;radius(actuators)./demand(actuators)]);end
 if value<1,value=value*(1-1e-12);end
 scales(rows)=value;
end
h=scales.*half;
% Analytic certificate: worst-case servo excursion of the whole box.
allowed=abs(inverse)*h;margin=radius-allowed;
offset=inverse*(h.*signs');
vertex_delta=center+offset;
target_abs=B*center+(h.*signs');
residual=max(abs(B*vertex_delta-target_abs),[],1);
provable=all(margin>=-tol_delta) && all(vertex_delta>=lower-tol_delta,'all') ...
 && all(vertex_delta<=upper+tol_delta,'all') && all(residual<=tolerance);
if ~provable
 box=online_certified_torque_box(natural,B,lower,upper,center,tolerance);
 box.certified=true;box.certificate_type='legacy_unprovable';box.margin_delta=min(margin);box.fallback=true;return;
end
box=struct('tau_min',tau_center-h,'tau_max',tau_center+h,'center_delta',center, ...
 'scale',min(scales),'axis_scale',scales,'vertex_residual',residual, ...
 'vertex_delta',vertex_delta,'certified',true,'certificate_type','analytic_box', ...
 'margin_delta',min(margin),'fallback',false,'box_points',8);
end
