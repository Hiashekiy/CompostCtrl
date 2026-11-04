function [air,detail]=online_flight_condition(position,velocity_body)
% Current or predicted SI air data; no reference trajectory is consulted.
persistent env polar
if isempty(env)
 root=repo_root;env=load(fullfile(root,'simulink','data','native_environment.mat'));
 polar=load(fullfile(root,'simulink','data','public_surrogate','force_polar.mat'));
end
h=-position(3);speed=norm(velocity_body);alpha=atan2(velocity_body(3),velocity_body(1));
beta=asin(min(1,max(-1,velocity_body(2)/max(speed,1))));
assert(all(isfinite([position(:);velocity_body(:)])),'online:state','Nonfinite flight state');
assert(h>=env.height_m(1) && h<=env.height_m(end),'online:atmosphereRange','Height outside atmosphere table');
assert(alpha>=polar.alpha_rad(1) && alpha<=polar.alpha_rad(end),'online:alphaRange','Alpha outside force polar');
rho=interp1(env.height_m,env.rho,h,'linear');air=[.5*rho*speed^2;speed;alpha;beta];detail=[h;speed;rho;air(1);alpha;beta];
end
