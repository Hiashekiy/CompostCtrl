function prediction=online_translation_prediction(initial,times,quaternions,max_step)
% Numerical translation predictor, NOT an Aerospace 6DoF replacement.
% initial=[position_NED;velocity_NED]; attitude is scalar-first body-to-NED.
% Uses only existing vehicle/environment/polar MAT data; no reference bank.
if nargin<4,max_step=.001;end
initial=initial(:);times=times(:);
assert(numel(initial)==6 && all(isfinite(initial)));
assert(numel(times)>=2 && times(1)==0 && all(diff(times)>0));
assert(isequal(size(quaternions),[4 numel(times)]) && all(isfinite(quaternions),'all'));
assert(isscalar(max_step) && isfinite(max_step) && max_step>0);
persistent v env polar density lift drag
if isempty(v)
 p=vehicle_params;v=p.vehicle;root=repo_root;
 env=load(fullfile(root,'simulink','data','native_environment.mat'));
 polar=load(fullfile(root,'simulink','data','public_surrogate','force_polar.mat'));
 density=griddedInterpolant(env.height_m(:),env.rho(:),'linear','none');
 lift=griddedInterpolant(polar.alpha_rad(:),polar.CL(:),'linear','none');
 drag=griddedInterpolant(polar.alpha_rad(:),polar.CD(:),'linear','none');
end
assert(all(vecnorm(quaternions)>0));quaternions=quaternions./vecnorm(quaternions);
prediction.state=zeros(6,numel(times));prediction.state(:,1)=initial;
prediction.air=zeros(6,numel(times));
for k=1:numel(times)-1
 z=prediction.state(:,k);n=ceil((times(k+1)-times(k))/max_step);dt=(times(k+1)-times(k))/n;
 for j=1:n
  a=(j-1)/n;b=j/n;c=(a+b)/2;
  f1=rhs(z,attitude(a));f2=rhs(z+dt*f1/2,attitude(c));
  f3=rhs(z+dt*f2/2,attitude(c));f4=rhs(z+dt*f3,attitude(b));
  z=z+dt*(f1+2*f2+2*f3+f4)/6;
 end
 prediction.state(:,k+1)=z;
end
for k=1:numel(times)
 [~,prediction.air(:,k)]=rhs(prediction.state(:,k),quaternions(:,k));
end
prediction.time=times; % air rows: height,speed,rho,qbar,alpha,beta (predicted)
 function q=attitude(fraction)
  qa=quaternions(:,k);qb=quaternions(:,k+1);d=dot(qa,qb);
  if d<0,qb=-qb;d=-d;end
  d=min(1,d);
  if d>.9995,q=(1-fraction)*qa+fraction*qb;q=q/norm(q);
  else,angle=acos(d);q=(sin((1-fraction)*angle)*qa+sin(fraction*angle)*qb)/sin(angle);end
 end
 function [dz,air]=rhs(z,q)
  w=q(1);x=q(2);y=q(3);zz=q(4);
  R=[1-2*(y*y+zz*zz) 2*(x*y-w*zz) 2*(x*zz+w*y);2*(x*y+w*zz) 1-2*(x*x+zz*zz) 2*(y*zz-w*x);2*(x*zz-w*y) 2*(y*zz+w*x) 1-2*(x*x+y*y)];
  vb=R'*z(4:6);speed=norm(vb);h=-z(3);
  alpha=atan2(vb(3),vb(1));beta=asin(min(1,max(-1,vb(2)/max(speed,1))));
  assert(h>=env.height_m(1) && h<=env.height_m(end),'online:atmosphereRange','Height outside atmosphere table');
  assert(alpha>=polar.alpha_rad(1) && alpha<=polar.alpha_rad(end),'online:alphaRange','Alpha outside force polar');
  rho=density(h);Q=.5*rho*speed^2;
  L=Q*v.S*lift(alpha);
  D=Q*v.S*drag(alpha);
  force=[-D*cos(alpha)*cos(beta)+L*sin(alpha);-D*sin(beta);-D*sin(alpha)*cos(beta)-L*cos(alpha)];
  dz=[z(4:6);R*force/v.mass+[0;0;9.80665]];
  air=[h;speed;rho;Q;alpha;beta];
 end
end
