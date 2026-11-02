function [u,residual]=bounded_control_allocation(M,target,lower,upper)
% Exact active-set enumeration for the small 3/4-actuator bounded LS problem.
lower=lower(:);upper=upper(:);target=target(:);n=numel(lower);
assert(n<=4 && all(lower<=upper),'Invalid bounded allocation dimensions/bounds');
best=inf;u=lower;
for code=0:3^n-1
 modes=mod(floor(code./3.^(0:n-1)),3);fixed=modes~=1;free=~fixed;
 trial=zeros(n,1);trial(modes==0)=lower(modes==0);trial(modes==2)=upper(modes==2);
 if any(free),trial(free)=pinv(M(:,free))*(target-M(:,fixed)*trial(fixed));end
 if any(trial<lower-1e-10 | trial>upper+1e-10),continue;end
 trial=min(max(trial,lower),upper);cost=norm(M*trial-target)^2;
 if cost<best,best=cost;u=trial;end
end
residual=M*u-target;
end
