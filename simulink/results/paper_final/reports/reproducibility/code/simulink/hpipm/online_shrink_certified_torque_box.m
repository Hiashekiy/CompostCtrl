function [mpc_min,mpc_max,center,halfwidth]=online_shrink_certified_torque_box(raw_min,raw_max,alpha)
% Uniform relative shrinkage of an already certified three-axis torque box.
%
% The certified box is raw_min <= tau <= raw_max, built at each prediction stage by
% online_fast_torque_box / online_certified_torque_box.  With
%     c = (raw_max+raw_min)/2,  h = (raw_max-raw_min)/2
% the box fed to the QP is
%     tau_min = c-(1-alpha).*h,  tau_max = c+(1-alpha).*h.
% The centre is untouched and (1-alpha) <= 1, so the new box is a SUBSET of the
% certified one: every point it contains was certified by the eight-vertex /
% monotone-interval certificate of the raw box.  The zero-width axes stay zero
% width, and a degenerate or non-finite box is refused instead of being widened.
%
% alpha == 0 returns the raw bounds bit for bit (no arithmetic at all), so the
% production configuration cannot drift by rounding.
%
% This is only a more conservative TORQUE CONSTRAINT for the upper-level MPC.  It
% does NOT assert that the real servo can hold that torque over a whole 0.2 s.
raw_min=raw_min(:);raw_max=raw_max(:);
assert(numel(raw_min)==3 && numel(raw_max)==3,'online:shrink','torque box must have three axes');
assert(all(isfinite([raw_min;raw_max])),'online:shrink','non-finite torque box');
assert(all(raw_min<=raw_max),'online:shrink','torque box endpoints are reversed or empty');
assert(isscalar(alpha) && isfinite(alpha) && alpha>=0 && alpha<1, ...
 'online:shrink','shrink ratio must satisfy 0 <= alpha < 1 (got %g)',alpha);
if alpha==0
 mpc_min=raw_min;mpc_max=raw_max;
else
 c=(raw_max+raw_min)/2;h=(raw_max-raw_min)/2;
 mpc_min=c-(1-alpha).*h;mpc_max=c+(1-alpha).*h;
end
center=(mpc_max+mpc_min)/2;halfwidth=(mpc_max-mpc_min)/2;
assert(all(isfinite([mpc_min;mpc_max;center;halfwidth])) && all(mpc_min<=mpc_max), ...
 'online:shrink','shrink produced an invalid torque box');
end
