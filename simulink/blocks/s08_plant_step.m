function [xn, cmg_avg] = s08_plant_step(P, x, u, td_sub)
%S08_PLANT_STEP  One control interval of the P0 plant, exactly as Python does it.
%
%   [xn, cmg_avg] = s08_plant_step(P, x, u, td_sub)
%
% Mirrors nearspace.torque_gap_sim.ClosedLoop._step_P0:
%
%   for i = 1:substeps
%       x = rk4(x, u, td_sub(:, i), dt/substeps)
%   end
%   cmg_avg = (1/dt) * sum_i h * (-A(gamma_mid) nu)
%
% td_sub is 3 x substeps (per sub-step external torque) or 3 x 1 (constant).
% The quaternion is renormalised after every RK4 stage, as in rk4_step_tg.

sub = P.substeps;
h = P.dt / sub;
if size(td_sub, 2) == 1
    td_sub = repmat(td_sub, 1, sub);
end
u = u(:);
x = x(:);
if P.cmg_present
    nu = u(4:7);
    acc = zeros(3, 1);
    for i = 1:sub
        g0 = x(8:11);
        x = local_rk4(P, x, u, td_sub(:, i), h);
        gm = 0.5 * (g0 + x(8:11));
        s = P.s0 .* cos(gm(:).') + P.gxs0 .* sin(gm(:).');
        A = P.h0 * s08_cross3(P.g_axis, s);
        acc = acc + h * (-A * nu);
    end
    cmg_avg = acc / P.dt;
else
    for i = 1:sub
        x = local_rk4(P, x, u, td_sub(:, i), h);
    end
    cmg_avg = zeros(3, 1);
end
xn = x;
end

function x = local_rk4(P, x, u, td, h)
k1 = s08_dyn(P, x, u, td);
k2 = s08_dyn(P, x + 0.5 * h * k1, u, td);
k3 = s08_dyn(P, x + 0.5 * h * k2, u, td);
k4 = s08_dyn(P, x + h * k3, u, td);
x = x + (h / 6) * (k1 + 2 * k2 + 2 * k3 + k4);
q = x(1:4);
x(1:4) = q / norm(q);
end
