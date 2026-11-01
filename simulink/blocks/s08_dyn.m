function dx = s08_dyn(P, x, u, td)
%S08_DYN  State derivative of the S08 P0 plant (nearspace.dynamics_tg).
%
%   B group (cmg_present): J wdot = tau_a - A(gamma) nu + tau_d - w x (J w + H)
%   A group             : J wdot = tau_a + tau_d - w x (J w)
dx = zeros(P.nx, 1);
q = x(1:4);
w = x(5:7);
tau_a = u(1:3);
dx(1:4) = 0.5 * s08_quat_mul(q, [0; w]);
if P.cmg_present
    nu = u(4:7);
    G = x(8:11);
    s = P.s0 .* cos(G(:).') + P.gxs0 .* sin(G(:).');
    H = P.h0 * sum(s, 2);
    A = P.h0 * s08_cross3(P.g_axis, s);
    dx(5:7) = P.Jinv * (tau_a - A * nu + td - cross(w, P.J * w + H));
    dx(8:11) = nu;
else
    dx(5:7) = P.Jinv * (tau_a + td - cross(w, P.J * w));
end
end
