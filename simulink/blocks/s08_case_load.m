function P = s08_case_load(case_file)
%S08_CASE_LOAD  Load one exported case snapshot (<tag>_case.json).
%
% Returns a flat parameter struct with exactly the quantities the plant needs.
% Everything here originates in the Python reference model; see
% simulink/py/export_for_simulink.py.

S = jsondecode(fileread(case_file));
P = struct();
P.tag = S.tag;
P.nx = double(S.nx);
P.nu = double(S.nu);
P.cmg_present = logical(S.cmg_present);
P.cmg_enabled = logical(S.cmg_enabled);
P.substeps = double(S.substeps);
P.n_steps = double(S.n_steps);
P.dt = double(S.dt);
P.t_start = double(S.t_start);
P.t_end = double(S.t_end);
P.J = diag(double(S.J(:)));
P.Jinv = diag(1 ./ double(S.J(:)));
P.x0 = double(S.x0(:));
td = double(S.tau_d);
if ndims(td) == 2 && size(td, 2) == 3 * P.substeps
    td = permute(reshape(td, size(td, 1), 3, P.substeps), [1 3 2]);
end
P.tau_d = td;                                  % (n_steps, substeps, 3)

if P.cmg_present
    g = S.cmg_geometry;
    P.h0 = double(g.h0);
    P.g_axis = double(g.g_axis);
    P.s0 = double(g.s0);
    P.gxs0 = double(g.gxs0);
    P.gdot_max = double(g.gdot_max);
    P.gddot_max = double(g.gddot_max);
    P.tau_g = double(g.tau_g);
else
    P.h0 = 0; P.g_axis = zeros(3, 4); P.s0 = zeros(3, 4);
    P.gxs0 = zeros(3, 4); P.gdot_max = 0; P.gddot_max = 0; P.tau_g = 0;
end
P.gamma_unbounded = logical(S.gamma_unbounded);

% recorded reference run (replay command + comparison baseline)
P.ref_u = double(S.ref_run.u);                 % (n_steps, nu)
P.ref_x = double(S.ref_run.x);                 % (n_steps, nx)
P.ref_att_err_deg = double(S.ref_run.att_err_deg(:));
P.diag = S.diag;
end
