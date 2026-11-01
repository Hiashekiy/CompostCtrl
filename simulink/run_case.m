function res = run_case(tag, mode, varargin)
%RUN_CASE  Build the Simulink model for one exported case, run it, compare.
%
%   run_case('A4_descent_replay_S00_with_cmg_P0')               % open-loop replay
%   run_case('A4_descent_replay_S00_with_cmg_P0','closedloop')  % TCP controller
%
% mode 'replay'      the recorded Python control sequence drives the plant, so
%                    only *plant* fidelity is exercised (doc criterion C3 for the
%                    plant: every logged state must match Python).
% mode 'closedloop'  the controller block talks to the Python RTI controller
%                    (simulink/py/ctrl_server.py) over TCP: same controller, same
%                    plant, same scenario -> a true closed-loop port.
%
% Artifacts
%   results/simulink_port/runs/<tag>_<mode>.csv            logged time series
%   results/simulink_port/runs/<tag>_<mode>_metrics.json   comparison metrics
%   results/simulink_port/runs/<tag>_<mode>.slx            generated model
%   figures/simulink_port/<tag>_<mode>.png                 comparison figure
%
% Before the model is built, the run snaps its inputs to fixed paths
% (data/CURRENT_case.json, data/CURRENT_u.csv) because the Level-2 S-function
% setup runs as soon as a block is added and the MATLAB workspace turned out to
% be an unreliable transport in this installation.

if nargin < 2 || isempty(mode), mode = 'replay'; end
p = inputParser;
p.addParameter('Port', 55901);
p.addParameter('SaveModel', true);
p.parse(varargin{:});
opt = p.Results;

this = fileparts(mfilename('fullpath'));
addpath(this);
addpath(fullfile(this, 'blocks'));
datadir = fullfile(pwd, 'results', 'simulink_port', 'data');
outdir = fullfile(pwd, 'results', 'simulink_port', 'runs');
figdir = fullfile(pwd, 'figures', 'simulink_port');
if ~exist(outdir, 'dir'), mkdir(outdir); end
if ~exist(figdir, 'dir'), mkdir(figdir); end

case_file = fullfile(datadir, [tag '_case.json']);
P = s08_case_load(case_file);
fprintf('\n==== %s  (%s, nx=%d, nu=%d, %d steps of %.2f s) ====\n', ...
        tag, mode, P.nx, P.nu, P.n_steps, P.dt);

% ---------------------------------------------------------- run snapshots --
copyfile(case_file, fullfile(datadir, 'CURRENT_case.json'));
write_replay_csv(fullfile(datadir, 'CURRENT_u.csv'), P);

% ---------------------------------------------------------------- model ----
m = ['s08_' tag];
if bdIsLoaded(m), close_system(m, 0); end
new_system(m);
set_param(m, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', ...
          'FixedStep', sprintf('%.10g', P.dt), ...
          'StopTime', sprintf('%.10g', P.n_steps * P.dt), ...
          'SaveTime', 'on', 'SaveOutput', 'on', ...
          'SaveFormat', 'StructureWithTime', 'ReturnWorkspaceOutputs', 'on');

lvl2 = 'simulink/User-Defined Functions/Level-2 MATLAB S-Function';
add_block(lvl2, [m '/plant'], 'FunctionName', 's08_plant_sfun', ...
          'Position', [220 91 300 149]);
add_block('simulink/Sinks/Out1', [m '/x'], 'Position', [360 100 390 140]);
add_line(m, 'plant/1', 'x/1');

srv = [];
if strcmpi(mode, 'closedloop')
    srv = start_controller(case_file, opt.Port);
    add_block(lvl2, [m '/ctrl'], 'FunctionName', 's08_ctrl_sfun', ...
              'Position', [90 91 170 149]);
    add_line(m, 'plant/1', 'ctrl/1', 'autorouting', 'on');
    add_line(m, 'ctrl/1', 'plant/1', 'autorouting', 'on');
else
    add_block(lvl2, [m '/u'], 'FunctionName', 's08_replay_sfun', ...
              'Position', [90 96 170 144]);
    add_line(m, 'u/1', 'plant/1');
end

% ----------------------------------------------------------------- run ----
t0 = tic;
out = sim(m);
wall = toc(t0);
fprintf('[run] model wall clock %.1f s\n', wall);
if opt.SaveModel
    try
        save_system(m, fullfile(outdir, [tag '_' mode '.slx']));
    catch err
        fprintf('[warn] save_system failed: %s\n', err.message);
    end
end
close_system(m, 0);
if ~isempty(srv), stop_controller(srv); end

y = out.get('yout');
t = y.time;
X = y.signals.values;
if size(X, 2) ~= P.nx, X = X.'; end
% the model logs x(0) first; the Python reference logs the state after each
% control interval, so drop x(0) and compare the remaining samples one to one
if size(X, 1) == P.n_steps + 1
    X = X(2:end, :);
    t = t(2:end);
end
if size(X, 1) ~= P.n_steps
    error('unexpected log size: %d samples (expect %d)', size(X, 1), P.n_steps);
end

% ------------------------------------------------------------- compare ----
ref = P.ref_x;
n = size(X, 1);
dX = abs(X - ref);
scale = max(1.0, max(abs(ref), [], 'all'));
err_py = P.ref_att_err_deg;
err_m = attitude_error_deg(X(:, 1:4), P.diag.q_ref);
d_err = abs(err_m - err_py);

metrics = struct();
metrics.tag = tag;
metrics.mode = mode;
metrics.samples = n;
metrics.nx = P.nx;
metrics.wall_s = wall;
metrics.state_max_abs_diff = max(dX, [], 'all');
metrics.state_max_rel_diff = max(dX, [], 'all') / scale;
[~, ws] = max(max(dX, [], 1));
metrics.state_worst_state = ws;
metrics.att_err_max_abs_diff_deg = max(d_err);
metrics.rmse_python_deg = sqrt(mean(err_py .^ 2));
metrics.rmse_matlab_deg = sqrt(mean(err_m .^ 2));
metrics.max_err_python_deg = max(err_py);
metrics.max_err_matlab_deg = max(err_m);
metrics.final_err_python_deg = err_py(end);
metrics.final_err_matlab_deg = err_m(end);
metrics.C3_state_rel_tol = 1e-9;
metrics.C3_err_tol_deg = 1e-6;
metrics.C3_state_pass = metrics.state_max_rel_diff <= metrics.C3_state_rel_tol;
metrics.C3_err_pass = metrics.att_err_max_abs_diff_deg <= metrics.C3_err_tol_deg;

if P.cmg_present
    cmg = zeros(n, 3);
    for k = 1:n
        G = X(k, 8:11);
        s = P.s0 .* cos(G(:).') + P.gxs0 .* sin(G(:).');
        A = P.h0 * s08_cross3(P.g_axis, s);
        if strcmpi(mode, 'replay')
            nu = P.ref_u(k, 4:7).';
        else
            nu = zeros(4, 1);          % not logged in closed loop yet
        end
        cmg(k, :) = (-A * nu).';
    end
    metrics.cmg_x_peak_matlab = max(abs(cmg(:, 1)));
    metrics.cmg_y_peak_matlab = max(abs(cmg(:, 2)));
    metrics.cmg_z_peak_matlab = max(abs(cmg(:, 3)));
end

fprintf('[cmp] state max|dx| = %.3e (rel %.3e), worst state #%d\n', ...
        metrics.state_max_abs_diff, metrics.state_max_rel_diff, ...
        metrics.state_worst_state);
fprintf('[cmp] attitude error: RMSE py=%.6f deg  ml=%.6f deg  max|d|=%.3e deg\n', ...
        metrics.rmse_python_deg, metrics.rmse_matlab_deg, ...
        metrics.att_err_max_abs_diff_deg);
fprintf('[cmp] C3 state %s, C3 error %s\n', local_tag(metrics.C3_state_pass), ...
        local_tag(metrics.C3_err_pass));

% ------------------------------------------------------------------ I/O ----
write_csv(fullfile(outdir, [tag '_' mode '.csv']), t, X, err_m, P);
fid = fopen(fullfile(outdir, [tag '_' mode '_metrics.json']), 'w');
fprintf(fid, '%s', jsonencode(metrics, 'PrettyPrint', true));
fclose(fid);
plot_result(fullfile(figdir, [tag '_' mode '.png']), t, err_m, err_py, X, P, metrics);
res = metrics;
end

% ==========================================================================
function write_replay_csv(f, P)
fid = fopen(f, 'w');
fprintf(fid, 'time');
for j = 1:P.nu, fprintf(fid, ',u%d', j); end
fprintf(fid, '\n');
tvec = (0:P.n_steps-1).' * P.dt;
for k = 1:P.n_steps
    fprintf(fid, '%.10g', tvec(k));
    fprintf(fid, ',%.10g', P.ref_u(k, :));
    fprintf(fid, '\n');
end
fclose(fid);
end

function srv = start_controller(case_file, port)
%START_CONTROLLER  Write the controller config and wait for the Python server.
%
% The server itself is launched by the caller (scripts that own the shell), so
% this function only publishes data/CURRENT_ctrl.json and blocks until the
% server answers a ping.  Keeping the launch outside MATLAB avoids cmd/encoding
% issues with the non-ASCII workspace path.
info = struct('case_file', case_file, 'port', port);
fid = fopen(fullfile(pwd, 'results', 'simulink_port', 'data', 'CURRENT_ctrl.json'), 'w');
fprintf(fid, '%s', jsonencode(info));
fclose(fid);
log = fullfile(pwd, 'results', 'simulink_port', 'runs', 'ctrl_server.log');
srv = struct('port', port, 'log', log, 'ok', false);
for k = 1:240
    try
        j = local_jsocket(port, 2);
        local_jsend(j, '{"cmd":"ping"}');
        r = local_jrecv(j);
        local_jclose(j);
        if contains(r, 'ok'), srv.ok = true; break, end
    catch
        pause(0.5);
    end
end
if ~srv.ok
    error(['controller server did not answer on port %d. Start it first:' ...
           '\n  python simulink/py/ctrl_server.py --case <case yaml> --port %d'], ...
          port, port);
end
end

function stop_controller(srv)
try
    j = local_jsocket(srv.port, 2);
    local_jsend(j, '{"cmd":"quit"}');
    pause(0.2);
    local_jclose(j);
catch
end
end

function j = local_jsocket(port, timeout_s)
addr = java.net.InetAddress.getByName('127.0.0.1');
j.sock = java.net.Socket(addr, port);
j.sock.setSoTimeout(int32(timeout_s * 1000));
j.in = java.io.BufferedReader(java.io.InputStreamReader(j.sock.getInputStream()));
j.out = java.io.PrintWriter(j.sock.getOutputStream(), true);
end

function local_jsend(j, msg)
j.out.println(msg);
end

function r = local_jrecv(j)
r = char(j.in.readLine());
end

function local_jclose(j)
try, j.in.close(); catch, end
try, j.out.close(); catch, end
try, j.sock.close(); catch, end
end

function e = attitude_error_deg(q, qref)
% Same definition as Python's attitude_error_vector: |2 * vec(qe)| in degrees.
% (2*acos(qe_w) would differ from it by ~1e-3 deg at 5 deg of error.)
n = size(q, 1);
e = zeros(n, 1);
for k = 1:n
    a = qref(k, :); b = q(k, :);
    a = a / norm(a); b = b / norm(b);
    qe = [a(1) * b(1) + a(2) * b(2) + a(3) * b(3) + a(4) * b(4);
          a(1) * b(2) - a(2) * b(1) - a(3) * b(4) + a(4) * b(3);
          a(1) * b(3) + a(2) * b(4) - a(3) * b(1) - a(4) * b(2);
          a(1) * b(4) - a(2) * b(3) + a(3) * b(2) - a(4) * b(1)];
    if qe(1) < 0, qe = -qe; end
    e(k) = 2 * norm(qe(2:4)) * 180 / pi;
end
end

function s = local_tag(c, a, b)
if nargin < 2, a = 'PASS'; b = 'FAIL'; end
if c, s = a; else, s = b; end
end

function write_csv(f, t, X, err, P)
fid = fopen(f, 'w');
fprintf(fid, 't_next,q0,q1,q2,q3,wx,wy,wz');
if P.cmg_present, fprintf(fid, ',g1,g2,g3,g4'); end
fprintf(fid, ',att_err_deg\n');
for k = 1:numel(t)
    fprintf(fid, '%.10g', t(k));
    fprintf(fid, ',%.10g', X(k, 1:7));
    if P.cmg_present, fprintf(fid, ',%.10g', X(k, 8:11)); end
    fprintf(fid, ',%.10g\n', err(k));
end
fclose(fid);
end

function plot_result(fig, t, err_m, err_py, X, P, metrics) %#ok<INUSD>
try
    f = figure('Visible', 'off', 'Position', [100 100 900 700]);
    subplot(2, 1, 1);
    plot(t, err_py, 'k--', 'LineWidth', 1.0); hold on;
    plot(t, err_m, 'r-', 'LineWidth', 1.0);
    grid on; ylabel('attitude error [deg]'); xlabel('t [s]');
    legend({'Python reference', 'MATLAB/Simulink'}, 'Location', 'northwest');
    title(sprintf('%s (%s): max|dx|=%.2e, max|d err|=%.2e deg', ...
          metrics.tag, metrics.mode, metrics.state_max_abs_diff, ...
          metrics.att_err_max_abs_diff_deg), 'Interpreter', 'none');
    subplot(2, 1, 2);
    plot(t, err_m - err_py, 'b-');
    grid on; ylabel('error diff [deg]'); xlabel('t [s]');
    print(f, fig, '-dpng', '-r120');
    close(f);
catch err
    fprintf('[warn] plot failed: %s\n', err.message);
end
end
