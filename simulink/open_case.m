function open_case(tag, mode)
%OPEN_CASE  Re-snapshot the inputs of a case and open its generated model.
%
%   open_case('A4_descent_replay_S00_with_cmg_P0')            % replay model
%   open_case('A4_descent_replay_S00_with_cmg_P0','closedloop')
%
% The .slx files under results/simulink_port/runs/ are *products* of run_case.m
% and read their inputs from fixed paths (data/CURRENT_case.json, CURRENT_u.csv,
% CURRENT_ctrl.json).  After a fresh clone those paths hold whichever run was
% committed last, so opening a model and pressing Run without this helper would
% silently simulate a different case.  This function refreshes the snapshots of
% the requested case and then opens the model.
%
% For 'closedloop' the Python controller must be running, e.g.
%   python simulink/py/ctrl_server.py --case configs/torque_gap_s08/cases/<case>.yaml --port 55901

if nargin < 2 || isempty(mode), mode = 'replay'; end
addpath(fileparts(mfilename('fullpath')));
addpath(fullfile(fileparts(mfilename('fullpath')), 'blocks'));

datadir = fullfile(pwd, 'results', 'simulink_port', 'data');
runs = fullfile(pwd, 'results', 'simulink_port', 'runs');
case_file = fullfile(datadir, [tag '_case.json']);
if exist(case_file, 'file') ~= 2
    error('open_case:noCase', ['case snapshot missing: %s\n' ...
          'run: python simulink/py/export_for_simulink.py --all'], case_file);
end
P = s08_case_load(case_file);
copyfile(case_file, fullfile(datadir, 'CURRENT_case.json'));
fid = fopen(fullfile(datadir, 'CURRENT_u.csv'), 'w');
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
if strcmpi(mode, 'closedloop')
    cfg = struct('case_file', fullfile(pwd, 'configs', 'torque_gap_s08', 'cases', ...
                 [tag(1:end-3) '.yaml']), 'port', 55901);
    fid = fopen(fullfile(datadir, 'CURRENT_ctrl.json'), 'w');
    fprintf(fid, '%s', jsonencode(cfg));
    fclose(fid);
    fprintf(['[open_case] closed-loop model: start the controller first:\n' ...
             '  python simulink/py/ctrl_server.py --case "%s" --port %d\n'], ...
            cfg.case_file, cfg.port);
end
mdl = fullfile(runs, sprintf('%s_%s.slx', tag, mode));
if exist(mdl, 'file') ~= 2
    error('open_case:noModel', ['model missing: %s\n' ...
          'run: run_case(''%s'',''%s'')'], mdl, tag, mode);
end
fprintf('[open_case] %s  (nx=%d, nu=%d, %d steps)\n', mdl, P.nx, P.nu, P.n_steps);
if usejava('desktop')
    open_system(mdl);
else
    % headless session (matlab -batch): the Simulink Editor needs a desktop, so
    % only load the model; `sim` works from here (on the file path, because the
    % model name becomes the file base name when it is loaded from disk)
    load_system(mdl);
    fprintf('[open_case] headless: model loaded, run sim(''%s'')\n', mdl);
end
end
