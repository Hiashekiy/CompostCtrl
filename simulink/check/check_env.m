function check_env(out_dir)
%CHECK_ENV  Probe the MATLAB/Simulink environment for the S08 port.
%
%   check_env                       % writes results/simulink_port/env_report.txt
%   check_env('some/dir')           % writes <dir>/env_report.txt
%
% Text only (no figures), safe for `matlab -batch`.  Every check is wrapped in
% try/catch so one missing product does not hide the rest.  The script returns
% normally when the *critical* checks pass (MATLAB, Simulink, headless sim,
% workspace writability) and errors otherwise, so the process exit status is
% usable from the shell.

if nargin < 1 || isempty(out_dir)
    out_dir = fullfile(pwd, 'results', 'simulink_port');
end
if ~exist(out_dir, 'dir'), mkdir(out_dir); end
report = fullfile(out_dir, 'env_report.txt');
L = {};
fail = {};

    function say(varargin)
        s = sprintf(varargin{:});
        L{end+1} = s;                                        %#ok<AGROW>
        fprintf('%s\n', s);
    end

    function tried(name, fn, critical)
        if nargin < 3, critical = false; end
        try
            msg = fn();
            say('[OK]    %-28s %s', name, msg);
        catch err
            say('[FAIL]  %-28s %s', name, err.message);
            if critical, fail{end+1} = name; end             %#ok<AGROW>
        end
    end

say('==== S08 Simulink port: environment check ====');
say('time            : %s', datestr(now, 'yyyy-mm-dd HH:MM:SS'));
say('version         : %s', version);
say('release         : %s', version('-release'));
say('burst           : %s', version('-description'));
say('matlabroot      : %s', matlabroot);
say('computer        : %s   ispc=%d  is64=%d', computer, ispc, ~isempty(strfind(computer, '64')));
say('prefdir         : %s', prefdir);
say('pwd             : %s', pwd);
say('');

% ---------------------------------------------------------------- products --
say('---- installed products (ver) ----');
v = ver;
for k = 1:numel(v)
    say('%-42s %-10s %s', v(k).Name, v(k).Version, v(k).Release);
end
say('');

% ---------------------------------------------------------------- licenses --
say('---- license(''test'', ...) ----');
prod_ids = {'MATLAB', 'Simulink', 'Aerospace_Blockset', 'Aerospace_Toolbox', ...
            'Simulink_Coder', 'Embedded_Coder', 'MATLAB_Coder', ...
            'Stateflow', 'Optimization_Toolbox', 'MPC_Toolbox', ...
            'Control_Toolbox', 'Simulink_Control_Design', ...
            'Real-Time_Workshop', 'Simulink_Real_Time'};
for k = 1:numel(prod_ids)
    try
        ok = license('test', prod_ids{k});
    catch
        ok = 0;
    end
    say('%-28s %d', prod_ids{k}, ok);
end
say('');

% ---------------------------------------------------------------- compiler --
tried('C compiler (mex)', @() local_c_compiler(), true);
tried('mex -setup writable', @() local_mex_dir());
say('');

% --------------------------------------------------------------- simulator --
tried('Simulink load', @() local_load_simulink(), true);
tried('headless sim', @() local_minimal_sim(out_dir), true);
tried('workspace writable', @() local_write_test(out_dir));
say('');

% ------------------------------------------------------- aerospace blockset --
tried('Aerospace Blockset blocks', @() local_aeroblks(), false);
say('');

say('---- summary ----');
if isempty(fail)
    say('critical checks: OK (%d)', 5);
else
    say('critical FAILED: %s', strjoin(fail, ', '));
end
fid = fopen(report, 'w');
fprintf(fid, '%s\n', L{:});
fclose(fid);
say('report written  : %s', report);

if ~isempty(fail)
    error('check_env:criticalFailed', 'critical check(s) failed: %s', strjoin(fail, ', '));
end
end

% ==========================================================================
function msg = local_c_compiler()
cfg = mex.getCompilerConfigurations('C', 'Selected');
if isempty(cfg)
    error('no C compiler selected: run mex -setup -client api');
end
msg = sprintf('%s %s (%s)', cfg.Name, cfg.Version, cfg.Location);
end

function msg = local_mex_dir()
d = fullfile(tempdir, 's08_mex_probe');
if ~exist(d, 'dir'), mkdir(d); end
msg = sprintf('temp dir writable: %s', d);
end

function msg = local_load_simulink()
load_system('simulink');
msg = sprintf('simulink library loaded (%s)', get_param('simulink', 'Version'));
end

function msg = local_minimal_sim(out_dir)
m = 's08_env_probe';
if bdIsLoaded(m), close_system(m, 0); end
new_system(m);
set_param(m, 'Solver', 'ode4', 'FixedStep', '0.01', 'StopTime', '1');
set_param(m, 'SaveFormat', 'StructureWithTime');
add_block('simulink/Sources/Constant', [m '/c'], 'Value', '3.5', ...
          'Position', [30 30 70 60]);
add_block('simulink/Math Operations/Gain', [m '/g'], 'Gain', '2', ...
          'Position', [120 30 160 60]);
add_line(m, 'c/1', 'g/1');
out = sim(m);
y = out.yout;
if isstruct(y), y = y.signals.values; end
final = y(end);
saved = fullfile(out_dir, [m '.slx']);
save_system(m, saved);
close_system(m, 0);
delete(saved);
if abs(final - 7) > 1e-9
    error('unexpected sim result: %g', final);
end
msg = sprintf('ode4 fixed step -> 3.5*2 = %g, model saved/loaded/deleted in %s', ...
              final, out_dir);
end

function msg = local_write_test(out_dir)
f = fullfile(out_dir, 'write_probe.txt');
fid = fopen(f, 'w');
if fid < 0, error('cannot write to %s', out_dir); end
fprintf(fid, 'ok\n');
fclose(fid);
delete(f);
msg = sprintf('%s', out_dir);
end

function msg = local_aeroblks()
try
    load_system('aeroblks');
catch
    error('aeroblks library not available (Aerospace Blockset not installed/licensed)');
end
b = find_system('aeroblks', 'LookUnderMasks', 'all', 'BlockType', 'SubSystem');
b = b(~cellfun(@isempty, strfind(b, 'DoF')) | ~cellfun(@isempty, strfind(b, 'DOF')));
if isempty(b)
    msg = 'library loaded, but no 6DoF block found by name (check manually)';
else
    msg = sprintf('%d 6DoF block(s): %s', numel(b), strjoin(b(:)', ' | '));
end
end
