function probe2(out_dir)
%PROBE2  Second-pass environment probe: headless sim, block libraries, encodings.
if nargin < 1 || isempty(out_dir), out_dir = fullfile(pwd, 'results', 'simulink_port'); end
if ~exist(out_dir, 'dir'), mkdir(out_dir); end
L = {};

    function say(varargin)
        s = sprintf(varargin{:});
        L{end+1} = s;                                        %#ok<AGROW>
        fprintf('%s\n', s);
    end

say('==== probe2 ====');
say('path handling: pwd=%s', pwd);

% ---- headless sim with a real root outport -------------------------------
m = 's08_env_probe';
try
    if bdIsLoaded(m), close_system(m, 0); end
    new_system(m);
    set_param(m, 'Solver', 'FixedStepDiscrete', 'FixedStep', '0.01', ...
              'StopTime', '1', 'SaveTime', 'on', 'SaveOutput', 'on', ...
              'SaveFormat', 'StructureWithTime', 'ReturnWorkspaceOutputs', 'on');
    add_block('simulink/Sources/Constant', [m '/c'], 'Value', '3.5', ...
              'Position', [30 30 70 60]);
    add_block('simulink/Math Operations/Gain', [m '/g'], 'Gain', '2', ...
              'Position', [120 30 160 60]);
    add_block('simulink/Sinks/Out1', [m '/y'], 'Position', [200 30 230 60]);
    add_line(m, 'c/1', 'g/1');
    add_line(m, 'g/1', 'y/1');
    out = sim(m);
    y = out.get('yout');
    vals = y.signals.values;
    say('[OK] headless sim: final=%g (expect 7), %d samples', vals(end), numel(vals));
    save_system(m, fullfile(out_dir, 'env_probe.slx'));
    close_system(m, 0);
    say('[OK] model save/close');
catch err
    say('[FAIL] headless sim: %s', err.message);
    if bdIsLoaded(m), close_system(m, 0); end
end

% ---- which block libraries are actually loadable ------------------------
say('---- block library probe ----');
libs = {'simulink', 'aeroblks', 'aeroblks_lib', 'aerospaceblks', 'aero_vroot', ...
        'aerolib', 'aerospace', 'aerosim', 'aeroanim', 'sl3d', 'sim3dblocks'};
for k = 1:numel(libs)
    nm = libs{k};
    try
        load_system(nm);
        b = find_system(nm, 'LookUnderMasks', 'all', 'SearchDepth', 1, ...
                        'BlockType', 'SubSystem');
        say('[OK]   %-14s %d top-level subsystem blocks', nm, numel(b));
    catch
        say('[--]   %-14s not loadable', nm);
    end
end

% ---- 6DoF blocks anywhere in the toolbox directories -------------------
say('---- 6DoF block search in toolbox/aeroblks ----');
d = fullfile(matlabroot, 'toolbox', 'aeroblks');
if exist(d, 'dir')
    files = dir(fullfile(d, '**', '*.slx'));
    say('aeroblks dir exists, %d .slx files', numel(files));
    hit = files(~cellfun(@isempty, regexpi({files.name}, '6dof|eom|equations')));
    for k = 1:min(numel(hit), 12)
        say('   %s', fullfile(hit(k).folder, hit(k).name));
    end
else
    say('no toolbox/aeroblks directory');
end

% ---- file I/O with the non-ASCII workspace path -------------------------
say('---- I/O with the chinese path ----');
f = fullfile(out_dir, 'io_probe.txt');
fid = fopen(f, 'w', 'n', 'UTF-8');
if fid < 0
    say('[FAIL] cannot fopen for write');
else
    fprintf(fid, 'alpha_deg,3.5\n');
    fclose(fid);
    fid = fopen(f, 'r');
    l = fgetl(fid); fclose(fid);
    say('[OK] write/read back: %s', l);
    delete(f);
end
csvf = fullfile(pwd, 'results', 'torque_gap_s08', 'A4_descent_replay', ...
                'A4_descent_replay_S00_with_cmg', 'timeseries.csv');
if exist(csvf, 'file')
    say('[OK] python result csv visible from MATLAB (%d bytes)', dir(csvf).bytes);
else
    say('[FAIL] python result csv not visible');
end

fid = fopen(fullfile(out_dir, 'probe2.txt'), 'w');
fprintf(fid, '%s\n', L{:});
fclose(fid);
say('done -> %s', fullfile(out_dir, 'probe2.txt'));
end
