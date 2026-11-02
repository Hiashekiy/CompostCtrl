function s08_startup()
%S08_STARTUP  One-shot path setup for running the S08 Simulink models by hand.
%
%   >> cd D:\项目\飞轮          % repository root
%   >> run('simulink/s08_startup.m')     % or: addpath simulink; s08_startup
%
% Then open a model *through* open_case so that the input snapshot is refreshed
% for that case (the models read data/CURRENT_case.json, which is a single
% shared slot):
%
%   >> open_case('A4_descent_replay_S00_with_cmg_P0','replay')   % then press Run
%   >> open_case('A4_descent_replay_S00_with_cmg_P0','closedloop') % needs the
%      %  Python controller: python simulink/py/ctrl_server.py --case <case yaml> --port 55901
%
% Pressing Run on a model that was opened directly from Explorer would reuse the
% snapshot of whichever case was simulated last - open_case is what prevents that.

here = fileparts(mfilename('fullpath'));
root = fileparts(here);
addpath(here);
addpath(fullfile(here, 'blocks'));
fprintf('\n[s08] paths added:\n   %s\n   %s\n', here, fullfile(here, 'blocks'));

runs = fullfile(root, 'results', 'simulink_port', 'runs');
f = dir(fullfile(runs, '*.slx'));
if isempty(f)
    fprintf('[s08] no models yet - run: python simulink/py/run_closedloop.py --all\n');
    return
end
fprintf('[s08] models in results/simulink_port/runs (%d):\n', numel(f));
for i = 1:numel(f)
    fprintf('   %s\n', f(i).name);
end
fprintf(['[s08] to open one with its own inputs:\n' ...
         '   open_case(''A4_descent_replay_S00_with_cmg_P0'',''replay'')      %% press Run\n' ...
         '   open_case(''A4_descent_replay_S00_with_cmg_P0'',''closedloop'')   %% needs ctrl_server.py\n' ...
         '   (tag = file name without the trailing _replay/_closedloop; the A-group\n' ...
         '    cases have nx=7/nu=3, all others nx=11/nu=7 - the snapshot must match)\n\n']);
end
