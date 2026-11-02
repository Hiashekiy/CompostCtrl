function s08_model_preload(tag, mode)
%S08_MODEL_PRELOAD  Model PreLoadFcn: put the paths and inputs in place.
%
% The generated models (results/simulink_port/runs/<tag>_<mode>.slx) read their
% inputs from fixed paths (data/CURRENT_case.json, CURRENT_u.csv,
% CURRENT_ctrl.json) because the Level-2 S-function setup cannot be handed data
% any other way in this installation.  That makes a bare double-click dangerous:
% the snapshot on disk belongs to whichever case ran last.
%
% This function is therefore called from each model's PreLoadFcn.  It
%   1. adds simulink/ and simulink/blocks/ to the path (resolved relative to this
%      file, so the model works from anywhere), and
%   2. restores this model's own snapshot (runs/<tag>_<mode>_case.json / _u.csv),
%      which run_case.m writes next to the model.
%
% For mode 'closedloop' it also publishes data/CURRENT_ctrl.json pointing at the
% case, so the only thing missing at Run time is the Python controller itself.

here = fileparts(mfilename('fullpath'));
addpath(here);
addpath(fullfile(here, 'blocks'));

runs = fullfile(fileparts(here), 'results', 'simulink_port', 'runs');
data = fullfile(fileparts(here), 'results', 'simulink_port', 'data');
src_case = fullfile(runs, sprintf('%s_%s_case.json', tag, mode));
src_u = fullfile(runs, sprintf('%s_%s_u.csv', tag, mode));
if exist(src_case, 'file') ~= 2
    warning('s08_model_preload:noSnapshot', ...
            ['no snapshot next to the model (%s); the current data/CURRENT_* ' ...
             'files will be used, which may belong to another case'], src_case);
else
    copyfile(src_case, fullfile(data, 'CURRENT_case.json'));
    if exist(src_u, 'file') == 2
        copyfile(src_u, fullfile(data, 'CURRENT_u.csv'));
    end
end
if strcmpi(mode, 'closedloop')
    cfg = struct('case_file', fullfile(fileparts(here), 'configs', 'torque_gap_s08', ...
                 'cases', sprintf('%s.yaml', tag(1:end-3))), 'port', 55901);
    fid = fopen(fullfile(data, 'CURRENT_ctrl.json'), 'w');
    if fid > 0
        fprintf(fid, '%s', jsonencode(cfg));
        fclose(fid);
    end
    fprintf(['[s08] closed-loop model: start the controller first\n' ...
             '     python simulink/py/ctrl_server.py --case "%s" --port %d\n'], ...
            cfg.case_file, cfg.port);
end
end
