function s08_replay_sfun(block)
%S08_REPLAY_SFUN  Play back a recorded control sequence (open-loop validation).
%
% Reads results/simulink_port/data/CURRENT_u.csv (first column time, next nu
% columns the input) and outputs row k at t = k*dt.  Used instead of the
% "From Workspace" block because passing data through the MATLAB workspace
% proved unreliable in this installation; the file is written by
% simulink/run_case.m before the model is built.

setup(block);

    function setup(block)
        block.NumDialogPrms = 0;
        U = local_load();
        local_cache(U);
        block.NumInputPorts = 0;
        block.NumOutputPorts = 1;
        block.SetPreCompOutPortInfoToDynamic;
        block.OutputPort(1).Dimensions = U.nu;
        block.OutputPort(1).SamplingMode = 'Sample';
        block.SampleTimes = [U.dt 0];
        block.SimStateCompliance = 'DefaultSimState';
        block.RegBlockMethod('Outputs', @outputs);
    end

    function outputs(block)
        U = local_cache();
        k = round(block.CurrentTime / U.dt) + 1;
        k = min(max(k, 1), U.n);
        block.OutputPort(1).Data = U.values(k, :).';
    end
end

function U = local_load()
f = char(fullfile(pwd, 'results', 'simulink_port', 'data', 'CURRENT_u.csv'));
if exist(f, 'file') ~= 2
    error('s08_replay_sfun:noFile', 'replay input missing: %s', f);
end
M = dlmread(f, ',', 1, 0);
U.dt = M(2, 1) - M(1, 1);
U.values = M(:, 2:end);
U.nu = size(U.values, 2);
U.n = size(U.values, 1);
end

function U = local_cache(newU)
persistent CACHE
if nargin > 0
    CACHE = newU;
end
U = CACHE;
end
