function s08_plant_sfun(block)
%S08_PLANT_SFUN  Level-2 MATLAB S-Function wrapper around s08_plant_step.
%
% The plant state lives in Dwork(1); one input (u), one output (x), discrete at
% the control period of the case.  Direct feedthrough is off, so the controller
% sees x(t) and drives x(t+dt) without an algebraic loop.
%
% The case snapshot is read from results/simulink_port/data/CURRENT_case.json,
% which simulink/run_case.m refreshes before the block is created (the Level-2
% setup runs as soon as the block is added; dialog parameters, base-workspace
% variables and block handles all behaved unreliably in this installation).
%
% The parsed parameters are held in a function persistent, because block.UserData
% is only writable from the runtime callbacks, not from setup.

setup(block);

    function setup(block)
        block.NumDialogPrms = 0;
        case_file = local_case_file();
        P = s08_case_load(case_file);
        local_cache(P);

        block.NumInputPorts = 1;
        block.NumOutputPorts = 1;
        block.SetPreCompInpPortInfoToDynamic;
        block.SetPreCompOutPortInfoToDynamic;
        block.InputPort(1).Dimensions = P.nu;
        block.InputPort(1).DirectFeedthrough = false;
        block.InputPort(1).SamplingMode = 'Sample';
        block.OutputPort(1).Dimensions = P.nx;
        block.OutputPort(1).SamplingMode = 'Sample';

        block.NumContStates = 0;
        block.SampleTimes = [P.dt 0];
        block.SimStateCompliance = 'DefaultSimState';
        block.SetAccelRunOnTLC(false);

        block.RegBlockMethod('PostPropagationSetup', @postPropSetup);
        block.RegBlockMethod('Start', @start);
        block.RegBlockMethod('Outputs', @outputs);
        block.RegBlockMethod('Update', @update);
        block.RegBlockMethod('Terminate', @terminate);
    end

    function postPropSetup(block)
        P = local_cache();
        block.NumDworks = 1;
        block.Dwork(1).Name = 'x';
        block.Dwork(1).Dimensions = P.nx;
        block.Dwork(1).DatatypeID = 0;              % double
        block.Dwork(1).Complexity = 'Real';
    end

    function start(block)
        P = local_cache();
        block.Dwork(1).Data = P.x0;
    end

    function outputs(block)
        block.OutputPort(1).Data = block.Dwork(1).Data;
    end

    function update(block)
        P = local_cache();
        x = block.Dwork(1).Data;
        u = block.InputPort(1).Data;
        k = round(block.CurrentTime / P.dt) + 1;     % 1-based control index
        k = min(max(k, 1), P.n_steps);
        td_sub = reshape(P.tau_d(k, :, :), P.substeps, 3).';
        block.Dwork(1).Data = s08_plant_step(P, x, u, td_sub);
    end

    function terminate(block)                        %#ok<INUSD>
    end
end

function case_file = local_case_file()
case_file = char(fullfile(pwd, 'results', 'simulink_port', 'data', ...
                         'CURRENT_case.json'));
if exist(case_file, 'file') ~= 2
    error('s08_plant_sfun:noCase', 'current case file missing: %s', case_file);
end
end

function P = local_cache(newP)
persistent CACHE
if nargin > 0
    CACHE = newP;
end
P = CACHE;
end
