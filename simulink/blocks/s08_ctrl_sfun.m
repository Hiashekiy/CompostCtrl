function s08_ctrl_sfun(block)
%S08_CTRL_SFUN  Controller block: the Python RTI controller, over TCP.
%
% The block owns no control logic.  At every control instant it sends the
% measured state to simulink/py/ctrl_server.py (which runs the *same*
% Scenario/TorqueGapRTI/accept-fallback logic as the reference model) and returns
% the action it gets back.  This keeps the port honest: the controller is not
% re-implemented, so a difference can only come from the plant or the interface.
%
% Configuration comes from results/simulink_port/data/CURRENT_ctrl.json
% {case_file, port} plus CURRENT_case.json for the port widths; both are written
% by simulink/run_case.m before the model is built.
%
% Requests are de-duplicated on the current time, so an extra Outputs call by
% the solver cannot desynchronise the server.

setup(block);

    function setup(block)
        block.NumDialogPrms = 0;
        cfg = local_config();
        P = s08_case_load(cfg.case_file);
        local_cache(struct('cfg', cfg, 'P', P, 'sock', [], 'last_t', NaN, ...
                           'last_u', zeros(P.nu, 1), 'n', 0));

        block.NumInputPorts = 1;
        block.NumOutputPorts = 1;
        block.SetPreCompInpPortInfoToDynamic;
        block.SetPreCompOutPortInfoToDynamic;
        block.InputPort(1).Dimensions = P.nx;
        block.InputPort(1).DirectFeedthrough = true;   % u depends on x now
        block.InputPort(1).SamplingMode = 'Sample';
        block.OutputPort(1).Dimensions = P.nu;
        block.OutputPort(1).SamplingMode = 'Sample';

        block.SampleTimes = [P.dt 0];
        block.SimStateCompliance = 'DefaultSimState';
        block.RegBlockMethod('Start', @start);
        block.RegBlockMethod('Outputs', @outputs);
        block.RegBlockMethod('Terminate', @terminate);
    end

    function start(block)                                        %#ok<INUSD>
        C = local_cache();
        C.sock = local_open(C.cfg);
        local_cache(C);
    end

    function outputs(block)
        C = local_cache();
        t = block.CurrentTime;
        if t == C.last_t && C.n > 0
            block.OutputPort(1).Data = C.last_u;
            return
        end
        x = block.InputPort(1).Data;
        t_abs = C.P.t_start + t;
        req = struct('cmd', 'step', 't', t_abs, 'x', x(:).');
        resp = local_call(C.sock, req);
        if isfield(resp, 'error')
            error('s08_ctrl_sfun:server', 'controller error at t=%.2f: %s', ...
                  t_abs, resp.error);
        end
        u = resp.u(:);
        C.last_t = t;
        C.last_u = u(1:C.P.nu);
        C.n = C.n + 1;
        local_cache(C);
        block.OutputPort(1).Data = C.last_u;
    end

    function terminate(block)                                    %#ok<INUSD>
        C = local_cache();
        if ~isempty(C.sock)
            try
                local_call(C.sock, struct('cmd', 'quit'));
            catch
            end
            local_close(C.sock);
            C.sock = [];
            local_cache(C);
        end
    end
end

% --------------------------------------------------------------------------
function cfg = local_config()
f = char(fullfile(pwd, 'results', 'simulink_port', 'data', 'CURRENT_ctrl.json'));
if exist(f, 'file') ~= 2
    error('s08_ctrl_sfun:noCfg', 'controller config missing: %s', f);
end
cfg = jsondecode(fileread(f));
cfg.case_file = char(cfg.case_file);
cfg.port = double(cfg.port);
end

function C = local_cache(newC)
persistent CACHE
if nargin > 0
    CACHE = newC;
end
C = CACHE;
end

function s = local_open(cfg)
addr = java.net.InetAddress.getByName('127.0.0.1');
s.sock = java.net.Socket(addr, cfg.port);
s.sock.setSoTimeout(int32(120000));
s.in = java.io.BufferedReader(java.io.InputStreamReader(s.sock.getInputStream()));
s.out = java.io.PrintWriter(s.sock.getOutputStream(), true);
r = local_call(s, struct('cmd', 'ping'));
if ~isfield(r, 'ok')
    error('s08_ctrl_sfun:ping', 'controller did not acknowledge the ping');
end
end

function r = local_call(s, req)
s.out.println(jsonencode(req));
line = s.in.readLine();
if isempty(line)
    error('s08_ctrl_sfun:closed', 'controller closed the connection');
end
r = jsondecode(char(line));
end

function local_close(s)
try, s.in.close(); catch, end
try, s.out.close(); catch, end
try, s.sock.close(); catch, end
end
