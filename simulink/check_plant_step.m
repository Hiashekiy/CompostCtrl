function ok = check_plant_step(tags, varargin)
%CHECK_PLANT_STEP  C1 acceptance: MATLAB plant vs the Python plant, one step.
%
%   check_plant_step                     % every exported case
%   check_plant_step({'A4_descent_replay_S00_with_cmg_P0'})
%
% For each case the Python exporter wrote <tag>_plantcheck.json with 8 random
% reachable states/inputs and the exact one-step result of
% ClosedLoop._step_P0.  Here the same step is run through s08_plant_step, the
% function the Simulink S-function calls, and the difference is reported.
%
% Acceptance (doc C1): max relative difference <= 1e-12.

if nargin < 1 || isempty(tags)
    tags = local_tags();
elseif ischar(tags)
    tags = {tags};
end
tol = 1e-12;
addpath(fullfile(fileparts(mfilename('fullpath')), 'blocks'));
fprintf('==== C1: one-step plant equivalence (tol %.0e) ====\n', tol);
ok = true;
summary = struct('tol', tol, 'cases', [], 'passed', false);
for t = 1:numel(tags)
    tag = tags{t};
    d = fullfile(pwd, 'results', 'simulink_port', 'data');
    pc = jsondecode(fileread(fullfile(d, [tag '_plantcheck.json'])));
    P = s08_case_load(fullfile(d, [tag '_case.json']));
    worst = 0; worstRel = 0; nfail = 0;
    for i = 1:numel(pc.tests)
        tc = pc.tests(i);
        x1 = s08_plant_step(P, double(tc.x0(:)), double(tc.u(:)), double(tc.tau_d(:)));
        dx = abs(x1(:) - double(tc.x1(:)));
        scale = max(1.0, max(abs(double(tc.x1(:)))));
        worst = max(worst, max(dx));
        worstRel = max(worstRel, max(dx) / scale);
        if ~all(isfinite(x1)) || numel(x1) ~= numel(tc.x1)
            nfail = nfail + 1;
        end
    end
    pass = (worstRel <= tol) && (nfail == 0);
    ok = ok && pass;
    rec = struct('tag', tag, 'nx', P.nx, 'tests', numel(pc.tests), ...
                 'max_abs_dx', worst, 'max_rel_dx', worstRel, 'passed', pass);
    summary.cases = [summary.cases, rec];
    fprintf('[%s] %-40s nx=%2d  tests=%d  max|dx|=%.3e  rel=%.3e\n', ...
            local_tag(pass), tag, P.nx, numel(pc.tests), worst, worstRel);
end
summary.passed = ok;
out = fullfile(pwd, 'results', 'simulink_port', 'plant_check.json');
fid = fopen(out, 'w');
fprintf(fid, '%s', jsonencode(summary, 'PrettyPrint', true));
fclose(fid);
fprintf('---- C1 %s ----\n', local_tag(ok, 'PASSED', 'FAILED'));
end

function s = local_tag(c, a, b)
if nargin < 2, a = 'OK  '; b = 'FAIL'; end
if c, s = a; else, s = b; end
end

function tags = local_tags()
d = fullfile(pwd, 'results', 'simulink_port', 'data');
f = dir(fullfile(d, '*_plantcheck.json'));
tags = cell(1, numel(f));
for k = 1:numel(f)
    tags{k} = strrep(f(k).name, '_plantcheck.json', '');
end
tags = sort(tags);
end
