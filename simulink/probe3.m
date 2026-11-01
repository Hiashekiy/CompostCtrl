function probe3(out_dir)
%PROBE3  Diagnose the S08Plant construction failure.
if nargin < 1 || isempty(out_dir), out_dir = fullfile(pwd, 'results', 'simulink_port'); end
if ~exist(out_dir, 'dir'), mkdir(out_dir); end
L = {};
    function say(varargin)
        s = sprintf(varargin{:});
        L{end+1} = s;                                        %#ok<AGROW>
        fprintf('%s\n', s);
    end

addpath(fullfile(pwd, 'simulink')); addpath(fullfile(pwd, 'simulink', 'blocks'));
say('which S08Plant: %s', which('S08Plant'));
say('exist class     : %d', exist('S08Plant', 'class'));

say('---- checkcode (lint) ----');
try
    msgs = checkcode(fullfile(pwd, 'simulink', 'blocks', 'S08Plant.m'), '-id');
    for k = 1:numel(msgs)
        say('L%d  %s  %s', msgs(k).line, msgs(k).id, msgs(k).message);
    end
    if isempty(msgs), say('no lint messages'); end
catch err
    say('lint failed: %s', err.message);
end

say('---- meta.class ----');
try
    mc = meta.class.fromName('S08Plant');
    say('class ok, superclasses: %s', strjoin({mc.SuperclassList.Name}, ', '));
    say('properties: %s', strjoin({mc.PropertyList.Name}, ', '));
catch err
    say('meta failed: %s', getReport(err, 'basic', 'hyperlinks', 'off'));
end

say('---- minimal system object ----');
f = fullfile(out_dir, 'S08ProbeSysObj.m');
fid = fopen(f, 'w');
fprintf(fid, 'classdef S08ProbeSysObj < matlab.System\n');
fprintf(fid, '    properties (Nontunable)\n        Gain = 2\n        Val = []\n    end\n');
fprintf(fid, '    methods (Access = protected)\n');
fprintf(fid, '        function y = stepImpl(obj, u)\n            y = obj.Gain * u;\n        end\n');
fprintf(fid, '        function sts = getSampleTimeImpl(obj)\n');
fprintf(fid, '            sts = createSampleTime(obj, ''Type'', ''Discrete'', ''SampleTime'', 0.2);\n');
fprintf(fid, '        end\n    end\nend\n');
fclose(fid);
addpath(out_dir);
try
    o = S08ProbeSysObj('Gain', 3, 'Val', [1 2 3]);
    y = o(2.0);
    say('[OK] minimal system object: y=%.3f', y);
catch err
    say('[FAIL] minimal system object: %s', getReport(err, 'basic', 'hyperlinks', 'off'));
end

say('---- construct S08Plant() ----');
d = fullfile(pwd, 'results', 'simulink_port', 'data');
cf = fullfile(d, 'A4_descent_replay_S00_with_cmg_P0_case.json');
for k = 1:2
    try
        if k == 1
            obj = S08Plant();
            say('[OK] S08Plant() default');
        else
            obj = S08Plant('CaseFile', cf, 'SampleTime', 0.2);
            say('[OK] S08Plant(name,value)');
            x = obj(zeros(7, 1));
            say('[OK] step -> %d states', numel(x));
        end
    catch err
        say('[FAIL] attempt %d: %s', k, getReport(err, 'basic', 'hyperlinks', 'off'));
    end
end

fid = fopen(fullfile(out_dir, 'probe3.txt'), 'w');
fprintf(fid, '%s\n', L{:});
fclose(fid);
end
