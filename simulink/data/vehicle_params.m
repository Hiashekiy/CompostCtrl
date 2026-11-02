function params = vehicle_params(case_name)
%VEHICLE_PARAMS Load exported SI data; no physical dynamics hidden here.
% Regenerate with python simulink/py/export_native_baseline.py --config PATH.
if nargin < 1, case_name='A4_descent_replay_S00_with_cmg'; end
base=fileparts(mfilename('fullpath'));
file=fullfile(base,'native_baseline',case_name,'vehicle_params.mat');
assert(isfile(file),'Export the selected Python baseline first: %s',file);
params=load(file);
end
