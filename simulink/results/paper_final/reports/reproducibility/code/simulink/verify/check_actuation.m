function check_actuation(model,results_dir)
% Regression guard for the P0 CMG path: the gimbal angles must actually move.
%
% Background: the P0 builder replaces the tested gimbal drive with an ideal
% rate-tracking drive.  Simulink.SubSystem.deleteContents also drops the lines
% attached to that subsystem's ports, and the resulting model still compiles and
% runs - but with gamma frozen at its initial value, so the cluster produces no
% reaction torque while the MPC keeps optimising four gimbal-rate commands.
% This check fails loudly instead of returning a plausible but inert B-arm run.
%
% Usage: check_actuation                      % default S00 pitch 30 case
%        check_actuation('NearSpace_CMG_AttitudeValidation_S02_pitch10')
if nargin<1,model='NearSpace_CMG_AttitudeValidation_S00_pitch30';end
if nargin<2,results_dir='';end
root=repo_root;
addpath(fullfile(root,'simulink'));addpath(fullfile(root,'simulink','hpipm'));hpipm_startup;
load_system(fullfile(root,'simulink','cases',[model '.slx']));
if ~isempty(results_dir)
 mw=get_param(model,'ModelWorkspace');assignin(mw,'native_results_dir',results_dir);
end
saved=get_param(model,'StopTime');
set_param(model,'StopTime','2');      % in-memory override, the saved model is untouched
out=sim(model);set_param(model,'StopTime',saved);
st=out.states;x=st.Data;if size(x,1)~=numel(st.Time),x=x';end
gamma=x(:,7:10);peak=max(abs(gamma),[],1);rates=out.commands.Data(:,4:7);rate_peak=max(abs(rates),[],1);
fprintf('CMG check %s: gamma_peak=%s nu_peak=%s\n',model,mat2str(peak,6),mat2str(rate_peak,6));
close_system(model,0);
% The MPC S-Function rewrites the runtime evidence for this model name on every
% run; keep the full-window file authoritative after this short check.
outdir=fullfile(root,'simulink','results','native_closedloop');
if ~isempty(results_dir),outdir=results_dir;end
short=fullfile(outdir,sprintf('%s_runtime_%gs.json',model,st.Time(end)));
try
 if exist(short,'file'),delete(short);end
 for tag={'60s','90s'}
  long=fullfile(outdir,[model '_runtime_' tag{1} '.json']);
  if exist(long,'file'),copyfile(long,fullfile(outdir,[model '_runtime.json']));break;end
 end
catch
 warning('runtime evidence file is locked; canonical full-window copy left untouched');
end
assert(all(peak>1e-3),'CMG gimbal angles did not move: the P0 gimbal drive is disconnected');
assert(all(any(abs(rates)>1e-6,1)),'no gimbal-rate command reached the CMG subsystem');
fprintf('CMG_ACTUATION_PASS %s\n',model);
end
