function init_paths
% Add native model/controller paths; independent from the historical S08 port.
here=fileparts(mfilename('fullpath'));addpath(here,fullfile(here,'modelgen'),fullfile(here,'verify'),fullfile(here,'data'),fullfile(here,'models'),fullfile(here,'hpipm'));
hpipm_startup;
fprintf('Native migration paths ready: %s\n',here);
end
