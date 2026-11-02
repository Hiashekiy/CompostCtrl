function hpipm_startup(hpipm_root,blasfeo_root)
if nargin<1, hpipm_root='D:\Tools\hpipm'; end
if nargin<2, blasfeo_root='D:\Tools\blasfeo'; end
here=fileparts(mfilename('fullpath'));
addpath(fullfile(hpipm_root,'interfaces','matlab_octave'));
addpath(fullfile(here,'mex'));
setenv('PATH',[fullfile(hpipm_root,'build') pathsep fullfile(blasfeo_root,'build') pathsep getenv('PATH')]);
end
