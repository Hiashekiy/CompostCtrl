function hpipm_startup(hpipm_root,blasfeo_root)
persistent initialized
if nargin<1, hpipm_root='D:\Tools\hpipm'; end
if nargin<2, blasfeo_root='D:\Tools\blasfeo'; end
signature=[hpipm_root '|' blasfeo_root];
if isequal(initialized,signature) && exist('ocp_qp_create','file')==3,return;end
here=fileparts(mfilename('fullpath'));
addpath(fullfile(hpipm_root,'interfaces','matlab_octave'));
addpath(fullfile(here,'mex'));
% This function is called on each QP solve. Repeated PATH prefixing eventually
% exceeds Windows environment limits and makes unrelated DLL loads fail.
entries=strsplit(getenv('PATH'),pathsep);
entries=[{fullfile(hpipm_root,'build'),fullfile(blasfeo_root,'build')} entries];
[~,keep]=unique(lower(string(entries)),'stable');
setenv('PATH',strjoin(entries(sort(keep)),pathsep));
initialized=signature;
end
