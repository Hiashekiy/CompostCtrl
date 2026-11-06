function root=repo_root
% Repository root, independent of which sub-folder the caller lives in.
% Use this instead of counting fileparts(mfilename('fullpath')) per file depth,
% so modelgen/, verify/ or hpipm/ scripts can move without breaking path resolution.
root=fileparts(fileparts(mfilename('fullpath')));
end
