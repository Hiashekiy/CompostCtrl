function root=native_root
% Repository root, independent of which sub-folder the caller lives in.
% Use this instead of counting fileparts(mfilename('fullpath')) per file depth,
% so build/, check/ or hpipm/ scripts can move without breaking path resolution.
root=fileparts(fileparts(mfilename('fullpath')));
end
