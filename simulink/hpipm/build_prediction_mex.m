function build_prediction_mex
here=fileparts(mfilename('fullpath'));out=fullfile(here,'mex');
files=dir(fullfile(here,'generated','native_*.c'));
for k=1:numel(files),mex('-R2018a',fullfile(files(k).folder,files(k).name),'-outdir',out);end
fprintf('PREDICTION_MEX_PASS count=%d\n',numel(files));
end
