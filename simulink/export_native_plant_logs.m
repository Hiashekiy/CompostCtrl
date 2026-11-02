function export_native_plant_logs
root=fileparts(fileparts(mfilename('fullpath')));out=fullfile(root,'simulink','results','native_plant');
for enabled=[false true]
 if enabled,name='Plant_6DOF_CMG';else,name='Plant_6DOF_SurfaceOnly';end
v=load(fullfile(out,[name '.mat']));r=v.r;t=r.yout{6}.Values.Time;
w=samples(r.yout{6}.Values);e=samples(r.yout{3}.Values);
if enabled,g=samples(r.yout{8}.Values);nu=samples(r.state);else,g=zeros(numel(t),4);nu=g;end
writematrix([t w e g nu],fullfile(out,[name '.csv']));
end
end
function value=samples(ts)
value=squeeze(ts.Data);if size(value,1)~=numel(ts.Time),value=value';end
end
