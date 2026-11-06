function hpipm_interface_sfun(block)
% Algorithm-only interpreted simulation adapter. No physical plant is hidden here.
block.NumDialogPrms=0;
s=template; packed=hpipm_pack_qp(s);
block.NumInputPorts=1;block.NumOutputPorts=3;
block.SetPreCompPortInfoToDefaults;
block.InputPort(1).Dimensions=numel(packed);block.InputPort(1).DirectFeedthrough=true;
block.OutputPort(1).Dimensions=double(s.nu);
block.OutputPort(2).Dimensions=1;block.OutputPort(3).Dimensions=4;
block.SampleTimes=[.2 0];block.SimStateCompliance='DefaultSimState';
block.RegBlockMethod('Outputs',@outputs);
end
function outputs(block)
s=template;[~,fields]=hpipm_pack_qp(s);v=block.InputPort(1).Data;
assert(all(isfinite(v)),'QP input contains nonfinite values');offset=0;
for k=1:numel(fields)
 key=fields{k}; n=numel(s.(key));s.(key)=reshape(v(offset+(1:n)),size(s.(key)));offset=offset+n;
end
assert(all(s.lu<=s.uu,'all'),'Invalid control bound order');
r=solve_hpipm_snapshot(s);
block.OutputPort(1).Data=r.U(:,1);block.OutputPort(2).Data=r.status;
block.OutputPort(3).Data=r.residuals(:);
end
function s=template
root=fileparts(fileparts(fileparts(mfilename('fullpath'))));
s=load(fullfile(root,'simulink','tests','05_python_matlab_qp_equivalence','production_qp.mat'));
end
