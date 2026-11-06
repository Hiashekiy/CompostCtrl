function bank=online_resample_reference_bank(bank,old_dts,new_dts,period)
% Re-time both the update instants and the prediction nodes; never hold a
% 200-ms reference window while executing a 100-ms control loop.
old_time=bank.time(:);new_time=(old_time(1):period:old_time(end))';
old_nodes=[0;cumsum(old_dts(:))];new_nodes=[0;cumsum(new_dts(:))];
assert(numel(old_nodes)==numel(new_nodes) && new_nodes(end)<=old_nodes(end)+1e-10);
for name={'q_ref','omega_ref','tau_d','tau_min','tau_max','tau_nom','gap'}
 key=name{1};value=bank.(key);shape=size(value);
 flat=reshape(value,[],numel(old_time))';
 value=reshape(interp1(old_time,flat,new_time,'linear')',shape(1),shape(2),[]);
 if shape(2)==numel(old_nodes)
  moved=zeros(shape(1),numel(new_nodes),numel(new_time));
  for k=1:numel(new_time)
   moved(:,:,k)=interp1(old_nodes,value(:,:,k)',new_nodes,'linear')';
  end
  value=moved;
 end
 if strcmp(key,'q_ref'),value=value./sqrt(sum(value.^2,1));end
 bank.(key)=value;
end
bank.time=new_time;
end
