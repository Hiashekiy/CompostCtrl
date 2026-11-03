function upgrade_reference_wiring(only)
% Upgrade saved closed-loop models in place so the reference enters through a real
% signal (controller port 2 + Reference_Manager), WITHOUT re-simulating and WITHOUT
% re-exporting scenarios: each model's own embedded bank supplies the data.
%
% Usage: upgrade_reference_wiring            % every closed-loop model
%        upgrade_reference_wiring('S00')     % only names matching this pattern
if nargin<1||isempty(only),only='';end
root=repo_root;
list=[dir(fullfile(root,'simulink','models','NearSpace_*.slx')); ...
      dir(fullfile(root,'simulink','cases','*.slx'))];
wired=0;skipped=0;comp=0;
for k=1:numel(list)
 name=list(k).name(1:end-4);
 if ~isempty(only) && isempty(strfind(name,only)),continue;end
 file=fullfile(list(k).folder,list(k).name);
 if bdIsLoaded(name),close_system(name,0);end
 load_system(file);
 ctl=[name '/Online_RTI_HPIPM'];
 if isempty(find_system(name,'SearchDepth',1,'Name','Online_RTI_HPIPM'))
  comp=comp+1;close_system(name,0);continue;
 end
 mw=get_param(name,'ModelWorkspace');
 if hasVariable(mw,'reference_window')
  fprintf('%-56s already wired\n',list(k).name);skipped=skipped+1;close_system(name,0);continue;
 end
 if ~hasVariable(mw,'native_reference_bank')
  fprintf('%-56s no embedded bank, skipped\n',list(k).name);skipped=skipped+1;close_system(name,0);continue;
 end
 bank=getVariable(mw,'native_reference_bank');
 assignin(mw,'reference_window',[bank.time(:)-1e-6 flatten(bank)]);
 assignin(mw,'reference_attitude_deg',[bank.time(:) euldeg(squeeze(bank.q_ref(:,1,:))')]);
 assignin(mw,'reference_body_rate',[bank.time(:) squeeze(bank.omega_ref(:,1,:))']);
 assignin(mw,'reference_torque_nom',[bank.time(:) squeeze(bank.tau_nom(:,1,:))']);
 rm=[name '/Reference_Manager'];
 if isempty(find_system(name,'SearchDepth',1,'Name','Reference_Manager'))
  add_block('built-in/SubSystem',rm,'Position',[1290 30 1620 300]);
  srcs={'window','reference_window';'attitude_deg','reference_attitude_deg'; ...
        'body_rate','reference_body_rate';'torque_nom','reference_torque_nom'};
  for i=1:size(srcs,1)
   add_block('simulink/Sources/From Workspace',[rm '/' srcs{i,1}],'VariableName',srcs{i,2}, ...
    'Interpolate','off','OutputAfterFinalValue','Holding final value','Position',[25 25+70*(i-1) 185 60+70*(i-1)]);
   add_block('built-in/Outport',[rm sprintf('/ref%d',i)],'Port',num2str(i),'Position',[230 30+70*(i-1) 260 55+70*(i-1)]);
   add_line(rm,[srcs{i,1} '/1'],sprintf('ref%d/1',i));
  end
 end
 ph=get_param(ctl,'PortHandles');
 inLine=get_param(ph.Inport(1),'Line');inSrc=get_param(inLine,'SrcPortHandle');
 outLine=get_param(ph.Outport(1),'Line');outDst=get_param(outLine,'DstPortHandle');
 pos=get_param(ctl,'Position');
 delete_line(inLine);delete_line(outLine);delete_block(ctl);
 add_block('simulink/User-Defined Functions/Level-2 MATLAB S-Function',ctl, ...
  'FunctionName','native_mpc_sfun','Position',pos);
 ph=get_param(ctl,'PortHandles');
 if numel(ph.Inport)<2
  try,set_param(name,'SimulationCommand','update');catch,end
  ph=get_param(ctl,'PortHandles');
 end
 if numel(ph.Inport)<2
  fprintf('%-56s COULD NOT ADD PORT 2\n',list(k).name);save_system(name);close_system(name,0);continue;
 end
 add_line(name,inSrc,ph.Inport(1),'autorouting','on');
 for d=1:numel(outDst),add_line(name,ph.Outport(1),outDst(d),'autorouting','on');end
 rmph=get_param(rm,'PortHandles');
 add_line(name,rmph.Outport(1),ph.Inport(2),'autorouting','on');
 if isempty(find_system(name,'SearchDepth',1,'Name','reference_log'))
  add_block('built-in/ToWorkspace',[name '/reference_log'],'VariableName','reference_log_att_deg', ...
   'SaveFormat','Timeseries','MaxDataPoints','inf','Position',[1310 320 1450 360]);
  add_line(name,rmph.Outport(2),get_param([name '/reference_log'],'PortHandles').Inport(1),'autorouting','on');
 end
 save_system(name);close_system(name,0);
 fprintf('%-56s wired (%d in-ports)\n',list(k).name,numel(ph.Inport));
 wired=wired+1;
end
fprintf('UPGRADE_PASS wired=%d skipped=%d component=%d\n',wired,skipped,comp);
end
function win=flatten(bank)
nk=numel(bank.time);nN=size(bank.tau_min,2)-1;win=[];
for k=1:nk
 qk=squeeze(bank.q_ref(:,:,k));wk=squeeze(bank.omega_ref(:,:,k));tk=squeeze(bank.tau_d(:,:,k));
 lo=bank.tau_min(:,1:nN,k);hi=bank.tau_max(:,1:nN,k);nm=bank.tau_nom(:,1:nN,k);
 v=[qk(:);wk(:);tk(:);lo(:);hi(:);nm(:);bank.gap(:,1,k)];
 if isempty(win),win=zeros(nk,numel(v));end
 win(k,:)=v.';
end
end
function e=euldeg(q)
q=q./vecnorm(q,2,2);
w=q(:,1);x=q(:,2);y=q(:,3);z=q(:,4);
e=rad2deg([atan2(2*(w.*x+y.*z),1-2*(x.^2+y.^2)), ...
           asin(max(-1,min(1,2*(w.*y-z.*x)))), ...
           atan2(2*(w.*z+x.*y),1-2*(y.^2+z.^2))]);
end
