function report_closedloop
root=repo_root;bank=load(fullfile(root,'simulink','data','online_plan_bank.mat'));
out=fullfile(root,'simulink','results','native_closedloop');files=dir(fullfile(out,'*_surrogate_flight_*s.mat'));
summary=struct;
for f=1:numel(files)
 data=load(fullfile(files(f).folder,files(f).name));r=data.r;t=r.states.Time;
 state=samples(r.states);e=state(:,1:3);roll=e(:,1);pitch=e(:,2);yaw=e(:,3);
 cr=cos(roll/2);sr=sin(roll/2);cp=cos(pitch/2);sp=sin(pitch/2);cy=cos(yaw/2);sy=sin(yaw/2);
 q=[cr.*cp.*cy+sr.*sp.*sy,sr.*cp.*cy-cr.*sp.*sy,cr.*sp.*cy+sr.*cp.*sy,cr.*cp.*sy-sr.*sp.*cy];
 ref=interp1(bank.time(:),squeeze(bank.q_ref(:,1,:))',t,'linear');ref=ref./vecnorm(ref,2,2);
 angle=2*acos(min(1,abs(sum(q.*ref,2))))*180/pi;
 report=struct('duration_s',t(end),'max_attitude_error_deg',max(angle),'rms_attitude_error_deg',sqrt(mean(angle.^2)), ...
 'final_attitude_error_deg',angle(end),'finite',all(isfinite(state),'all'));
 if any(strcmp(r.who,'flight_conditions'))
  condition=samples(r.flight_conditions);report.alpha_min_deg=min(condition(:,3))*180/pi;report.alpha_max_deg=max(condition(:,3))*180/pi;
  report.qbar_min_Pa=min(condition(:,1));report.qbar_max_Pa=max(condition(:,1));
 end
 key=matlab.lang.makeValidName(erase(files(f).name,'.mat'));summary.(key)=report;
 writematrix([t angle],fullfile(out,[key '_attitude_error.csv']));
end
fid=fopen(fullfile(out,'tracking_report.json'),'w');fprintf(fid,'%s',jsonencode(summary,'PrettyPrint',true));fclose(fid);disp(summary);
end
function data=samples(ts)
data=squeeze(ts.Data);if size(data,1)~=numel(ts.Time),data=data';end
end
