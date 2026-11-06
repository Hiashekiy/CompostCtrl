function v3_e2_evidence(T,out)
% Full mechanism traces use measured geometry/actuators; added command is labeled separately.
for task=1:2
 f=figure('Visible','off');tiledlayout(6,2);S=T(strcmp(T.pair_id,sprintf('V2_M%d',task)),:);
 for method=1:height(S),folder=fullfile(out,'E2','cases',char(string(S.id(method))));D=readtable(fullfile(folder,'actuator_timeseries_20ms.csv'));label=char(string(S.method(method)));c=v3_color(label);if strcmp(label,'MPC'),label='Joint MPC';elseif contains(label,'unload'),label='PID + unload';else,label='PID';end;
 nexttile(1);hold on;plot(D.time_s,vecnorm(D{:,28:30},2,2),'Color',c,'DisplayName',label);
 nexttile(2);hold on;plot(D.time_s,D.sigma_min,'Color',c,'DisplayName',label);
 nexttile(3);hold on;plot(D.time_s,rad2deg(D.gamma1_actual_rad),'Color',c,'DisplayName',label);
 nexttile(4);hold on;plot(D.time_s,rad2deg(D.elevator_actual_rad),'Color',c,'DisplayName',label);
 nexttile(5);hold on;plot(D.time_s,D.aero_actual_y_Nm,'Color',c,'DisplayName',label);
 nexttile(6);hold on;plot(D.time_s,D.cmg_actual_y_Nm,'Color',c,'DisplayName',label);
 nexttile(7);hold on;plot(D.time_s,D.aero_actual_y_Nm+D.cmg_actual_y_Nm,'Color',c,'DisplayName',label);
 nexttile(8);hold on;plot(D.time_s,D.next_y_positive_support_Nm,'Color',c,'DisplayName',label);plot(D.time_s,D.next_y_negative_support_Nm,'--','Color',c,'HandleVisibility','off');
 nexttile(9);hold on;plot(D.time_s,D.next_x_positive_support_Nm,'Color',c,'DisplayName',label);plot(D.time_s,D.next_x_negative_support_Nm,'--','Color',c,'HandleVisibility','off');
 nexttile(10);hold on;plot(D.time_s,D.next_z_positive_support_Nm,'Color',c,'DisplayName',label);plot(D.time_s,D.next_z_negative_support_Nm,'--','Color',c,'HandleVisibility','off');
 pid=fullfile(folder,'pid_allocation_timeseries.csv');if isfile(pid),P=readtable(pid);nexttile(11);hold on;stairs(P.time_s,P.unload_scale,'Color',c,'DisplayName',label);nexttile(12);hold on;plot(P.time_s,P.unload_c_2,'Color',c,'DisplayName',[label ' requested extra']);end
 end
 labels={'|H| (N m s)','Sigma min A','Gimbal 1 (deg)','Elevator (deg)','Aero pitch (N m)','CMG pitch (N m)','Total pitch (N m)','Pitch support (N m)','Roll support (N m)','Yaw support (N m)','Unload scale','Unload request (N m)'};for k=1:12,nexttile(k);ylabel(labels{k});grid on;xline(35,'k:','HandleVisibility','off');legend('Interpreter','none','Location','best');end;xlabel('Time (s)');v3_savefig(f,out,sprintf('E2_mechanism_%d',task));
 f=figure('Visible','off');tiledlayout(2,2);for method=1:height(S),D=readtable(fullfile(out,'E2','cases',char(string(S.id(method))),'actuator_timeseries_20ms.csv'));for frame=1:4,nexttile(frame);hold on;plot(D.time_s,rad2deg(D{:,30+frame}),'Color',v3_color(S.method(method)),'DisplayName',char(string(S.method(method))));ylabel(sprintf('Gamma %d (deg)',frame));grid on;xline(35,'k:','HandleVisibility','off');legend('Interpreter','none','Location','best');xlabel('Time (s)');end;end;v3_savefig(f,out,sprintf('E2_gamma_%d',task));
end
for task=1:2,f=figure('Visible','off');tiledlayout(2,1);S=T(strcmp(T.pair_id,sprintf('V2_C%d',task)),:);for k=1:height(S),D=readtable(fullfile(out,'E2','cases',char(string(S.id(k))),'timeseries.csv'));idx=1:max(1,ceil(height(D)/2000)):height(D);c=v3_color(S.method(k));nexttile(1);hold on;plot(D.time_s(idx),D.pitch_deg(idx),'Color',c,'DisplayName',char(string(S.method(k))));if k==1,plot(D.time_s(idx),D.ref_pitch_deg(idx),'k--','DisplayName','Reference');end;nexttile(2);hold on;plot(D.time_s(idx),D.attitude_error_deg(idx),'Color',c,'DisplayName',char(string(S.method(k))));end;nexttile(1);ylabel('Absolute pitch (deg)');legend('Interpreter','none');grid on;nexttile(2);ylabel('Quaternion error (deg)');xlabel('Time (s)');grid on;v3_savefig(f,out,sprintf('E2_continuous_%d',task));end
end
