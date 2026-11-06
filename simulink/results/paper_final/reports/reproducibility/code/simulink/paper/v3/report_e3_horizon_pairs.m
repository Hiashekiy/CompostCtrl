function report_e3_horizon_pairs(old,out)
% Paired task-relative analysis, all samples used for sustained thresholds.
revision_controller_audit(old,out);
Hs=[5 8 12 16 23.2];rows={};pairs={};sources={};data=cell(5,2);metrics=cell(5,2);
for h=1:5,for mode=1:2
 H=Hs(h);tag='M';if mode==2,tag='Z';end
 reused=(h==1 && mode==1)||h==5;
 if h==5,id='V3_L';if mode==2,id='V3_Z';end;elseif reused,id='V3_S';else,id=sprintf('V3_H%02d_%s',H,tag);end
 root=out;if reused,root=old;end;folder=fullfile(root,'E3/cases',id);
 c=jsondecode(fileread(fullfile(folder,'case_config.json')));m=jsondecode(fileread(fullfile(folder,'metrics.json')));assert(abs(sum(c.dts)-H)<1e-9 && c.onset==35 && c.amplitude==10*(mode==1));
 C=readtable(fullfile(folder,'cmg_timeseries.csv'));data{h,mode}=C;
 d=load(fullfile(folder,'sim_output.mat'),'r');t=d.r.states.Time(:);x=squeeze(d.r.states.Data);bank=load(fullfile(folder,'reference_bank.mat'));[~,ref]=paper_reference_series(bank,t);
 axiserr=rad2deg(x(:,1:3))-ref;axiserr=mod(axiserr+180,360)-180;
 windows={t>=35 & t<=40,t>=40 & t<=50,true(size(t)),t>=10 & t<35};wn={'maneuver','post','full','hold'};
 for w=1:4,for a=1:3,an={'roll','pitch','yaw'};mask=windows{w};if any(mask),m.([wn{w} '_' an{a} '_rmse_deg'])=sqrt(mean(axiserr(mask,a).^2));m.([wn{w} '_' an{a} '_peak_deg'])=max(abs(axiserr(mask,a)));else,m.([wn{w} '_' an{a} '_rmse_deg'])=NaN;m.([wn{w} '_' an{a} '_peak_deg'])=NaN;end;end;end
 m.source_path=folder;m.reused=reused;m.task=tag;rows{end+1}=m;metrics{h,mode}=m;
 sources{end+1}=struct('id',id,'horizon_s',H,'task',tag,'reused',reused,'source_path',folder,'config_sha256',paper_hash(fullfile(folder,'case_config.json')),'metrics_sha256',paper_hash(fullfile(folder,'metrics.json')),'status',m.status,'complete',m.complete);
end;end
R=struct2table([rows{:}]);R.maneuver_rank=nan(height(R),1);mi=find(strcmp(R.task,'M'));values=R.maneuver_rmse_deg(mi);[~,order]=sort(values);R.maneuver_rank(mi(order))=(1:numel(mi))';writetable(R,fullfile(out,'tables/E3_10cases.csv'));
fig=figure('Visible','off');tiledlayout(5,2);
for h=1:5
 M=data{h,1};Z=data{h,2};assert(isequal(M.time_s,Z.time_s),'Paired full native logs have different physical times');t=M.time_s;
 dg=rad2deg(M{:,{'g1','g2','g3','g4'}}-Z{:,{'g1','g2','g3','g4'}});dnu=M{:,{'nu1','nu2','nu3','nu4'}}-Z{:,{'nu1','nu2','nu3','nu4'}};dH=M{:,{'Hx','Hy','Hz'}}-Z{:,{'Hx','Hy','Hz'}};dtau=M{:,{'tau_cmg_x','tau_cmg_y','tau_cmg_z'}}-Z{:,{'tau_cmg_x','tau_cmg_y','tau_cmg_z'}};
 movement=max(abs(dg),[],2);preview=35-Hs(h);before=t<preview-1e-9;
 p=struct('horizon_s',Hs(h),'M_id',metrics{h,1}.id,'Z_id',metrics{h,2}.id,'earliest_preview_s',preview,'prepreview_max_delta_gamma_deg',max(movement(before)),'prepreview_max_delta_H_Nms',max(abs(dH(before,:)),[],'all'),'prepreview_max_delta_nu_radps',max(abs(dnu(before,:)),[],'all'),'prepreview_max_delta_torque_Nm',max(abs(dtau(before,:)),[],'all'));
 p.prepreview_near_zero=p.prepreview_max_delta_gamma_deg<1e-5 && p.prepreview_max_delta_H_Nms<1e-5 && p.prepreview_max_delta_torque_Nm<1e-5;
 for th=[.5 2 5],key=strrep(num2str(th),'.','p');p.(['migration_' key 'deg_s'])=sustained(t,movement,th);end
 p.advance_s=35-p.migration_2deg_s;p.task_preposition_evidence=p.prepreview_near_zero && isfinite(p.migration_2deg_s) && p.migration_2deg_s>=preview-1e-9;
 for mode=1:2,m=metrics{h,mode};tag='M';if mode==2,tag='Z';end
 for field={'Hy_at_onset_Nms','Hnorm_at_onset_Nms','sigma_min_at_onset','pitch_positive_support_at_onset_Nm','pitch_negative_support_at_onset_Nm','maneuver_rmse_deg','maneuver_peak_deg','post_rmse_deg','rmse_full_deg','prepare_rmse_deg','opposite_torque_pct','opposite_effective_samples','mean_compute_s','p95_compute_s','max_compute_s','over200ms_pct','fallback_count'},key=field{1};v=m.(key);if isempty(v),v=NaN;end;p.([tag '_' key])=v;end
 end
 p.delta_Hy_at_34p8_Nms=p.M_Hy_at_onset_Nms-p.Z_Hy_at_onset_Nms;
 pairs{end+1}=p;
 D=array2table([t dg dnu dH dtau movement],'VariableNames',{'time_s','delta_g1_deg','delta_g2_deg','delta_g3_deg','delta_g4_deg','delta_nu1','delta_nu2','delta_nu3','delta_nu4','delta_Hx','delta_Hy','delta_Hz','delta_tau_x','delta_tau_y','delta_tau_z','max_delta_gamma_deg'});writetable(D,fullfile(out,'tables',sprintf('E3_H%04d_M_minus_Z.csv',round(100*Hs(h)))));
 nexttile(2*h-1);plot(t,movement);yline(2,'k--');xline(preview,'k:');xline(35,'k-');xlim([0 35]);ylabel(sprintf('H=%g: max |Dgamma| (deg)',Hs(h)));grid on;
 nexttile(2*h);plot(t,dH(:,2));xline(preview,'k:');xline(35,'k-');xlim([0 35]);ylabel('Delta Hy (N m s)');grid on;
end
v3_savefig(fig,out,'E3_relative_migration');P=struct2table([pairs{:}]);writetable(P,fullfile(out,'tables/E3_5pairs.csv'));
fig=figure('Visible','off');tiledlayout(2,2);M=R(strcmp(R.task,'M'),:);
nexttile;plot(Hs,M.maneuver_rmse_deg,'-o');hold on;plot(Hs,M.post_rmse_deg,'-s');plot(Hs,M.rmse_full_deg,'-^');legend('35-40s','40-50s','0-60s');ylabel('Attitude RMSE (deg)');grid on;
nexttile;plot(Hs,M.maneuver_roll_rmse_deg,'-o',Hs,M.maneuver_pitch_rmse_deg,'-s',Hs,M.maneuver_yaw_rmse_deg,'-^');legend('roll','pitch','yaw');ylabel('35-40s axis RMSE (deg)');grid on;
nexttile;plot(Hs,M.mean_compute_s,'-o',Hs,M.p95_compute_s,'-s',Hs,M.max_compute_s,'-^');yline(.2,'k:');legend('mean','P95','max');ylabel('Solve time (s)');xlabel('Horizon (s)');grid on;
nexttile;plot(Hs,M.over200ms_pct,'-o');ylabel('Solve time >200ms (%)');xlabel('Horizon (s)');grid on;v3_savefig(fig,out,'E3_horizon_performance');
fig=figure('Visible','off');tiledlayout(2,2);fields={'Hy_at_onset_Nms','Hnorm_at_onset_Nms','pitch_positive_support_at_onset_Nm','pitch_negative_support_at_onset_Nm'};yl={'Hy (N m s)','H norm (N m s)','Positive pitch support (N m)','Negative pitch support (N m)'};
for j=1:4,nexttile;plot(Hs,P.(['M_' fields{j}]),'-o',Hs,P.(['Z_' fields{j}]),'-s');legend('M','Z');ylabel(yl{j});xlabel('Horizon (s)');grid on;end;v3_savefig(fig,out,'E3_pitch_authority');
paper_json(fullfile(out,'reports/E3_acceptance.json'),struct('expected',10,'actual',height(R),'new_simulations',7,'reused_cases',3,'complete',all(R.complete),'sources',{sources},'prepreview_consistent',all(P.prepreview_near_zero),'preposition_evidence_by_horizon',P.task_preposition_evidence','invalid_ids',{cellstr(string(R.id(ismember(string(R.status),["MODEL_DOMAIN_FAIL","CONTROLLER_FAIL","INCOMPLETE"]))))},'historical_onset8_excluded',true,'sustained_threshold_definition','strictly > threshold for >=0.6s, entirely before35s; onset is first crossing, no preview censoring'));
fid=fopen(fullfile(out,'reports/E3_revised.md'),'w','n','UTF-8');cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'# 实验三：五档预测时域×有/无机动\n\n最终10例，新增7次真实Simulink闭环仿真，旧S/L/Z三例只读复用。onset=35s，transition=5s，duration=60s；M俯仰增量10°，Z为真实零机动。未混入onset=8s历史协议，未更改物理模型、控制目标/权重或4轮SQP上限。\n\n主迁移判据为同H的M−Z最大框架角差严格超过2°、持续至少0.6s、且完整持续段在35s前。首次时刻取跨阈值起点；未发生者保留NaN；0.5°和5°仅做敏感性分析。绝对10°迁移受共同启动瞬态影响，仅为诊断。\n\n|H(s)|35–40s姿态RMSE(°)|40–50s RMSE(°)|迁移时刻(s)|提前量(s)|预览前最大角差(°)|预置证据|\n|---:|---:|---:|---:|---:|---:|---|\n');
for h=1:5,fprintf(fid,'|%.1f|%.6f|%.6f|%.3f|%.3f|%.6g|%d|\n',Hs(h),M.maneuver_rmse_deg(h),M.post_rmse_deg(h),P.migration_2deg_s(h),P.advance_s(h),P.prepreview_max_delta_gamma_deg(h),P.task_preposition_evidence(h));end
fprintf(fid,'\n预览前一致性全部通过：%d。若不通过，不能将差异归因为任务预置；应先排查同时间参考、初态、扰动和控制器一致性。Z只报告10–35s保持误差，不参与机动排名。三轴RMSE和峰值由完整1ms物理输出独立重算；几何姿态误差、准备段有效反向力矩样本数、求解耗时和回退等沿用原分析口径。\n\n方向控制权限取34.8s的真实状态，沿用v3_analyze速率/加速度约束算法。这只是方向投影上界，不等于另两轴力矩为零的纯俯仰可达力矩；Hy更大不必然更好。正负支撑可为负值，表示该加速度受限瞬时可达集合在该方向上的有符号支撑。sigma_min、Hy、H模和正负支撑均保留。\n\n执行配合仅统计准备段两种实际俯仰力矩绝对值均大于1Nm的样本；反向比例必须结合样本数和姿态保持误差，不能单独称为完美补偿。不同预测网格的差异也不能全部归因为提前迁移。真实失败、模型越界和未迁移情况全部保留在表与验收来源中。\n\n',all(P.prepreview_near_zero));
for name={'E3_horizon_performance','E3_relative_migration','E3_pitch_authority'},fprintf(fid,'![%s](../figures/%s.png)\n\n',name{1},name{1});end
fprintf(fid,'[10例完整指标](../tables/E3_10cases.csv)；[5组配对指标](../tables/E3_5pairs.csv)；[验收来源](E3_acceptance.json)。\n');
fprintf(fid,'\n求解耗时包含真实离线运行开销，8/12/16s部分控制步超过200ms；未由这些结果证明硬实时能力。5s/23.2s有机动数据来自旧运行，其他时域为新运行，运行负载和启动开销可能不同，因此耗时的非单调差异不能全部归因为预测时域。该限制不改变同时间物理轨迹的配对分析。\n');
end
function onset=sustained(t,movement,threshold)
onset=NaN;active=movement>threshold & t<35;edges=diff([false;active;false]);starts=find(edges==1);ends=find(edges==-1)-1;
for j=1:numel(starts),if t(ends(j))-t(starts(j))>=.6-1e-9,onset=t(starts(j));return;end;end
end
