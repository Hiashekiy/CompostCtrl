function report_e4_1deg(old,out)
% Read-only selection of the original 27 fixed-one-degree cases.
T=readtable(fullfile(old,'tables/E4_all.csv'),'TextType','string');T=T(T.amplitude_deg==1,:);assert(height(T)==27);
freq=[.02 .04 .06 .08 .10 .12 .15 .18 .20];labels={'SURFACE_MPC','PID_CHAIN_CMG','JOINT_MPC_CMG'};sources={};rows={};
fields={'gain_ratio','gain_db','phase_lag_deg','pitch_rmse_deg','fit_residual_deg','bias_deg','trend_deg_per_s','attitude_rmse_deg','qbar_min_Pa','qbar_max_Pa','height_min_m','height_max_m','surface_sat_pct','cmg_rate_sat_pct','mean_compute_s','p95_compute_s','fallback_count'};
for method=1:3,for f=freq
 id=sprintf('V4_010_%03d_%d',round(f*1000),method);folder=fullfile(old,'E4/cases',id);assert(sum(T.id==string(id))==1);
 for file={'case_config.json','metrics.json','sim_output.mat','timeseries.csv'},assert(isfile(fullfile(folder,file{1})),'Missing %s/%s',id,file{1});end
 c=jsondecode(fileread(fullfile(folder,'case_config.json')));m=jsondecode(fileread(fullfile(folder,'metrics.json')));
 assert(c.amplitude==1 && abs(c.frequency-f)<1e-10 && c.onset==8 && abs(c.duration-(8+4/f))<1e-9);
 assert(c.cmg==(method~=1));if method==2,assert(strcmp(c.method,'PID_CHAIN_CMG') && ~c.pid.unload);else,assert(strcmp(c.method,'MPC'));end
 for j=1:numel(fields),key=fields{j};assert(isfield(m,key));value=T.(key)(T.id==string(id));assert((isempty(m.(key)) && isnan(value)) || isequaln(value,m.(key)) || abs(value-m.(key))<1e-8,'Metric mismatch %s/%s',id,key);end
 m.display_method=labels{method};m.source_path=folder;m.source_config=fullfile(folder,'case_config.json');m.source_metrics=fullfile(folder,'metrics.json');rows{end+1}=m;
 sources{end+1}=struct('id',id,'method',labels{method},'frequency_Hz',f,'status',m.status,'complete',m.complete,'config_path',m.source_config,'metrics_path',m.source_metrics,'raw_path',fullfile(folder,'sim_output.mat'),'metrics_sha256',paper_hash(m.source_metrics),'config_sha256',paper_hash(m.source_config));
end;end
R=struct2table([rows{:}]);writetable(R,fullfile(out,'tables/E4_1deg_27cases.csv'));
names={'gain_db','phase_lag_deg','pitch_rmse_deg'};files={'gain','phase','rmse'};yl={'Amplitude gain (dB)','Phase lag (deg)','Pitch tracking RMSE (deg)'};
for j=1:3
 fig=figure('Visible','off');hold on;
 for k=1:3,Q=R(strcmp(R.display_method,labels{k}),:);Q=sortrows(Q,'frequency_Hz');plot(Q.frequency_Hz,Q.(names{j}),'-o','DisplayName',labels{k});if j==2,unreliable=Q.phase_unreliable;plot(Q.frequency_Hz(unreliable),Q.phase_lag_deg(unreliable),'kx','MarkerSize',12,'LineWidth',2,'HandleVisibility','off');end;end
 xlabel('Frequency (Hz)');ylabel(yl{j});grid on;legend('Location','best','Interpreter','none');v3_savefig(fig,out,['E4_1deg_' files{j}]);
end
counts=zeros(1,3);for k=1:3,counts(k)=sum(R.acceptable_tracking(strcmp(R.display_method,labels{k})));end
paper_json(fullfile(out,'reports/E4_acceptance.json'),struct('expected',27,'actual',height(R),'new_simulations',0,'reused_cases',27,'complete',all(R.complete),'auxiliary_pass_counts',counts,'sources',{sources},'unreliable_phase_ids',{cellstr(string(R.id(R.phase_unreliable)))},'invalid_ids',{cellstr(string(R.id(~startsWith(string(R.status),'VALID_'))))}));
fid=fopen(fullfile(out,'reports/E4_1deg.md'),'w','n','UTF-8');cleanup=onCleanup(@()fclose(fid));
fprintf(fid,'# 实验四：固定1°峰值周期跟踪\n\n只读复用旧批次27例（3方法×9频率），新增仿真0次。3°/5°的54例保持归档。各频率运行8+4/f秒，最后两周期分析。\n\n三条核心曲线分别展示幅值增益、相位滞后、直接俯仰跟踪RMSE。正相位表示滞后，增益比低于0.05的相位用黑色叉号标记，不作为可靠相位解释。\n\n辅助工程通过数：仅舵面MPC %d/9、冻结PID+CMG %d/9、联合MPC %d/9；判据沿用增益≥−3dB、|相位|≤45°、RMSE≤0.5°，不能代替连续曲线。\n\n不同频率对应不同飞行时长、高度和动压，结果不是严格固定工作点LTI频率响应，不能直接推断带宽倍数。没有重整PID、修改控制器或重跑仿真。完整表保留失败和较差跟踪状态、拟合残差、偏差、趋势、动压/高度、执行器饱和、耗时与回退。\n\n',counts);
for j=1:3,fprintf(fid,'![%s](../figures/E4_1deg_%s.png)\n\n',yl{j},files{j});end
fprintf(fid,'数据：[27例完整表](../tables/E4_1deg_27cases.csv)；来源与验收：[E4_acceptance.json](E4_acceptance.json)。\n');
end
