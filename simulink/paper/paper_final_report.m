function paper_final_report(out)
paper_reference_manifest(out);
if ~isfolder(fullfile(out,'tables')),mkdir(fullfile(out,'tables'));end
T=readtable(fullfile(out,'metrics.csv'),'TextType','string');mapping={'E1_baseline','E1_typical';'E1','E1_feasible_grid';'E2','E2_pid_vs_mpc';'E3','E3_horizon';'E4','E4_frequency_response';'E2_followup','E2_valid_domain_followup';'E3_followup','E3_valid_domain_followup';'E4_followup','E4_extended_frequency_response'};
for k=1:size(mapping,1),S=T(T.experiment==mapping{k,1},:);writetable(S,fullfile(out,'tables',[mapping{k,2} '.csv']));end
S=T(T.experiment=="E1" & T.profile=="step",:);rows={};for cmg=[0 1],for a=unique(S.amplitude)',M=S(S.cmg==cmg & S.amplitude==a,:);yes=M.transition(M.status=="SUCCESS");if isempty(yes),duration=NaN;bound='no_success_in_grid';else,duration=min(yes);if numel(yes)==height(M),bound='all_succeed_upper_bound_only';else,bound='minimum_success_tested_grid';end;end;rows{end+1}=struct('cmg',cmg,'amplitude_deg',a,'shortest_successful_reference_duration_s',duration,'interpretation',bound);end;end
writetable(struct2table([rows{:}],'AsArray',true),fullfile(out,'tables','E1_min_duration.csv'));
p=jsondecode(fileread(fullfile(out,'preflight.json')));checks={};for k=1:3,e=p.protected{k};checks{k}=struct('path',e.path,'initial_sha256',e.sha256,'final_sha256',paper_hash(e.path),'unchanged',strcmp(e.sha256,paper_hash(e.path)));end;paper_json(fullfile(out,'protected_inputs_final.json'),checks);
f=fopen(fullfile(out,'四组论文实验结果.md'),'w','n','UTF-8');c=onCleanup(@()fclose(f));fprintf(f,'# 四组论文实验结果\n\n基线 `%s`。全部结果来自原生 MATLAB/Simulink。正式模型、默认参考库的前后 SHA256 校验见 protected_inputs_final.json。\n\n',p.git_sha);
fprintf(f,'## 结果范围\n\n本批次保留完整扫描点、有效域中断及基础设施失败。判据修订记录见 criteria_v2.md 和 criteria_E4_v3.md（后者是分析阶段透明修订，不是预注册）；旧标记保留在 legacy_status。附加 10° E2/E3 对照的原因和矩阵见 matrix_followup.json。原始 E3 缩短网格未对齐 1ms 物理时钟，不能用于时域因果结论；校正网格见 E3_followup。未通过卸载物理验收的 PID 卸载方法不计为有效基线。\n\n');
paper_report_findings(f,T);
for exp=unique(T.experiment)'
 S=T(T.experiment==exp,:);fprintf(f,'## %s\n\n|工况|方法|状态|RMSE (deg)|峰值 (deg)|终止时间 (s)|\n|---|---|---|---:|---:|---:|\n',exp);
 for k=1:height(S),fprintf(f,'|%s|%s|%s|%.4f|%.4f|%.3f|\n',S.id(k),S.method(k),S.status(k),S.rmse_deg(k),S.peak_deg(k),S.t_end(k));end
 fprintf(f,'\nRMSE 包含各次实际有效运行区间；截断工况不得与完整运行直接比较。每个工况目录保留 sim_output.mat、timeseries.csv、metrics.json、extra_metrics.json、跟踪 PNG/PDF 和失败堆栈。\n\n');
end
fprintf(f,'## 未生成完整指标的失败与阻断\n\n');files=dir(fullfile(out,'E*','*','run_status.json'));for k=1:numel(files),folder=files(k).folder;if ~isfile(fullfile(folder,'metrics.json')),st=jsondecode(fileread(fullfile(folder,files(k).name)));fprintf(f,'- %s: %s。错误堆栈见该目录 run_status.json。\n',folder,st.status);end;end;fprintf(f,'\n');
for k=1:4,path=fullfile(out,'figures',sprintf('comparison_%d.png',k));if isfile(path),fprintf(f,'![对照图%d](%s)\n\n',k,strrep(path,'\','/'));end;end
fprintf(f,'## 复现命令\n\n从仓库根目录经 simulink/mlab.cmd 启动 MATLAB，先 addpath(''simulink''); init_paths; addpath(''simulink/paper''); 再执行 run_paper_suite(''all'',''20261105_paper_v2'')。支持 preflight/smoke/E1/E2/E3/E4/report；report 只从已有日志生成报告，不启动仿真。原始数据不提交版本库。\n\n');
paper_report_pid(f,out);
fprintf(f,'## 结论限制\n\n仅在共同有效、完整运行和相同参考下比较。E1 是离散闭环任务成功区域，不是理论可达域。E3 框架提前运动不能单独证明主动卸载或奇异性优化。E4 是含未来参考预瞄的时变飞行段正弦跟踪，不能等同常规因果反馈闭环带宽；本报告不宣称严格 LTI 带宽。低频长时间参考的扰动延长验证误差见 bank.disturbance_validation，使用原有公式和原生空气密度表，没有调用 Python。计算耗时是本机同步仿真测量，不是硬件实时验证。所有验收未覆盖项仍属未完成，不得把批处理退出码视为论文验收通过。\n');
end












