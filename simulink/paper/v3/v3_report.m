function v3_report(out)
guard=v3_file_lock(fullfile(out,'reports','.report.lock'));
groups={'E1','E2','E3','E4'};expected=[144 21 3 81];allrows={};acceptance=struct;
for gi=1:4
 g=groups{gi};files=dir(fullfile(out,g,'cases','*','metrics.json'));rows={};for k=1:numel(files),if isfile(fullfile(files(k).folder,'.run.lock')),continue;end;sp=fullfile(files(k).folder,'run_status.json');if isfile(sp),try,st=jsondecode(fileread(sp));if strcmp(st.status,'RUNNING'),continue;end;catch,continue;end;end;m=jsondecode(fileread(fullfile(files(k).folder,files(k).name)));keys=fieldnames(m);for j=1:numel(keys),v=m.(keys{j});if isempty(v) && ~ischar(v),m.(keys{j})=NaN;end;end;rows{end+1}=m;end
statuses=dir(fullfile(out,g,'cases','*','run_status.json'));complete=numel(statuses)==expected(gi);infra=0;for k=1:numel(statuses),if isfile(fullfile(statuses(k).folder,'.run.lock')),st=struct('status','RUNNING');else,st=jsondecode(fileread(fullfile(statuses(k).folder,statuses(k).name)));end;infra=infra+ismember(st.status,{'RUNNING','INFRA_FAIL','ANALYSIS_FAIL'});end
acceptance.(g)=struct('expected',expected(gi),'status_count',numel(statuses),'analyzed_count',numel(rows),'matrix_completed',complete && infra==0,'infra_or_analysis_failures',infra);
f=fopen(fullfile(out,'reports',[g '.md']),'w','n','UTF-8');cleanup=onCleanup(@()fclose(f));fprintf(f,'# %s 新批次实验结果\n\n实际状态 %d/%d，已分析 %d。历史数据不计入本批次。模型有效域中断不等于不可完成机动；仅完整且有效的运行可比较 RMSE。\n\n',g,numel(statuses),expected(gi),numel(rows));
if isempty(rows),fprintf(f,'尚无可评估原始数据。\n');clear cleanup;continue;end
T=struct2table([rows{:}],'AsArray',true);writetable(T,fullfile(out,'tables',[g '_all.csv']));for k=1:numel(rows),allrows{end+1}=rows{k};end
fprintf(f,'|配置|方法|状态|完整|全程 RMSE °|部分 RMSE °|实际终点 s|\n|---|---|---|---:|---:|---:|---:|\n');for k=1:height(T),fprintf(f,'|%s|%s|%s|%d|%.4f|%.4f|%.3f|\n',string(T.id(k)),string(T.method(k)),string(T.status(k)),T.complete(k),T.rmse_full_deg(k),T.rmse_partial_deg(k),T.actual_end_s(k));end
fprintf(f,'\n原始数据：%s/%s/cases/<case_id>/sim_output.mat；参考、配置、状态、指标、timeseries 和 domain_diagnostics 与原始数据同目录。真实 CMG 速率/力矩直接取 cluster 物理端口，并通过原生几何交叉核对。\n\n',strrep(out,'\','/'),g);
clear cleanup
switch g
case 'E1',v3_report_e1(T,out);
case 'E3',v3_report_e3(T,out);
case 'E4',v3_report_e4(T,out);
case 'E2',v3_report_e2(T,out);
end
v3_report_notes(g,T,out);
end
p=jsondecode(fileread(fullfile(out,'config','preflight.json')));unchanged=true;checks={};for j=1:3,e=p.protected{j};current=paper_hash(e.path);same=strcmp(current,e.sha256);unchanged=unchanged && same;checks{j}=struct('path',e.path,'initial_sha256',e.sha256,'current_sha256',current,'unchanged',same);end
acceptance.protected_inputs_unchanged=unchanged;acceptance.protected_evidence=checks;acceptance.all_matrices_attempted=all(cellfun(@(g)acceptance.(g).matrix_completed,groups));acceptance.all_analysis_complete=all(cellfun(@(g)acceptance.(g).analyzed_count==acceptance.(g).expected,groups));acceptance.paper_acceptance_complete=false;paper_json(fullfile(out,'reports','acceptance.json'),acceptance);
if ~isempty(allrows),writetable(struct2table([allrows{:}],'AsArray',true),fullfile(out,'tables','all_experiments.csv'));end
f=fopen(fullfile(out,'reports','综合报告.md'),'w','n','UTF-8');c=onCleanup(@()fclose(f));fprintf(f,'# 四组原生六自由度闭环实验综合报告\n\n源提交 %s。E2独立标定与冻结先于E4正式测试。仅原生MATLAB/Simulink，MPC最多4 SQP，上层0.2s，F20=0.02s，物理0.001s。\n\n',p.git_sha);for j=1:4,g=groups{j};a=acceptance.(g);fprintf(f,'- [%s报告](%s.md)：状态 %d/%d，分析 %d，基础设施/分析失败 %d。\n',g,g,a.status_count,a.expected,a.analyzed_count,a.infra_or_analysis_failures);end;fprintf(f,'\n正式模型和默认bank哈希一致：%d。各任务原始失败和中止均保留。验收详情见 acceptance.json；矩阵完成不等于所有任务跟踪成功。任何未完成或无有效数据的项不生成虚构数值。E3旧时域扫描单独标为历史数据；E4不同频率对应不同飞行时间窗，不能称严格LTI带宽。\n\n复现：PowerShell运行 simulink/paper/v3/run.ps1；MATLAB入口 paper_v3(''all'',''20261105_redesign_v1'')，只读报告 paper_v3(''report'',...)。\n',unchanged);
end
