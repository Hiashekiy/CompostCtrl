"""Paper tables, evidence-bound summaries and safe cleanup planning/verification."""
import csv,hashlib,json,shutil,subprocess,sys,zipfile
from pathlib import Path
_HERE=Path(__file__).resolve();OUT=_HERE.parents[2] if (_HERE.parents[2]/'manifest.csv').exists() else _HERE.parents[3]/'simulink/results/paper_final';ROOT=OUT.parents[2];ARCHIVE=ROOT/'simulink/results/paper_archive/20261106_paper_final_cleanup.zip'
GROUPS=['E1_CMG_Benefit','E2_Controller_Comparison','E3_Prediction_Horizon','E4_Frequency_Tracking']
def rows(p):
 with p.open(encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))
def load(p):return json.loads(p.read_text(encoding='utf-8'))
def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(8*1024*1024),b''):h.update(b)
 return h.hexdigest()
def dump(p,v):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(v,ensure_ascii=False,indent=2),encoding='utf-8')
def csvout(p,r):
 p.parent.mkdir(parents=True,exist_ok=True)
 with p.open('w',encoding='utf-8',newline='') as f:
  w=csv.DictWriter(f,fieldnames=list(r[0]));w.writeheader();w.writerows(r)
def text(p,s):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(s,encoding='utf-8')
def reports():
 index=rows(OUT/'manifest.csv');payload=rows(OUT/'reports/payload_sha256.csv');rep=load(OUT/'reports/reproduction_receipt.json');assert rep['passed'] and rep['raw_cases']==197
 metrics={r['case_id']:load(OUT/r['metrics_path']) for r in index};official={r['id']:r for r in rows(OUT/GROUPS[2]/'tables/E3_10cases.csv')};official.update({r['id']:r for r in rows(OUT/GROUPS[3]/'tables/E4_1deg_27cases.csv')})
 # Add accurate provenance labels without changing any archived original configuration/metric.
 for r in index:
  r['repository_snapshot_git']=r['source_git_commit'];r['formal_selection_git']='08b2053' if r['role']=='latest_dynamic_unload' else '5413f48' if r['experiment'] in GROUPS[2:] else 'b069828';r['original_result_archive_git']='08b2053' if r['role']=='latest_dynamic_unload' else 'b069828' if '20261105_redesign_v1' in r['source_batch'] else '5413f48'
  provenance=load((OUT/r['scenario_path']).parent/'provenance.json')
  r['run_baseline_git']=provenance.get('upstream_baseline_git_sha',provenance.get('baseline_git_sha',provenance.get('baseline_git',{}).get('pre_revision_commit','not_recorded')))
  r['case_config_sha256']=next(p['sha256'] for p in payload if p['path']==r['scenario_path'])
  m=metrics[r['case_id']];r['periodic_tracking_criterion']=m['acceptable_tracking'] if r['experiment']==GROUPS[3] else 'not_applicable';r['actual_end_s']=m['actual_end_s']
 csvout(OUT/'manifest.csv',index)
 canonical=[]
 fields=('rmse_full_deg','rmse_partial_deg','peak_attitude_error_deg','maneuver_rmse_deg','maneuver_peak_deg','post_rmse_deg','mean_compute_s','p95_compute_s','max_compute_s','over200ms_pct','fallback_count','surface_sat_pct','surface_rate_sat_pct','cmg_rate_sat_pct','qbar_min_Pa','qbar_max_Pa','height_min_m','height_max_m','gain_db','gain_ratio','phase_lag_deg','phase_unreliable','pitch_rmse_deg','acceptable_tracking')
 for r in index:
  m=metrics[r['case_id']];canonical.append({**{k:r[k] for k in ('experiment','case_id','method','status','complete','original_stability_status','raw_path','scenario_path','metrics_path')},**{k:m.get(k) for k in fields},'generic_window_note':'For continuous tasks use events_from_raw.csv; generic legacy window is not each event'})
 for group in GROUPS:
  csvout(OUT/group/'tables/paper_case_metrics.csv',[r for r in canonical if r['experiment']==group]);csvout(OUT/group/'tables/case_index.csv',[r for r in index if r['experiment']==group])
 singles=[r for r in canonical if r['case_id'].startswith('V2_S')];csvout(OUT/GROUPS[1]/'tables/single_maneuver_metrics.csv',singles)
 e1=[r for r in index if r['experiment']==GROUPS[0]];stats=[]
 for method in ('SURFACE_MPC','JOINT_MPC'):
  q=[r for r in e1 if r['method']==method];passed=sum(r['status']=='SUCCESS' for r in q);stats.append({'method':method,'cases':len(q),'success':passed,'success_rate_pct':100*passed/len(q),'model_domain_failures':sum(r['status']=='MODEL_DOMAIN_FAIL' for r in q)})
 csvout(OUT/GROUPS[0]/'tables/success_rates.csv',stats)
 maxalpha=max(metrics[r['case_id']]['max_abs_alpha_deg'] for r in e1);alpha_hi=rep['physical_alpha_domain_deg'][1]
 pair=rows(OUT/GROUPS[1]/'tables/M1M2_paired_results.csv');e3=rows(OUT/GROUPS[2]/'tables/pairs_from_raw.csv');e4=[r for r in index if r['experiment']==GROUPS[3]];e4counts={m:sum(metrics[r['case_id']]['acceptable_tracking'] for r in e4 if r['method']==m) for m in ('SURFACE_MPC','PID_CMG','JOINT_MPC')}
 summaries=[]
 summaries.append(f'''# 实验一：CMG对姿态机动能力的提升

目的：在相同原生飞行模型与控制设置下，比较仅舵面MPC和联合MPC的机动可行性。

正式来源为20261105_redesign_v1的144例：9档俯仰增量×8档过渡时间×2构型，72个配对点。完整原始MAT、参考bank、配置和生成模型全部保留，而非只留成功点。

仅舵面成功22/72（30.56%），联合成功64/72（88.89%），新增42个成功点。完整矩阵包含58个TRACKING_FAIL。最短时间是扫描网格内最短成功值，不能外推为连续空间的绝对最优时间。

正式144例中没有MODEL_DOMAIN_FAIL。最高记录迎角为{maxalpha:.6f}°，有效域上界{alpha_hi:g}°，仍相差{alpha_hi-maxalpha:.6f}°。25°/20s配对只作为正式数据范围代表，不冒称触及或突破有效域。未将过时历史域失败案例混入正式矩阵；其历史原始证据未删除。

代表配对：5°/2s两者成功；7.5°/2s仅联合成功；25°/20s为高迎角范围代表。曲线保留参考/实际、姿态误差、实际舵偏、气动和CMG力矩、驱动限幅利用率。

正文建议：E1_feasibility_matrix、E1_minimum_transition，以及E1_representative_joint_only；其他代表图放补充材料。表格使用success_rates.csv、E1_all.csv和E1_min_duration.csv。

证据均在[E1目录](../E1_CMG_Benefit/)，逐例见[总清单](../manifest.csv)。
''')
 summaries.append('''# 实验二：传统控制与联合MPC比较

目的：保留固定PID与联合MPC的实际差异，并透明呈现连续机动与卸载的负面结果。

正式集16例：5°/10°/15°单次机动各保留PID和联合MPC（6例），C1/C2各保留PID和联合MPC（4例），最新20261106_e2_unload_revision的M1/M2×3方法（6例）。旧固定2Nm卸载及单次/连续任务的重合卸载PID曲线不混入主表，原历史数据仍保留。

单次主表使用single_maneuver_metrics.csv。连续任务使用events_from_raw.csv的逐事件窗口，不把旧通用maneuver_rmse字段当作C2每次机动值。C1最后一次机动只观测8s后段，不宣称完整稳定性观察通过。

C2第二次联合MPC机动RMSE=3.232611°，后段8s RMSE=4.626222°；第三次分别为0.557230°和0.023094°。12–24s有60次QP成功但全步因objective_increase被拒、接受步数为0；这是优化停滞的近端证据，不能认定已唯一查明深层原因，也不能用角动量“耗尽/补充”解释。完整原始MAT、分段状态、实际力矩与求解日志均保留。

最新动态卸载：M1机动RMSE 2.288939→2.194236°、后10s 2.348221→2.030444°；M2机动RMSE 2.030638→2.515378°、后10s 2.134792→3.847160°。机动前H模分别相对无卸载降低8.917572/7.212534Nms，但M2反而恶化。四个PID案例均未过原稳定判据，联合MPC两例通过。不得以H模下降或局部方向投影支撑增加代替跟踪收益。

卸载窗口依据真实动压与耦合舵面权限，A=2–12s、B=12–35s、C=35–40s，后完整观测40–50s。逐周期实际补偿采用同态无卸载条件端点参照，不冒充另一条反事实飞行。源实现和全部参数保持不变；本次没有新增仿真。

正文建议：E2_single_maneuvers、E2_C1_continuous、E2_C2_continuous、E2_unload_following_maneuver。权限窗口、构型变化和实际补偿图作为机理/补充证据。主表用events_from_raw.csv及M1M2_paired_results.csv；C2诊断原文见reports/source_evidence。
''')
 summaries.append('''# 实验三：预测时域与CMG提前构型迁移

正式来源为20261106_e3_e4_revision，完整保留5/8/12/16/23.2s×有机动M/无机动Z共10例。35s开始、5s完成+10°俯仰；同H的M/Z初态和物理环境配对。三例只读复用来源也已实际复制到本目录，不需要去旧目录找数据。

M机动段RMSE按H依次为2.517184、2.255163、1.993398、1.760107、1.317855°。任务相关迁移定义为同H的M−Z最大框架差严格超过2°且持续至少0.6s；提前量依次为3.357、6.678、10.603、14.560、21.510s，预览前框架差均为0。Z只参与保持/配对，不混入机动排名。

保留40–50s误差、完整框架与H差、34.8s正负方向投影支撑、求解均值/P95/最大与超过200ms比例。方向支撑不是其他两轴为零的纯俯仰可达力矩。更早迁移与较低误差是本组观察关联，尚非已证明的唯一因果关系。不同网格、执行负载及新旧运行来源限制计算开销比较，也不能据此证明硬实时能力。

未纳入onset=8s历史预测时域扫描；失败状态没有删除。正文建议E3_horizon_performance与E3_task_relative_migration，局部方向权限图可放机理说明。表使用E3_10cases.csv、E3_5pairs.csv及独立原始复算pairs_from_raw.csv。
''')
 summaries.append(f'''# 实验四：固定1°周期姿态跟踪

正式来源为20261106_e3_e4_revision的固定1°峰值（非峰峰值）正弦，9个频率0.02/0.04/0.06/0.08/0.10/0.12/0.15/0.18/0.20Hz×3方法，共27例。所有原始输出均已复制，3°/5°历史数据不进入本目录。

每例运行8+4/f秒，以最后两周期按原v3_fit_periodic定义统计。只整理E4_gain、E4_phase、E4_rmse三个核心图。增益比<0.05的相位以叉号保留，并断开不可靠相位的连线；不将其当作可靠相位比较。没有平滑或删除不利频点。

旧辅助工程判据通过数：仅舵面{e4counts['SURFACE_MPC']}/9、冻结PID+CMG {e4counts['PID_CMG']}/9、联合MPC {e4counts['JOINT_MPC']}/9。主结论依据三条连续指标曲线，不用通过数替代。状态、拟合残差、动压/高度范围、耗时和饱和信息均在E4_1deg_27cases.csv与paper_case_metrics.csv保留。

不同频率对应不同真实飞行时长、高度和动压。这是时变飞行任务下的周期跟踪比较，不能直接称为固定工作点严格LTI带宽或宣称带宽倍数提升。
''')
 for i,s in enumerate(summaries):text(OUT/'reports'/f'E{i+1}_summary.md',s);text(OUT/GROUPS[i]/'summary.md',s.replace('../manifest.csv','../manifest.csv'))
 text(OUT/'reports/paper_experiment_overview.md','''# 论文实验总览

唯一论文数据入口为paper_final。197例：E1=144，E2=16，E3=10，E4=27；全部包含实际复制的原始sim_output.mat、bank、配置、案例模型和SHA256。没有新增Simulink仿真。

建议正文顺序：E1先回答机动能力范围，E2比较传统控制并保留连续任务/卸载负面结果，E3展示预测时域与配对迁移的观察关系，E4用固定1°的三项周期跟踪指标收尾。不得将更早迁移写成已证明唯一原因，也不得将周期比较写成严格LTI带宽。

|章节|正文主表|正文主图|补充材料|
|---|---|---|---|
|E1|success_rates、E1_all、E1_min_duration|可行矩阵、最短时间、仅联合成功代表|两者成功、高迎角范围代表|
|E2|single_maneuver_metrics、events_from_raw、M1M2_paired_results|单次、C1/C2、卸载后机动|动压权限、构型、补偿、C2求解日志|
|E3|E3_10cases、E3_5pairs、pairs_from_raw|时域性能、五组相对迁移|局部正负方向支撑与计算限制|
|E4|E4_1deg_27cases|增益、相位、RMSE三图|状态、环境、耗时与饱和列|

读者应同时看到：C2第二次退化及原因未决、C1后观察窗口不足、M2卸载恶化、E1没有正式域中断点。详见四份分组总结、manifest.csv、figure_registry.csv和reproduction_checks.csv。

本资料属于实验事实底稿，不是已经由人类作者确认的论文正文。后续作者需核对论述与原始证据；未引入统计显著性、虚构重复样本或未测因果效应。
''')
 text(OUT/'README.md','''# 论文专用实验数据集

今后论文写作只从本目录取数据。197个正式案例，E1=144、E2=16、E3=10、E4=27。原始MAT、参考bank与模型为真实文件复制，不是仅有旧路径的占位引用。

入口：[总览](reports/paper_experiment_overview.md)、[197例清单](manifest.csv)、[图表目录](reports/figure_registry.csv)、[逐文件SHA256](reports/file_sha256.csv)、[原始复算](reports/reproduction_checks.csv)。每组有summary.md，reports下亦有四份总结。

scenarios/<id>/保存未改动的原始case_config、metrics、build_config、provenance、reference_bank.mat及model.slx；raw/<id>/保存sim_output.mat和原控制/执行器诊断MAT。manifest记录本目录相对路径、Git来源、完成/有效域/原稳定判据。旧表/JSON中原有绝对路径为不改写的历史证据；论文使用manifest和reports/path_mapping.csv解析本地副本，不需要搜索历史目录。

reports/reproducibility/code保存相关原生模型、控制/分析源码与物理/预测数据依赖快照。源案例模型的旧绝对初始化路径作为历史证据保留，未为重仿修改模型；本次验证的是原始输出到指标/图表的后处理复现，未宣称跨机器无需配置即可重新仿真。MATLAB/Simulink版本和核验结果见reproduction_receipt.json。

Git字段区分：repository_snapshot_git/source_git_commit是本次归档读取仓库的快照；run_baseline_git是原始provenance记录的运行前基线，不能据此声称当时未提交的新代码已经存在于该提交。formal_selection_git与original_result_archive_git分别记录正式筛选与原始结果归档提交，实际运行版本由逐案例模型/bank/源码SHA和reports/source_evidence内原始来源清单约束。

## 后处理复现（零新增仿真）

在仓库根目录运行：

```powershell
$env:MATLAB_HOME='D:\\Program Files\\MATLAB\\R2024a'
& .\\simulink\\mlab.cmd "addpath('simulink/results/paper_final/reports/reproducibility'); reproduce_paper_final('D:/项目/飞轮/simulink/results/paper_final'); export_e2_gimbal_pairs('D:/项目/飞轮/simulink/results/paper_final');"
python simulink/results/paper_final/reports/reproducibility/render_paper_final.py
python simulink/results/paper_final/reports/reproducibility/finalize_paper_final.py verify
```

归档内也保存组织/绘图脚本；Python绘图和哈希核验脚本可直接从本目录的reports/reproducibility运行，无需调用历史实验目录。绘图仅使用MATLAB从原始输出导出的真实点，20ms显示分箱保留各列极值，不做平滑；所有指标在完整1ms原始输出上复算。Python不模拟物理系统。

## 数据保留与清理

原始MAT、模型/数据依赖留在本地，Git仅归档清单、配置、源码、摘要与图表，避免上传约7GB原始实验。不要执行全仓git add。旧MAT与原始保护基线未删除。

清理仅针对cleanup_manifest.csv列出的已完整备份缓存及未跟踪重复启动日志。备份位于../paper_archive/20261106_paper_final_cleanup.zip；删除逐文件验证路径和SHA，不递归清空结果目录。具体删除状态及前后统计见reports/deleted_files.csv、cleanup_statistics.json和final_verification.json。
''')
 text(OUT/'.gitignore','*.mat\n*.slx\n*.mexw64\n*.dll\n.work/\n*.log\n')
 # Copy the exact postprocessing entrypoints into the standalone evidence directory.
 for name in ('reproduce_paper_final.m','export_e2_gimbal_pairs.m','prepare_paper_final.py','render_paper_final.py','finalize_paper_final.py'):
  shutil.copy2(ROOT/'simulink/paper/final'/name,OUT/'reports/reproducibility'/name)
 evidence=subprocess.check_output(['git','log','-8','--format=fuller','--stat'],cwd=ROOT)
 (OUT/'reports/reproducibility/git_evidence.raw').write_bytes(evidence)
 text(OUT/'reports/reproducibility/git_evidence.txt',evidence.decode('utf-8',errors='backslashreplace'))
 print('REPORTS_COMPLETE')
def allowed(p):
 p=p.resolve();p.relative_to(ROOT.resolve());rel=p.relative_to(ROOT).as_posix()
 if p.is_relative_to((OUT/'.work').resolve()) and p.suffix=='.csv':return True
 caches=[ROOT/'simulink/results/paper_experiments'/name/'cache' for name in ('20261105_redesign_v1','20261106_e3_e4_revision','20261106_e2_unload_revision')]
 if any(p.is_relative_to(c.resolve()) for c in caches):return True
 return p.parent==ROOT/'simulink/paper' and (p.name.startswith('e2_revision_') or p.name in ('revision_launcher.log','revision_report_launcher.log')) and p.suffix=='.log'
def plan():
 assert load(OUT/'reports/reproduction_receipt.json')['passed'];assert load(OUT/'reports/figure_QA.json')['automated_passed']
 targets=[]
 for name in ('20261105_redesign_v1','20261106_e3_e4_revision','20261106_e2_unload_revision'):
  c=ROOT/'simulink/results/paper_experiments'/name/'cache'
  if c.exists():targets.extend(p for p in c.rglob('*') if p.is_file())
 targets+=list((ROOT/'simulink/paper').glob('e2_revision_*.log'))
 targets += [p for p in [ROOT/'simulink/paper/revision_launcher.log',ROOT/'simulink/paper/revision_report_launcher.log'] if p.is_file()]
 targets += list((OUT/'.work').glob('*.csv'))
 targets=sorted(set(targets));assert not ARCHIVE.exists();ARCHIVE.parent.mkdir(parents=True,exist_ok=True);records=[]
 with zipfile.ZipFile(ARCHIVE,'w',compression=zipfile.ZIP_DEFLATED,compresslevel=6) as z:
  for p in targets:
   assert allowed(p) and not p.is_symlink();rel=p.relative_to(ROOT).as_posix();tracked=subprocess.run(['git','ls-files','--error-unmatch','--',rel],cwd=ROOT,stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL).returncode==0
   assert not tracked,'Never remove Git evidence: '+rel
   digest=sha(p);z.write(p,rel);records.append({'path':rel,'bytes':p.stat().st_size,'sha256':digest,'original_experiment':next((b for b in ('20261105_redesign_v1','20261106_e3_e4_revision','20261106_e2_unload_revision') if b in rel),'paper_final_postprocessing' if '/.work/' in rel else 'launch_logs'),'reason':'reproducible_display_intermediate' if '/.work/' in rel else 'reproducible_cache' if '/cache/' in rel else 'duplicate_untracked_launcher_log','backup_path':ARCHIVE.relative_to(ROOT).as_posix(),'backup_member':rel,'planned_action':'delete_after_commit_and_verified_backup'})
 with zipfile.ZipFile(ARCHIVE) as z:
  assert z.testzip() is None
  for r in records:assert hashlib.sha256(z.read(r['backup_member'])).hexdigest()==r['sha256']
 csvout(OUT/'cleanup_manifest.csv',records);dump(OUT/'reports/cleanup_statistics.json',{'state':'planned_backed_up','before_files':len(records),'before_bytes':sum(int(r['bytes']) for r in records),'backup_bytes':ARCHIVE.stat().st_size,'backup_sha256':sha(ARCHIVE),'raw_MAT_deleted':0,'protected_baselines_deleted':0,'final_dataset_files':sum(p.is_file() for p in OUT.rglob('*')),'final_dataset_bytes':sum(p.stat().st_size for p in OUT.rglob('*') if p.is_file()),'free_bytes_before_cleanup':shutil.disk_usage(ROOT).free})
 print('CLEANUP_PLANNED',len(records),sum(int(r['bytes']) for r in records))
def verify():
 index=rows(OUT/'manifest.csv');assert len(index)==197
 check=rows(OUT/'reports/reproduction_checks.csv');assert len(check)==197 and all(r['passed']=='1' for r in check)
 pairs=rows(OUT/GROUPS[2]/'tables/pairs_from_raw.csv');official=rows(OUT/GROUPS[2]/'tables/E3_5pairs.csv')
 for a,b in zip(pairs,official):assert abs(float(a['migration_2deg_s'])-float(b['migration_2deg_s']))<1e-8 and float(a['prepreview_delta_deg'])==0
 for row in index:
  for key in ('raw_path','scenario_path','bank_path','model_path','metrics_path'):assert (OUT/row[key]).is_file()
 immutable=[]
 exclude={'reports/file_sha256.csv','reports/final_verification.json','reports/cleanup_statistics.json','reports/deleted_files.csv','reports/pre_cleanup_commit.txt'}
 for p in sorted(OUT.rglob('*')):
  if not p.is_file() or '.work' in p.relative_to(OUT).parts:continue
  rel=p.relative_to(OUT).as_posix()
  if rel not in exclude:immutable.append({'path':rel,'bytes':p.stat().st_size,'sha256':sha(p)})
 digests={r['path']:r['sha256'] for r in immutable}
 code_prefix='reports/reproducibility/code/'
 for row in index:
  build=load((OUT/row['scenario_path']).parent/'build_config.json')
  assert digests[row['model_path']]==build['model_sha256']
  assert digests[row['bank_path']]==build['bank_sha256']
  provenance=load((OUT/row['scenario_path']).parent/'provenance.json')
  expected=provenance.get('case_config_sha256',provenance.get('config_sha256'))
  if expected:assert digests[row['scenario_path']]==expected,row['case_id']
  if 'PID_step_sha256' in provenance:assert digests[code_prefix+'simulink/paper/v3/v3_pid_step.m']==provenance['PID_step_sha256']
  for name,digest in provenance.get('simulation_sources',{}).items():
   matching=[key for key in digests if key.startswith(code_prefix) and key.endswith('/'+name+'.m')]
   assert len(matching)==1 and digests[matching[0]]==digest,(row['case_id'],name)
 for row in rows(OUT/'reports/payload_sha256.csv'):assert digests[row['path']]==row['sha256'],row['path']
 prior=OUT/'reports/file_sha256.csv'
 if prior.exists():
  for row in rows(prior):assert digests.get(row['path'])==row['sha256'],'Core file changed after archival: '+row['path']
 for row in rows(OUT/'reports/figure_registry.csv'):
  for key in ('pdf','png'):assert (OUT/row[key]).is_file()
 csvout(prior,immutable);dump(OUT/'reports/final_verification.json',{'passed':True,'formal_cases':197,'group_counts':{g:sum(r['experiment']==g for r in index) for g in GROUPS},'native_metric_checks_passed':True,'payload_copy_hashes_passed':True,'recorded_case_config_hashes_passed':True,'recorded_controller_source_hashes_passed':True,'all_local_paths_exist':True,'E3_pair_times_match_original':True,'hashed_files':len(immutable),'hashed_bytes':sum(int(r['bytes']) for r in immutable),'new_simulations':0,'figures':len(rows(OUT/'reports/figure_registry.csv'))})
 print('FINAL_VERIFY_PASS',len(immutable))
def delete(commit):
 assert subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()==commit
 assert subprocess.run(['git','cat-file','-e',commit+':simulink/results/paper_final/cleanup_manifest.csv'],cwd=ROOT).returncode==0
 records=rows(OUT/'cleanup_manifest.csv');stats=load(OUT/'reports/cleanup_statistics.json');assert sha(ARCHIVE)==stats['backup_sha256'];deleted=[]
 def inventory():
  roots=[ROOT/'simulink/results/paper_experiments'/b for b in ('20261105_redesign_v1','20261106_e3_e4_revision','20261106_e2_unload_revision')]+[OUT,ARCHIVE.parent]
  result={}
  for folder in roots:
   files=[p for p in folder.rglob('*') if p.is_file() and p.name not in ('cleanup_statistics.json','deleted_files.csv','pre_cleanup_commit.txt','final_verification.json')]
   result[folder.relative_to(ROOT).as_posix()]={'files':len(files),'bytes':sum(p.stat().st_size for p in files)}
  return result
 stats['inventory_before_cleanup']=inventory()
 stats['inventory_scope']='Three source batches, paper_final and paper_archive; excludes four rolling cleanup/verification receipts.'
 with zipfile.ZipFile(ARCHIVE) as z:
  for r in records:
   p=ROOT/r['path'];assert allowed(p) and p.is_file() and not p.is_symlink();assert sha(p)==r['sha256'];assert hashlib.sha256(z.read(r['backup_member'])).hexdigest()==r['sha256'];p.unlink();deleted.append({**r,'action':'deleted_after_verified_backup','pre_cleanup_git_commit':commit})
 # No recursive removal: only already-empty cache directories, deepest first.
 parents={parent for r in records for parent in (ROOT/r['path']).parents if parent!=ROOT and '/cache' in parent.as_posix() and parent.is_relative_to(ROOT)}
 for p in sorted(parents,key=lambda p:len(p.parts),reverse=True):
  if p.is_dir() and not any(p.iterdir()):p.rmdir()
 if (OUT/'.work').is_dir() and not any((OUT/'.work').iterdir()):(OUT/'.work').rmdir()
 csvout(OUT/'reports/deleted_files.csv',deleted);stats.update({'state':'completed','after_files':sum((ROOT/r['path']).is_file() for r in records),'after_bytes':sum((ROOT/r['path']).stat().st_size for r in records if (ROOT/r['path']).is_file()),'deleted_files':len(deleted),'deleted_bytes':sum(int(r['bytes']) for r in deleted),'free_bytes_after_cleanup':shutil.disk_usage(ROOT).free,'pre_cleanup_git_commit':commit});text(OUT/'reports/pre_cleanup_commit.txt',commit+'\n');verify();stats['inventory_after_cleanup']=inventory();dump(OUT/'reports/cleanup_statistics.json',stats);print('CLEANUP_COMPLETE',len(deleted))
if __name__=='__main__':
 mode=sys.argv[1]
 if mode=='reports':reports()
 elif mode=='plan':plan()
 elif mode=='verify':verify()
 elif mode=='delete':delete(sys.argv[2])
 else:raise SystemExit('reports | plan | verify | delete COMMIT')
