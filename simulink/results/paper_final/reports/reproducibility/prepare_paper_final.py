"""Assemble the paper's immutable, self-contained evidence set. No simulations."""
import csv,hashlib,json,shutil,subprocess,time
from pathlib import Path
_HERE=Path(__file__).resolve()
FINAL=_HERE.parents[2] if (_HERE.parents[2]/'manifest.csv').exists() else _HERE.parents[3]/'simulink/results/paper_final'
ROOT=FINAL.parents[2]
OLD=ROOT/'simulink/results/paper_experiments/20261105_redesign_v1'
REV=ROOT/'simulink/results/paper_experiments/20261106_e3_e4_revision'
E2R=ROOT/'simulink/results/paper_experiments/20261106_e2_unload_revision'
GROUPS=('E1_CMG_Benefit','E2_Controller_Comparison','E3_Prediction_Horizon','E4_Frequency_Tracking')
def sha(p):
 h=hashlib.sha256()
 with p.open('rb') as f:
  for b in iter(lambda:f.read(8*1024*1024),b''):h.update(b)
 return h.hexdigest()
def readcsv(p):
 with p.open(encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))
def writecsv(p,rows):
 p.parent.mkdir(parents=True,exist_ok=True)
 with p.open('w',encoding='utf-8',newline='') as f:
  w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
def dump(p,value):p.parent.mkdir(parents=True,exist_ok=True);p.write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf-8')
def main():
 assert not (FINAL/'manifest.csv').exists(),'Final dataset already assembled; verify instead of overwriting'
 for group in GROUPS:
  for sub in ('scenarios','raw','tables','figures'):(FINAL/group/sub).mkdir(parents=True,exist_ok=True)
 (FINAL/'reports').mkdir(exist_ok=True)
 head=subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()
 cases=[]
 def add(group,folder,role):cases.append((group,folder,role))
 for r in readcsv(OLD/'tables/E1_all.csv'):add(GROUPS[0],OLD/'E1/cases'/r['id'],'full_feasibility_matrix')
 for amp in (5,10,15):
  for method in (1,3):add(GROUPS[1],OLD/'E2/cases'/f'V2_S{amp:02d}_{method}','single_maneuver')
 for task in (1,2):
  for method in (1,3):add(GROUPS[1],OLD/'E2/cases'/f'V2_C{task}_{method}','continuous_maneuver')
 for task in (1,2):
  for method in (1,2,3):add(GROUPS[1],E2R/'E2/cases'/f'E2R_M{task}_{method}','latest_dynamic_unload')
 for r in readcsv(REV/'tables/E3_10cases.csv'):add(GROUPS[2],Path(r['source_path']),'horizon_MZ_pair')
 for r in readcsv(REV/'tables/E4_1deg_27cases.csv'):add(GROUPS[3],Path(r['source_path']),'one_degree_peak_sine')
 assert len(cases)==197 and len({p.name for _,p,_ in cases})==197
 payload=[]
 def copy(src,dst,role):
  assert src.is_file(),str(src);dst.parent.mkdir(parents=True,exist_ok=True)
  size=src.stat().st_size;before=sha(src)
  if dst.exists():assert sha(dst)==before,'Existing target differs: '+str(dst)
  else:shutil.copy2(src,dst)
  assert sha(dst)==before and sha(src)==before,'Copy/source integrity failure'
  payload.append({'path':dst.relative_to(FINAL).as_posix(),'bytes':size,'sha256':before,'source':src.relative_to(ROOT).as_posix() if src.is_relative_to(ROOT) else str(src),'role':role})
 manifest=[]
 for i,(group,folder,role) in enumerate(cases):
  cid=folder.name;base=FINAL/group;sc=base/'scenarios'/cid;raw=base/'raw'/cid
  cfg=json.loads((folder/'case_config.json').read_text(encoding='utf-8'));metrics=json.loads((folder/'metrics.json').read_text(encoding='utf-8'));build=json.loads((folder/'build_config.json').read_text(encoding='utf-8'))
  for p in folder.glob('*.mat'):copy(p,(sc if p.name=='reference_bank.mat' else raw)/p.name,'reference_bank' if p.name=='reference_bank.mat' else 'native_or_diagnostic_MAT')
  for p in folder.glob('*.json'):copy(p,sc/p.name,'original_configuration_or_metric')
  for p in folder.glob('dynamic_unload_ticks.csv'):copy(p,base/'tables'/f'{cid}_dynamic_unload_ticks.csv','measured_unload_evidence')
  model=Path(build['model_path']);copy(model,sc/'model.slx','exact_generated_case_model')
  method='JOINT_MPC' if cfg['method']=='MPC' and cfg['cmg'] else 'SURFACE_MPC' if cfg['method']=='MPC' else 'PID_DYNAMIC_UNLOAD' if 'DynamicUnload' in cfg['method'] else 'PID_CMG'
  row={'experiment':group,'case_id':cid,'role':role,'method':method,'source_batch':str(folder.relative_to(ROOT)),'scenario_path':(sc/'case_config.json').relative_to(FINAL).as_posix(),'raw_path':(raw/'sim_output.mat').relative_to(FINAL).as_posix(),'bank_path':(sc/'reference_bank.mat').relative_to(FINAL).as_posix(),'model_path':(sc/'model.slx').relative_to(FINAL).as_posix(),'metrics_path':(sc/'metrics.json').relative_to(FINAL).as_posix(),'raw_sha256':sha(raw/'sim_output.mat'),'bank_sha256':sha(sc/'reference_bank.mat'),'model_sha256':sha(sc/'model.slx'),'source_git_commit':head,'status':metrics['status'],'complete':metrics['complete'],'model_domain_status':'failed' if metrics['status']=='MODEL_DOMAIN_FAIL' else 'valid_recorded' if metrics['complete'] and not metrics['simulation_aborted'] else 'unverified_or_incomplete','original_stability_status':'not_applicable_periodic' if cfg['profile']=='sine' else 'passed' if metrics['status']=='SUCCESS' else 'failed','amplitude_deg':cfg['amplitude'],'onset_s':cfg['onset'],'transition_s':cfg['transition'],'frequency_Hz':cfg['frequency'],'horizon_s':sum(cfg['dts']),'initial_conditions':'reference_bank.mat:x0; model.slx:initial flight state; case_config.json:gamma0','reference_and_disturbance':'reference_bank.mat:q_ref/omega_ref/tau_d/time','analysis_script':'reports/reproducibility/code/simulink/paper/v3/v3_analyze.m','paper_reproduction_script':'reports/reproducibility/reproduce_paper_final.m'}
  manifest.append(row)
  if i%25==0:print(f'ARCHIVE {i+1}/197 {cid}',flush=True)
 writecsv(FINAL/'manifest.csv',manifest)
 table_sources={GROUPS[0]:[(OLD/'tables'/n,n) for n in ('E1_all.csv','E1_paired.csv','E1_min_duration.csv','E1_domain_failures.csv')],GROUPS[1]:[(OLD/'tables/E2_event_metrics.csv','original_event_metrics_selected.csv')]+[(E2R/'tables'/n,n) for n in ('M1M2_6cases.csv','M1M2_stage_metrics.csv','M1M2_paired_results.csv','M1M2_offline_feasibility.csv','C2_segment_metrics.csv','C2_event_boundary_states.csv','C2_3maneuvers_recomputed.csv','C2_solver_diagnostics.csv','C2_required_torque_segments.csv','C2_per_actuator_segment_saturation.csv')],GROUPS[2]:[(REV/'tables'/n,n) for n in ('E3_10cases.csv','E3_5pairs.csv')],GROUPS[3]:[(REV/'tables/E4_1deg_27cases.csv','E4_1deg_27cases.csv')]}
 selected={row['case_id'] for row in manifest}
 for group,pairs in table_sources.items():
  for src,name in pairs:
   if name=='original_event_metrics_selected.csv':writecsv(FINAL/group/'tables'/name,[r for r in readcsv(src) if r['case_id'] in selected])
   else:copy(src,FINAL/group/'tables'/name,'unchanged_official_summary_table')
 for group in GROUPS:
  writecsv(FINAL/group/'tables/case_index.csv',[r for r in manifest if r['experiment']==group])
 # Tables with historic absolute paths remain byte-identical evidence; local indices below resolve them.
 writecsv(FINAL/'reports/path_mapping.csv',[{'case_id':r['case_id'],'original_case_folder':r['source_batch'],'local_raw':r['raw_path'],'local_scenario':r['scenario_path']} for r in manifest])
 e1=[r for r in manifest if r['experiment']==GROUPS[0]];official=readcsv(OLD/'tables/E1_all.csv')
 representative=['V1_0050_0020_A','V1_0050_0020_B','V1_0075_0020_A','V1_0075_0020_B','V1_0250_0200_A','V1_0250_0200_B']
 dump(FINAL/'reports/selection_decisions.json',{'counts':{g:sum(r['experiment']==g for r in manifest) for g in GROUPS},'E1_representatives':representative,'E1_domain_failure_count':sum(r['status']=='MODEL_DOMAIN_FAIL' for r in official),'E1_boundary_rule':'Highest recorded alpha, not a fabricated domain failure; compare against archived polar range','E2_excluded':'fixed2Nm unload; duplicate old unload S/C curves; old M1/M2','E3_excluded':'onset8s historical horizon studies','E4_excluded':'3/5degree data','simulation_count_added':0,'source_git_commit':head})
 # Copy minimal native physical dependencies and the actual analysis/controller source tree.
 code=FINAL/'reports/reproducibility/code'
 paths=subprocess.check_output(['git','ls-files','-z','simulink/paper','simulink/hpipm','simulink/modelgen','simulink/data','simulink/verify','simulink/models','simulink/repo_root.m','simulink/init_paths.m'],cwd=ROOT).decode().split('\0')
 for rel in paths:
  if not rel:continue
  p=ROOT/rel
  if p.suffix in ('.m','.mexw64','.slx') and '/results/' not in rel:copy(p,code/rel,'frozen_code_or_physical_model_dependency')
 deps=['simulink/data/native_environment.mat','simulink/data/public_surrogate/force_polar.mat','simulink/data/native_baseline/A4_descent_replay_S00_with_cmg/vehicle_params.mat','simulink/data/paper_banks/archive/S00_pitch30_B_bank.mat','simulink/tests/05_python_matlab_qp_equivalence/production_online_qp.mat','simulink/tests/05_python_matlab_qp_equivalence/production_online_A_qp.mat']
 for rel in deps:copy(ROOT/rel,code/rel,'frozen_physical_or_controller_data_dependency')
 for p in sorted((ROOT/'simulink/hpipm/mex').glob('*.mexw64')):copy(p,code/p.relative_to(ROOT),'exact_executed_MEX_dependency')
 for name,path in [('libhpipm.dll',Path('D:/Tools/hpipm/build/libhpipm.dll')),('libblasfeo.dll',Path('D:/Tools/blasfeo/build/libblasfeo.dll'))]:copy(path,FINAL/'reports/reproducibility/runtime'/name,'original_external_HPIPM_runtime_DLL')
 for src in (OLD/'reports/acceptance.json',REV/'reports/E3_acceptance.json',REV/'reports/E4_acceptance.json',E2R/'reports/E2_acceptance.json',E2R/'reports/C2_第二次退化诊断.md',E2R/'reports/M1M2_场景可行性.md',E2R/'reports/independent_physical_verification.json'):
  copy(src,FINAL/'reports/source_evidence'/src.name,'original_acceptance_and_limitations')
 for batch,names in {
  OLD:['config/source_manifest.json','config/source_git_sha.txt','config/git_delivery.json'],
  REV:['config/E3_compatibility.json','config/E3_incremental_matrix.json','reports/E3_controller_audit.json'],
  E2R:['config/baseline_git.json','config/frozen_pid.json','config/physical_parameters.json','config/formal_matrix.json','config/M1_native_basis.mat','config/M2_native_basis.mat','config/scenario_offline.json','config/scenario_feasibility_gate.json','reports/revision_source_manifest.json']
 }.items():
  for name in names:copy(batch/name,FINAL/'reports/source_evidence'/batch.name/name,'original_batch_provenance_or_initial_state_basis')
 writecsv(FINAL/'reports/payload_sha256.csv',payload)
 dump(FINAL/'reports/assembly_receipt.json',{'cases':len(cases),'copied_files':len(payload),'copied_bytes':sum(r['bytes'] for r in payload),'source_git_commit':head,'source_and_copy_SHA256_equal':True,'new_simulations':0,'free_bytes_after':shutil.disk_usage(ROOT).free})
 print('ARCHIVE_COMPLETE',len(cases),sum(r['bytes'] for r in payload),flush=True)
if __name__=='__main__':main()
