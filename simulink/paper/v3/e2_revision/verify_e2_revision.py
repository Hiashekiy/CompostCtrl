"""Independent standard-library checks on native-derived evidence and run provenance."""
import csv,hashlib,json,math,subprocess
from pathlib import Path
ROOT=Path(__file__).resolve().parents[4]
OUT=ROOT/'simulink/results/paper_experiments/20261106_e2_unload_revision'
OLD=ROOT/'simulink/results/paper_experiments/20261105_redesign_v1'
def load(p):return json.loads(p.read_text(encoding='utf-8'))
def rows(p):
    with p.open(encoding='utf-8-sig',newline='') as f:return list(csv.DictReader(f))
def sha(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(4*1024*1024),b''):h.update(b)
    return h.hexdigest()
def write(p,value):p.write_text(json.dumps(value,ensure_ascii=False,indent=2),encoding='utf-8')
def main():
    events=rows(OUT/'tables/C2_3maneuvers_recomputed.csv');reference=[r for r in rows(OLD/'tables/E2_event_metrics.csv') if r['case_id']=='V2_C2_3']
    assert len(events)==len(reference)==3
    for event,old in zip(events,reference):
        for field in ('maneuver_rmse_deg','post_rmse_deg'):
            assert abs(float(event[field])-float(old[field]))<1e-8
    accum=[[0.,0] for _ in range(3)];post=[[0.,0] for _ in range(3)];n=0;first_error=None
    params=load(OUT/'config/physical_parameters.json');segments=((0,8),(8,12),(12,20),(20,24),(24,32),(32,36),(36,46));counts=[0]*7
    actuator_limits={**{f'nu{i}':params['cmg']['gdot_max'] for i in range(1,5)},**{f'acc{i}':params['cmg']['gddot_max'] for i in range(1,5)},**{f'delta{i}':params['vehicle']['delta_max'][i-1] for i in range(1,4)},**{f'delta_dot{i}':params['vehicle']['delta_dot_max'][i-1] for i in range(1,4)}}
    saturation=[{key:0 for key in actuator_limits} for _ in segments]
    with (OUT/'tables/C2_native_diagnostic.csv').open(encoding='utf-8-sig',newline='') as f:
        for r in csv.DictReader(f):
            t=float(r['time_s']);e=float(r['attitude_error_deg']);n+=1
            if first_error is None and 20<=t<=24 and abs(float(r['pitch_error_deg']))>1:first_error=t
            for j,(lo,hi) in enumerate(segments):
                if lo<=t<hi or (j==6 and t==hi):
                    counts[j]+=1
                    for key,limit in actuator_limits.items():saturation[j][key]+=abs(float(r[key]))>=limit-1e-5
            for j,start in enumerate((8,20,32)):
                if start-1e-9<=t<=start+4+1e-9:accum[j][0]+=e*e;accum[j][1]+=1
                stop=(20,32,46)[j]
                if start+4+1e-9<t<=stop+1e-9:post[j][0]+=e*e;post[j][1]+=1
    assert n==60001
    per_actuator=[]
    for j,(lo,hi) in enumerate(segments):per_actuator.append({'start_s':lo,'end_s':hi,'native_samples':counts[j],**{key+'_sat_pct':100*value/counts[j] for key,value in saturation[j].items()}})
    with (OUT/'tables/C2_per_actuator_segment_saturation.csv').open('w',encoding='utf-8',newline='') as f:
        writer=csv.DictWriter(f,fieldnames=list(per_actuator[0]));writer.writeheader();writer.writerows(per_actuator)
    for j,r in enumerate(events):
        assert abs(math.sqrt(accum[j][0]/accum[j][1])-float(r['maneuver_rmse_deg']))<1e-8
        assert abs(math.sqrt(post[j][0]/post[j][1])-float(r['post_rmse_deg']))<1e-8
    solver=rows(OUT/'tables/C2_solver_diagnostics.csv');stuck=[r for r in solver if 12<=float(r['time_s'])<24]
    assert len(stuck)==60 and all(float(r['qp_solved'])==1 and float(r['accepted_steps'])==0 and float(r['fallback'])==0 and r['early_stop_reason']=='objective_increase' for r in stuck)
    physical=load(OUT/'reports/independent_physical_verification.json');assert physical['passed']
    acceptance=load(OUT/'reports/E2_acceptance.json');assert acceptance['formal_actual']==6 and acceptance['formal_complete'] and acceptance['physical_and_period_checks_passed'] and acceptance['paired_protocol_passed']
    source_dir=ROOT/'simulink/paper/v3/e2_revision';source={p.relative_to(ROOT).as_posix():sha(p) for p in sorted(source_dir.iterdir()) if p.suffix in ('.m','.py')}
    write(OUT/'reports/revision_source_manifest.json',{'role':'Final analysis source manifest; each simulation has its own immutable source_manifest and configuration hash','files':source})
    baseline=OUT/'config/baseline_git.json'
    if not baseline.exists():write(baseline,{'pre_revision_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()})
    run_sources=[];raw_hashes=[]
    for group,count in (('E2',6),('E2_PRECHECK',2)):
        folders=sorted((OUT/group/'cases').iterdir());assert len(folders)==count
        for folder in folders:
            cfg=load(folder/'case_config.json');st=load(folder/'run_status.json');build=load(folder/'build_config.json');sim_sources=load(folder/'source_manifest.json');raw=folder/'sim_output.mat';assert raw.is_file() and not raw.is_symlink()
            assert st['status'] not in ('RUNNING','INFRA_FAIL','ANALYSIS_FAIL','MODEL_DOMAIN_FAIL','CONTROLLER_FAIL')
            for name,expected in sim_sources.items():
                candidates=list(source_dir.glob(name+'.m'))+list((ROOT/'simulink/paper/v3').glob(name+'.m'))
                assert len(candidates)==1 and sha(candidates[0])==expected,(folder.name,name)
            assert abs(sum(cfg['dts'])-23.2)<1e-9
            assert build['fixed_step']==.001 and build['inner_period']==.02 and build['dt_ctrl']==.2 and build['sqp_limit']==4
            rawsha=sha(raw);raw_hashes.append(rawsha)
            proof={'id':folder.name,'group':group,'phase':cfg['phase'],'raw_path':str(raw),'raw_sha256':rawsha,'config_sha256':sha(folder/'case_config.json'),'model_sha256':build['model_sha256'],'bank_sha256':build['bank_sha256'],'baseline_git':load(baseline),'simulation_sources':sim_sources,'analysis_sources':'reports/revision_source_manifest.json','status':st['status'],'actual_end_s':st['actual_end_s']}
            write(folder/'provenance.json',proof);run_sources.append(proof)
    assert len(set(raw_hashes))==8
    assert not list((OUT/'E2/cases').glob('*C2*'))
    result={'passed':True,'C2_native_samples':n,'C2_event_metrics_match_original_and_independent_native_reduction':True,'C2_first_pitch_error_over1deg_during_second_s':first_error,'C2_successful_QPs_but_rejected_full_steps_12to24':len(stuck),'formal_simulations':6,'informal_prechecks':2,'C2_new_simulations':0,'distinct_new_raw_outputs':8,'all_simulation_sources_unchanged_since_execution':True,'physical_endpoint_check_passed':True,'runs':run_sources}
    write(OUT/'reports/independent_evidence_verification.json',result)
    print(json.dumps({k:v for k,v in result.items() if k!='runs'},ensure_ascii=False))
if __name__=='__main__':main()
