"""Independently verify paired migration against original full-resolution CSVs."""
import csv
import hashlib
import json
import math
from itertools import zip_longest
from pathlib import Path

ROOT=Path(__file__).resolve().parents[1]
OUT=ROOT/'simulink/results/paper_experiments/20261106_e3_e4_revision'

def read(path):
    with path.open(encoding='utf-8-sig',newline='') as f:
        return list(csv.DictReader(f))

def number(text):
    return float(text) if text else math.nan

def same(a,b):
    return (math.isnan(a) and math.isnan(b)) or abs(a-b)<1e-7

def main():
    cases=read(OUT/'tables/E3_10cases.csv')
    pairs=read(OUT/'tables/E3_5pairs.csv')
    e4=read(OUT/'tables/E4_1deg_27cases.csv')
    assert len(cases)==10 and len(pairs)==5 and len(e4)==27
    lookup={row['id']:row for row in cases}
    assert len(lookup)==10
    results=[]
    for pair in pairs:
        paths=[Path(lookup[pair[tag+'_id']]['source_path'])/'cmg_timeseries.csv' for tag in ('M','Z')]
        starts={th:None for th in (.5,2.,5.)}
        onsets={th:math.nan for th in starts}
        max_before=0.
        count=0
        with paths[0].open(newline='') as f,paths[1].open(newline='') as g:
            for m,z in zip_longest(csv.DictReader(f),csv.DictReader(g)):
                assert m is not None and z is not None
                t=float(m['time_s']);assert abs(t-float(z['time_s']))<1e-9
                delta=max(abs(float(m[f'g{i}'])-float(z[f'g{i}'])) for i in range(1,5))*180/math.pi
                if t<number(pair['earliest_preview_s'])-1e-9:max_before=max(max_before,delta)
                for th in starts:
                    if delta>th and t<35:
                        if starts[th] is None:starts[th]=t
                        if math.isnan(onsets[th]) and t-starts[th]>=.6-1e-9:onsets[th]=starts[th]
                    else:starts[th]=None
                count+=1
        assert same(max_before,number(pair['prepreview_max_delta_gamma_deg']))
        for th,key in ((.5,'0p5'),(2.,'2'),(5.,'5')):
            assert same(onsets[th],number(pair[f'migration_{key}deg_s']))
        results.append({'H':number(pair['horizon_s']),'samples_per_case':count,'independent_migration_2deg_s':None if math.isnan(onsets[2.]) else onsets[2.],'prepreview_max_delta_gamma_deg':max_before,'passed':True})
    for row in e4:
        source=json.loads(Path(row['source_metrics']).read_text(encoding='utf-8'))
        for key in ('gain_db','phase_lag_deg','pitch_rmse_deg'):
            a=source[key] if source[key] is not None else math.nan
            assert same(a,number(row[key]))
    statuses=[json.loads(p.read_text()) for p in sorted((OUT/'E3/cases').glob('*/run_status.json'))]
    assert len(statuses)==7 and all(s['status'] not in ('RUNNING','INFRA_FAIL','ANALYSIS_FAIL') for s in statuses)
    raw=list((OUT/'E3/cases').glob('*/sim_output.mat'));assert len(raw)==7
    assert not (OUT/'E4').exists(), 'E4 must not have new simulations'
    code={p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in (ROOT/'simulink/paper/v3').glob('*e3*') if p.is_file()}
    code.update({p.relative_to(ROOT).as_posix():hashlib.sha256(p.read_bytes()).hexdigest() for p in (ROOT/'simulink/paper/v3').glob('*e4_1deg*') if p.is_file()})
    audit=ROOT/'simulink/paper/v3/revision_controller_audit.m'
    code[audit.relative_to(ROOT).as_posix()]=hashlib.sha256(audit.read_bytes()).hexdigest()
    baseline=subprocess_sha()
    for path in raw:
        folder=path.parent
        cfg=json.loads((folder/'build_config.json').read_text())
        provenance={'case_id':folder.name,'baseline_git_sha':baseline,'controller_audit':'reports/E3_controller_audit.json','config_sha256':hashlib.sha256((folder/'case_config.json').read_bytes()).hexdigest(),'model_sha256':cfg['model_sha256'],'bank_sha256':cfg['bank_sha256'],'entrypoint_sha256':code,'raw_path':str(path)}
        (folder/'provenance.json').write_text(json.dumps(provenance,indent=2,ensure_ascii=False),encoding='utf-8')
    result={'passed':True,'E3_new_raw_simulations':7,'E3_reused':3,'E4_new_simulations':0,'E4_reused':27,'independent_pairs':results,'E4_core_metrics_match_sources':True,'entrypoint_sha256':code}
    (OUT/'reports/independent_revision_verification.json').write_text(json.dumps(result,indent=2,ensure_ascii=False),encoding='utf-8')
    print(json.dumps(result,ensure_ascii=False))

def subprocess_sha():
    import subprocess
    return subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip()

if __name__=='__main__':main()
