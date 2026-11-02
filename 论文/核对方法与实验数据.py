"""只读重算正文表2、表3，并检查原始时序与指标文件一致性。"""
import csv
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BASE = ROOT / 'results/torque_gap_no_cmg_comparison_rti2'
SCENARIOS = ['S00_pitch30', 'S01_pitch_minus30', 'S02_pitch10', 'S03_pitch45',
             'S04_roll30', 'S05_yaw30', 'S06_pitch30_roll20', 'S07_sequential']
records = []
for phase, scenarios in [('P0', SCENARIOS), ('P1', [SCENARIOS[i] for i in [0, 1, 6]])]:
    for scenario in scenarios:
        for arm in ['no_cmg', 'with_cmg']:
            folder = BASE / phase / f'{scenario}_{arm}'
            metrics = json.loads((folder / 'metrics.json').read_text(encoding='utf-8'))
            with (folder / 'timeseries.csv').open(encoding='utf-8', newline='') as f:
                rows = list(csv.DictReader(f))
            window = metrics['windows']['combined']
            errors = [float(r['att_err_deg']) for r in rows
                      if window['t0'] <= float(r['t_next']) < window['t1']]
            rmse = math.sqrt(sum(e*e for e in errors) / len(errors))
            peak = max(errors)
            expected = metrics['stats']['combined']
            assert abs(rmse - expected['rmse_deg']) < 1e-8, folder
            assert abs(peak - expected['max_err_deg']) < 1e-8, folder
            records.append(dict(phase=phase, scenario=scenario, arm=arm,
                                rmse=rmse, peak=peak, samples=len(errors)))
print(json.dumps({'verified_runs': len(records), 'records': records}, ensure_ascii=False, indent=2))
