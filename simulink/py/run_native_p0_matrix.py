"""Run the eight existing P0 maneuvers with native Aerospace plants and MATLAB MPC.

Each saved SLX embeds its controller template and future reference schedule.
"""
from pathlib import Path
import subprocess,os,sys,json,zipfile
from scipy.io import loadmat
ROOT=Path(__file__).resolve().parents[2]
sys.stdout.reconfigure(encoding='utf-8',errors='replace')
sys.stderr.reconfigure(encoding='utf-8',errors='replace')
matrix=ROOT/'configs/torque_gap_no_cmg_comparison'
cases=sorted((matrix/'cases').glob('*_with_cmg.yaml'))+[matrix/'sequential/S07_sequential_with_cmg.yaml']
env=os.environ.copy();env['MATLAB_HOME']=r'D:\Program Files\MATLAB\R2024a'
env['PYTHONIOENCODING']='utf-8'
progress=ROOT/'simulink/results/native_closedloop/p0_matrix_progress.json'
results=json.loads(progress.read_text(encoding='utf-8')) if progress.exists() else []
for path in cases:
    name=path.stem.removesuffix('_with_cmg')
    for enabled in [True,False]:
        arm='B' if enabled else 'A'
        resolved=ROOT/'results/torque_gap_no_cmg_comparison_rti2/P0'/f"{name}_{'with_cmg' if enabled else 'no_cmg'}"/'config_resolved.yaml'
        source=str(resolved.relative_to(ROOT)) if resolved.exists() else ''
        already=any(r['case']==name and r['arm']==arm for r in results)
        if enabled and source:
            already=already and any(r['case']==name and r['arm']==arm and r.get('config_source')==source for r in results)
        # Early development runs used the P1 drive lag. Do not count those as P0.
        model=ROOT/'simulink'/f"NearSpace_CMG_AttitudeValidation{'_SurfaceOnly' if not enabled else ''}_{name}.slx"
        if already and enabled and model.exists():
            with zipfile.ZipFile(model) as z:
                already=not any(b'Inverse_time_constant' in z.read(n) for n in z.namelist() if n.endswith('.xml'))
        if already:
            print('SKIP completed',name,arm,flush=True)
            continue
        case=path if enabled else path.with_name(path.name.replace('_with_cmg','_no_cmg'))
        if resolved.exists():case=resolved
        args=[sys.executable,str(ROOT/'simulink/py/export_hpipm_qp.py'),'--config',str(case)]
        if not enabled:args+=['--surface-only']
        print('EXPORT',name,'B' if enabled else 'A',flush=True)
        subprocess.run(args,cwd=ROOT,env=env,check=True)
        subprocess.run([sys.executable,str(ROOT/'simulink/py/export_online_plan_bank.py'),'--config',str(case)],cwd=ROOT,env=env,check=True)
        bank=loadmat(ROOT/'simulink/data/online_plan_bank.mat');duration=float(bank['time'].ravel()[-1])
        expression=f"maxNumCompThreads(1); addpath('simulink'); build_native_closedloop({duration},false,{'true' if enabled else 'false'},'{name}')"
        print('RUN',name,'B' if enabled else 'A','duration',duration,flush=True)
        completed=subprocess.run([str(ROOT/'simulink/mlab.cmd'),expression],cwd=ROOT,env=env,check=True,
            stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,encoding='utf-8',errors='replace')
        log=ROOT/'simulink/results/native_closedloop'/f'{name}_{arm}_console.log'
        log.write_text(completed.stdout,encoding='utf-8')
        if 'NATIVE_CLOSEDLOOP_PASS' not in completed.stdout:raise RuntimeError('MATLAB did not report completed simulation')
        print('PASS',name,arm,flush=True)
        results=[r for r in results if not (r['case']==name and r['arm']==arm)]
        results.append(dict(case=name,arm='B' if enabled else 'A',duration_s=duration,status='SIMULATION_FINITE_PASS',plant_mode='P0_IDEAL',config_source=source))
        (ROOT/'simulink/results/native_closedloop/p0_matrix_progress.json').write_text(json.dumps(results,indent=2))
print('NATIVE_EIGHT_CASE_P0_MATRIX_FINISHED',flush=True)
