"""Hash only protected experiment batches and their physical/controller inputs."""
import hashlib,json,subprocess,sys
from pathlib import Path
ROOT=Path(__file__).resolve().parents[4]
OUT=ROOT/'simulink/results/paper_experiments/20261106_e2_unload_revision'
SNAP=OUT/'config/protected_snapshot.json'
def digest(p):
    h=hashlib.sha256()
    with p.open('rb') as f:
        for b in iter(lambda:f.read(8*1024*1024),b''):h.update(b)
    return h.hexdigest()
def main():
    if sys.argv[1]=='snapshot':
        assert not SNAP.exists()
        paths=set()
        for name in ('20261105_redesign_v1','20261106_e3_e4_revision'):
            folder=ROOT/'simulink/results/paper_experiments'/name
            paths.update(p for p in folder.rglob('*') if p.is_file() and 'cache' not in p.relative_to(folder).parts and p.name!='.run.lock')
        tracked=subprocess.check_output(['git','ls-files','-z','simulink/models','simulink/data','simulink/hpipm','simulink/modelgen','simulink/paper/v3','simulink/tests/05_python_matlab_qp_equivalence'],cwd=ROOT).decode().split('\0')
        paths.update(ROOT/p for p in tracked if p and '/e2_revision/' not in p)
        # These eight pre-existing dirty files belong to a different experiment.
        for arm in ('A','B'):
            folder=ROOT/f'simulink/results/online_authority/fast_certificate/ablation/analytic_full_{arm}_60s'
            paths.update(p for p in folder.iterdir() if p.is_file() and p.suffix in ('.mat','.json'))
        data={p.relative_to(ROOT).as_posix():digest(p) for p in sorted(paths)}
        SNAP.write_text(json.dumps(data,indent=2,ensure_ascii=False),encoding='utf-8')
        print('Protected task-scoped files:',len(data))
    else:
        data=json.loads(SNAP.read_text(encoding='utf-8'))
        changed=[p for p,h in data.items() if not (ROOT/p).is_file() or digest(ROOT/p)!=h]
        result={'passed':not changed,'files_checked':len(data),'changed_paths':changed,'scope':'Entire original338-run batch, E3/E4 revision, formal physical/controller inputs, pre-existing dirty ablation results'}
        (OUT/'reports/protected_verification.json').write_text(json.dumps(result,indent=2,ensure_ascii=False),encoding='utf-8');print(json.dumps(result));assert not changed
if __name__=='__main__':main()
