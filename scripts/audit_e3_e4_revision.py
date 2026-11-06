"""Protect only the revision's reused cases, old reports, and physical inputs."""
import hashlib
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OLD = ROOT / 'simulink/results/paper_experiments/20261105_redesign_v1'
OUT = ROOT / 'simulink/results/paper_experiments/20261106_e3_e4_revision'
SNAPSHOT = OUT / 'config/read_only_snapshot.json'

def digest(path):
    h = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()

def targets():
    paths = set()
    ids = [('E3', n) for n in ('V3_S', 'V3_L', 'V3_Z')]
    ids += [('E4', f'V4_010_{f:03d}_{m}') for f in (20,40,60,80,100,120,150,180,200) for m in (1,2,3)]
    for group, name in ids:
        paths.update(p for p in (OLD/group/'cases'/name).iterdir() if p.is_file())
    for sub in ('reports', 'tables', 'config', 'calibration'):
        paths.update(p for p in (OLD/sub).rglob('*') if p.is_file())
    tracked = subprocess.check_output(['git','ls-files','-z','simulink/models','simulink/data','simulink/hpipm','simulink/modelgen','simulink/tests/05_python_matlab_qp_equivalence'],cwd=ROOT).decode().split('\0')
    paths.update(ROOT/p for p in tracked if p)
    return sorted(paths)

def main():
    if sys.argv[1] == 'snapshot':
        assert not SNAPSHOT.exists(), 'Do not replace the protection baseline'
        data = {p.relative_to(ROOT).as_posix(): digest(p) for p in targets()}
        SNAPSHOT.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding='utf-8')
        print(f'Protected {len(data)} task-related files')
    else:
        data = json.loads(SNAPSHOT.read_text(encoding='utf-8'))
        changed = [p for p, sha in data.items() if not (ROOT/p).is_file() or digest(ROOT/p) != sha]
        result = {'passed': not changed, 'checked_files': len(data), 'changed_paths': changed, 'scope': '30 reused cases including all MAT, old report/table/config/calibration outputs, tracked physical/controller inputs'}
        (OUT/'reports/read_only_verification.json').write_text(json.dumps(result,ensure_ascii=False,indent=2),encoding='utf-8')
        print(json.dumps(result,ensure_ascii=False))
        assert not changed

if __name__ == '__main__':
    main()
