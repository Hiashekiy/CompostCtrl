"""Remove only verified historical output duplicates retained in paper_final.

No recursive deletion, simulation, controller changes, or historical MAT selection.
"""
import csv
import hashlib
import json
from pathlib import Path
import shutil
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[3]
FINAL = ROOT / 'simulink/results/paper_final'
OUT = ROOT / 'simulink/results/paper_archive/20261106_duplicate_cleanup'
PREFIX = ROOT / 'simulink/results/paper_experiments'
ALLOWED_BATCHES = {'20261105_redesign_v1', '20261106_e3_e4_revision', '20261106_e2_unload_revision'}


def sha(p):
    h = hashlib.sha256()
    with p.open('rb') as f:
        for block in iter(lambda: f.read(8 * 1024 * 1024), b''):
            h.update(block)
    return h.hexdigest()


def rows(p):
    with p.open(encoding='utf-8-sig', newline='') as f:
        return list(csv.DictReader(f))


def dump(p, value):
    p.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')


def protected():
    data = json.loads((PREFIX / '20261105_redesign_v1/config/protected_old_raw.json').read_text(encoding='utf-8'))
    return {Path(r['path']).resolve() for r in data}


def resolve_record(r):
    source = (ROOT / r['old_path']).resolve()
    kept = (ROOT / r['retained_path']).resolve()
    relative = source.relative_to(PREFIX.resolve())
    assert relative.parts[0] in ALLOWED_BATCHES
    assert len(relative.parts) >= 5 and relative.parts[1] in ('E1', 'E2', 'E3', 'E4') and relative.parts[2] == 'cases'
    assert source.suffix == '.mat' and source.name != 'reference_bank.mat'
    assert source not in protected()
    kept_relative = kept.relative_to(FINAL.resolve())
    assert kept_relative.parts[1] == 'raw' and source.name == kept.name
    assert not source.is_symlink() and not kept.is_symlink()
    return source, kept


def validate(records, require_source=True):
    tracked = set(subprocess.check_output(['git', 'ls-files', '-z'], cwd=ROOT).decode('utf-8').split('\0'))
    for i, r in enumerate(records, 1):
        source, kept = resolve_record(r)
        assert r['old_path'] not in tracked
        assert kept.is_file() and kept.stat().st_size == int(r['bytes']) and sha(kept) == r['sha256']
        if require_source:
            assert source.is_file() and source.stat().st_size == int(r['bytes']) and sha(source) == r['sha256']
        if i % 100 == 0:
            print('DUPLICATE_HASH_CHECK', i, '/', len(records), flush=True)


def plan():
    assert not OUT.exists(), 'Existing cleanup evidence must not be overwritten'
    retained = {r['path']: r['sha256'] for r in rows(FINAL / 'reports/file_sha256.csv')}
    records = []
    for r in rows(FINAL / 'reports/payload_sha256.csv'):
        if r['role'] != 'native_or_diagnostic_MAT' or not r['source'].startswith('simulink/results/paper_experiments/'):
            continue
        assert retained[r['path']] == r['sha256']
        record = {'old_path': r['source'], 'retained_path': 'simulink/results/paper_final/' + r['path'],
                  'bytes': r['bytes'], 'sha256': r['sha256'], 'case_id': Path(r['path']).parent.name,
                  'reason': 'byte_identical_historical_output_duplicate',
                  'recovery': 'copy retained_path back to old_path using restore mode'}
        resolve_record(record)
        records.append(record)
    assert records and len({r['old_path'] for r in records}) == len(records)
    validate(records)
    OUT.mkdir(parents=True)
    with (OUT / 'cleanup_manifest.csv').open('w', encoding='utf-8', newline='') as f:
        writer = csv.DictWriter(f, fieldnames=list(records[0]))
        writer.writeheader()
        writer.writerows(records)
    dump(OUT / 'plan_receipt.json', {'passed': True, 'files': len(records), 'bytes': sum(int(r['bytes']) for r in records),
         'source_and_retained_SHA256_match': True, 'protected_old_raw_excluded': len(protected()),
         'tracked_files_excluded': True, 'retained_archive': 'simulink/results/paper_final',
         'new_simulations': 0, 'free_bytes_before': shutil.disk_usage(ROOT).free})
    (OUT / 'README.md').write_text('''# 历史重复原始输出清理

仅清理cleanup_manifest.csv逐项列出的历史原始输出副本；每一份均已完整复制到paper_final，且旧文件、保留文件与已提交的SHA清单三方一致。paper_final是保留归档，不再额外复制一份相同大文件。99份原始保护结果、参考bank、场景配置、模型、源码、数据依赖、未筛选的历史MAT及Git证据不在删除范围内。

历史revision的read_only_snapshot/protected_snapshot是当时验收的原始证据，保持原文件不变。部分旧路径在本次获准清理后迁移至paper_final；旧快照不再表示这些工作区副本仍然存在。以本迁移清单解析保留位置，数据内容与原SHA未变。

先提交清单和验证凭证，再逐文件删除。删除后验证paper_final全部197例原始数据和3303份不可变文件。执行结果见deletion_receipt.json；论文集原有验收凭证保持不变。

需要恢复旧路径时，从仓库根目录运行：

```powershell
python simulink/paper/final/cleanup_history_duplicates.py restore
```

恢复逐文件核对SHA；已有同名文件不覆盖。恢复会重新占用约6.75GB。本次没有删除未在论文集保留的历史负面结果。
''', encoding='utf-8')
    print('DUPLICATE_PLAN_COMPLETE', len(records), sum(int(r['bytes']) for r in records), flush=True)


def delete(commit):
    assert subprocess.check_output(['git', 'rev-parse', 'HEAD'], cwd=ROOT, text=True).strip() == commit
    manifest_rel = (OUT / 'cleanup_manifest.csv').relative_to(ROOT).as_posix()
    committed = subprocess.check_output(['git', 'show', commit + ':' + manifest_rel], cwd=ROOT)
    assert hashlib.sha256(committed).hexdigest() == sha(OUT / 'cleanup_manifest.csv')
    records = rows(OUT / 'cleanup_manifest.csv')
    # Confirm scope and file presence first; hash both copies immediately before each unlink.
    tracked = set(subprocess.check_output(['git', 'ls-files', '-z'], cwd=ROOT).decode('utf-8').split('\0'))
    for r in records:
        source, kept = resolve_record(r)
        assert r['old_path'] not in tracked and source.is_file() and kept.is_file()
        assert source.stat().st_size == kept.stat().st_size == int(r['bytes'])
    deleted = []
    for i, r in enumerate(records, 1):
        source, kept = resolve_record(r)
        assert sha(source) == r['sha256'] and sha(kept) == r['sha256']
        source.unlink()
        deleted.append(r['old_path'])
        if i % 100 == 0:
            print('DUPLICATES_REMOVED', i, '/', len(records), flush=True)
    # Pure postprocessing integrity check, no MATLAB or new simulations.
    result = subprocess.run([sys.executable, str(ROOT / 'simulink/paper/final/finalize_paper_final.py'), 'verify'], cwd=ROOT, capture_output=True, text=True)
    (OUT / 'post_cleanup_verification.txt').write_text(result.stdout + result.stderr, encoding='utf-8')
    assert result.returncode == 0, result.stdout + result.stderr
    assert all(not (ROOT / r['old_path']).exists() for r in records)
    dump(OUT / 'deletion_receipt.json', {'passed': True, 'pre_cleanup_commit': commit, 'deleted_files': len(deleted),
         'deleted_bytes': sum(int(r['bytes']) for r in records), 'duplicate_paths_before': len(records),
         'duplicate_paths_after': 0, 'retained_files': len(records), 'paper_final_integrity_passed': True,
         'protected_old_raw_still_present': all(p.is_file() for p in protected()),
         'unselected_historical_MAT_deleted': 0, 'new_simulations': 0, 'free_bytes_after': shutil.disk_usage(ROOT).free})
    print('DUPLICATE_CLEANUP_COMPLETE', len(deleted), flush=True)


def restore():
    for r in rows(OUT / 'cleanup_manifest.csv'):
        source, kept = resolve_record(r)
        assert kept.is_file() and sha(kept) == r['sha256']
        if source.exists():
            assert sha(source) == r['sha256'], 'Existing file differs; refuse overwrite'
            continue
        source.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(kept, source)
        assert sha(source) == r['sha256']
    print('HISTORICAL_PATHS_RESTORED')


if __name__ == '__main__':
    if sys.argv[1] == 'plan':
        plan()
    elif sys.argv[1] == 'delete':
        delete(sys.argv[2])
    elif sys.argv[1] == 'restore':
        restore()
    else:
        raise SystemExit('plan | delete PRE_CLEANUP_COMMIT | restore')
