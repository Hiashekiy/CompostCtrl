#!/usr/bin/env python
"""Stage-0 archiver for the legacy CMG project.

Builds ``archive/`` from the live working tree and records a SHA-256 manifest so
the snapshot can be verified later without touching the historical results.

Policy (chosen by the project owner):

* source, configs, docs/reports and environment metadata are copied verbatim;
* generated artefacts (``results/``, ``figures/``) are *registered only*
  -- path, size, SHA-256 and the git commit that tracks them -- so the 50 MB of
  historical data is not duplicated inside the same repository;
* nothing is ever moved, deleted or rewritten: the live tree stays untouched.

Usage (from the repository root)::

    python tools/archive_legacy.py build
    python tools/archive_legacy.py verify

``--include-results`` switches to a full physical snapshot (copies ``results/``
and ``figures/`` into ``archive/legacy_results/``) for callers who want the
archive to be self-contained.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
import platform
import shutil
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
ARCHIVE = REPO / "archive"

# Directories copied verbatim into archive/legacy_source/.
SOURCE_TREES = ("nearspace", "legacy", "scripts", "tests", "configs")
SOURCE_FILES = (".gitignore", "README.md", "config.yaml", "requirements.txt")
REPORTS_TREE = "docs"
# Registered but not copied unless --include-results is given.
RESULT_TREES = ("results", "figures")
# Never archived: interpreter/build artefacts and ignored scratch space.
EXCLUDED_DIRS = {"__pycache__", ".git", "tools"}
# New-project material that lives in the live tree but is not a legacy artefact.
EXCLUDED_FILES = {
    "docs/CMG增量式构型路径搜索_实验指导实施文件.md": "新实验（阶段 2 起）的指导实施文件，属于新流程，不是旧项目产物",
}

MANIFEST = ARCHIVE / "ARCHIVE_MANIFEST.csv"

CATEGORY_NOTES = {
    "code_current": "当前有效代码（nearspace 实现与 scripts 运行入口），新实验只作参考，不直接依赖",
    "code_legacy": "已被取代的早期实现（反作用轮/俯仰通道/第一代 CMG），仅可追溯性保留",
    "tests": "旧项目单元测试（无 pytest 依赖，脚本式自检）",
    "config": "YAML 配置：飞行器/大气/控制器权重/实验矩阵",
    "report": "旧项目报告与实施方案（历史结论，不得冒充新实验结果）",
    "reference": "参考文献原文（PDF）",
    "doc": "仓库说明文档",
    "environment": "依赖与忽略规则，复现环境依据",
    "data": "历史结果原始时序/指标/元数据（只登记不复制）",
    "figure": "历史图件（只登记不复制，可由旧脚本重绘）",
    "scratch": "被 .gitignore 排除的临时草稿，不纳入归档",
    "other": "未分类文件",
}


def log(msg: str) -> None:
    print(msg, flush=True)


def sha256_of(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


def git(*args: str) -> str:
    out = subprocess.run(
        ["git", *args], cwd=REPO, capture_output=True, text=True, encoding="utf-8"
    )
    return out.stdout.strip() if out.returncode == 0 else ""


def classify(rel: str) -> str:
    top = rel.split("/", 1)[0]
    if top in ("nearspace", "scripts"):
        return "code_current"
    if top == "legacy":
        return "code_legacy"
    if top == "tests":
        return "tests"
    if top == "configs" or rel == "config.yaml":
        return "config"
    if top == "docs":
        return "reference" if rel.startswith("docs/references/") else "report"
    if rel in ("README.md",):
        return "doc"
    if rel in ("requirements.txt", ".gitignore"):
        return "environment"
    if top == "results":
        return "data"
    if top == "figures":
        return "figure"
    if top == "tmp":
        return "scratch"
    return "other"


def archive_target(rel: str, copy_results: bool) -> str | None:
    """Return the archive-relative destination, or None when only registered."""
    category = classify(rel)
    if category in ("code_current", "code_legacy", "tests", "config", "environment", "doc"):
        return f"legacy_source/{rel}"
    if category == "report":
        return f"legacy_reports/{rel[len('docs/'):]}"
    if category == "reference":
        return f"legacy_reports/{rel[len('docs/'):]}"
    if category in ("data", "figure"):
        return f"legacy_results/{rel}" if copy_results else None
    return None


def walk_worktree() -> list[str]:
    paths: list[str] = []
    for root, dirs, files in os.walk(REPO):
        dirs[:] = sorted(d for d in dirs if d not in EXCLUDED_DIRS)
        rel_root = Path(root).relative_to(REPO).as_posix()
        for name in sorted(files):
            rel = name if rel_root == "." else f"{rel_root}/{name}"
            if rel.startswith("archive/"):
                continue
            paths.append(rel)
    return sorted(paths)


def git_blobs() -> dict[str, str]:
    out = git("ls-tree", "-r", "-z", "HEAD")
    blobs: dict[str, str] = {}
    for entry in out.split("\0"):
        if not entry:
            continue
        meta, _, path = entry.partition("\t")
        parts = meta.split()
        if len(parts) == 3:
            blobs[path] = parts[2]
    return blobs


def build(copy_results: bool) -> int:
    if ARCHIVE.exists() and any(ARCHIVE.iterdir()):
        log(f"[abort] {ARCHIVE} 已存在且非空；归档禁止覆盖。先人工确认再删除。")
        return 2

    head = git("rev-parse", "HEAD")
    branch = git("rev-parse", "--abbrev-ref", "HEAD")
    status = git("status", "--short", "--branch")
    blobs = git_blobs()
    tracked = set(blobs)
    paths = walk_worktree()

    for sub in ("legacy_source", "legacy_reports", "legacy_environment"):
        (ARCHIVE / sub).mkdir(parents=True, exist_ok=True)
    for sub in ("csv", "figures", "logs", "metrics"):
        (ARCHIVE / "legacy_results" / sub).mkdir(parents=True, exist_ok=True)

    rows: list[dict[str, str]] = []
    copied = 0
    excluded: list[tuple[str, str]] = []
    for rel in paths:
        src = REPO / rel
        category = classify(rel)
        if rel in EXCLUDED_FILES:
            excluded.append((rel, EXCLUDED_FILES[rel]))
            continue
        if category == "scratch":
            excluded.append((rel, "被 .gitignore 排除的临时草稿"))
            continue
        dest_rel = archive_target(rel, copy_results)
        digest = sha256_of(src)
        archive_path = "-"
        if dest_rel is not None:
            dest = ARCHIVE / dest_rel
            dest.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy2(src, dest)
            if sha256_of(dest) != digest:
                log(f"[fail] 复制后哈希不一致：{rel}")
                return 3
            archive_path = dest_rel
            copied += 1
        rows.append(
            {
                "original_path": rel,
                "archive_path": archive_path,
                "category": category,
                "archived": "yes" if dest_rel is not None else "no",
                "size_bytes": str(src.stat().st_size),
                "sha256": digest,
                "tracked_in_git": "yes" if rel in tracked else "no",
                "git_blob": blobs.get(rel, "-"),
                "purpose": CATEGORY_NOTES[category],
            }
        )

    with MANIFEST.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)

    write_environment(head, branch, status)
    write_result_pointers(copy_results)
    write_inventory(rows, excluded, head, branch, status, copy_results)

    stats = {
        "generated_utc": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "repo": str(REPO),
        "head": head,
        "branch": branch,
        "total_files": len(rows),
        "copied_files": copied,
        "registered_only_files": len(rows) - copied,
        "total_bytes": sum(int(r["size_bytes"]) for r in rows),
        "copied_bytes": sum(int(r["size_bytes"]) for r in rows if r["archived"] == "yes"),
        "excluded_files": len(excluded),
    }
    (ARCHIVE / "ARCHIVE_STATS.json").write_text(
        json.dumps(stats, ensure_ascii=False, indent=2) + "\n", encoding="utf-8", newline="\n"
    )
    log(json.dumps(stats, ensure_ascii=False, indent=2))
    return 0


def write_environment(head: str, branch: str, status: str) -> None:
    env = ARCHIVE / "legacy_environment"
    freeze = subprocess.run(
        [sys.executable, "-m", "pip", "freeze"], capture_output=True, text=True
    ).stdout
    (env / "pip-freeze.txt").write_text(freeze, encoding="utf-8", newline="\n")
    shutil.copy2(REPO / "requirements.txt", env / "requirements.txt")
    info = [
        f"git root: {REPO}",
        f"branch: {branch}",
        f"HEAD: {head}",
        f"HEAD date: {git('log', '-1', '--format=%ad', '--date=iso')}",
        f"HEAD subject: {git('log', '-1', '--format=%s')}",
        f"captured_utc: {datetime.now(timezone.utc).isoformat(timespec='seconds')}",
        f"python: {sys.version.split()[0]} ({platform.python_implementation()})",
        f"platform: {platform.platform()}",
        f"machine: {platform.machine()}",
        "note: 旧结果采集时的解释器版本未记录，仅此快照日期可考。",
        "",
        "git status --short --branch:",
        status,
    ]
    (env / "git-and-runtime.txt").write_text("\n".join(info) + "\n", encoding="utf-8", newline="\n")


def write_result_pointers(copy_results: bool) -> None:
    body = (
        "# legacy_results\n\n"
        + (
            "本目录是 `results/` 与 `figures/` 的**完整物理副本**（`--include-results` 模式）。\n"
            "它的哈希记录在 `archive/ARCHIVE_MANIFEST.csv` 中。\n"
            if copy_results
            else (
                "本目录**不存放数据副本**。按归档策略，`results/`（原始时序/指标/元数据）与\n"
                "`figures/`（图件）只登记在 `../ARCHIVE_MANIFEST.csv` 中，逐文件记录\n"
                "原始路径、字节数与 SHA-256，并注明其所在 git 提交，避免同一仓库内\n"
                "重复存储约 50 MB 历史数据。\n\n"
                "取回方式：\n\n"
                "```bash\n"
                "git checkout 6fcc0f9 -- results figures      # 按提交号恢复\n"
                "python tools/archive_legacy.py verify        # 校核当前工作区哈希\n"
                "```\n\n"
                "子目录 `csv/ figures/ logs/ metrics/` 保留为空骨架，用于将来确实需要\n"
                "物理副本时（`python tools/archive_legacy.py build --include-results`）。\n"
            )
        )
    )
    (ARCHIVE / "legacy_results" / "README.md").write_text(body, encoding="utf-8", newline="\n")


def write_inventory(
    rows: list[dict[str, str]],
    excluded: list[tuple[str, str]],
    head: str,
    branch: str,
    status: str,
    copy_results: bool,
) -> None:
    by_cat: dict[str, list[dict[str, str]]] = {}
    for row in rows:
        by_cat.setdefault(row["category"], []).append(row)
    untracked = [r for r in rows if r["tracked_in_git"] == "no"]
    total_bytes = sum(int(r["size_bytes"]) for r in rows)

    lines: list[str] = []
    a = lines.append
    a("# 归档清单（ARCHIVE INVENTORY）")
    a("")
    a("> 本文件与 `ARCHIVE_MANIFEST.csv`、`ARCHIVE_STATS.json` 均由")
    a("> `python tools/archive_legacy.py build` 自动生成，请勿手工编辑；")
    a("> 手工说明写在 `archive/README.md`。")
    a("")
    a("## 1. 盘点对象")
    a("")
    a(f"- 仓库根目录：`{REPO}`")
    a(f"- 分支：`{branch}`")
    a(f"- HEAD 提交：`{head}`")
    a(f"- 盘点时间（UTC）：{datetime.now(timezone.utc).isoformat(timespec='seconds')}")
    a(f"- 纳入清单文件数：{len(rows)}（{total_bytes / 1e6:.2f} MB）")
    a("- `git status --short --branch`：")
    a("")
    a("```")
    a(status or "(empty)")
    a("```")
    a("")
    a("## 2. 归档策略")
    a("")
    a("混合式：源代码、配置、测试、文档/报告、环境元数据**全量复制**到 `archive/`；")
    a("生成的 `results/` 与 `figures/` **只登记不复制**，逐文件记录 SHA-256 与所在提交。")
    if copy_results:
        a("本次构建使用 `--include-results`，`results/` 与 `figures/` 已复制为物理快照。")
    a("历史结果与代码在原位保持不变，归档过程不移动、不删除、不重写任何旧文件。")
    a("")
    a("## 3. 按类别统计")
    a("")
    a("| 类别 | 文件数 | 字节数 | 是否复制 | 说明 |")
    a("|---|---:|---:|---|---|")
    for cat in sorted(by_cat):
        files = by_cat[cat]
        size = sum(int(r["size_bytes"]) for r in files)
        yes = all(r["archived"] == "yes" for r in files)
        a(f"| `{cat}` | {len(files)} | {size:,} | {'是' if yes else '登记'} | {CATEGORY_NOTES[cat]} |")
    a("")
    a("## 4. 目录布局")
    a("")
    a("```text")
    a("archive/")
    a("  legacy_source/      nearspace/ legacy/ scripts/ tests/ configs/ config.yaml README.md requirements.txt .gitignore")
    a("  legacy_reports/     旧项目报告与参考资料（docs/*.md、docs/references/AST-V1.pdf）")
    a("  legacy_results/     csv/ figures/ logs/ metrics/ 骨架 + README 指针（见策略）")
    a("  legacy_environment/ pip-freeze.txt requirements.txt git-and-runtime.txt")
    a("  ARCHIVE_INVENTORY.md  本文件")
    a("  ARCHIVE_MANIFEST.csv  逐文件清单：原路径/归档路径/字节数/SHA-256/来源提交")
    a("  ARCHIVE_STATS.json    构建统计")
    a("  README.md             人工说明与取回方式")
    a("```")
    a("")
    a("## 5. 运行入口与测试入口（旧项目）")
    a("")
    a("运行入口（`scripts/`，见原 `README.md`）：")
    a("")
    for row in by_cat.get("code_current", []):
        if row["original_path"].startswith("scripts/"):
            a(f"- `{row['original_path']}`")
    a("")
    a("旧实现入口（`legacy/`，已不在实验链路内）：")
    a("")
    for row in by_cat.get("code_legacy", []):
        if Path(row["original_path"]).name.startswith("run_"):
            a(f"- `{row['original_path']}`")
    a("")
    a("测试入口（脚本式自检，无 pytest 依赖，在工作区根目录运行）：")
    a("")
    for row in by_cat.get("tests", []):
        a(f"- `python {row['original_path']}`")
    a("")
    a("## 6. 已知问题与不可复现项（转述 `legacy/README.md`，未擅自修复）")
    a("")
    a("- 早期 RW / CMG 图件生成脚本（`make_figures.py` 等 5 个）在旧归档移动时按 ANSI 编码")
    a("  读写导致中文字符串损坏，已删除；其 PNG 仍在 `results/legacy_rw/`、`results/cmg_ab/baseline_legacy/`，")
    a("  重绘须按 `scripts/plot_cmg_ab.py` 风格重写。")
    a("- `results/cmp_A.csv`（145 s、仅舵面、100 km 顶点脉冲）在整理前被一次试运行覆盖并删除；")
    a("  同类工况由 `results/cmg_ab/disturbance/A/` 复现。")
    a("- `legacy/README.md` 第 1 行含字面转义符（`n`r`n`），是旧归档过程中的编码损伤；")
    a("  为保证哈希可核验，本归档保留原样未改写。")
    a("- 旧结果采集时的解释器版本未记录；当前环境见 `legacy_environment/`。")
    a("")
    a("## 7. 未纳入归档的内容")
    a("")
    scratch = [rel for rel, _ in excluded if rel.startswith("tmp/")]
    others = [(rel, why) for rel, why in excluded if not rel.startswith("tmp/")]
    a(f"- 被 `.gitignore` 排除的临时草稿 `tmp/`：{len(scratch)} 个文件")
    for rel in scratch:
        a(f"  - `{rel}`")
    for rel, why in others:
        a(f"- `{rel}`：{why}")
    a("- `__pycache__/`、`*.pyc`：解释器缓存，构建产物，不具归档价值。")
    a("- `tools/archive_legacy.py`：本归档工具本身，属于新流程而非旧项目。")
    a("")
    a("## 8. 历史结果溯源")
    a("")
    a(f"- `results/` 与 `figures/` 由提交 `{head}` 跟踪；清单中每行 `git_blob` 是该文件在")
    a("  该提交中的对象哈希，可与 `git cat-file -p` 交叉核对，用于确认工作区内容与提交一致。")
    a("- 归档不改变任何历史结果文件，`verify` 会重新计算清单中每个文件的 SHA-256 并逐一比对。")
    a("")
    a("## 9. 校验方式")
    a("")
    a("```bash")
    a("python tools/archive_legacy.py build     # 生成清单与副本（已完成）")
    a("python tools/archive_legacy.py verify    # 重新计算哈希并与清单比对")
    a("# 任意单文件：")
    a("#   PowerShell: (Get-FileHash -Algorithm SHA256 <path>).Hash.ToLower()")
    a("```")
    a("")
    a("## 10. 未提交改动保护")
    a("")
    if untracked:
        a("构建时未跟踪但已纳入清单的文件：")
        a("")
        for row in untracked:
            a(f"- `{row['original_path']}`（{row['size_bytes']} B）")
    else:
        a("构建时无未跟踪文件。")
    a("")
    (ARCHIVE / "ARCHIVE_INVENTORY.md").write_text("\n".join(lines), encoding="utf-8", newline="\n")


def verify() -> int:
    if not MANIFEST.exists():
        log(f"[abort] 缺少 {MANIFEST}")
        return 2
    mismatches = 0
    checked = 0
    with MANIFEST.open(encoding="utf-8", newline="") as fh:
        for row in csv.DictReader(fh):
            checked += 1
            src = REPO / row["original_path"]
            if not src.exists():
                log(f"[missing] {row['original_path']}")
                mismatches += 1
                continue
            if str(src.stat().st_size) != row["size_bytes"]:
                log(f"[size] {row['original_path']}")
                mismatches += 1
            if sha256_of(src) != row["sha256"]:
                log(f"[sha256] {row['original_path']}")
                mismatches += 1
            if row["archived"] == "yes":
                dst = ARCHIVE / row["archive_path"]
                if not dst.exists():
                    log(f"[missing-copy] {row['archive_path']}")
                    mismatches += 1
                elif sha256_of(dst) != row["sha256"]:
                    log(f"[copy-sha256] {row['archive_path']}")
                    mismatches += 1
    log(f"checked={checked} mismatches={mismatches}")
    return 0 if mismatches == 0 else 1


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("build", "verify"))
    parser.add_argument(
        "--include-results",
        action="store_true",
        help="同时把 results/ 与 figures/ 复制进 archive/legacy_results/（完整物理快照）",
    )
    args = parser.parse_args()
    return build(args.include_results) if args.command == "build" else verify()


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    raise SystemExit(main())
