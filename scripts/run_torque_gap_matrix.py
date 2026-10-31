#!/usr/bin/env python
"""Run a matrix of torque-gap experiments sequentially.

Each entry is (case, mode, plant, overrides, tag).  Runs whose output directory
already contains metrics.json are skipped unless --force is given, so the matrix
can be resumed.  Identical references / initial state / authority are guaranteed
because every entry is produced by run_torque_gap.py from the same case file with
only the listed overrides.

python scripts/run_torque_gap_matrix.py --group core
python scripts/run_torque_gap_matrix.py --group intensity --force
python scripts/run_torque_gap_matrix.py --group configurations
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

# ---- matrices -------------------------------------------------------------
# Entry format: (case, plant, overrides, tag).  The B0/B1/B2 modulation modes are
# gone: the J_cfg cost was removed from the optimiser (see
# docs/torque_gap/modulation_removed_report.md), so there is only one controller.
VERIFY = [("C1", "P0", [], "")]
CORE = [(c, "P0", [], "") for c in ("C0", "C1", "C2", "C3", "C4", "C5")]
P1 = [(c, "P1", [], "") for c in ("C1", "C2")]
# gap intensity scan (the 30 deg arm is C1 itself)
INTENSITY = [("C1", "P0", ["trajectory3.maneuver.delta_deg=%s" % amp],
              "amp%s" % amp.replace(".", "p")) for amp in ("10", "20")]
# alternative initial gimbal configurations
CONFIGS = [("C1", "P0", ["torque_gap.init.gamma0_deg=[60.0, -60.0, 60.0, -60.0]"], "g60"),
           ("C1", "P0", ["torque_gap.init.gamma0_deg=[0.0, 60.0, 0.0, 60.0]"], "g0p60")]
# unlimited-gimbal arm: same C cases, P0
UNLIMITED = [(c, "P0", [], "") for c in ("C0", "C1", "C2", "C3", "C4", "C5")]
GROUPS = dict(verify=VERIFY, core=CORE, p1=P1, intensity=INTENSITY,
              configurations=CONFIGS, unlimited=UNLIMITED,
              all=CORE + P1 + INTENSITY + CONFIGS)


def run_dir(case, plant, tag, results_root):
    name = "%s_%s%s" % (case.upper(), plant.upper(), ("_" + tag) if tag else "")
    root = Path(results_root)
    root = root if root.is_absolute() else ROOT / root
    return root / name


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--group", default="core", choices=sorted(GROUPS))
    ap.add_argument("--force", action="store_true")
    ap.add_argument("--only", default=None, help="comma separated case names")
    ap.add_argument("--config-root", default="configs/torque_gap/cases")
    ap.add_argument("--results-root", default="results/torque_gap")
    ap.add_argument("--log-tag", default="", help="suffix for matrix_log_<group>_<tag>.json")
    args = ap.parse_args()
    entries = GROUPS[args.group]
    if args.only:
        keep = {s.strip().upper() for s in args.only.split(",")}
        entries = [e for e in entries if e[0].upper() in keep]
    t_all = time.perf_counter()
    done = []
    for case, plant, overrides, tag in entries:
        out = run_dir(case, plant, tag, args.results_root)
        if (out / "metrics.json").exists() and not args.force:
            print("[skip] %s" % out.name)
            continue
        cmd = [sys.executable, str(ROOT / "scripts" / "run_torque_gap.py"),
               "--case", case, "--plant", plant, "--tag", tag,
               "--config-root", args.config_root,
               "--results-root", args.results_root]
        for o in overrides:
            cmd += ["--set", o]
        t0 = time.perf_counter()
        print("[run] %s" % " ".join(cmd[2:]))
        proc = subprocess.run(cmd, cwd=str(ROOT), capture_output=True, text=True,
                              errors="replace")
        dt = time.perf_counter() - t0
        ok = (out / "metrics.json").exists()
        done.append(dict(name=out.name, ok=ok, wall_s=dt))
        print("      -> %s in %.1fs" % ("ok" if ok else "FAILED", dt))
        if not ok:
            print(proc.stdout[-2000:])
            print(proc.stderr[-3000:])
    log_root = Path(args.results_root)
    log_root = log_root if log_root.is_absolute() else ROOT / log_root
    log_root.mkdir(parents=True, exist_ok=True)
    log_name = "matrix_log_%s%s.json" % (args.group,
                                         ("_" + args.log_tag) if args.log_tag else "")
    with open(log_root / log_name, "w",
              encoding="utf-8") as fh:
        json.dump(dict(group=args.group, entries=done,
                       total_wall_s=time.perf_counter() - t_all), fh, indent=2)
    print("[done] group=%s entries=%d wall=%.1fs"
          % (args.group, len(done), time.perf_counter() - t_all))
    return 0


if __name__ == "__main__":
    sys.exit(main())
