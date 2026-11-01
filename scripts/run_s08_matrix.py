#!/usr/bin/env python
"""Run matrix of the S08 degradation study (A = surfaces only, B = surfaces + CMG).

Phases
    smoke   A1_alpha_p30 and A4_descent_replay_S00, both arms, 4 s window
            (98-102 s) -- an infrastructure check, not a result
    A1      angle-of-attack sweep, 6 cases x 2 arms, P0
    A2      dynamic-pressure sweep, 4 cases x 2 arms, P0
    A3      2x2 coupled combinations, 4 cases x 2 arms, P0
    A4      original S00/S03 descent replay, 2 cases x 2 arms, P0
    P0      A1 + A2 + A3 + A4
    P1      the same cases on the 18-state truth plant (real servo, bounded
            deflection allocation): A1, A2, A4
    seed    A4 with seeds 1-4 (disturbance phase randomised, identical for both
            arms of a pair)
    all     P0 + P1 + seed

Every run reuses exactly one generated case file
(configs/torque_gap_s08/cases/<case>_<arm>.yaml); nothing is overridden except
the plant, the seed and (smoke) the window.

Outputs
    results/torque_gap_s08/<group>/<run_id>/
        timeseries.csv  metrics.json  meta.json  config_resolved.yaml  run.log

Examples
    python scripts/run_s08_matrix.py --phase smoke --jobs 2
    python scripts/run_s08_matrix.py --phase P0 --jobs 5
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from nearspace.config import deep_merge                     # noqa: E402

CFG = ROOT / "configs" / "torque_gap_s08"
OUT = ROOT / "results" / "torque_gap_s08"
PY = sys.executable
ARMS = ("no_cmg", "with_cmg")
GROUPS = ("A1_alpha", "A2_qbar", "A3_coupled", "A4_descent_replay")
P1_GROUPS = ("A1_alpha", "A2_qbar", "A4_descent_replay")
SEED_CASES = ("A4_descent_replay_S00", "A4_descent_replay_S03")


def design_cases():
    d = json.loads((OUT / "design.json").read_text(encoding="utf-8"))
    return d["cases"]


def cases_by_group(groups):
    return [c["name"] for c in design_cases() if c["group"] in groups]


def specs(phase, root=None, seeds=(0,)):
    out = []
    root = Path(root) if root else OUT
    if not root.is_absolute():
        root = ROOT / root

    def add(case, group, arm, plant, seed=0, randomize=False, window=None):
        rid = "%s_%s" % (case, arm)
        sub = group
        if plant == "P1":
            rid += "_P1"
        if randomize:
            rid += "_seed%d" % seed
        ov = {}
        if window:
            ov = {"torque_gap": {"start": window[0], "end": window[1]}}
        out.append(dict(run_id=rid, phase=sub, case=case, group=group, arm=arm,
                        plant=plant, seed=int(seed), seed_randomize=bool(randomize),
                        case_file=str((CFG / "cases" / ("%s_%s.yaml" % (case, arm)))
                                      .relative_to(ROOT)),
                        out_dir=str((root / sub / rid).relative_to(ROOT)),
                        overrides=ov))

    def add_group(groups, plant, seed=0, randomize=False):
        for group in groups:
            for case in cases_by_group([group]):
                for arm in ARMS:
                    add(case, group, arm, plant, seed=seed, randomize=randomize)

    if phase == "smoke":
        for case in ("A1_alpha_p30", "A4_descent_replay_S00"):
            for arm in ARMS:
                add(case, "smoke", arm, "P0", window=(98.0, 102.0))
    if phase in ("A1", "P0", "all"):
        add_group(["A1_alpha"], "P0")
    if phase in ("A2", "P0", "all"):
        add_group(["A2_qbar"], "P0")
    if phase in ("A3", "P0", "all"):
        add_group(["A3_coupled"], "P0")
    if phase in ("A4", "P0", "all"):
        add_group(["A4_descent_replay"], "P0")
    if phase in ("P1", "all"):
        add_group(list(P1_GROUPS), "P1")
    if phase in ("seed", "all"):
        for seed in seeds:
            if seed == 0:
                continue
            for case in SEED_CASES:
                for arm in ARMS:
                    add(case, "A4_descent_replay", arm, "P0", seed=seed, randomize=True)
    return out


# --------------------------------------------------------------------------
def run_one(spec):
    from nearspace.config import _to_ns, dump, read_yaml
    from nearspace.torque_gap_ocp import TorqueGapOCP
    from nearspace.torque_gap_rti import TorqueGapRTI
    from nearspace.torque_gap_scenario import Scenario
    from nearspace.torque_gap_sim import ClosedLoop, write_run

    raw = read_yaml(str(ROOT / spec["case_file"]))
    ov = dict(spec.get("overrides") or {})
    ov.setdefault("torque_gap", {})
    ov["torque_gap"]["plant"] = spec["plant"]
    ov["torque_gap"]["seed"] = int(spec["seed"])
    ov["torque_gap"]["seed_randomize"] = bool(spec["seed_randomize"])
    raw = deep_merge(raw, ov)
    cfg = _to_ns(raw)
    scn = Scenario(cfg)
    scn.case = spec["run_id"]
    if spec.get("backend") == "rti":
        ocp = TorqueGapRTI.from_config(scn)
        backend = "rti"
        rti_options = ocp.options()
    else:
        ocp = TorqueGapOCP(scn)
        backend = "ipopt"
        rti_options = None
    cl = ClosedLoop(scn, ocp=ocp, plant=spec["plant"])
    out_dir = ROOT / spec["out_dir"]
    print("[run] %-40s plant=%s cmg_present=%s nx=%d nu=%d nlp=%d/%d steps=%d"
          % (spec["run_id"], cl.plant, cl.cmg_present, scn.nx, scn.nu,
             ocp.n_vars, ocp.n_cons, cl.n_steps), flush=True)
    t0 = time.perf_counter()
    metrics = cl.run()
    wall = time.perf_counter() - t0
    write_run(str(out_dir), cl, metrics, cfg,
              extra_meta=dict(run_id=spec["run_id"], phase=spec["phase"],
                              scenario=spec["case"], group=spec["group"],
                              arm=spec["arm"], config_path=spec["case_file"],
                              overrides=ov.get("torque_gap", {}),
                              wall_clock_s=wall, backend=backend,
                              rti_options=rti_options,
                              command="scripts/run_s08_matrix.py --run-one"))
    (out_dir / "config_resolved.yaml").write_text(
        yaml.safe_dump(raw, allow_unicode=True, sort_keys=False), encoding="utf-8")
    print("[done] %-40s wall=%.1fs rmse=%s fb=%.3f cpu_mean=%.3f"
          % (spec["run_id"], wall,
             {k: round(v, 3) for k, v in metrics["rmse_deg"].items()},
             metrics["fallback_ratio"], metrics["cpu_s"]["mean"]), flush=True)
    return metrics


def driver(phase, jobs, force, only, root=None, backend="rti", seeds=(0,)):
    planned = specs(phase, root, seeds)
    todo = [s for s in planned
            if (only is None or s["case"] in only or s["run_id"] in only)
            and (force or not (ROOT / s["out_dir"] / "metrics.json").exists())]
    print("[matrix] phase=%s runs=%d (of %d planned) jobs=%d backend=%s"
          % (phase, len(todo), len(planned), jobs, backend), flush=True)
    running, done, failed = [], [], []
    while todo or running:
        while todo and len(running) < jobs:
            spec = dict(todo.pop(0))
            spec["backend"] = backend
            out_dir = ROOT / spec["out_dir"]
            out_dir.mkdir(parents=True, exist_ok=True)
            log = open(out_dir / "run.log", "w", encoding="utf-8")
            proc = subprocess.Popen(
                [PY, str(Path(__file__).resolve()), "--run-one",
                 json.dumps(spec, ensure_ascii=False)],
                cwd=str(ROOT), stdout=log, stderr=subprocess.STDOUT,
                env={**os.environ, "PYTHONIOENCODING": "utf-8"})
            proc._log = log                                     # noqa: SLF001
            running.append((spec, proc))
            print("[start] %s" % spec["run_id"], flush=True)
        time.sleep(1.0)
        still = []
        for spec, proc in running:
            rc = proc.poll()
            if rc is None:
                still.append((spec, proc))
                continue
            proc._log.close()                                   # noqa: SLF001
            if rc == 0 and (ROOT / spec["out_dir"] / "metrics.json").exists():
                done.append(spec["run_id"])
                print("[ok]    %s" % spec["run_id"], flush=True)
            else:
                failed.append((spec["run_id"], rc))
                print("[FAIL]  %s rc=%s (see %s/run.log)"
                      % (spec["run_id"], rc, spec["out_dir"]), flush=True)
        running = still
    print("[matrix] done=%d failed=%d" % (len(done), len(failed)), flush=True)
    for rid, rc in failed:
        print("   FAILED", rid, rc)
    return 1 if failed else 0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--phase", default="P0",
                    choices=["smoke", "A1", "A2", "A3", "A4", "P0", "P1", "seed", "all"])
    ap.add_argument("--jobs", type=int, default=1)
    ap.add_argument("--force", action="store_true")
    ap.add_argument("--only", default=None,
                    help="comma separated case or run ids to restrict the phase")
    ap.add_argument("--run-one", default=None)
    ap.add_argument("--results-root", default=None)
    ap.add_argument("--backend", default="rti", choices=["rti", "ipopt"])
    ap.add_argument("--seeds", default="1,2,3,4",
                    help="seeds used by the seed phase (seed 0 is always a base run)")
    ap.add_argument("--list", action="store_true")
    args = ap.parse_args()
    seeds = tuple(int(s) for s in str(args.seeds).split(",") if s.strip())
    if args.run_one:
        run_one(json.loads(args.run_one))
        return 0
    if args.list:
        for s in specs(args.phase, args.results_root, (0,) + seeds):
            done = (ROOT / s["out_dir"] / "metrics.json").exists()
            print("%-52s %-4s %s" % (s["run_id"], s["plant"], "done" if done else "-"))
        return 0
    only = set(x.strip() for x in args.only.split(",")) if args.only else None
    return driver(args.phase, max(int(args.jobs), 1), args.force, only,
                  args.results_root, args.backend, (0,) + seeds)


if __name__ == "__main__":
    sys.exit(main())
