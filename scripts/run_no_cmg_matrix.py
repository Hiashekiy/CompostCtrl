#!/usr/bin/env python
"""Run matrix of the no-CMG (A) vs surface+CMG (B) comparison.

Phases
    smoke   S00, S01, S04, A/B, P0, short window (98-118 s)
    P0      S00-S06, A/B, P0, 90-150 s           (14 runs)
    P0seq   S07, A/B, P0, 90-180 s               (2 runs)
    P1      S00, S01, S06, A/B, P1, 90-150 s     (6 runs)
    seed    S00, S01, S06, A/B, P0, seeds 1,2 (disturbance phase randomised,
            identical for both arms of a pair)
    all     P0 + P0seq + P1
    seedall seed

Every run reuses exactly the generated case file (configs/torque_gap_no_cmg_
comparison/cases/*.yaml, sequential/*.yaml); nothing is overridden except the
plant, the seed, the seed-randomisation switch and (smoke phase) the window.

Outputs
    results/torque_gap_no_cmg_comparison/<phase>/<run_id>/
        timeseries.csv  metrics.json  meta.json  config_resolved.yaml  run.log

Examples
    python scripts/run_no_cmg_matrix.py --phase P0 --jobs 4
    python scripts/run_no_cmg_matrix.py --phase smoke --jobs 2
    python scripts/run_no_cmg_matrix.py --run-one '<spec json>'
"""
from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from nearspace.config import deep_merge                     # noqa: E402

CFG = ROOT / "configs" / "torque_gap_no_cmg_comparison"
OUT = ROOT / "results" / "torque_gap_no_cmg_comparison_rti2"
PY = sys.executable

# Solver profiles.  `baseline` is exactly what the first experiment used; `fast`
# is the Tier-1 speed profile.  Both arms of a scenario always share one profile,
# so the A/B comparison stays fair.
SOLVER_PROFILES = {
    # Pinned to the settings of the *first* experiment: the Tier-1 acceptance
    # policy defaults to True in the code now, so an empty profile would silently
    # re-run the baseline arm with Tier-1 behaviour.
    "baseline": {"torque_gap": {"solver": {"tol": 1e-6, "max_iter": 600,
                                           "accept_feasible": False,
                                           "warm_shift_on_fallback": False}}},
    # 3.8x faster on S00/B with no tracking loss (manoeuvre RMSE 0.096 -> 0.090);
    # the win comes from accepting feasible non-converged solutions and keeping
    # the warm start alive, not from the looser tolerance.
    "fast": {"torque_gap": {"solver": {"tol": 1e-4, "max_iter": 200,
                                       "accept_feasible": True,
                                       "accept_feasible_viol": 1e-4,
                                       "warm_shift_on_fallback": True}}},
    # extreme end of the speed/quality trade-off: no further speed-up on S00/B
    # and 4x worse manoeuvre RMSE, kept as evidence, not as a default.
    "fastest": {"torque_gap": {"solver": {"tol": 1e-3, "max_iter": 150,
                                          "accept_feasible": True,
                                          "accept_feasible_viol": 1e-4,
                                          "warm_shift_on_fallback": True}}},
}

S0 = ["S00_pitch30", "S01_pitch_minus30", "S02_pitch10", "S03_pitch45",
      "S04_roll30", "S05_yaw30", "S06_pitch30_roll20"]
P1_SET = ["S00_pitch30", "S01_pitch_minus30", "S06_pitch30_roll20"]
SEED_SET = ["S00_pitch30", "S01_pitch_minus30", "S06_pitch30_roll20"]
ARMS = ("no_cmg", "with_cmg")


def case_path(scenario, arm):
    for sub in ("cases", "sequential"):
        p = CFG / sub / ("%s_%s.yaml" % (scenario, arm))
        if p.exists():
            return p
    raise SystemExit("missing case file for %s/%s" % (scenario, arm))


def specs(phase, root=None, solver_profile="baseline", backend="ipopt", qp_iters=12,
          rti_options=None):
    out = []
    root = Path(root) if root else OUT
    if not root.is_absolute():
        root = ROOT / root
    profile = SOLVER_PROFILES[solver_profile]

    def add(sub, scenario, arm, plant, seed=0, randomize=False, extra=None):
        rid = "%s_%s" % (scenario, arm)
        if randomize:
            rid += "_seed%d" % seed
        ov = deep_merge(profile, extra or {})
        if rti_options:
            ov = deep_merge(ov, {"torque_gap": {"rti": dict(rti_options)}})
        out.append(dict(run_id=rid, phase=sub, scenario=scenario, arm=arm,
                        case=str(case_path(scenario, arm).relative_to(ROOT)),
                        plant=plant, seed=int(seed), seed_randomize=bool(randomize),
                        out_dir=str((root / sub / rid).relative_to(ROOT)),
                        solver_profile=solver_profile,
                        backend=backend, qp_iters=int(qp_iters),
                        overrides=ov))

    if phase in ("smoke", "all_smoke"):
        for sc in ("S00_pitch30", "S01_pitch_minus30", "S04_roll30"):
            for arm in ARMS:
                add("smoke", sc, arm, "P0",
                    extra={"torque_gap": {"start": 98.0, "end": 118.0}})
    if phase in ("P0", "all"):
        for sc in S0:
            for arm in ARMS:
                add("P0", sc, arm, "P0")
    if phase in ("P0seq", "all"):
        for arm in ARMS:
            add("P0", "S07_sequential", arm, "P0")
    if phase in ("P1", "all"):
        for sc in P1_SET:
            for arm in ARMS:
                add("P1", sc, arm, "P1")
    if phase in ("seed", "seedall", "all"):
        for sc in SEED_SET:
            for seed in (1, 2):
                for arm in ARMS:
                    add("P0", sc, arm, "P0", seed=seed, randomize=True)
    return out


# --------------------------------------------------------------------------
def run_one(spec):
    from nearspace.config import _to_ns, deep_merge, dump, read_yaml
    from nearspace.torque_gap_ocp import TorqueGapOCP
    from nearspace.torque_gap_scenario import Scenario
    from nearspace.torque_gap_sim import ClosedLoop, write_run

    raw = read_yaml(str(ROOT / spec["case"]))
    ov = dict(spec.get("overrides") or {})
    ov.setdefault("torque_gap", {})
    ov["torque_gap"]["plant"] = spec["plant"]
    ov["torque_gap"]["seed"] = int(spec["seed"])
    ov["torque_gap"]["seed_randomize"] = bool(spec["seed_randomize"])
    raw = deep_merge(raw, ov)
    cfg = _to_ns(raw)
    scn = Scenario(cfg)
    scn.case = spec["run_id"]
    backend = spec.get("backend", "ipopt")
    rti_options = None
    if backend == "rti":
        from nearspace.torque_gap_rti import TorqueGapRTI
        ocp = TorqueGapRTI.from_config(scn, qp_iters=int(spec.get("qp_iters", 12)))
        rti_options = ocp.options()
    else:
        ocp = TorqueGapOCP(scn)
    cl = ClosedLoop(scn, ocp=ocp, plant=spec["plant"])
    out_dir = ROOT / spec["out_dir"]
    print("[run] %-34s plant=%s backend=%s cmg_present=%s nx=%d nu=%d nlp=%d/%d steps=%d"
          % (spec["run_id"], cl.plant, backend, cl.cmg_present, scn.nx, scn.nu,
             ocp.n_vars, ocp.n_cons, cl.n_steps), flush=True)
    t0 = time.perf_counter()
    metrics = cl.run()
    wall = time.perf_counter() - t0
    meta = write_run(str(out_dir), cl, metrics, cfg,
                     extra_meta=dict(run_id=spec["run_id"], phase=spec["phase"],
                                     scenario=spec["scenario"], arm=spec["arm"],
                                     config_path=spec["case"],
                                     overrides=ov.get("torque_gap", {}),
                                     wall_clock_s=wall,
                                     backend=backend,
                                     qp_iters=int(spec.get("qp_iters", 0)),
                                     rti_options=rti_options,
                                     command="scripts/run_no_cmg_matrix.py --run-one"))
    (out_dir / "config_resolved.yaml").write_text(
        yaml.safe_dump(raw, allow_unicode=True, sort_keys=False), encoding="utf-8")
    print("[done] %-34s wall=%.1fs rmse=%s fb=%.3f cpu_mean=%.3f"
          % (spec["run_id"], wall,
             {k: round(v, 3) for k, v in metrics["rmse_deg"].items()},
             metrics["fallback_ratio"], metrics["cpu_s"]["mean"]), flush=True)
    return meta


def driver(phase, jobs, force, only, root=None, solver_profile="baseline",
           backend="ipopt", qp_iters=12, rti_options=None):
    todo = [s for s in specs(phase, root, solver_profile, backend, qp_iters, rti_options)
            if (only is None or s["scenario"] in only)
            and (force or not (ROOT / s["out_dir"] / "metrics.json").exists())]
    print("[matrix] phase=%s runs=%d (of %d planned) jobs=%d"
          % (phase, len(todo), len(specs(phase, root, solver_profile, backend, qp_iters,
                                        rti_options)), jobs), flush=True)
    running = []
    done, failed = [], []
    while todo or running:
        while todo and len(running) < jobs:
            spec = todo.pop(0)
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


def _rti_opts(args):
    return json.loads(args.rti_options) if args.rti_options else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--phase", default="P0",
                    choices=["smoke", "P0", "P0seq", "P1", "seed", "seedall", "all"])
    ap.add_argument("--jobs", type=int, default=1)
    ap.add_argument("--force", action="store_true")
    ap.add_argument("--only", default=None,
                    help="comma separated scenario ids to restrict the phase")
    ap.add_argument("--run-one", default=None)
    ap.add_argument("--results-root", default=None,
                    help="results directory (default results/torque_gap_no_cmg_comparison)")
    ap.add_argument("--solver-profile", default="baseline",
                    choices=sorted(SOLVER_PROFILES))
    ap.add_argument("--backend", default="ipopt", choices=["ipopt", "rti"])
    ap.add_argument("--qp-iters", type=int, default=12)
    ap.add_argument("--rti-options", default=None,
                    help="JSON dict of torque_gap.rti.* overrides, e.g. "
                         "state_tr_gamma / state_tr_omega / qp_warm_start")
    ap.add_argument("--list", action="store_true")
    args = ap.parse_args()
    if args.run_one:
        run_one(json.loads(args.run_one))
        return 0
    if args.list:
        for s in specs(args.phase, args.results_root, args.solver_profile,
                       args.backend, args.qp_iters, _rti_opts(args)):
            done = (ROOT / s["out_dir"] / "metrics.json").exists()
            print("%-40s %s %s" % (s["run_id"], s["plant"], "done" if done else "-"))
        return 0
    only = set(x.strip() for x in args.only.split(",")) if args.only else None
    return driver(args.phase, max(int(args.jobs), 1), args.force, only,
                  args.results_root, args.solver_profile, args.backend, args.qp_iters,
                  _rti_opts(args))


if __name__ == "__main__":
    sys.exit(main())
