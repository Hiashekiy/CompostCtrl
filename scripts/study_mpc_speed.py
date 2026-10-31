#!/usr/bin/env python
"""MPC solve-speed study: solver profiles x backends x problem formulations.

Every row is a complete closed-loop run of one scenario, so the numbers are
"control step time actually spent in the optimiser" and "tracking quality
actually achieved", not a single-solve microbenchmark.

Variants
    baseline        IPOPT, the settings of the original experiment
    fast            IPOPT, Tier 1 (tol 1e-4, iteration cap, accept-feasible,
                    warm start kept through fallbacks)
    fast+n0         ... plus the redundant unit-norm constraints removed
    fast+g14        ... plus a 14-node / 9.2 s grid
    fast+g14+n0     ... plus both
    rti12           HPIPM real-time iteration, 12 Gauss-Newton QPs per step
    rti12+g14       ... on the 14-node grid
    rti12+g10       ... on a 10-node / 4.4 s grid

Outputs
    results/torque_gap_mpc_speed/<variant>/<scenario>_<arm>/...
    results/torque_gap_mpc_speed/study.csv
    results/torque_gap_mpc_speed/study.md

python scripts/study_mpc_speed.py [--jobs 6] [--only S00_pitch30,S03_pitch45]
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

OUT = ROOT / "results" / "torque_gap_mpc_speed"
PY = sys.executable
CFG = ROOT / "configs" / "torque_gap_no_cmg_comparison"

GRID14 = [dict(n=6, dt=0.2), dict(n=4, dt=0.5), dict(n=4, dt=1.5)]
GRID10 = [dict(n=6, dt=0.2), dict(n=4, dt=0.8)]

VARIANTS = {
    "baseline": dict(profile="baseline", backend="ipopt", over={}),
    "fast": dict(profile="fast", backend="ipopt", over={}),
    "fast_n0": dict(profile="fast", backend="ipopt",
                    over={"torque_gap": {"solver": {"quat_norm": "n0"}}}),
    "fast_g14": dict(profile="fast", backend="ipopt",
                     over={"torque_gap": {"horizon": GRID14}}),
    "fast_g14_n0": dict(profile="fast", backend="ipopt",
                        over={"torque_gap": {"horizon": GRID14,
                                             "solver": {"quat_norm": "n0"}}}),
    "rti12": dict(profile="baseline", backend="rti", qp_iters=12, over={}),
    "rti12_g14": dict(profile="baseline", backend="rti", qp_iters=12,
                      over={"torque_gap": {"horizon": GRID14}}),
    "rti12_g10": dict(profile="baseline", backend="rti", qp_iters=12,
                      over={"torque_gap": {"horizon": GRID10}}),
}

SCENARIOS = [("S00_pitch30", "with_cmg"), ("S03_pitch45", "with_cmg"),
             ("S05_yaw30", "with_cmg"), ("S00_pitch30", "no_cmg")]


def case_path(scenario, arm):
    for sub in ("cases", "sequential"):
        p = CFG / sub / ("%s_%s.yaml" % (scenario, arm))
        if p.exists():
            return str(p.relative_to(ROOT))
    raise SystemExit("missing case %s/%s" % (scenario, arm))


def specs(only=None):
    out = []
    for vname, v in VARIANTS.items():
        for sc, arm in SCENARIOS:
            if only and sc not in only:
                continue
            rid = "%s_%s" % (sc, arm)
            out.append(dict(
                run_id="%s__%s" % (vname, rid), phase=vname, scenario=sc, arm=arm,
                case=case_path(sc, arm), plant="P0", seed=0, seed_randomize=False,
                out_dir=str((OUT / vname / rid).relative_to(ROOT)),
                solver_profile=v["profile"], backend=v["backend"],
                qp_iters=int(v.get("qp_iters", 0)),
                overrides=v["over"]))
    return out


def run_all(jobs, only):
    todo = [s for s in specs(only)
            if not (ROOT / s["out_dir"] / "metrics.json").exists()]
    planned = len(specs(only))
    print("[study] %d runs to do (%d planned), jobs=%d" % (len(todo), planned, jobs),
          flush=True)
    running, failed = [], []
    while todo or running:
        while todo and len(running) < jobs:
            spec = todo.pop(0)
            d = ROOT / spec["out_dir"]
            d.mkdir(parents=True, exist_ok=True)
            log = open(d / "run.log", "w", encoding="utf-8")
            proc = subprocess.Popen(
                [PY, str(ROOT / "scripts" / "run_no_cmg_matrix.py"), "--run-one",
                 json.dumps(spec, ensure_ascii=False)],
                cwd=str(ROOT), stdout=log, stderr=subprocess.STDOUT,
                env={**os.environ, "PYTHONIOENCODING": "utf-8"})
            proc._log = log                                     # noqa: SLF001
            running.append((spec, proc))
            print("[start]", spec["run_id"], flush=True)
        time.sleep(1.0)
        still = []
        for spec, proc in running:
            rc = proc.poll()
            if rc is None:
                still.append((spec, proc))
                continue
            proc._log.close()                                   # noqa: SLF001
            ok = rc == 0 and (ROOT / spec["out_dir"] / "metrics.json").exists()
            if not ok:
                failed.append(spec["run_id"])
            print("[%s] %s" % ("ok" if ok else "FAIL", spec["run_id"]), flush=True)
        running = still
    return failed


COLS = ["variant", "scenario", "arm", "backend", "qp_iters", "nx_model", "nlp_vars",
        "nlp_constraints", "grid_nodes", "horizon_s", "rmse_all_deg",
        "rmse_man_deg", "rmse_pre_deg", "max_err_man_deg", "cpu_mean_s",
        "cpu_p95_s", "cpu_max_s", "iters_mean", "ms_per_iter", "fallback_ratio",
        "accepted_ratio", "realtime_met", "max_violation", "tau_rms_x", "tau_rms_y",
        "tau_rms_z", "cmg_rms_y", "H_norm_max"]


def collect():
    rows = []
    for vname in VARIANTS:
        for sc, arm in SCENARIOS:
            d = OUT / vname / ("%s_%s" % (sc, arm))
            if not (d / "metrics.json").exists():
                continue
            m = json.loads((d / "metrics.json").read_text(encoding="utf-8"))
            meta = json.loads((d / "meta.json").read_text(encoding="utf-8"))
            st = m.get("stats", {}).get("combined") or {}
            exp = meta.get("experiment", {})
            rows.append(dict(
                variant=vname, scenario=sc, arm=arm,
                backend=meta.get("backend", exp.get("backend", "ipopt")),
                qp_iters=meta.get("qp_iters", 0),
                nx_model=m["nx_model"], nlp_vars=m["nlp_vars"],
                nlp_constraints=m["nlp_constraints"],
                grid_nodes=exp.get("horizon_nodes"), horizon_s=exp.get("horizon_s"),
                rmse_all_deg=m["rmse_deg"]["all"], rmse_man_deg=m["rmse_deg"]["combined"],
                rmse_pre_deg=m["rmse_deg"].get("pre"),
                max_err_man_deg=st.get("max_err_deg"),
                cpu_mean_s=m["cpu_s"]["mean"], cpu_p95_s=m["cpu_s"]["p95"],
                cpu_max_s=m["cpu_s"]["max"], iters_mean=m["iterations"]["mean"],
                ms_per_iter=m.get("ms_per_iteration"),
                fallback_ratio=m["fallback_ratio"],
                accepted_ratio=m.get("accepted_ratio", 0.0),
                realtime_met=m.get("realtime_met"),
                max_violation=m["max_violation"],
                tau_rms_x=((st.get("tau_rms") or [None] * 3)[0]),
                tau_rms_y=((st.get("tau_rms") or [None] * 3)[1]),
                tau_rms_z=((st.get("tau_rms") or [None] * 3)[2]),
                cmg_rms_y=((st.get("cmg_tau_rms") or [None, None, None])[1]),
                H_norm_max=st.get("H_norm_max")))
    return rows


def fmt(v, nd=3):
    if v is None or v == "":
        return "—"
    if isinstance(v, bool):
        return "yes" if v else "no"
    try:
        f = float(v)
    except (TypeError, ValueError):
        return str(v)
    return ("%%.%df" % nd) % f


def figure(rows):
    """Grid length versus tracking and cost (the key Tier-2 negative result)."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        plt.rcParams.update({
            "font.family": "sans-serif",
            "font.sans-serif": ["Microsoft YaHei", "SimHei", "DejaVu Sans"],
            "axes.unicode_minus": False, "font.size": 8.5, "axes.labelsize": 8.5,
            "axes.titlesize": 9.5, "legend.fontsize": 7.5,
        })
        FIG = ROOT / "figures" / "torque_gap_mpc_speed"
        FIG.mkdir(parents=True, exist_ok=True)
        series = [
            ("fast", "S00_pitch30", "with_cmg", "IPOPT 22 nodes"),
            ("fast_g14", "S00_pitch30", "with_cmg", "IPOPT 14 nodes"),
            ("rti12", "S00_pitch30", "with_cmg", "RTI 22 nodes"),
            ("rti12_g14", "S00_pitch30", "with_cmg", "RTI 14 nodes"),
            ("rti12_g10", "S00_pitch30", "with_cmg", "RTI 10 nodes"),
        ]
        idx = {(r["variant"], r["scenario"], r["arm"]): r for r in rows}
        fig, axs = plt.subplots(1, 2, figsize=(12.0, 3.8))
        combos = [("S00_pitch30", "S00"), ("S03_pitch45", "S03"), ("S05_yaw30", "S05")]
        variants = ["fast", "fast_g14", "rti12", "rti12_g14", "rti12_g10"]
        w = 0.16
        xp = np.arange(len(combos))
        for j, v in enumerate(variants):
            rm, tm = [], []
            for sc, _ in combos:
                r = idx.get((v, sc, "with_cmg"))
                rm.append(r["rmse_man_deg"] if r else np.nan)
                tm.append(1000 * r["cpu_mean_s"] if r else np.nan)
            axs[0].bar(xp + (j - 2) * w, rm, w, label=v)
            axs[1].bar(xp + (j - 2) * w, tm, w, label=v)
        axs[0].set_yscale("log")
        axs[0].set_ylabel("manoeuvre RMSE [deg]")
        axs[0].set_title("Shorter horizon costs tracking (B group)")
        axs[1].set_yscale("log")
        axs[1].axhline(200.0, color="k", ls="--", lw=0.9, label="0.2 s period")
        axs[1].set_ylabel("mean solve time [ms]")
        axs[1].set_title("... and buys speed")
        for a in axs:
            a.set_xticks(xp)
            a.set_xticklabels([c[1] for c in combos])
            a.grid(alpha=0.3, axis="y")
            a.legend()
        fig.tight_layout()
        fig.savefig(FIG / "tier2_grid_study.png", dpi=150)
        plt.close(fig)
        print("wrote", FIG / "tier2_grid_study.png")
    except Exception as exc:                                   # noqa: BLE001
        print("figure skipped:", exc)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--jobs", type=int, default=6)
    ap.add_argument("--only", default=None)
    ap.add_argument("--skip-runs", action="store_true")
    args = ap.parse_args()
    only = set(x.strip() for x in args.only.split(",")) if args.only else None
    OUT.mkdir(parents=True, exist_ok=True)
    if not args.skip_runs:
        failed = run_all(args.jobs, only)
        if failed:
            print("[study] FAILED:", failed)
    rows = collect()
    with open(OUT / "study.csv", "w", encoding="utf-8", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=COLS)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, "") for k in COLS})

    lines = ["# MPC solve-speed study", "",
             "Generated by `scripts/study_mpc_speed.py`; every row is a full "
             "closed-loop P0 run of one scenario, so `cpu_mean_s` is the control-step "
             "time actually spent inside the optimiser and `rmse_man_deg` is the "
             "tracking quality actually achieved over the manoeuvre window.", "",
             "## Variants", "",
             "| variant | optimiser | problem |", "|---|---|---|",
             "| baseline | IPOPT, tol 1e-6, no acceptance | 22 nodes / 23.2 s |",
             "| fast | IPOPT, tol 1e-4, iteration cap 200, accept feasible, warm start kept | 22 nodes / 23.2 s |",
             "| fast_n0 | fast | unit-norm quaternion constraints only at node 0 |",
             "| fast_g14 | fast | 14 nodes / 9.2 s |",
             "| fast_g14_n0 | fast | 14 nodes / 9.2 s + n0 |",
             "| rti12 | HPIPM RTI, 12 Gauss-Newton QPs | 22 nodes / 23.2 s |",
             "| rti12_g14 | HPIPM RTI, 12 QPs | 14 nodes / 9.2 s |",
             "| rti12_g10 | HPIPM RTI, 12 QPs | 10 nodes / 4.4 s |",
             "", "## Results", "",
             "| variant | scenario/arm | nodes | mean [ms] | p95 [ms] | max [ms] | "
             "manoeuvre RMSE [deg] | pre RMSE [deg] | fallback | real-time |",
             "|---|---|---|---|---|---|---|---|---|---|"]
    for r in sorted(rows, key=lambda x: (list(VARIANTS).index(x["variant"]),
                                         x["scenario"], x["arm"])):
        lines.append("| %s | %s/%s | %s | %s | %s | %s | %s | %s | %s | %s |" % (
            r["variant"], r["scenario"], r["arm"], fmt(r["grid_nodes"], 0),
            fmt(1000 * r["cpu_mean_s"], 1), fmt(1000 * r["cpu_p95_s"], 1),
            fmt(1000 * r["cpu_max_s"], 1), fmt(r["rmse_man_deg"], 4),
            fmt(r["rmse_pre_deg"], 4), fmt(r["fallback_ratio"]),
            fmt(r["realtime_met"])))
    # speed-up table against the baseline variant
    base = {(r["scenario"], r["arm"]): r for r in rows if r["variant"] == "baseline"}
    if base:
        lines += ["", "## Speed-up and tracking change versus `baseline`", "",
                  "| variant | scenario/arm | speed-up (mean) | speed-up (p95) | "
                  "Δ manoeuvre RMSE [deg] |", "|---|---|---|---|---|"]
        for r in sorted(rows, key=lambda x: (list(VARIANTS).index(x["variant"]),
                                             x["scenario"], x["arm"])):
            b = base.get((r["scenario"], r["arm"]))
            if not b or r["variant"] == "baseline":
                continue
            lines.append("| %s | %s/%s | %.1fx | %.1fx | %+0.4f |" % (
                r["variant"], r["scenario"], r["arm"],
                b["cpu_mean_s"] / max(r["cpu_mean_s"], 1e-9),
                b["cpu_p95_s"] / max(r["cpu_p95_s"], 1e-9),
                r["rmse_man_deg"] - b["rmse_man_deg"]))
    (OUT / "study.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    figure(rows)
    print("wrote", (OUT / "study.csv"))
    print("wrote", (OUT / "study.md"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
