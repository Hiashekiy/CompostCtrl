#!/usr/bin/env python
"""Sweep the RTI linearisation-stability options on the hard scenarios.

Goal: keep every control step inside the 0.2 s period *and* recover the tracking
that plain RTI loses on the authority-limited scenarios (S03 pitch 45 deg,
S05 yaw 30 deg), without regressing the easy ones.

Options swept (see nearspace/torque_gap_rti.py):
    state_tr_gamma   hard trust region on |dgamma| per stage          [rad]
    state_tr_omega   hard trust region on |domega| per stage         [rad/s]
    state_tr_sigma   tighten that trust region when sigma_min is low
    nu_scale_sigma   tighten the gimbal-rate trust region near singularity
    qp_warm_start    reuse the previous QP solution inside HPIPM
    cost_scale       normalise the QP cost to O(1)
    gamma_damp       damping on |dgamma|^2 in the QP cost
    qp_iters_hi      extra QPs when sigma_min < floor

python scripts/sweep_rti_stability.py [--jobs 6] [--only S03_pitch45,S05_yaw30]
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

OUT = ROOT / "results" / "torque_gap_rti_stability"
PY = sys.executable
CFG = ROOT / "configs" / "torque_gap_no_cmg_comparison"

VARIANTS = {
    # name: (qp_iters, rti options, note)
    "v0_plain": (12, {}, "plain RTI (reference)"),
    "v1_tr03": (12, {"state_tr_gamma": 0.3}, "dgamma trust region 0.3 rad"),
    "v2_tr05w": (12, {"state_tr_gamma": 0.5, "state_tr_omega": 1.0},
                 "dgamma 0.5 rad + domega 1 rad/s"),
    "v3_tr_sigma": (12, {"state_tr_gamma": 0.3, "state_tr_sigma": True,
                         "nu_scale_sigma": True}, "sigma-aware trust regions"),
    "v4_ws": (12, {"qp_warm_start": True}, "QP warm start"),
    "v5_cs": (12, {"cost_scale": True}, "QP cost scaling"),
    "v6_combo": (12, {"state_tr_gamma": 0.3, "state_tr_sigma": True,
                      "nu_scale_sigma": True, "qp_warm_start": True,
                      "cost_scale": True}, "trust regions + warm start + scaling"),
    "v7_combo_hi": (12, {"state_tr_gamma": 0.3, "state_tr_sigma": True,
                         "nu_scale_sigma": True, "qp_warm_start": True,
                         "cost_scale": True, "qp_iters_hi": 24},
                    "combo + 24 QPs near singularity"),
    "v8_combo_damp": (12, {"state_tr_gamma": 0.3, "state_tr_sigma": True,
                           "nu_scale_sigma": True, "qp_warm_start": True,
                           "cost_scale": True, "gamma_damp": 0.05},
                      "combo + gamma damping"),
    # the trust region 0.5 rad / 1 rad/s is the winner of the first round; these
    # variants test whether the cheap numerical options add anything on top
    "v9_tr_ws": (12, {"state_tr_gamma": 0.5, "state_tr_omega": 1.0,
                      "qp_warm_start": True}, "winner + QP warm start"),
    "v10_tr_ws_cs": (12, {"state_tr_gamma": 0.5, "state_tr_omega": 1.0,
                          "qp_warm_start": True, "cost_scale": True},
                     "winner + warm start + cost scaling"),
    "v11_tr08": (12, {"state_tr_gamma": 0.8, "state_tr_omega": 1.5},
                 "looser trust region 0.8 rad / 1.5 rad/s"),
    "v12_tr_ws_sigma": (12, {"state_tr_gamma": 0.5, "state_tr_omega": 1.0,
                             "qp_warm_start": True, "state_tr_sigma": True,
                             "nu_scale_sigma": True},
                        "winner + warm start + sigma-aware tightening"),
}

SCENARIOS = ["S00_pitch30", "S02_pitch10", "S03_pitch45", "S04_roll30",
             "S05_yaw30", "S07_sequential"]

COLS = ["variant", "note", "scenario", "qp_iters", "state_tr_gamma", "state_tr_sigma",
        "nu_scale_sigma", "qp_warm_start", "cost_scale", "gamma_damp", "qp_iters_hi",
        "rmse_man_deg", "rmse_all_deg", "max_err_man_deg", "cpu_mean_ms",
        "cpu_p95_ms", "cpu_max_ms", "iters_mean", "fallback_ratio", "realtime_met"]


def case_path(scenario, arm="with_cmg"):
    for sub in ("cases", "sequential"):
        p = CFG / sub / ("%s_%s.yaml" % (scenario, arm))
        if p.exists():
            return str(p.relative_to(ROOT))
    raise SystemExit("missing case")


def specs(only=None):
    out = []
    for name, (qit, opts, _note) in VARIANTS.items():
        for sc in SCENARIOS:
            if only and sc not in only:
                continue
            out.append(dict(run_id="%s__%s" % (name, sc), phase=name, scenario=sc,
                            arm="with_cmg", case=case_path(sc), plant="P0",
                            seed=0, seed_randomize=False,
                            out_dir=str((OUT / name / sc).relative_to(ROOT)),
                            solver_profile="baseline", backend="rti", qp_iters=qit,
                            overrides={"torque_gap": {"rti": dict(opts)}}))
    return out


def run_all(jobs, only):
    todo = [s for s in specs(only) if not (ROOT / s["out_dir"] / "metrics.json").exists()]
    print("[sweep] %d runs to do (of %d), jobs=%d"
          % (len(todo), len(specs(only)), jobs), flush=True)
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


def collect():
    rows = []
    for name, (qit, opts, note) in VARIANTS.items():
        for sc in SCENARIOS:
            d = OUT / name / sc
            if not (d / "metrics.json").exists():
                continue
            m = json.loads((d / "metrics.json").read_text(encoding="utf-8"))
            meta = json.loads((d / "meta.json").read_text(encoding="utf-8"))
            st = (m.get("stats") or {}).get("combined") or {}
            ro = meta.get("rti_options") or {}
            rows.append(dict(
                variant=name, note=note, scenario=sc, qp_iters=m and ro.get("qp_iters"),
                state_tr_gamma=ro.get("state_tr_gamma"), state_tr_sigma=ro.get("state_tr_sigma"),
                nu_scale_sigma=ro.get("nu_scale_sigma"), qp_warm_start=ro.get("qp_warm_start"),
                cost_scale=ro.get("cost_scale"), gamma_damp=ro.get("gamma_damp"),
                qp_iters_hi=ro.get("qp_iters_hi"),
                rmse_man_deg=m["rmse_deg"]["combined"], rmse_all_deg=m["rmse_deg"]["all"],
                max_err_man_deg=st.get("max_err_deg"),
                cpu_mean_ms=1000 * m["cpu_s"]["mean"], cpu_p95_ms=1000 * m["cpu_s"]["p95"],
                cpu_max_ms=1000 * m["cpu_s"]["max"],
                iters_mean=m["iterations"]["mean"], fallback_ratio=m["fallback_ratio"],
                realtime_met=m.get("realtime_met")))
    return rows


def fmt(v, nd=3):
    if v is None or v == "":
        return "—"
    if isinstance(v, bool):
        return "yes" if v else "no"
    try:
        return ("%%.%df" % nd) % float(v)
    except (TypeError, ValueError):
        return str(v)


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
            print("[sweep] FAILED:", failed)
    rows = collect()
    with open(OUT / "sweep.csv", "w", encoding="utf-8", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=COLS, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)

    idx = {(r["variant"], r["scenario"]): r for r in rows}
    base = {sc: idx.get(("v0_plain", sc)) for sc in SCENARIOS}
    L = ["# RTI linearisation-stability sweep", "",
         "Every row is a full closed-loop P0 run (B group, 90-150 s / 90-180 s).",
         "Reference = `v0_plain`: plain RTI, 12 Gauss-Newton QPs, 22 nodes.", "",
         "Targets: S03 <= 3.0 deg, S05 <= 4.0 deg, P95 <= 200 ms, no regression on",
         "S00 <= 0.15 / S02 <= 0.01 / S04 <= 0.01 / S07 <= 0.10 deg.", "",
         "## Tracking (manoeuvre-window RMSE, deg)", ""]
    hdr = "| variant | " + " | ".join(s.split("_")[0] for s in SCENARIOS) + " | mean |"
    L += [hdr, "|---" * (len(SCENARIOS) + 2) + "|"]
    for name, (qit, opts, note) in VARIANTS.items():
        vals = [idx.get((name, sc)) for sc in SCENARIOS]
        if not any(vals):
            continue
        cells = [fmt(v["rmse_man_deg"], 4) if v else "—" for v in vals]
        mean = np.mean([v["rmse_man_deg"] for v in vals if v])
        L.append("| %s | %s | %.4f |" % (name, " | ".join(cells), mean))
    L += ["", "## Speed (mean / P95 per control step, ms)", "",
          hdr.replace("| variant |", "| variant |"), "|---" * (len(SCENARIOS) + 2) + "|"]
    for name, (qit, opts, note) in VARIANTS.items():
        vals = [idx.get((name, sc)) for sc in SCENARIOS]
        if not any(vals):
            continue
        cells = ["%s / %s" % (fmt(v["cpu_mean_ms"], 0), fmt(v["cpu_p95_ms"], 0))
                 if v else "—" for v in vals]
        mean = np.mean([v["cpu_mean_ms"] for v in vals if v])
        L.append("| %s | %s | %s |" % (name, " | ".join(cells), fmt(mean, 1)))
    L += ["", "## Verdict versus v0_plain", "",
          "| variant | note | S03 improve | S05 improve | S00/S02/S04/S07 regression | all P95 <= 200 ms | verdict |",
          "|---|---|---|---|---|---|---|"]
    for name, (qit, opts, note) in VARIANTS.items():
        if name == "v0_plain":
            continue
        v03, v05 = idx.get((name, "S03_pitch45")), idx.get((name, "S05_yaw30"))
        b03, b05 = base["S03_pitch45"], base["S05_yaw30"]
        d03 = (v03["rmse_man_deg"] - b03["rmse_man_deg"]) if (v03 and b03) else None
        d05 = (v05["rmse_man_deg"] - b05["rmse_man_deg"]) if (v05 and b05) else None
        reg = []
        for sc in ("S00_pitch30", "S02_pitch10", "S04_roll30", "S07_sequential"):
            v, b = idx.get((name, sc)), base[sc]
            if v and b and v["rmse_man_deg"] > b["rmse_man_deg"] * 1.25 + 0.01:
                reg.append("%s %.3f->%.3f" % (sc.split("_")[0], b["rmse_man_deg"],
                                              v["rmse_man_deg"]))
        p95 = max((idx[(name, sc)]["cpu_p95_ms"] for sc in SCENARIOS
                   if idx.get((name, sc))), default=np.nan)
        ok = (d03 is not None and idx[(name, "S03_pitch45")]["rmse_man_deg"] <= 3.0
              and idx[(name, "S05_yaw30")]["rmse_man_deg"] <= 4.0
              and p95 <= 200.0 and not reg)
        L.append("| %s | %s | %s | %s | %s | %s | %s |" % (
            name, note,
            "%+.3f" % d03 if d03 is not None else "—",
            "%+.3f" % d05 if d05 is not None else "—",
            ", ".join(reg) if reg else "none",
            fmt(p95, 0), "**PASS**" if ok else "no"))
    (OUT / "sweep.md").write_text("\n".join(L) + "\n", encoding="utf-8")
    print("wrote", OUT / "sweep.csv")
    print("wrote", OUT / "sweep.md")
    return 0


if __name__ == "__main__":
    sys.exit(main())
