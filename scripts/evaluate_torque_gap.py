#!/usr/bin/env python
"""Aggregate the torque-gap experiment results.

Reads every results/torque_gap/<run>/metrics.json + meta.json and writes

    results/torque_gap/summary.csv      one row per run, raw numbers
    results/torque_gap/summary.md       same table + B0/B1 paired deltas
                                        + a fairness diff of the paired configs

No improvement ratio is invented: the raw numbers are reported, and pairs whose
configuration differs in anything except the allowed modulation switches are
flagged.

python scripts/evaluate_torque_gap.py
"""
from __future__ import annotations

import json
import os
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
RES = ROOT / "results" / "torque_gap"      # overridden by --root in main()

FIELDS = [
    ("case", lambda m: m.get("case", "")),
    ("mode", lambda m: m.get("mode", "")),
    ("plant", lambda m: m.get("plant", "")),
    ("tag", lambda m: m.get("tag", "")),
    ("rmse_all", lambda m: m["rmse_deg"]["all"]),
    ("rmse_man", lambda m: m["rmse_deg"]["maneuver"]),
    ("max_man", lambda m: m["max_err_deg"]["maneuver"]),
    ("settle_s", lambda m: m.get("settle_time_s")),
    ("mig_onset", lambda m: m.get("migration_onset_t")),
    ("mig_lead_s", lambda m: m.get("migration_lead_s")),
    ("pre_travel", lambda m: m.get("pre_gimbal_travel_rad")),
    ("pre_H", lambda m: m.get("pre_H_norm_at_start")),
    ("H_max", lambda m: m.get("H_norm_max")),
    ("sigma_min_man", lambda m: m.get("sigma_min_maneuver_min")),
    ("E_gap_man", lambda m: m.get("E_gap_maneuver_mean")),
    ("gap_res_man", lambda m: m.get("gap_residual_maneuver_mean")),
    ("nu_sat_man", lambda m: m.get("nu_saturation_ratio_maneuver")),
    ("taubnd", lambda m: m.get("tau_bound_used_mean")),
    ("fallback", lambda m: m.get("fallback_ratio")),
    ("cpu_mean", lambda m: m["cpu_s"]["mean"]),
    ("cpu_p95", lambda m: m["cpu_s"]["p95"]),
    ("cpu_max", lambda m: m["cpu_s"]["max"]),
    ("viol", lambda m: m.get("max_violation")),
    ("iters", lambda m: m["iterations"]["mean"]),
]

# configuration keys that are allowed to differ inside a B0/B1 pair
ALLOWED_DIFF = {"mode", "lambda_cfg"}


def load_runs(res=None):
    runs = []
    res = res or RES
    for d in sorted(res.iterdir()):
        if not d.is_dir():
            continue
        mp, mt = d / "metrics.json", d / "meta.json"
        if not (mp.exists() and mt.exists()):
            continue
        metrics = json.loads(mp.read_text(encoding="utf-8"))
        meta = json.loads(mt.read_text(encoding="utf-8"))
        metrics["run"] = d.name
        metrics["case"] = meta.get("case", "")
        parts = d.name.split("_")
        metrics["tag"] = "_".join(parts[3:]) if len(parts) > 3 else ""
        exp = meta.get("experiment", {})
        metrics["plant"] = metrics.get("plant", exp.get("plant", ""))
        metrics["mode"] = metrics.get("mode", exp.get("mode", ""))
        metrics["_meta"] = meta
        runs.append(metrics)
    return runs


def flat(obj, prefix=""):
    out = {}
    if isinstance(obj, dict):
        for k, v in obj.items():
            out.update(flat(v, prefix + k + "."))
    elif isinstance(obj, list):
        out[prefix[:-1]] = json.dumps(obj, sort_keys=True)
    else:
        out[prefix[:-1]] = obj
    return out


def pair_diff(meta_a, meta_b):
    a = flat(meta_a.get("experiment", {}))
    b = flat(meta_b.get("experiment", {}))
    keys = set(a) | set(b)
    diffs = {k: (a.get(k), b.get(k)) for k in sorted(keys)
             if a.get(k) != b.get(k)}
    bad = {k: v for k, v in diffs.items()
           if k.split(".")[-1] not in ALLOWED_DIFF and k not in ALLOWED_DIFF}
    return diffs, bad


def fmt(v):
    if v is None:
        return "-"
    if isinstance(v, str):
        return v
    if isinstance(v, (int, np.integer)):
        return str(int(v))
    if isinstance(v, float):
        if v == 0.0:
            return "0"
        if abs(v) < 1e-3 or abs(v) >= 1e5:
            return "%.2e" % v
        return "%.4g" % v
    return str(v)


def main():
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="results/torque_gap")
    ap.add_argument("--compare-root", default=None,
                    help="second result root for a paired baseline comparison")
    args = ap.parse_args()
    global RES
    RES = (ROOT / args.root) if not Path(args.root).is_absolute() else Path(args.root)
    RES.mkdir(parents=True, exist_ok=True)
    runs = load_runs(RES)
    if not runs:
        print("no runs found under", RES)
        return 1
    runs.sort(key=lambda m: (m["case"], m["mode"], m["plant"], m["run"]))
    # ---- csv ----
    header = ["run"] + [f[0] for f in FIELDS]
    lines = [",".join(header)]
    for m in runs:
        row = [m["run"]] + [fmt(f[1](m)) for f in FIELDS]
        lines.append(",".join(row))
    (RES / "summary.csv").write_text("\n".join(lines) + "\n", encoding="utf-8")

    # ---- markdown ----
    md = ["# Torque-gap experiment summary (raw numbers)", "",
          "Generated by `python scripts/evaluate_torque_gap.py --root %s` from every "
          "`%s/*/metrics.json`." % (args.root, args.root), "",
          "Columns: `rmse_all/rmse_man` attitude RMSE [deg] over the whole run and "
          "over the manoeuvre window; `max_man` maximum attitude error in the window; "
          "`settle_s` time after the window start until the error stays below 1 deg; "
          "`mig_onset` first time the gimbal deviation from the initial configuration "
          "exceeds 3 deg *before* the manoeuvre; `mig_lead_s` = window start - "
          "mig_onset; `pre_travel` accumulated max gimbal rate before the manoeuvre "
          "[rad]; `pre_H` |H| at the manoeuvre start [N m s]; `H_max` peak |H|; "
          "`sigma_min_man` minimum sigma_min(A) during the manoeuvre (diagnostic "
          "only); `E_gap_man` mean instantaneous capability residual; "
          "`gap_res_man` mean ||tau_cmg - d|| during the manoeuvre [N m]; "
          "`nu_sat_man` fraction of saturated gimbal rates in the manoeuvre; "
          "`taubnd` mean fraction of axes sitting on an aerodynamic bound; "
          "`fallback` fraction of control steps whose NLP result was rejected; "
          "`cpu_*` wall clock per control step [s]; `viol` max constraint violation.",
          ""]
    cols = [f[0] for f in FIELDS]
    md.append("| " + " | ".join(["run"] + cols) + " |")
    md.append("|" + "|".join(["---"] * (len(cols) + 1)) + "|")
    for m in runs:
        md.append("| " + " | ".join([m["run"]] + [fmt(f[1](m)) for f in FIELDS]) + " |")

    # ---- paired deltas (only meaningful for archives that still contain the
    # removed modulation modes B1/B2) ----
    # ---- paired deltas (only for archives that still contain the removed
    # modulation modes B1/B2; new runs have a single controller) -----------
    pair_keys = []
    if any(m["mode"] for m in runs):
        md += ["", "## Paired B0 -> B1 comparison (same case/plant/tag)", "",
               "| pair | dRMSE_all | dRMSE_man | dmax_man | dsettle_s | dpre_travel | "
               "dH_max | dgap_res | dfallback | config diff |",
               "|---|---|---|---|---|---|---|---|---|---|"]
        by_key = {}
        for m in runs:
            by_key.setdefault((m["case"], m["plant"], m["tag"]), {})[m["mode"]] = m
        for key, modes in sorted(by_key.items()):
            if "B0" in modes and "B1" in modes:
                a, b = modes["B0"], modes["B1"]
                diffs, bad = pair_diff(a["_meta"], b["_meta"])
                md.append("| %s/%s/%s | %s | %s | %s | %s | %s | %s | %s | %s | %s |" % (
                    key[0], key[1], key[2] or "-",
                    fmt(None if a["rmse_deg"]["all"] is None or b["rmse_deg"]["all"] is None
                        else b["rmse_deg"]["all"] - a["rmse_deg"]["all"]),
                    fmt(None if a["rmse_deg"]["maneuver"] is None
                        or b["rmse_deg"]["maneuver"] is None
                        else b["rmse_deg"]["maneuver"] - a["rmse_deg"]["maneuver"]),
                    fmt(_d(a, b, ("max_err_deg", "maneuver"))),
                    fmt(_d(a, b, ("settle_time_s",))),
                    fmt(_d(a, b, ("pre_gimbal_travel_rad",))),
                    fmt(_d(a, b, ("H_norm_max",))),
                    fmt(_d(a, b, ("gap_residual_maneuver_mean",))),
                    fmt(None if a["fallback_ratio"] is None else
                        b["fallback_ratio"] - a["fallback_ratio"]),
                    ("only mode/lambda: " + ", ".join(sorted(diffs))) if not bad
                    else "**UNEXPECTED: " + ", ".join(sorted(bad)) + "**"))
                pair_keys.append((key, diffs, bad))

    md += ["", "## Fairness check", "",
           "Every B0/B1 pair must differ only in `mode` / `lambda_cfg` (only the "
           "archived ±90°/unlimited arms contain both; the modulation mechanism has "
           "since been removed, so new runs have a single controller).  Pairs with "
           "other differences are flagged:", ""]
    flagged = [(k, bad) for k, _, bad in pair_keys if bad]
    if flagged:
        for k, bad in flagged:
            md.append("- **%s/%s/%s**: unexpected differences %s"
                      % (k[0], k[1], k[2], sorted(bad)))
    elif pair_keys:
        md.append("- none: all pairs differ only in the modulation switch.")
    else:
        md.append("- no B0/B1 pairs in this root (single-controller runs).")

    # ---- optional paired comparison against another result root ----------
    if args.compare_root:
        other = (ROOT / args.compare_root if not Path(args.compare_root).is_absolute()
                 else Path(args.compare_root))
        base_runs = {(m["case"], m["mode"], m["plant"], m["tag"]): m
                     for m in load_runs(other)}
        md += ["", "## Paired comparison with `%s`" % args.compare_root, "",
               "`delta = rmse(new) - rmse(base)`; negative means the arm in `%s` "
               "tracks better." % args.root, "",
               "| case | mode | plant | tag | rmse_base | rmse_new | delta | "
               "fb_base | fb_new | H_max base | H_max new |",
               "|---|---|---|---|---|---|---|---|---|---|---|"]
        for m in runs:
            base = base_runs.get((m["case"], m["mode"], m["plant"], m["tag"]))
            if not base:
                continue
            rb = base["rmse_deg"]["maneuver"]
            rn = m["rmse_deg"]["maneuver"]
            md.append("| %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |" % (
                m["case"], m["mode"], m["plant"], m["tag"] or "-",
                fmt(rb), fmt(rn),
                fmt(None if (rb is None or rn is None) else rn - rb),
                fmt(base["fallback_ratio"]), fmt(m["fallback_ratio"]),
                fmt(base.get("H_norm_max")), fmt(m.get("H_norm_max"))))

    (RES / "summary.md").write_text("\n".join(md) + "\n", encoding="utf-8")
    print("runs:", len(runs), "pairs:", len(pair_keys),
          "flagged:", len(flagged))
    print("written:", RES / "summary.md", "and summary.csv")
    return 0


def _d(a, b, path):
    va = _get(a, path)
    vb = _get(b, path)
    if va is None or vb is None:
        return None
    return vb - va


def _get(obj, path):
    cur = obj
    for k in path:
        if not isinstance(cur, dict) or k not in cur:
            return None
        cur = cur[k]
    return cur


if __name__ == "__main__":
    raise SystemExit(main())
