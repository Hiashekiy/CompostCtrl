#!/usr/bin/env python
"""Quantitative analysis of the unlimited-gimbal experiment.

Computes, from the raw `timeseries.csv` of both arms (bounded +-90 deg and
continuous rotation), everything the report must answer:

  * gimbal travel: max |gamma|, number of 360 deg wraps, whether the +-90 deg wall
    is still reached, total gimbal path versus net rotation (multi-turn sanity),
  * CMG capability: |H| peak and components, sigma_min minimum, gimbal-rate
    saturation fraction, achieved CMG torque versus the predicted gap,
  * attitude: RMSE / peak error in the manoeuvre window, settling time,
  * aerodynamics: fraction of axes on a bound, commanded versus achieved torque
    (P1 only),
  * solver: fallback fraction, wall clock mean / p95 / max, iterations,
  * migration: onset time before the manoeuvre, gimbal state at the manoeuvre
    start, H at the manoeuvre start.

Writes `results/torque_gap_unlimited/analysis.md` and `analysis.csv`.

python scripts/analyze_torque_gap_unlimited.py
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
DEG = np.rad2deg


def read_ts(path):
    with open(path, encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    out = {}
    for key in rows[0]:
        vals = []
        for r in rows:
            try:
                vals.append(float(r[key]))
            except (TypeError, ValueError):
                vals.append(np.nan)
        out[key] = np.array(vals)
    return out


def run_dirs(root):
    root = Path(root)
    if not root.exists():
        return {}
    out = {}
    for d in sorted(root.iterdir()):
        if not d.is_dir():
            continue
        if not ((d / "timeseries.csv").exists() and (d / "meta.json").exists()
                and (d / "metrics.json").exists()):
            continue
        meta = json.loads((d / "meta.json").read_text(encoding="utf-8"))
        exp = meta.get("experiment", {})
        key = (meta.get("case", ""), exp.get("mode", "") or "ctl", exp.get("plant", "P0"))
        h0 = float(meta.get("experiment", {}).get("c_mg", {}).get("h0", 50.0))
        out[key] = dict(dir=d, meta=meta, h0=h0,
                        metrics=json.loads((d / "metrics.json").read_text(encoding="utf-8")))
    return out


def analyse(entry):
    ts = read_ts(entry["dir"] / "timeseries.csv")
    m = entry["metrics"]
    meta = entry["meta"]
    exp = meta.get("experiment", {})
    gamma = np.column_stack([ts["g%d" % (i + 1)] for i in range(4)])
    nu = np.column_stack([ts["nu%d" % (i + 1)] for i in range(4)])
    H = np.column_stack([ts["Hx"], ts["Hy"], ts["Hz"]])
    win = m.get("window")
    t = ts["t"]
    mw = (t >= win[0]) & (t <= win[1]) if win else np.ones_like(t, bool)
    prem = (t < win[0]) if win else np.zeros_like(t, bool)
    cmg = np.column_stack([ts["cmg_x"], ts["cmg_y"], ts["cmg_z"]])
    d = np.column_stack([ts["d_x"], ts["d_y"], ts["d_z"]])
    tau_cmd = np.column_stack([ts["tauc_cmd_x"], ts["tauc_cmd_y"], ts["tauc_cmd_z"]])
    tau_act = np.column_stack([ts["tau_act_x"], ts["tau_act_y"], ts["tau_act_z"]])
    gap_norm = np.linalg.norm(d[mw], axis=1) if mw.any() else np.array([0.0])
    cover = np.linalg.norm(cmg[mw] - d[mw], axis=1) if mw.any() else np.array([0.0])
    path = float(np.sum(np.abs(np.diff(gamma, axis=0))))
    net = float(np.sum(np.abs(gamma[-1] - gamma[0])))
    H_net = float(np.linalg.norm(H[-1] - H[0]))
    # dH/dt = A nu with |A nu| <= h0 * sum|nu|, so path >= |delta H| / h0
    h0 = float(entry.get("h0", 50.0))
    min_path = H_net / h0
    geom = np.abs((gamma + np.pi) % (2 * np.pi) - np.pi)   # geometric excursion
    # tighter lower bound on the required gimbal path: |dH/dt| = |A nu| and
    # |A nu| <= h0 sum|nu_i|, so path >= integral |A nu| dt / h0.  The ratio
    # path_min/path separates "travel that changes the cluster momentum" from
    # cyclic / null-space travel.
    dt_ctrl = float(np.median(np.diff(t))) if t.size > 1 else 0.2
    path_min_torque = float(np.sum(np.linalg.norm(cmg, axis=1)) * dt_ctrl) / h0
    # control chatter: how far the applied commands move at every control step
    chat_tau = (float(np.mean(np.linalg.norm(np.diff(tau_cmd, axis=0), axis=1)))
                if tau_cmd.shape[0] > 1 else 0.0)
    dnu = np.diff(nu, axis=0)
    chat_nu = float(np.mean(np.linalg.norm(dnu, axis=1))) if dnu.size else 0.0
    flips = (float(np.mean(np.any(np.sign(dnu[:-1]) * np.sign(dnu[1:]) < 0, axis=1)))
             if dnu.shape[0] > 2 else 0.0)

    i0 = int(np.argmax(t >= win[0])) if win else 0
    return dict(
        case=meta.get("case", ""), mode=exp.get("mode", ""),
        plant=exp.get("plant", "P0"),
        gamma_limit_mode=exp.get("gamma_limit_mode", "bounded"),
        rmse_man=m["rmse_deg"]["maneuver"], rmse_all=m["rmse_deg"]["all"],
        max_man=m["max_err_deg"]["maneuver"],
        settle_s=m.get("settle_time_s"),
        mig_onset=m.get("migration_onset_t"), mig_lead=m.get("migration_lead_s"),
        gamma_deg_start=[float(DEG(v)) for v in gamma[i0]],
        gamma_deg_max=float(DEG(np.max(np.abs(gamma)))),
        gamma_deg_final=[float(DEG(v)) for v in gamma[-1]],
        turns=float(np.max(np.abs(gamma)) / (2.0 * np.pi)),
        path_rad=path, net_rad=net,
        path_over_net=float(path / net) if net > 1e-9 else float("inf"),
        H_net_change=H_net,
        path_efficiency=float(min_path / path) if path > 1e-9 else float("nan"),
        gamma_geom_max_deg=float(DEG(np.max(geom))),
        gimbal_path_rad=path,
        path_min_rad=path_min_torque,
        path_eff_torque=float(path_min_torque / path) if path > 1e-9 else float("nan"),
        chatter_tau=chat_tau, chatter_nu=chat_nu, nu_sign_flip=flips,
        frac_gamma_gt90=float(np.mean(np.any(np.abs(gamma) > np.pi / 2, axis=1))),
        H_max=m.get("H_norm_max"),
        H_at_start=float(np.linalg.norm(H[i0])),
        H_components_max=[float(np.max(np.abs(H[:, i]))) for i in range(3)],
        sigma_min_min=m.get("sigma_min_min"),
        sigma_min_man=m.get("sigma_min_maneuver_min"),
        nu_sat=m.get("nu_saturation_ratio_maneuver"),
        nu_max=float(np.max(np.abs(nu))),
        gap_mean=float(np.mean(gap_norm)),
        gap_cover=float(np.mean(cover)),
        gap_cover_rel=float(np.mean(cover) / max(np.mean(gap_norm), 1e-9)),
        cmg_max=float(np.max(np.linalg.norm(cmg, axis=1))),
        taubnd=m.get("tau_bound_used_mean"),
        tau_track=(float(np.mean(np.linalg.norm(tau_act - tau_cmd, axis=1)))
                   if exp.get("plant") == "P1" else None),
        fallback=m.get("fallback_ratio"),
        cpu_mean=m["cpu_s"]["mean"], cpu_p95=m["cpu_s"]["p95"],
        cpu_max=m["cpu_s"]["max"], iters=m["iterations"]["mean"],
        E_gap=m.get("E_gap_maneuver_mean"),
        n_steps=len(t),
    )


def fmt(v, nd=3):
    if v is None:
        return "-"
    if isinstance(v, (int, np.integer)):
        return str(int(v))
    if isinstance(v, (float, np.floating)):
        if v == 0.0:
            return "0"
        if abs(v) < 1e-3 or abs(v) >= 1e5:
            return "%.2e" % v
        return ("%%.%df" % nd) % v
    if isinstance(v, list):
        return "[" + ", ".join(fmt(x, 1) for x in v) + "]"
    return str(v)


COLS = ["case", "mode", "plant", "gamma_limit_mode", "rmse_man", "max_man",
        "settle_s", "mig_onset", "mig_lead", "gamma_deg_max", "turns",
        "frac_gamma_gt90", "path_over_net", "path_efficiency", "gimbal_path_rad",
        "path_min_rad", "path_eff_torque", "chatter_tau", "chatter_nu",
        "nu_sign_flip",
        "H_net_change", "gamma_geom_max_deg", "gamma_deg_start", "gamma_deg_final",
        "H_max", "H_at_start", "sigma_min_min", "nu_sat", "nu_max", "gap_mean",
        "gap_cover", "gap_cover_rel", "cmg_max", "taubnd", "tau_track",
        "fallback", "cpu_mean", "cpu_p95", "cpu_max", "iters", "E_gap", "n_steps"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--limited-root", default="results/torque_gap")
    ap.add_argument("--free-root", default="results/torque_gap_unlimited")
    ap.add_argument("--out", default="results/torque_gap_unlimited")
    args = ap.parse_args()
    out = (ROOT / args.out) if not Path(args.out).is_absolute() else Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    lim = {k: analyse(v) for k, v in run_dirs(ROOT / args.limited_root).items()}
    free = {k: analyse(v) for k, v in run_dirs(ROOT / args.free_root).items()}

    rows = list(free.values())
    with open(out / "analysis.csv", "w", encoding="utf-8", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(COLS)
        for r in rows:
            w.writerow([fmt(r.get(c)) for c in COLS])

    md = ["# Unlimited-gimbal analysis (raw, per run)", "",
          "Generated by `python scripts/analyze_torque_gap_unlimited.py` from "
          "`timeseries.csv`.", "",
          "`gamma_deg_max` = max |gamma| over the run; `turns` = that value in "
          "revolutions; `frac_gamma_gt90` = fraction of control steps with any "
          "|gamma| > 90 deg; `path_over_net` = total gimbal path / net rotation "
          "(>> 1 means unnecessary winding); `gap_cover_rel` = mean "
          "||tau_cmg - d|| / mean ||d|| inside the manoeuvre window (< 1 means the "
          "CMG covered most of the gap).", "",
          "| " + " | ".join(COLS) + " |",
          "|" + "|".join(["---"] * len(COLS)) + "|"]
    for r in rows:
        md.append("| " + " | ".join(fmt(r.get(c)) for c in COLS) + " |")

    md += ["", "## Paired bounded vs unlimited (same case / mode / plant)", "",
           "| case | mode | plant | rmse bounded | rmse unlimited | delta | "
           "max|gamma| bounded | max|gamma| unlimited | turns unbounded | "
           "H_max b/u | gap_cover_rel b/u | fallback b/u |",
           "|---|---|---|---|---|---|---|---|---|---|---|---|"]
    for key in sorted(set(lim) & set(free)):
        a, b = lim[key], free[key]
        rb = a["rmse_man"]
        rn = b["rmse_man"]
        md.append("| %s | %s | %s | %s | %s | %s | %s | %s | %s | %s / %s | "
                  "%s / %s | %s / %s |" % (
                      key[0], key[1], key[2], fmt(rb), fmt(rn),
                      fmt(None if (rb is None or rn is None) else rn - rb),
                      fmt(a["gamma_deg_max"], 1), fmt(b["gamma_deg_max"], 1),
                      fmt(b["turns"]), fmt(a["H_max"]), fmt(b["H_max"]),
                      fmt(a["gap_cover_rel"]), fmt(b["gap_cover_rel"]),
                      fmt(a["fallback"]), fmt(b["fallback"])))
    (out / "analysis.md").write_text("\n".join(md) + "\n", encoding="utf-8")
    print("runs: bounded %d, unlimited %d" % (len(lim), len(free)))
    print("written:", out / "analysis.md", "and analysis.csv")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
