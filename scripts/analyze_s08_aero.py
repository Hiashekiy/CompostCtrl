#!/usr/bin/env python
"""Per-run aerodynamic / control-capability metrics of the S08 experiment.

    python scripts/analyze_s08_aero.py

Reads every finished run under results/torque_gap_s08/<group>/<run_id>/ and
writes

    results/torque_gap_s08/summary.csv     one row per run
    results/torque_gap_s08/summary.json    the same numbers plus the time stamps

Statistical windows (doc §12).  Every run reports the three of them:
    all   the whole evaluation window 90-150 s
    main  the primary load interval: the commanded manoeuvre window for the
          descent replays (A4), and the first 20 s of the window for the
          constant-condition cases (A1-A3), which have no manoeuvre
    term  the terminal interval, the last 20 s of the window

Definitions used here (all of them are measurements of the logged time series):
    rmse/max/end     attitude error |e_R| [deg]
    t_exceed_X       first time the error exceeds X deg
    err_growth       last minus first time the error is above 1 deg (a growing
                     interval shows up as a large positive value)
    alpha_mean/max   actual angle of attack [deg] (from the body velocity, never
                     theta_ref)
    qbar_mean/max    dynamic pressure [Pa]
    surf_util        |tau_a_command - tau_nat| / half width, 1.0 = the demand fills
                     the available aerodynamic control moment
    surf_sat         fraction of axes sitting on the aerodynamic box edge
    nu_sat / H / sigma_min   CMG rate saturation, cluster momentum, singularity
"""
from __future__ import annotations

import csv
import json
import math
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "results" / "torque_gap_s08"
PLANTS = ("P0", "P1")
THRESH = (1.0, 5.0, 10.0)


# --------------------------------------------------------------------------
def load_csv(path):
    with open(path, "r", encoding="utf-8", newline="") as fh:
        reader = csv.DictReader(fh)
        cols = {k: [] for k in reader.fieldnames}
        for row in reader:
            for k, v in row.items():
                cols[k].append(v)
    out = {}
    for k, vals in cols.items():
        try:
            out[k] = np.array([float(v) if v not in ("", None) else np.nan
                               for v in vals], float)
        except ValueError:
            out[k] = np.array(vals, dtype=object)
    return out


def seg_mask(ts, lo, hi):
    return (ts >= lo) & (ts <= hi)


def stats_segment(d, mask):
    if not mask.any():
        return {}
    t = d["t_next"][mask]
    err = d["att_err_deg"][mask]
    out = dict(
        n=int(mask.sum()),
        rmse=float(np.sqrt(np.mean(err ** 2))),
        max=float(np.max(err)),
        end=float(err[-1]),
        err_mean=float(np.mean(err)),
        alpha_mean=float(np.nanmean(d["alpha_deg"][mask])),
        alpha_max=float(np.nanmax(d["alpha_deg"][mask])),
        alpha_min=float(np.nanmin(d["alpha_deg"][mask])),
        qbar_mean=float(np.nanmean(d["qbar_pa"][mask])),
        qbar_max=float(np.nanmax(d["qbar_pa"][mask])),
        tau_req_y_mean=float(np.nanmean(np.abs(d["tau_req_y"][mask]))),
        tau_req_y_max=float(np.nanmax(np.abs(d["tau_req_y"][mask]))),
        surf_util_mean=float(np.nanmean(d["surf_util_y"][mask])),
        surf_util_max=float(np.nanmax(d["surf_util_y"][mask])),
        surf_sat=float(np.nanmean(d["tau_bound_used"][mask])),
        fallback=float(np.nanmean(d["fallback"][mask])),
        cpu_mean=float(np.nanmean(d["cpu_s"][mask])),
    )
    for x in THRESH:
        idx = np.flatnonzero(err > x)
        out["t_exceed_%g" % x] = float(t[idx[0]]) if idx.size else None
    above = np.flatnonzero(err > 1.0)
    out["err_growth_s"] = float(t[above[-1]] - t[above[0]]) if above.size > 1 else 0.0
    # surface / CMG capability
    if "de_act" in d and np.isfinite(d["de_act"][mask]).any():
        dmax = np.deg2rad([25.0, 20.0, 20.0])
        out["delta_pos_sat"] = float(np.nanmean(
            np.abs(d["de_act"][mask]) >= 0.999 * dmax[0]))
        out["alloc_res_y"] = float(np.nanmean(np.abs(d["alloc_res_y"][mask])))
    if "cmg_y" in d and np.isfinite(d["cmg_y"][mask]).any():
        for ax in ("x", "y", "z"):
            out["cmg_%s_peak" % ax] = float(np.nanmax(np.abs(d["cmg_%s" % ax][mask])))
        out["H_norm_max"] = float(np.nanmax(d["H_norm"][mask]))
        out["sigma_min_min"] = float(np.nanmin(d["sigma_min"][mask]))
        out["nu_sat"] = float(np.nanmean(d["nu_sat"][mask]))
        out["gap_residual_mean"] = float(np.nanmean(d["gap_residual"][mask]))
        for ax in ("x", "y", "z"):
            out["H_%s_peak" % ax] = float(np.nanmax(np.abs(d["H%s" % ax][mask])))
        # longest continuous push of the CMG about the pitch axis in one direction
        cy = d["cmg_y"][mask]
        sg = np.sign(np.where(np.abs(cy) > 1.0, cy, 0.0))
        ts = d["t_next"][mask]
        dt = float(np.median(np.diff(ts))) if len(ts) > 1 else 0.2
        best = cur = 0
        prev = 0
        for v in sg:
            if v != 0 and v == prev:
                cur += 1
            elif v != 0:
                cur = 1
            else:
                cur = 0
            prev = v
            best = max(best, cur)
        out["cmg_y_push_s"] = float(best * dt)
    return out


def sustained_time(t, flag, hold_s=1.0, frac=0.8):
    """First time a boolean signal stays on over `hold_s` (doc §8 'first sustained')."""
    flag = np.asarray(flag, float)
    flag = np.where(np.isfinite(flag), flag, 0.0) > 0.5
    if not flag.any():
        return None
    dt = float(np.median(np.diff(t))) if len(t) > 1 else 0.2
    n = max(int(round(hold_s / max(dt, 1e-6))), 1)
    if n > len(flag):
        n = len(flag)
    run = np.convolve(flag.astype(float), np.ones(n), mode="valid")
    idx = np.flatnonzero(run >= frac * n)
    return float(t[idx[0]]) if idx.size else None


def design_targets():
    """Nominal flight condition of each case (the value the case was built for)."""
    p = OUT / "design.json"
    if not p.exists():
        return {}
    d = json.loads(p.read_text(encoding="utf-8"))
    return {c["name"]: c for c in d.get("cases", [])}


def analyse_run(run_dir, targets=None):
    metrics = json.loads((run_dir / "metrics.json").read_text(encoding="utf-8"))
    meta = json.loads((run_dir / "meta.json").read_text(encoding="utf-8"))
    d = load_csv(run_dir / "timeseries.csv")
    ts = d["t_next"]
    t0, t1 = float(ts[0]), float(ts[-1])
    win = metrics.get("windows") or {}
    if "man0" in win:
        main = (win["man0"]["t0"], win["man0"]["t1"])
        main_kind = "manoeuvre"
    else:
        main = (t0, min(t0 + 20.0, t1))
        main_kind = "first20"
    term = (max(t1 - 20.0, t0), t1)
    rec = dict(
        run_id=run_dir.name, group=run_dir.parent.name,
        case=meta.get("scenario") or metrics.get("case") or run_dir.name,
        arm=meta.get("arm"), plant=metrics.get("plant"),
        seed=int(metrics.get("seed", 0)),
        cmg_present=bool(metrics.get("cmg_present")),
        nx=int(metrics.get("nx_model", 0)), nu=int(metrics.get("nu_model", 0)),
        t0=t0, t1=t1, main_kind=main_kind,
        main_window=[float(main[0]), float(main[1])],
        term_window=[float(term[0]), float(term[1])],
        mach_mean=float(np.nanmean(d["mach"])),
        knudsen_mean=float(np.nanmean(d["knudsen"])),
        qbar_over_window=float(np.nanmax(d["qbar_pa"]) / max(np.nanmin(d["qbar_pa"]), 1e-12)),
        half_y=float(np.nanmean(d["half_y"])),
        Cm_nat_mean=float(np.nanmean(d["Cm_nat"])),
        fallback_ratio=float(metrics.get("fallback_ratio", 0.0)),
        realtime_met=metrics.get("realtime_met"),
        cpu_p95=float((metrics.get("cpu_s") or {}).get("p95", float("nan"))),
        wall_s=meta.get("wall_clock_s"),
        nlp_vars=int(metrics.get("nlp_vars", 0)),
    )
    tgt = (targets or {}).get(rec["case"], {})
    if tgt:
        rec["alpha_target_deg"] = tgt.get("alpha_ref_deg")
        rec["qbar_target_pa"] = tgt.get("qbar_pa")
        rec["maneuver_deg"] = tgt.get("maneuver_deg")
    for name, m in (("all", seg_mask(ts, t0, t1)),
                    ("main", seg_mask(ts, main[0], main[1])),
                    ("term", seg_mask(ts, term[0], term[1]))):
        for k, v in stats_segment(d, m).items():
            rec["%s_%s" % (name, k)] = v
    # first *sustained* events (doc §8)
    rec["t_err_sus_1deg"] = sustained_time(ts, d["att_err_deg"] > 1.0)
    rec["t_surf_sat_sus"] = sustained_time(ts, d["tau_bound_used"] > 0.5)
    rec["t_surf_util_sus"] = sustained_time(ts, d["surf_util_y"] > 0.95)
    if "nu_sat" in d and np.isfinite(d["nu_sat"]).any():
        rec["t_nu_sat_sus"] = sustained_time(ts, d["nu_sat"] > 0.5)
        h0 = 50.0
        try:
            h0 = float((meta.get("experiment") or {}).get("c_mg", {}).get("h0", 50.0))
        except AttributeError:
            pass
        htot = 4.0 * h0
        rec["H_total"] = htot
        rec["t_H_half"] = sustained_time(ts, d["H_norm"] >= 0.5 * htot)
        rec["t_H_08"] = sustained_time(ts, d["H_norm"] >= 0.8 * htot)
        rec["t_sigma_floor"] = sustained_time(
            ts, d["sigma_min"] <= 0.30 * 0.816496580927726 * h0)
    return rec


def main():
    rows = []
    targets = design_targets()
    for group_dir in sorted(p for p in OUT.iterdir() if p.is_dir()):
        if group_dir.name in ("figures",):
            continue
        for run_dir in sorted(p for p in group_dir.iterdir() if p.is_dir()):
            if not (run_dir / "metrics.json").exists():
                continue
            rows.append(analyse_run(run_dir, targets))
    if not rows:
        print("[analyze] no finished runs found")
        return 1
    keys = []
    for r in rows:
        for k in r:
            if k not in keys:
                keys.append(k)
    order = {"A1_alpha": 0, "A2_qbar": 1, "A3_coupled": 2, "A4_descent_replay": 3,
             "smoke": 9}
    rows.sort(key=lambda r: (order.get(r["group"], 5), r["plant"], r["case"],
                             r["arm"], r["seed"]))
    with open(OUT / "summary.csv", "w", encoding="utf-8", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: ("" if isinstance(v, float) and not math.isfinite(v)
                            else (None if v is None else v)) for k, v in r.items()})
    (OUT / "summary.json").write_text(
        json.dumps(rows, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print("[analyze] %d runs -> %s" % (len(rows), (OUT / "summary.csv").relative_to(ROOT)))
    by = {}
    for r in rows:
        by.setdefault(r["group"], set()).add(r["plant"])
    for g, ps in by.items():
        print("   %-20s plants=%s runs=%d" % (g, sorted(ps),
                                              sum(1 for r in rows if r["group"] == g)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
