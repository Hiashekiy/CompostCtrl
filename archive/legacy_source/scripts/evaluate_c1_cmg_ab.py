"""Phase 6/7: metrics table, comparison figures and the machine-readable verdict
of the C1 reproduction / CMG A/B study.

    python scripts/evaluate_c1_cmg_ab.py

Inputs   results/c1_cmg_reproduction/cmg/c1/{A,B}/{timeseries.csv,metrics.json,meta.json}
         results/c1_cmg_reproduction/baseline/{verification.json,meta.json}
Outputs  results/c1_cmg_reproduction/summary.{csv,md}
         results/c1_cmg_reproduction/figures/*.png

Every number in the table comes from the run artefacts; nothing is typed in by
hand.  The judgement section applies the acceptance rules of the implementation
plan (§7.2) literally, including the ones the run fails.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                   # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
OUT = os.path.join(ROOT, "results", "c1_cmg_reproduction")
CMG = os.path.join(OUT, "cmg")
BASE = os.path.join(OUT, "baseline")
FIG = os.path.join(OUT, "figures")
CASE = "c1"
MODES = ("A", "B")

# segment windows, fixed before the runs (implementation plan §7.1)
SEGMENTS = [("pre-pulse 95-105 s", 95.0, 105.0),
            ("pulse 105-115 s", 105.0, 115.0),
            ("decay 115-125 s", 115.0, 125.0),
            ("ascent 125-140 s", 125.0, 140.0),
            ("recovery 140-200 s", 140.0, 200.0),
            ("descent 150-200 s", 150.0, 200.0)]

CORE = [
    ("err_rmse_deg", "attitude RMSE whole run [deg]", "min"),
    ("err_peak_deg", "peak attitude error [deg]", "min"),
    ("high_rmse_deg", "high-altitude (>=90 km) RMSE [deg]", "min"),
    ("high_peak_deg", "high-altitude peak error [deg]", "min"),
    ("delta_sat_frac_all", "surface saturation time fraction [-]", "min"),
    ("delta_peak_all_deg", "peak surface deflection [deg]", "info"),
    ("delta_travel_deg", "surface mean travel per step [deg]", "min"),
    ("aero_moment_peak_Nm", "peak aero moment [N m]", "info"),
    ("maneuver_settle_after_ref_s", "settle time after the reference pulse [s]", "min"),
    ("cmg_torque_peak_Nm", "CMG peak torque [N m]", "info"),
    ("cmg_momentum_peak_Nms", "CMG peak stored momentum [N m s]", "info"),
    ("cmg_gimbal_rate_peak_rad_s", "CMG peak gimbal rate [rad/s]", "info"),
    ("cmg_sigma_min", "CMG minimum singularity index [N m s]", "max"),
    ("qp_fail_count", "QP failure steps", "min"),
    ("fallback_count", "failsafe fallback steps", "min"),
    ("qp_fail_longest_run", "longest continuous fallback [steps]", "min"),
    ("max_constraint_violation", "max constraint violation", "min"),
    ("solve_time_mean_ms", "mean solve time [ms]", "min"),
    ("solve_time_p95_ms", "p95 solve time [ms]", "min"),
    ("solve_time_max_ms", "worst solve time [ms]", "min"),
]


# ---------------------------------------------------------------------------
# loading
# ---------------------------------------------------------------------------
def load_mode(mode):
    d = os.path.join(CMG, CASE, mode)
    with open(os.path.join(d, "metrics.json"), encoding="utf-8") as fh:
        metrics = json.load(fh)
    with open(os.path.join(d, "meta.json"), encoding="utf-8") as fh:
        meta = json.load(fh)
    ts = np.genfromtxt(os.path.join(d, "timeseries.csv"), delimiter=",", names=True)
    return dict(mode=mode, dir=d, metrics=metrics, meta=meta, ts=ts)


def segment_table(runs):
    rows = []
    windows = runs["A"]["meta"]["windows"]
    for name, t0, t1 in SEGMENTS:
        row = dict(segment=name, t0=t0, t1=t1)
        for m in MODES:
            t = runs[m]["ts"]["t"]
            e = runs[m]["ts"]["err_norm_deg"]
            sel = (t >= t0) & (t <= t1)
            row[m] = float(np.sqrt(np.mean(e[sel] ** 2))) if sel.any() else float("nan")
        row["change_pct"] = 100.0 * (row["B"] / row["A"] - 1.0) if row["A"] else float("nan")
        rows.append(row)
    if "high" in windows:
        t0, t1 = windows["high"]
        row = dict(segment="high-altitude window %.1f-%.1f s" % (t0, t1), t0=t0, t1=t1)
        for m in MODES:
            t = runs[m]["ts"]["t"]
            e = runs[m]["ts"]["err_norm_deg"]
            sel = (t >= t0) & (t <= t1)
            row[m] = float(np.sqrt(np.mean(e[sel] ** 2))) if sel.any() else float("nan")
        row["change_pct"] = 100.0 * (row["B"] / row["A"] - 1.0) if row["A"] else float("nan")
        rows.append(row)
    return rows


def legacy_reference():
    p = os.path.join(BASE, "verification.json")
    if not os.path.exists(p):
        alt = os.path.join(BASE, "meta.json")
        if os.path.exists(alt):
            with open(alt, encoding="utf-8") as fh:
                v = json.load(fh).get("verification", {})
            return v.get("historical"), v.get("rerun"), v
        return None, None, None
    with open(p, encoding="utf-8") as fh:
        v = json.load(fh)
    return v.get("historical"), v.get("rerun"), v


def legacy_surface_stats():
    """Surface saturation of the historical single-axis C1 (not in metrics.csv)."""
    p = os.path.join(BASE, "historical", "C1.csv")
    if not os.path.exists(p):
        return None
    d = np.genfromtxt(p, delimiter=",", names=True)
    delta = np.rad2deg(d["delta"])
    return dict(t=d["t"], delta_deg=delta, err_deg=np.rad2deg(d["err_deg"]))


# ---------------------------------------------------------------------------
# figures
# ---------------------------------------------------------------------------
def _fig(name):
    f, ax = plt.subplots(figsize=(11, 6))
    ax.grid(alpha=0.3)
    return f, ax


def make_figures(runs, segs):
    os.makedirs(FIG, exist_ok=True)
    A, B = runs["A"]["ts"], runs["B"]["ts"]
    lab = {"A": "A: surfaces only", "B": "B: surfaces + CMG"}
    written = []

    # 1 reference vs actual attitude
    f, ax = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for i, (k, r, nm) in enumerate((("theta", "theta_ref", "pitch"),
                                    ("phi", "phi_ref", "roll"),
                                    ("psi", "psi_ref", "yaw"))):
        ax[i].plot(A["t"], A[r], "k--", lw=1.0, label="reference")
        ax[i].plot(A["t"], A[k], color="C0", lw=1.2, label=lab["A"])
        ax[i].plot(B["t"], B[k], color="C1", lw=1.2, label=lab["B"])
        ax[i].set_ylabel("%s [deg]" % nm)
        ax[i].grid(alpha=0.3)
        ax[i].legend(fontsize=8, loc="best")
    ax[0].set_title("1  reference and realised attitude (C1 task, matched three-axis model)")
    ax[-1].set_xlabel("t [s]")
    f.tight_layout()
    p1 = os.path.join(FIG, "01_attitude.png")
    f.savefig(p1, dpi=130)
    plt.close(f)
    written.append(p1)

    # 2 tracking error
    f, ax = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
    ax[0].plot(A["t"], A["err_norm_deg"], color="C0", label=lab["A"])
    ax[0].plot(B["t"], B["err_norm_deg"], color="C1", label=lab["B"])
    ax[0].set_ylabel("attitude error (geodesic) [deg]")
    ax[0].axvspan(105.0, 115.0, color="gray", alpha=0.15, label="reference pulse")
    win = runs["A"]["meta"]["windows"].get("high")
    if win:
        ax[0].axvspan(win[0], win[1], color="orange", alpha=0.12,
                      label=">= 90 km window")
    ax[0].set_yscale("symlog", linthresh=1.0)
    ax[0].legend(fontsize=8)
    ax[0].grid(alpha=0.3)
    ax[1].plot(A["t"], A["err_pitch_deg"], color="C0", label="A pitch error")
    ax[1].plot(B["t"], B["err_pitch_deg"], color="C1", label="B pitch error")
    ax[1].set_ylabel("pitch error [deg]")
    ax[1].set_xlabel("t [s]")
    ax[1].grid(alpha=0.3)
    ax[1].legend(fontsize=8)
    ax[0].set_title("2  attitude tracking error")
    f.tight_layout()
    p = os.path.join(FIG, "02_error.png")
    f.savefig(p, dpi=130)
    plt.close(f)
    written.append(p)

    # 3 altitude and dynamic pressure
    f, ax = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
    ax[0].plot(A["t"], A["h"], "k")
    ax[0].set_ylabel("altitude [km]")
    if win:
        ax[0].axvspan(win[0], win[1], color="orange", alpha=0.12)
        ax[0].axhline(90.0, color="orange", ls=":", lw=1)
    ax[0].grid(alpha=0.3)
    ax[1].plot(A["t"], A["qinf"], "k")
    ax[1].set_yscale("log")
    ax[1].set_ylabel("dynamic pressure [Pa]")
    ax[1].set_xlabel("t [s]")
    ax[1].grid(alpha=0.3)
    ax[0].set_title("3  altitude and dynamic pressure (identical in A and B)")
    f.tight_layout()
    p = os.path.join(FIG, "03_altitude_qinf.png")
    f.savefig(p, dpi=130)
    plt.close(f)
    written.append(p)

    # 4 surface deflections and limits
    f, ax = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    lim = (25.0, 20.0, 20.0)
    for i, (k, nm) in enumerate((("de", "elevator"), ("da", "aileron"), ("dr", "rudder"))):
        ax[i].plot(A["t"], A[k], color="C0", lw=1.0, label=lab["A"])
        ax[i].plot(B["t"], B[k], color="C1", lw=1.0, label=lab["B"])
        ax[i].axhline(lim[i], color="r", ls="--", lw=0.8)
        ax[i].axhline(-lim[i], color="r", ls="--", lw=0.8)
        ax[i].set_ylabel("%s [deg]" % nm)
        ax[i].grid(alpha=0.3)
        ax[i].legend(fontsize=8)
    ax[0].set_title("4  surface deflection and its limit")
    ax[-1].set_xlabel("t [s]")
    f.tight_layout()
    p = os.path.join(FIG, "04_surface.png")
    f.savefig(p, dpi=130)
    plt.close(f)
    written.append(p)

    # 5 aero moment
    f, ax = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for i, k in enumerate(("M_aero_x", "M_aero_y", "M_aero_z")):
        ax[i].plot(A["t"], A[k], color="C0", lw=1.0, label=lab["A"])
        ax[i].plot(B["t"], B[k], color="C1", lw=1.0, label=lab["B"])
        ax[i].set_ylabel("%s [N m]" % k)
        ax[i].set_yscale("symlog", linthresh=1.0)
        ax[i].grid(alpha=0.3)
        ax[i].legend(fontsize=8)
    ax[0].set_title("5  aerodynamic surface moment")
    ax[-1].set_xlabel("t [s]")
    f.tight_layout()
    p = os.path.join(FIG, "05_aero_moment.png")
    f.savefig(p, dpi=130)
    plt.close(f)
    written.append(p)

    # 6 CMG moment
    f, ax = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for i, k in enumerate(("M_cmg_x", "M_cmg_y", "M_cmg_z")):
        ax[i].plot(A["t"], A[k], color="C0", lw=1.0, label=lab["A"] + " (must be 0)")
        ax[i].plot(B["t"], B[k], color="C1", lw=1.0, label=lab["B"])
        ax[i].set_ylabel("%s [N m]" % k)
        ax[i].grid(alpha=0.3)
        ax[i].legend(fontsize=8)
    ax[0].set_title("6  CMG output moment (A is exactly zero)")
    ax[-1].set_xlabel("t [s]")
    f.tight_layout()
    p = os.path.join(FIG, "06_cmg_moment.png")
    f.savefig(p, dpi=130)
    plt.close(f)
    written.append(p)

    # 7 CMG momentum
    f, ax = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
    for k in ("H_cmg_x", "H_cmg_y", "H_cmg_z"):
        ax[0].plot(B["t"], B[k], lw=1.0, label=k)
    ax[0].plot(B["t"], B["H_norm"], "k", lw=1.4, label="|H|")
    ax[0].axhline(60.0, color="r", ls="--", lw=1.0, label="controller budget 60 N m s")
    ax[0].axhline(107.7, color="m", ls=":", lw=1.0,
                  label="single-axis hardware 107.7 N m s")
    ax[0].set_ylabel("momentum [N m s]")
    ax[0].grid(alpha=0.3)
    ax[0].legend(fontsize=8)
    ax[1].plot(B["t"], B["H_norm"], "k", label=lab["B"])
    ax[1].plot(A["t"], A["H_norm"], color="C0", label=lab["A"] + " (must be 0)")
    ax[1].set_ylabel("|H_cmg| [N m s]")
    ax[1].set_xlabel("t [s]")
    ax[1].grid(alpha=0.3)
    ax[1].legend(fontsize=8)
    ax[0].set_title("7  CMG stored angular momentum")
    f.tight_layout()
    p = os.path.join(FIG, "07_cmg_momentum.png")
    f.savefig(p, dpi=130)
    plt.close(f)
    written.append(p)

    # 8 gimbal rates and singularity
    f, ax = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for i in range(1, 5):
        ax[0].plot(B["t"], B["gdot_%d" % i], lw=0.9, label="gimbal %d" % i)
    ax[0].axhline(1.5, color="r", ls="--", lw=0.8, label="rate limit 1.5 rad/s")
    ax[0].axhline(-1.5, color="r", ls="--", lw=0.8)
    ax[0].set_ylabel("gimbal rate [rad/s]")
    ax[0].grid(alpha=0.3)
    ax[0].legend(fontsize=7, ncol=2)
    for i in range(1, 5):
        ax[1].plot(B["t"], B["gimbal_%d" % i], lw=0.9, label="gimbal %d" % i)
    ax[1].set_ylabel("gimbal angle [deg]")
    ax[1].grid(alpha=0.3)
    ax[1].legend(fontsize=7, ncol=2)
    ax[2].plot(A["t"], A["sigma"], color="C0", lw=1.0, label=lab["A"])
    ax[2].plot(B["t"], B["sigma"], color="C1", lw=1.0, label=lab["B"])
    ax[2].axhline(12.25, color="r", ls="--", lw=0.9, label="singularity floor 12.25")
    ax[2].set_ylabel("sigma_min(A) [N m s]")
    ax[2].set_yscale("log")
    ax[2].set_xlabel("t [s]")
    ax[2].grid(alpha=0.3)
    ax[2].legend(fontsize=8)
    ax[0].set_title("8  CMG gimbal motion and singularity index")
    f.tight_layout()
    p = os.path.join(FIG, "08_cmg_gimbal_sigma.png")
    f.savefig(p, dpi=130)
    plt.close(f)
    written.append(p)

    # 9 solver status and computation time
    f, ax = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    ax[0].step(A["t"], A["fallback"], where="post", color="C0", label=lab["A"])
    ax[0].step(B["t"], B["fallback"], where="post", color="C1", label=lab["B"])
    ax[0].set_ylabel("fallback flag")
    ax[0].grid(alpha=0.3)
    ax[0].legend(fontsize=8)
    ax[1].plot(A["t"], A["viol"], color="C0", lw=1.0, label=lab["A"])
    ax[1].plot(B["t"], B["viol"], color="C1", lw=1.0, label=lab["B"])
    ax[1].set_yscale("symlog", linthresh=1e-6)
    ax[1].set_ylabel("max constraint violation")
    ax[1].grid(alpha=0.3)
    ax[1].legend(fontsize=8)
    ax[2].plot(A["t"], A["solve_time_ms"], color="C0", lw=1.0, label=lab["A"])
    ax[2].plot(B["t"], B["solve_time_ms"], color="C1", lw=1.0, label=lab["B"])
    ax[2].axhline(200.0, color="r", ls="--", lw=0.9, label="control period 200 ms")
    ax[2].set_ylabel("solve time [ms]")
    ax[2].set_yscale("log")
    ax[2].set_xlabel("t [s]")
    ax[2].grid(alpha=0.3)
    ax[2].legend(fontsize=8)
    ax[0].set_title("9  MPC solver status and computation time")
    f.tight_layout()
    p = os.path.join(FIG, "09_solver.png")
    f.savefig(p, dpi=130)
    plt.close(f)
    written.append(p)

    # legacy reference for the same numeric target
    leg = legacy_surface_stats()
    if leg is not None:
        f, ax = plt.subplots(1, 1, figsize=(11, 4.5))
        ax.plot(leg["t"], leg["err_deg"], color="k", lw=1.0,
                label="legacy C1 (single-axis, historical)")
        ax.plot(A["t"], A["err_pitch_deg"], color="C0", lw=1.0,
                label="A: matched three-axis, pitch error")
        ax.set_xlabel("t [s]")
        ax.set_ylabel("pitch error [deg]")
        ax.grid(alpha=0.3)
        ax.legend(fontsize=8)
        ax.set_title("10  historical single-axis C1 vs matched-model A")
        f.tight_layout()
        p = os.path.join(FIG, "10_legacy_vs_matched_A.png")
        f.savefig(p, dpi=130)
        plt.close(f)
        written.append(p)
    return written


# ---------------------------------------------------------------------------
# verdict (§7.2)
# ---------------------------------------------------------------------------
def verdict(runs, segs):
    A, B = runs["A"]["metrics"], runs["B"]["metrics"]
    rmse_a, rmse_b = A["err_rmse_deg"], B["err_rmse_deg"]
    improve = 100.0 * (1.0 - rmse_b / rmse_a) if rmse_a else float("nan")
    high = [s for s in segs if s["segment"].startswith("high-altitude")]
    checks = []

    def add(name, ok, value, rule):
        checks.append(dict(check=name, ok=bool(ok), value=value, rule=rule))

    add("B reduces the whole-run RMSE below A", rmse_b < rmse_a, [rmse_a, rmse_b],
        "B < A")
    add("B reduces the whole-run RMSE by at least 20%", improve >= 20.0, improve,
        "improvement >= 20 %")
    add("B reduces the peak attitude error", B["err_peak_deg"] < A["err_peak_deg"],
        [A["err_peak_deg"], B["err_peak_deg"]], "B < A")
    if high:
        add("B reduces the high-altitude RMSE", high[0]["B"] < high[0]["A"],
            [high[0]["A"], high[0]["B"]], "B < A")
    add("B reduces the surface saturation fraction",
        B["delta_sat_frac_all"] < A["delta_sat_frac_all"],
        [A["delta_sat_frac_all"], B["delta_sat_frac_all"]], "B < A")
    longest = int(B.get("fallback_longest_run", B["qp_fail_longest_run"]))
    add("no sustained QP failure in B", longest < 5, longest,
        "longest continuous fallback run < 5 steps (1 s)")
    add("B keeps the momentum inside the controller budget",
        B["cmg_momentum_peak_Nms"] <= 60.0 + 1e-6, B["cmg_momentum_peak_Nms"],
        "<= 60 N m s")
    add("B keeps the singularity index above the floor",
        B["cmg_sigma_min"] > 12.25, B["cmg_sigma_min"], "> 12.25 N m s")
    add("worst solve time is reported against the 200 ms control period",
        True, [A["solve_time_max_ms"], B["solve_time_max_ms"]], "reported, not judged")
    return dict(improvement_pct=improve, checks=checks,
                any_hard_failure=any(not c["ok"] for c in checks
                                     if c["rule"] != "reported, not judged"))


# ---------------------------------------------------------------------------
# tables
# ---------------------------------------------------------------------------
def write_tables(runs, segs, fig_paths, legacy):
    A, B = runs["A"]["metrics"], runs["B"]["metrics"]
    rows = []
    for key, label, better in CORE:
        va, vb = A.get(key), B.get(key)
        rows.append(dict(metric=key, label=label,
                         A=va if not isinstance(va, list) else json.dumps(va),
                         B=vb if not isinstance(vb, list) else json.dumps(vb),
                         better=better))
    with open(os.path.join(OUT, "summary.csv"), "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=["metric", "label", "A", "B", "better"])
        w.writeheader()
        for r in rows:
            w.writerow(r)
    with open(os.path.join(OUT, "segments.csv"), "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=["segment", "t0", "t1", "A", "B", "change_pct"])
        w.writeheader()
        for r in segs:
            w.writerow(r)

    v = verdict(runs, segs)
    hist, rerun, _ = legacy
    L = []
    L.append("# C1 reproduction + CMG unified-MPC A/B - generated summary\n")
    L.append("Generated by `scripts/evaluate_c1_cmg_ab.py` from the run artefacts; "
             "no number below is typed in by hand.\n")

    L.append("## Provenance\n")
    L.append("| item | A | B |")
    L.append("|---|---|---|")
    for k in ("case", "tag", "actuator_mode", "code_commit", "package_version",
              "python", "wall_time_s"):
        L.append("| %s | `%s` | `%s` |"
                 % (k, runs["A"]["meta"].get(k), runs["B"]["meta"].get(k)))
    L.append("| high-altitude window | %s | %s |"
             % (runs["A"]["meta"]["windows"].get("high"),
                runs["B"]["meta"]["windows"].get("high")))
    L.append("")

    if hist:
        L.append("## Historical C1 (single-axis, frozen) used as the reproduction target\n")
        L.append("| metric | committed value | re-run in this environment |")
        L.append("|---|---|---|")
        for k in hist:
            L.append("| %s | %.6g | %.6g |" % (k, hist[k], rerun.get(k, float("nan"))))
        L.append("")

    L.append("## Core metrics (A vs B, same task, same reference, same disturbance)\n")
    L.append("| metric | A (surfaces only) | B (surfaces + CMG) | change | better |")
    L.append("|---|---|---|---|---|")
    for r in rows:
        va, vb = A.get(r["metric"]), B.get(r["metric"])
        if isinstance(va, (int, float)) and isinstance(vb, (int, float)):
            with np.errstate(divide="ignore", invalid="ignore"):
                ratio = (vb / va) if va else float("inf")
            chg = ("%+.2f %%" % (100.0 * (ratio - 1.0))
                   if np.isfinite(ratio) and abs(ratio) < 1e4 else "n/a")
            L.append("| %s | %.6g | %.6g | %s | %s |"
                     % (r["label"], va, vb, chg, r["better"]))
        else:
            L.append("| %s | %s | %s | - | %s |"
                     % (r["label"], r["A"], r["B"], r["better"]))
    L.append("")

    L.append("## Window RMSE [deg]\n")
    L.append("| window | t0 [s] | t1 [s] | A | B | change |")
    L.append("|---|---|---|---|---|---|")
    for r in segs:
        L.append("| %s | %.1f | %.1f | %.3f | %.3f | %+.1f %% |"
                 % (r["segment"], r["t0"], r["t1"], r["A"], r["B"], r["change_pct"]))
    L.append("")

    L.append("## Judgement against the pre-registered rules (§7.2)\n")
    L.append("| requirement | rule | result | value |")
    L.append("|---|---|---|---|")
    for c in v["checks"]:
        L.append("| %s | %s | %s | %s |"
                 % (c["check"], c["rule"], "PASS" if c["ok"] else "FAIL", c["value"]))
    L.append("")
    L.append("Whole-run RMSE improvement of B over A: **%+.2f %%**.\n" % v["improvement_pct"])
    L.append("Figures: " + ", ".join("`%s`" % os.path.relpath(p, OUT) for p in fig_paths) + "\n")
    with open(os.path.join(OUT, "summary.md"), "w", encoding="utf-8") as fh:
        fh.write("\n".join(L) + "\n")
    return v


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--no-figures", action="store_true")
    a = ap.parse_args(argv)

    runs = {m: load_mode(m) for m in MODES}
    segs = segment_table(runs)
    legacy = legacy_reference()
    figs = [] if a.no_figures else make_figures(runs, segs)
    v = write_tables(runs, segs, figs, legacy)

    with open(os.path.join(OUT, "verdict.json"), "w", encoding="utf-8") as fh:
        json.dump(v, fh, indent=2, ensure_ascii=False)

    print("RMSE  A=%.3f deg  B=%.3f deg  (%+.1f %%)"
          % (runs["A"]["metrics"]["err_rmse_deg"], runs["B"]["metrics"]["err_rmse_deg"],
             v["improvement_pct"]))
    for c in v["checks"]:
        print("  %-56s %s  %s" % (c["check"], "PASS" if c["ok"] else "FAIL", c["value"]))
    print("wrote", os.path.join(OUT, "summary.md"), "+ figures:", len(figs))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
