"""方案 §8：为每个主工况生成 7 组图。

    python scripts/plot_cmg_maneuver.py                # 全部已完成工况
    python scripts/plot_cmg_maneuver.py --case M1_pitch_5deg

所有曲线都来自 `timeseries.csv`（直接记录）。唯一由模型重算的是
"总控制力矩 = M_aero + M_cmg"（图中标注 `recomputed`），其余均是记录值。
"""
from __future__ import annotations

import argparse
import json
import os
import sys

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                     # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

OUT = os.path.join(ROOT, "results", "cmg_maneuver")
FIG = os.path.join(OUT, "figures")
DT = 0.2
TOL = 1.0
FLOOR = 12.24744871391589
H_LIM = 60.0


def _series(path):
    return np.genfromtxt(path, delimiter=",", names=True)


def _load(case):
    runs = {}
    for m in ("A", "B"):
        runs[m] = _series(os.path.join(OUT, case, m, "timeseries.csv"))
    with open(os.path.join(OUT, case, "metrics_maneuver.json"), encoding="utf-8") as fh:
        ev = json.load(fh)
    return runs, ev


def plot_case(case):
    runs, ev = _load(case)
    A, B = runs["A"], runs["B"]
    man = ev["maneuver"]
    t0, Tm = man["t0"], man["Tm"]
    os.makedirs(FIG, exist_ok=True)
    out = []
    lab = {"A": "A: surfaces only", "B": "B: surfaces + CMG"}

    # 1 attitude
    f, ax = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    for i, (k, r, nm) in enumerate((("theta", "theta_ref", "pitch"),
                                    ("phi", "phi_ref", "roll"),
                                    ("psi", "psi_ref", "yaw"))):
        ax[i].plot(A["t"], A[r], "k--", lw=1.0, label="reference")
        ax[i].plot(A["t"], A[k], color="C0", lw=1.2, label=lab["A"])
        ax[i].plot(B["t"], B[k], color="C1", lw=1.2, label=lab["B"])
        ax[i].axvline(t0, color="gray", ls=":", lw=1.0)
        ax[i].axvline(t0 + Tm, color="gray", ls=":", lw=1.0)
        ax[i].set_ylabel("%s [deg]" % nm)
        ax[i].grid(alpha=0.3)
        ax[i].legend(fontsize=8)
    ax[0].set_title("%s  1  reference and realised attitude (dotted: manoeuvre window)" % case)
    ax[-1].set_xlabel("t [s]")
    f.tight_layout(); p = os.path.join(FIG, case + "_01_attitude.png")
    f.savefig(p, dpi=130); plt.close(f); out.append(p)

    # 2 error with completion markers
    f, ax = plt.subplots(figsize=(11, 5))
    for m, c in (("A", "C0"), ("B", "C1")):
        d = A if m == "A" else B
        ax.plot(d["t"], d["err_norm_deg"], color=c, lw=1.2, label=lab[m])
        ct = ev[m]["completion_time_s"]
        if ct is not None:
            ax.plot([t0 + ct], [0.0], "v", color=c, ms=9,
                    label="%s completed %.2f s" % (m, ct))
    ax.axhline(TOL, color="r", ls="--", lw=1.0, label="completion band %.1f°" % TOL)
    ax.axhline(-TOL, color="r", ls="--", lw=1.0)
    ax.axvspan(t0, t0 + Tm, color="gray", alpha=0.12, label="reference ramp")
    ax.set_yscale("symlog", linthresh=0.5)
    ax.set_xlabel("t [s]"); ax.set_ylabel("attitude error [deg]")
    ax.set_title("%s  2  tracking error, trigger and completion" % case)
    ax.grid(alpha=0.3); ax.legend(fontsize=8)
    f.tight_layout(); p = os.path.join(FIG, case + "_02_error.png")
    f.savefig(p, dpi=130); plt.close(f); out.append(p)

    # 3 surfaces + rates
    f, ax = plt.subplots(4, 1, figsize=(11, 11), sharex=True)
    lim = (25.0, 20.0, 20.0)
    for i, (k, nm) in enumerate((("de", "elevator"), ("da", "aileron"), ("dr", "rudder"))):
        ax[i].plot(A["t"], A[k], color="C0", lw=1.0, label=lab["A"])
        ax[i].plot(B["t"], B[k], color="C1", lw=1.0, label=lab["B"])
        ax[i].axhline(lim[i], color="r", ls="--", lw=0.8)
        ax[i].axhline(-lim[i], color="r", ls="--", lw=0.8)
        ax[i].set_ylabel("%s [deg]" % nm); ax[i].grid(alpha=0.3); ax[i].legend(fontsize=8)
    ax[3].plot(A["t"], A["de_dot"], color="C0", lw=0.9, label="A de_dot")
    ax[3].plot(B["t"], B["de_dot"], color="C1", lw=0.9, label="B de_dot")
    ax[3].axhline(60, color="r", ls="--", lw=0.8); ax[3].axhline(-60, color="r", ls="--", lw=0.8)
    ax[3].set_ylabel("elevator rate [deg/s]"); ax[3].set_xlabel("t [s]")
    ax[3].grid(alpha=0.3); ax[3].legend(fontsize=8)
    ax[0].set_title("%s  3  surface deflection, rate and limits" % case)
    f.tight_layout(); p = os.path.join(FIG, case + "_03_surface.png")
    f.savefig(p, dpi=130); plt.close(f); out.append(p)

    # 4 torque decomposition
    f, ax = plt.subplots(3, 1, figsize=(11, 10), sharex=True)
    for m, c, d in (("A", "C0", A), ("B", "C1", B)):
        ax[0].plot(d["t"], d["M_aero_y"], color=c, lw=1.0, label="%s aero pitch" % m)
    ax[0].set_ylabel("aero pitch moment [N m]"); ax[0].grid(alpha=0.3); ax[0].legend(fontsize=8)
    for m, c, d in (("A", "C0", A), ("B", "C1", B)):
        ax[1].plot(d["t"], d["M_cmg_y"], color=c, lw=1.0, label="%s CMG pitch" % m)
    ax[1].set_ylabel("CMG pitch moment [N m]"); ax[1].grid(alpha=0.3); ax[1].legend(fontsize=8)
    for m, c, d in (("A", "C0", A), ("B", "C1", B)):
        ax[2].plot(d["t"], d["M_aero_y"] + d["M_cmg_y"], color=c, lw=1.0,
                   label="%s total (recomputed)" % m)
    ax[2].set_ylabel("total control pitch [N m]"); ax[2].set_xlabel("t [s]")
    ax[2].grid(alpha=0.3); ax[2].legend(fontsize=8)
    ax[0].set_title("%s  4  aerodynamic, CMG and total control moment (pitch)" % case)
    f.tight_layout(); p = os.path.join(FIG, case + "_04_torque.png")
    f.savefig(p, dpi=130); plt.close(f); out.append(p)

    # 5 CMG state
    f, ax = plt.subplots(3, 1, figsize=(11, 10), sharex=True)
    ax[0].plot(B["t"], B["H_norm"], "C1", lw=1.2, label="B |H|")
    ax[0].axhline(H_LIM, color="r", ls="--", lw=1.0, label="controller budget 60")
    ax[0].axhline(107.7, color="m", ls=":", lw=1.0, label="single-axis hardware 107.7")
    ax[0].set_ylabel("|H_cmg| [N m s]"); ax[0].grid(alpha=0.3); ax[0].legend(fontsize=8)
    for i in range(1, 5):
        ax[1].plot(B["t"], B["gdot_%d" % i], lw=0.9, label="gimbal %d" % i)
    ax[1].axhline(1.5, color="r", ls="--", lw=0.8); ax[1].axhline(-1.5, color="r", ls="--", lw=0.8)
    ax[1].set_ylabel("gimbal rate [rad/s]"); ax[1].grid(alpha=0.3); ax[1].legend(fontsize=7, ncol=2)
    ax[2].plot(A["t"], A["sigma"], color="C0", lw=1.0, label=lab["A"])
    ax[2].plot(B["t"], B["sigma"], color="C1", lw=1.0, label=lab["B"])
    ax[2].axhline(FLOOR, color="r", ls="--", lw=0.9, label="floor 12.25")
    ax[2].set_yscale("log"); ax[2].set_ylabel("sigma_min"); ax[2].set_xlabel("t [s]")
    ax[2].grid(alpha=0.3); ax[2].legend(fontsize=8)
    ax[0].set_title("%s  5  CMG momentum, gimbal rate and singularity" % case)
    f.tight_layout(); p = os.path.join(FIG, case + "_05_cmg.png")
    f.savefig(p, dpi=130); plt.close(f); out.append(p)

    # 6 solver
    f, ax = plt.subplots(3, 1, figsize=(11, 10), sharex=True)
    ax[0].step(A["t"], A["fallback"], where="post", color="C0", label=lab["A"])
    ax[0].step(B["t"], B["fallback"], where="post", color="C1", label=lab["B"])
    ax[0].set_ylabel("fallback flag"); ax[0].grid(alpha=0.3); ax[0].legend(fontsize=8)
    ax[1].plot(A["t"], A["viol"], color="C0", lw=1.0, label=lab["A"])
    ax[1].plot(B["t"], B["viol"], color="C1", lw=1.0, label=lab["B"])
    ax[1].axhline(1e-3, color="r", ls="--", lw=0.9, label="fallback threshold")
    ax[1].set_yscale("symlog", linthresh=1e-6)
    ax[1].set_ylabel("constraint violation"); ax[1].grid(alpha=0.3); ax[1].legend(fontsize=8)
    ax[2].plot(A["t"], A["solve_time_ms"], color="C0", lw=1.0, label=lab["A"])
    ax[2].plot(B["t"], B["solve_time_ms"], color="C1", lw=1.0, label=lab["B"])
    ax[2].axhline(200.0, color="r", ls="--", lw=0.9, label="control period 200 ms")
    ax[2].set_yscale("log"); ax[2].set_ylabel("solve time [ms]"); ax[2].set_xlabel("t [s]")
    ax[2].grid(alpha=0.3); ax[2].legend(fontsize=8)
    ax[0].set_title("%s  6  solver status, violation and solve time" % case)
    f.tight_layout(); p = os.path.join(FIG, case + "_06_solver.png")
    f.savefig(p, dpi=130); plt.close(f); out.append(p)

    # 7 summary bars
    A_m, B_m = ev["A"], ev["B"]
    items = [("completion time [s]", A_m["completion_time_s"], B_m["completion_time_s"]),
             ("window RMSE [deg]", A_m["rmse_window_deg"], B_m["rmse_window_deg"]),
             ("peak error [deg]", A_m["peak_window_deg"], B_m["peak_window_deg"]),
             ("surface saturation [-]", A_m["delta_sat_frac_window"],
              B_m["delta_sat_frac_window"]),
             ("CMG |H| peak / 60 [-]", 0.0, B_m["H_peak_Nms"] / H_LIM)]
    f, ax = plt.subplots(figsize=(10, 5))
    x = np.arange(len(items))
    va = [(-1.0 if v is None else float(v)) for _, v, _ in items]
    vb = [(-1.0 if v is None else float(v)) for _, _, v in items]
    ax.bar(x - 0.2, va, width=0.4, color="C0", label="A: surfaces only")
    ax.bar(x + 0.2, vb, width=0.4, color="C1", label="B: surfaces + CMG")
    for i, (lbl, a, b) in enumerate(items):
        ya = 0.0 if a is None else max(float(a), 0.0) * 1.02 + 1e-3
        yb = 0.0 if b is None else max(float(b), 0.0) * 1.02 + 1e-3
        ax.text(i - 0.2, ya, "not completed" if a is None else "%.2f" % a,
                ha="center", fontsize=7, rotation=90 if a is None else 0)
        ax.text(i + 0.2, yb, "not completed" if b is None else "%.2f" % b,
                ha="center", fontsize=7, rotation=90 if b is None else 0)
    ax.set_xticks(x)
    ax.set_xticklabels([l for l, _, _ in items], fontsize=8)
    ax.set_ylabel("value (bar -1 = not completed)")
    ax.grid(alpha=0.3, axis="y")
    ax.legend(fontsize=8)
    ax.set_title("%s  7  A/B summary: %s" % (case, ev["verdict"]))
    f.tight_layout(); p = os.path.join(FIG, case + "_07_summary.png")
    f.savefig(p, dpi=130); plt.close(f); out.append(p)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--case", nargs="*", default=None)
    a = ap.parse_args(argv)
    cases = a.case or sorted(d for d in os.listdir(OUT)
                             if os.path.isdir(os.path.join(OUT, d))
                             and os.path.exists(os.path.join(OUT, d, "metrics_maneuver.json")))
    n = 0
    for c in cases:
        ps = plot_case(c)
        n += len(ps)
        print("%-18s -> %d figures" % (c, len(ps)))
    print("total %d figures in %s" % (n, FIG))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
