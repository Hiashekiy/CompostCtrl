#!/usr/bin/env python
"""Why does the degradation appear at the *end* of the window?  Answer with a run.

The heavy scenario is re-run with the simulation window extended from 150 s to 180 s,
same controller (HPIM RTI, 12 Gauss-Newton QPs, trust regions), same reference, same
grid.  Three things are then compared:

  1  the overlapping part (140-150 s) of the short and the long run - if the window
     end mattered, these would differ;
  2  what happens *after* 150 s in the long run - does the divergence continue?
  3  the physical driver: the dynamic pressure growth of the descending arc and the
     aerodynamic torque envelope it drags with it.

Outputs
    results/torque_gap_no_cmg_comparison_rti2/ext_window/comparison.csv
    figures/torque_gap_no_cmg_comparison/ext_window/S00_extension_180s.png

python scripts/plot_extension_180s.py
"""
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

RES = ROOT / "results" / "torque_gap_no_cmg_comparison_rti2" / "ext_window"
FIG = ROOT / "figures" / "torque_gap_no_cmg_comparison" / "ext_window"
T_WIN = 150.0          # end of the original experimental window


def load(name):
    rows = list(csv.DictReader(open(RES / name / "timeseries.csv", encoding="utf-8")))
    out = {}
    for k in rows[0]:
        if k == "status":
            continue
        try:
            out[k] = np.array([float(r[k]) if r[k] not in ("", None) else np.nan
                               for r in rows])
        except ValueError:
            pass
    out["_metrics"] = json.loads((RES / name / "metrics.json").read_text(encoding="utf-8"))
    return out


def main():
    data = {n: load(n) for n in ("A_150", "A_180", "B_150", "B_180")}

    # ---- 1. is the overlap identical? -------------------------------------
    m = lambda d: (d["t_next"] >= 140.0) & (d["t_next"] <= 149.9)
    overlap = {}
    for key in ("att_err_deg", "tau_act_y", "cmg_y", "H_norm", "nu1", "wy"):
        pairs = (("A_150", "A_180") if key in ("att_err_deg", "tau_act_y", "wy")
                 else ("B_150", "B_180"))
        a, b = data[pairs[0]], data[pairs[1]]
        if key in a and key in b:
            overlap[key] = float(np.nanmax(np.abs(a[key][m(a)] - b[key][m(b)])))

    # ---- 2. divergence after 150 s ---------------------------------------
    times = {}
    for tag, name in (("A", "A_180"), ("B", "B_180")):
        d = data[name]
        t, e = d["t_next"], d["att_err_deg"]
        row = {}
        for thr in (5, 10, 20, 40):
            idx = np.flatnonzero((e > thr) & (t > 130.0))
            row[thr] = float(t[idx[0]]) if idx.size else None
        times[tag] = row

    # ---- 3. the driver -----------------------------------------------------
    b = data["B_180"]
    rows = []
    for tt in range(135, 181, 5):
        i = int(np.argmin(np.abs(b["t_next"] - tt)))
        j = int(np.argmin(np.abs(data["A_180"]["t_next"] - tt)))
        rows.append(dict(t=float(b["t_next"][i]),
                         err_A=float(data["A_180"]["att_err_deg"][j]),
                         err_B=float(b["att_err_deg"][i]),
                         Q=float(b["Q"][i]),
                         hi_y=float(b["hi_y"][i]), lo_y=float(b["lo_y"][i]),
                         tau_nom_y=float(b["tau_nom_y"][i]),
                         tau_act_y=float(b["tau_act_y"][i]),
                         cmg_y=float(b["cmg_y"][i]),
                         H_norm=float(b["H_norm"][i]),
                         nu_sat=float(b["nu_sat"][i])))
    with open(RES / "comparison.csv", "w", encoding="utf-8", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        for r in rows:
            w.writerow({k: ("%.6g" % v) for k, v in r.items()})

    print("overlap 140-150 s, short vs extended run (max abs difference):")
    for k, v in overlap.items():
        print("   %-12s %.3e" % (k, v))
    print("\nfirst crossing of the attitude-error thresholds after 130 s:")
    for tag, row in times.items():
        print("   %s arm: %s" % (tag, ", ".join(
            ">%d deg at %s" % (k, ("%.1f s" % v) if v else "never")
            for k, v in row.items())))

    # ---- figure ------------------------------------------------------------
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["Microsoft YaHei", "SimHei", "DejaVu Sans"],
        "axes.unicode_minus": False, "mathtext.fontset": "dejavusans",
        "font.size": 9, "axes.labelsize": 9, "axes.titlesize": 9.5,
        "legend.fontsize": 7.5, "axes.linewidth": 0.8, "lines.linewidth": 1.2,
        "grid.linewidth": 0.5, "grid.alpha": 0.35, "savefig.bbox": "tight",
    })
    FIG.mkdir(parents=True, exist_ok=True)
    fig, axes = plt.subplots(3, 1, figsize=(7.6, 8.4), sharex=True,
                             gridspec_kw=dict(hspace=0.14))
    A, B = data["A_180"], data["B_180"]
    for ax in axes:
        ax.axvline(T_WIN, color="k", ls="--", lw=1.0)
        ax.grid(True)
    axes[0].text(T_WIN + 1.0, 0.95, "原实验窗口末端 $t=150$ s",
                 transform=axes[0].get_xaxis_transform(), fontsize=7.5, color="0.3",
                 va="top")
    # 1 attitude error, log scale
    axes[0].plot(A["t_next"], A["att_err_deg"], color="tab:red", lw=1.3,
                 label="A 无 CMG")
    axes[0].plot(B["t_next"], B["att_err_deg"], color="tab:blue", lw=1.3,
                 label="B 舵面+CMG")
    axes[0].set_yscale("log")
    axes[0].set_ylabel("姿态误差 [°]")
    axes[0].legend(loc="upper left", framealpha=0.9, ncol=2)
    axes[0].set_title("窗口延长到 180 s（同一控制器、同一参考）：150 s 处不是边界效应，而是散失的起点")
    # 2 the driver: Q and the aero envelope
    axes[1].plot(B["t_next"], B["Q"], color="tab:purple", lw=1.2, label="动压 $Q$ [Pa]")
    axes[1].set_ylabel("动压 $Q$ [Pa]")
    ax2 = axes[1].twinx()
    ax2.plot(B["t_next"], B["hi_y"], color="tab:orange", lw=1.2,
             label="气动抬头能力上界 $\\tau^+_y$")
    ax2.plot(B["t_next"], B["lo_y"], color="0.55", lw=1.0, ls=":",
             label="$\\tau^-_y$（已超出坐标范围，180 s 时约 −3×10³ N·m）")
    ax2.plot(B["t_next"], B["tau_nom_y"], color="tab:blue", lw=1.0, ls="--",
             label="参考需求 $\\tau_{nom,y}$")
    ax2.set_ylabel("俯仰力矩 [N·m]")
    ax2.set_ylim(-460.0, 40.0)
    h1, l1 = axes[1].get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    axes[1].legend(h1 + h2, l1 + l2, loc="lower left", framealpha=0.9, ncol=1)
    # 3 the CMG effort
    axes[2].plot(B["t_next"], B["cmg_y"], color="tab:red", lw=1.1,
                 label="$\\tau_{cmg,y}$")
    axes[2].plot(B["t_next"], B["tau_act_y"], color="tab:green", lw=1.0,
                 label="$\\tau_{a,y}$（舵面）")
    axes[2].set_ylabel("力矩 [N·m]")
    axes[2].set_xlabel("时间 $t$ [s]")
    ax3 = axes[2].twinx()
    ax3.plot(B["t_next"], B["H_norm"], color="0.3", lw=1.0, alpha=0.8,
             label="$|H|$ [N·m·s]")
    ax3.set_ylabel("$|H|$ [N·m·s]")
    axes[2].set_ylim(-260.0, 130.0)
    h1, l1 = axes[2].get_legend_handles_labels()
    h2, l2 = ax3.get_legend_handles_labels()
    axes[2].legend(h1 + h2, l1 + l2, loc="lower left", framealpha=0.9, ncol=3)
    axes[2].set_xlim(130.0, 180.0)
    fig.savefig(FIG / "S00_extension_180s.png", dpi=150)
    plt.close(fig)
    print("\nwrote", (FIG / "S00_extension_180s.png").relative_to(ROOT))
    print("wrote", (RES / "comparison.csv").relative_to(ROOT))
    return 0


if __name__ == "__main__":
    sys.exit(main())
