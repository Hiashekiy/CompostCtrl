"""Figures for the device-replacement sweep (RW -> CMG).

    python scripts/plot_device_sweep.py [--cases cseries tseries]

One figure per case, 5 panels overlaid for all arms of that case:

    1  quaternion geodesic attitude error  ‖e‖
    2  per-axis error (pitch and roll/yaw of the driven axes)
    3  elevator / aileron deflection with the limits drawn
    4  momentum-exchange device: stored momentum norm
    5  momentum-exchange device: applied body moment

House style: time in seconds, angles in degrees, moments in N·m, momentum in
N·m·s; the wheel arms are dashed, the CMG arms solid, "no device" in black.
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
import matplotlib.pyplot as plt                        # noqa: E402

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "Noto Sans SC",
                                   "DengXian", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RES = os.path.join(ROOT, "results", "cmg_ab", "device_sweep")
FIG = os.path.join(ROOT, "figures", "cmg_ab")

# arm -> (colour, linestyle, short legend label)
STYLE = {
    "C0_rw":    ("#7f7f7f", (0, (1, 1)), "C0 理想轮"),
    "C0_cmg":   ("#2ca02c", (0, (1, 1)), "C0' 理想 CMG"),
    "C1":       ("#000000", "-", "C1/T1 仅舵面"),
    "T1":       ("#000000", "-", "C1/T1 仅舵面"),
    "C2_cmg":   ("#ff7f0e", "-", "C2' 分层+CMG"),
    "C3_rw":    ("#1f77b4", "--", "C3/T3 统一 MPC+轮"),
    "T3_rw":    ("#1f77b4", "--", "C3/T3 统一 MPC+轮"),
    "C3_cmg":   ("#d62728", "-", "C3'/T3' 统一 MPC+CMG"),
    "T3_cmg":   ("#d62728", "-", "C3'/T3' 统一 MPC+CMG"),
    "C4_cmg":   ("#9467bd", "-.", "C4' 统一 MPC+CMG（失配 20%）"),
}


def load(case, arm):
    p = os.path.join(RES, case, arm, "timeseries.csv")
    if not os.path.exists(p):
        return None, None
    with open(p, newline="", encoding="utf-8") as fh:
        r = csv.reader(fh)
        cols = next(r)
        arr = np.array([[float(v) for v in row] for row in r], dtype=float)
    rec = {c: arr[:, i] for i, c in enumerate(cols)}
    mp = os.path.join(RES, case, arm, "metrics.json")
    meta = {}
    if os.path.exists(mp):
        with open(mp, encoding="utf-8") as fh:
            meta = json.load(fh)
    return rec, meta


def figure(case, arms_list):
    data = {}
    for a in arms_list:
        rec, meta = load(case, a)
        if rec is not None:
            data[a] = (rec, meta)
    if not data:
        print("no data for", case)
        return None
    dmax = 25.0
    fig, axs = plt.subplots(5, 1, figsize=(11, 13), sharex=True)

    ax = axs[0]
    for a, (rec, m) in data.items():
        c, ls, lab = STYLE.get(a, ("k", "-", a))
        ax.plot(rec["t"], rec["err_norm_deg"], color=c, ls=ls, lw=1.3,
                label="%s  RMSE %.4f°" % (lab, m["err_rmse_deg"]))
    ax.set_ylabel("‖e‖ [°]")
    ax.set_title("%s：四元数测地姿态误差（图例给出全程 RMSE）" % case, fontsize=10)
    ax.legend(fontsize=6.5, ncol=2, frameon=False)
    ax.grid(alpha=0.3)

    ax = axs[1]
    for a, (rec, m) in data.items():
        c, ls, lab = STYLE.get(a, ("k", "-", a))
        ax.plot(rec["t"], rec["err_pitch_deg"], color=c, ls=ls, lw=1.2,
                label="%s 俯仰" % lab if "CG" in a or a in ("C1", "T1") else None)
    ax.set_ylabel("俯仰误差 [°]")
    ax.set_title("俯仰轴误差（欧拉角差）", fontsize=10)
    ax.legend(fontsize=6.5, ncol=2, frameon=False)
    ax.grid(alpha=0.3)

    ax = axs[2]
    for a, (rec, m) in data.items():
        c, ls, lab = STYLE.get(a, ("k", "-", a))
        ax.plot(rec["t"], rec["de"], color=c, ls=ls, lw=1.1)
    ax.axhline(dmax, color="k", lw=0.7, ls=":")
    ax.axhline(-dmax, color="k", lw=0.7, ls=":")
    ax.text(0.99, 0.92, "点线 = 升降舵限幅 ±%.0f°" % dmax, transform=ax.transAxes,
            ha="right", fontsize=7)
    ax.set_ylabel("δe [°]")
    ax.set_title("升降舵偏角", fontsize=10)
    ax.grid(alpha=0.3)

    ax = axs[3]
    for a, (rec, m) in data.items():
        c, ls, lab = STYLE.get(a, ("k", "-", a))
        ax.plot(rec["t"], rec["dev_H_norm"], color=c, ls=ls, lw=1.3,
                label="%s  max %.1f N·m·s" % (lab, m["cmg_momentum_peak_Nms"]))
    ax.set_ylabel("‖H_dev‖ [N·m·s]")
    ax.set_title("动量交换装置储存的角动量范数（上限：反作用轮 3 轴各 366.5 → 范数 634.7；"
                 "CMG 控制器预算 60、硬件集群 200）", fontsize=9)
    ax.legend(fontsize=6.5, ncol=2, frameon=False)
    ax.grid(alpha=0.3)

    ax = axs[4]
    for a, (rec, m) in data.items():
        c, ls, lab = STYLE.get(a, ("k", "-", a))
        ax.plot(rec["t"], rec["dev_M_y"], color=c, ls=ls, lw=1.1)
    ax.set_ylabel("装置力矩 My [N·m]")
    ax.set_xlabel("时间 [s]")
    ax.set_title("装备施加在俯仰轴上的力矩（反作用轮 = −τ，CMG = −A·gdot）", fontsize=10)
    ax.grid(alpha=0.3)

    fig.tight_layout()
    p = os.path.join(FIG, "device_%s.png" % case)
    fig.savefig(p, dpi=130)
    plt.close(fig)
    return p


def bar_figure(cases):
    """RMSE / peak / surface saturation for every arm of every case."""
    rows = []
    for case in cases:
        d = os.path.join(RES, case)
        if not os.path.isdir(d):
            continue
        for a in sorted(os.listdir(d)):
            mp = os.path.join(d, a, "metrics.json")
            if os.path.isfile(mp):
                with open(mp, encoding="utf-8") as fh:
                    rows.append((case, a, json.load(fh)))
    if not rows:
        return None
    fields = [("err_rmse_deg", "三轴 RMSE [°]"), ("err_peak_deg", "峰值误差 [°]"),
              ("delta_sat_frac_all", "舵面饱和比例"),
              ("cmg_momentum_peak_Nms", "装置峰值角动量 [N·m·s]")]
    fig, axs = plt.subplots(1, len(fields), figsize=(4.2 * len(fields), 4.4))
    cases_u = sorted({r[0] for r in rows})
    for ax, (key, label) in zip(np.atleast_1d(axs), fields):
        labels, vals, cols = [], [], []
        for case in cases_u:
            for c_, a, m in rows:
                if c_ != case:
                    continue
                labels.append("%s\n%s" % (case, a))
                vals.append(m.get(key) or 0.0)
                cols.append(STYLE.get(a, ("k", "-", ""))[0])
        x = np.arange(len(labels))
        ax.bar(x, vals, 0.65, color=cols)
        ax.set_xticks(x)
        ax.set_xticklabels(labels, rotation=90, fontsize=5.5)
        ax.set_title(label, fontsize=9)
        ax.grid(axis="y", alpha=0.3)
    fig.tight_layout()
    p = os.path.join(FIG, "device_sweep_overview.png")
    fig.savefig(p, dpi=130)
    plt.close(fig)
    return p


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--cases", nargs="*", default=None)
    a = ap.parse_args(argv)
    os.makedirs(FIG, exist_ok=True)
    sys.path.insert(0, HERE)
    from run_device_sweep import load_sweep
    _, blocks = load_sweep()
    arm_map = blocks.get("arms", {})
    cases = a.cases or sorted(arm_map)
    for c in cases:
        print("wrote", figure(c, arm_map.get(c, [])))
    print("wrote", bar_figure(cases))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
