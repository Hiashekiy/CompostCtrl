#!/usr/bin/env python
"""Paired comparison figures: +-90 deg travel arm vs unlimited (continuous) arm.

Reads both result roots (`results/torque_gap` = bounded, and
`results/torque_gap_unlimited` = continuous rotation) and writes paired figures to
`figures/torque_gap_unlimited/`:

  cmp_<case>_<mode>_gimbal.png    four gimbal angles, both arms, with the +-90 deg
                                 lines and the turn count (how many 360 deg wraps)
  cmp_<case>_<mode>_attitude.png  attitude error, both arms
  cmp_<case>_<mode>_momentum.png  |H|, H components, sigma_min, gimbal-rate saturation
  cmp_summary_rmse.png            maneuver-window RMSE per case/mode for both arms

python scripts/plot_torque_gap_unlimited.py
"""
from __future__ import annotations

import argparse
import csv
import json
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
LIMITED = ROOT / "results" / "torque_gap"
FREE = ROOT / "results" / "torque_gap_unlimited"
FIG = ROOT / "figures" / "torque_gap_unlimited"
AX = ["x", "y", "z"]


def load_series(run_dir):
    ts = Path(run_dir) / "timeseries.csv"
    if not ts.exists():
        return None
    with open(ts, encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    data = {}
    for key in rows[0]:
        vals = []
        for r in rows:
            try:
                vals.append(float(r[key]))
            except (TypeError, ValueError):
                vals.append(np.nan)
        data[key] = np.array(vals)
    return data


def run_map(root):
    out = {}
    for d in sorted(Path(root).iterdir()):
        if not d.is_dir():
            continue
        m = d / "metrics.json"
        meta = d / "meta.json"
        if not (m.exists() and meta.exists()):
            continue
        mt = json.loads(meta.read_text(encoding="utf-8"))
        exp = mt.get("experiment", {})
        key = (mt.get("case", ""), exp.get("mode", ""), exp.get("plant", "P0"))
        out[key] = dict(dir=d, meta=mt,
                        metrics=json.loads(m.read_text(encoding="utf-8")))
    return out


def turns(gamma):
    """Number of completed 360 deg wraps (absolute, unwrapped angles)."""
    return float(np.max(np.abs(gamma)) / (2.0 * np.pi))


def plot_pair(key, lim, free):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    a, b = load_series(lim["dir"]), load_series(free["dir"])
    if a is None or b is None:
        return
    case, mode, plant = key
    tag = "%s_%s_%s" % (case, mode, plant)

    # ---- gimbal angles + turn count ---------------------------------
    fig, axes = plt.subplots(4, 1, figsize=(9, 9), sharex=True)
    for i, ax in enumerate(axes):
        ax.plot(a["t"], np.rad2deg(a["g%d" % (i + 1)]), color="tab:blue", lw=1.2,
                label="bounded +-90 deg")
        ax.plot(b["t"], np.rad2deg(b["g%d" % (i + 1)]), color="tab:red", lw=1.2,
                label="unlimited (unwrapped)")
        for yy in (-90.0, 90.0):
            ax.axhline(yy, color="0.6", ls="--", lw=0.8)
        ax.set_ylabel(r"$\gamma_%d$ [deg]" % (i + 1))
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7, loc="upper left")
    axes[-1].set_xlabel("t [s]")
    tl = turns(np.column_stack([a["g%d" % (i + 1)] for i in range(4)]))
    tf = turns(np.column_stack([b["g%d" % (i + 1)] for i in range(4)]))
    fig.suptitle("%s / %s / %s: gimbal angle (turns: bounded %.2f, unlimited %.2f)"
                 % (case, mode, plant, tl, tf))
    fig.tight_layout()
    fig.savefig(FIG / ("cmp_%s_gimbal.png" % tag), dpi=140)
    plt.close(fig)

    # ---- attitude error ---------------------------------------------
    fig, axes = plt.subplots(2, 1, figsize=(9, 5.6), sharex=True)
    axes[0].plot(a["t"], a["att_err_deg"], lw=1.3, label="bounded")
    axes[0].plot(b["t"], b["att_err_deg"], lw=1.3, label="unlimited")
    axes[0].set_yscale("log")
    axes[0].set_ylabel("attitude error [deg]")
    axes[0].legend(fontsize=8)
    axes[1].plot(a["t"], a["d_y"], lw=1.2, label="bounded: gap $d_y$")
    axes[1].plot(b["t"], b["d_y"], lw=1.2, label="unlimited: gap $d_y$")
    axes[1].axhline(0.0, color="k", lw=0.6)
    axes[1].set_ylabel("pitch gap [N m]")
    axes[1].set_xlabel("t [s]")
    axes[1].legend(fontsize=8)
    for ax in axes:
        ax.grid(alpha=0.3)
    axes[0].set_title("%s / %s / %s: attitude tracking and predicted gap" % (case, mode, plant))
    fig.tight_layout()
    fig.savefig(FIG / ("cmp_%s_attitude.png" % tag), dpi=140)
    plt.close(fig)

    # ---- momentum / singularity / rate saturation --------------------
    fig, axes = plt.subplots(4, 1, figsize=(9, 9), sharex=True)
    for i, ax in enumerate(axes[:3]):
        ax.plot(a["t"], a["H" + AX[i]], lw=1.2, label="bounded")
        ax.plot(b["t"], b["H" + AX[i]], lw=1.2, label="unlimited")
        ax.set_ylabel("H_%s [N m s]" % AX[i])
        ax.grid(alpha=0.3)
        ax.legend(fontsize=7)
    axes[0].set_title("%s / %s / %s: CMG momentum components" % (case, mode, plant))
    axes[3].plot(a["t"], a["H_norm"], lw=1.3, label="|H| bounded")
    axes[3].plot(b["t"], b["H_norm"], lw=1.3, label="|H| unlimited")
    ax2 = axes[3].twinx()
    ax2.plot(a["t"], a["sigma_min"], lw=0.9, ls=":", color="0.4", label="sigma_min bounded")
    ax2.plot(b["t"], b["sigma_min"], lw=0.9, ls=":", color="tab:green",
             label="sigma_min unlimited")
    ax2.set_ylabel(r"$\sigma_{min}(A)$")
    axes[3].set_ylabel("|H| [N m s]")
    axes[3].set_xlabel("t [s]")
    h1, l1 = axes[3].get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    axes[3].legend(h1 + h2, l1 + l2, fontsize=7)
    axes[3].grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(FIG / ("cmp_%s_momentum.png" % tag), dpi=140)
    plt.close(fig)


def plot_summary(lim_map, free_map):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    keys = sorted({k for k in free_map if k[2] == "P0"} & set(lim_map))
    if not keys:
        return
    cases = sorted({k[0] for k in keys})
    modes = sorted({k[1] for k in keys})
    fig, axes = plt.subplots(1, 2, figsize=(11, 3.8))
    w = 0.8 / max(len(modes), 1)
    for i, mode in enumerate(modes):
        xs, rl, rf = [], [], []
        for j, case in enumerate(cases):
            a = lim_map.get((case, mode, "P0"))
            b = free_map.get((case, mode, "P0"))
            if not (a and b):
                continue
            xs.append(j + i * w)
            rl.append(a["metrics"]["rmse_deg"]["maneuver"] or np.nan)
            rf.append(b["metrics"]["rmse_deg"]["maneuver"] or np.nan)
        axes[0].bar(np.array(xs) - w / 2, rl, width=w / 2, label="%s bounded" % mode)
        axes[0].bar(np.array(xs) + w / 2, rf, width=w / 2, label="%s unlimited" % mode)
    axes[0].set_xticks(np.arange(len(cases)) + 0.4 - w / 2)
    axes[0].set_xticklabels(cases)
    axes[0].set_ylabel("manoeuvre RMSE [deg]")
    axes[0].set_title("attitude tracking")
    axes[0].grid(alpha=0.3, axis="y")
    axes[0].legend(fontsize=7, ncol=2)
    for i, mode in enumerate(modes):
        fr = []
        for case in cases:
            b = free_map.get((case, mode, "P0"))
            fr.append(b["metrics"].get("fallback_ratio", np.nan) if b else np.nan)
        axes[1].bar(np.arange(len(cases)) + i * w, fr, width=w,
                    label="%s unlimited" % mode)
    axes[1].set_xticks(np.arange(len(cases)) + 0.4 - w / 2)
    axes[1].set_xticklabels(cases)
    axes[1].set_ylabel("NLP fallback fraction")
    axes[1].set_title("solver usage (unlimited arm)")
    axes[1].grid(alpha=0.3, axis="y")
    axes[1].legend(fontsize=7)
    fig.suptitle("Paired comparison: bounded +-90 deg vs continuous rotation")
    fig.tight_layout()
    fig.savefig(FIG / "cmp_summary_rmse.png", dpi=140)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--limited-root", default="results/torque_gap")
    ap.add_argument("--free-root", default="results/torque_gap_unlimited")
    ap.add_argument("--fig-root", default="figures/torque_gap_unlimited")
    ap.add_argument("--cases", default=None)
    args = ap.parse_args()
    global FIG
    FIG = (ROOT / args.fig_root) if not Path(args.fig_root).is_absolute() \
        else Path(args.fig_root)
    FIG.mkdir(parents=True, exist_ok=True)
    lim_map = run_map(ROOT / args.limited_root)
    free_map = run_map(ROOT / args.free_root)
    keep = {c.strip().upper() for c in args.cases.split(",")} if args.cases else None
    n = 0
    for key in sorted(set(free_map) & set(lim_map)):
        if keep and key[0].upper() not in keep:
            continue
        plot_pair(key, lim_map[key], free_map[key])
        n += 1
    plot_summary(lim_map, free_map)
    print("paired figures:", n, "->", FIG)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
