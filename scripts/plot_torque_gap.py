#!/usr/bin/env python
"""Core comparison figures for the torque-gap experiments.

Every figure is built from the raw `timeseries.csv` of the runs in
results/torque_gap/, and every mechanism figure marks

    - the manoeuvre window start / end (dashed vertical line), and
    - the first moment the predicted gap becomes non-zero (dotted line),

so that "the CMG migrated before the torque was needed" can be read off the
figure directly.

python scripts/plot_torque_gap.py [--cases C0,C1] [--plant P0]
"""
from __future__ import annotations

import argparse
import csv
import json
import os
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
RES = ROOT / "results" / "torque_gap"
FIG = ROOT / "figures" / "torque_gap"
AX = ["roll", "pitch", "yaw"]


def load(run_dir):
    ts = run_dir / "timeseries.csv"
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
            except (ValueError, TypeError):
                vals.append(np.nan)
        data[key] = np.array(vals)
    # text columns
    data["status"] = [r["status"] for r in rows]
    return data


def meta_of(run_dir):
    p = run_dir / "meta.json"
    return json.loads(p.read_text(encoding="utf-8")) if p.exists() else {}


def maneuver_window(meta):
    tr = meta.get("config", {}).get("trajectory3", {}) or {}
    man = tr.get("maneuver")
    man = man if isinstance(man, list) else ([man] if man else [])
    ts = []
    for m in man:
        if not m:
            continue
        t0 = float(m.get("start", 1e9))
        if t0 < 1e8 and abs(float(m.get("delta_deg", 0.0))) > 0:
            ts.append((t0, t0 + float(m.get("duration", 0.0))))
    if not ts:
        return None
    return (min(a for a, _ in ts), max(b for _, b in ts))


def gap_onset(d):
    mag = np.maximum.reduce([np.abs(d["d_x"]), np.abs(d["d_y"]), np.abs(d["d_z"])])
    idx = np.flatnonzero(mag > 1e-9)
    return float(d["t"][idx[0]]) if idx.size else None


def vertical_marks(ax, d, win, onset, label=True):
    if win:
        ax.axvline(win[0], color="0.35", ls="--", lw=1.0,
                   label="manoeuvre start" if label else None)
        ax.axvline(win[1], color="0.35", ls=":", lw=1.0,
                   label="manoeuvre end" if label else None)
    if onset is not None:
        ax.axvline(onset, color="tab:red", ls="-.", lw=1.0,
                   label="predicted gap onset (current step)" if label else None)


def plot_case(case, plant, runs):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    # --- attitude error comparison -----------------------------------
    fig, axes = plt.subplots(2, 1, figsize=(9, 6.5), sharex=True)
    for mode, d, meta in runs:
        win = maneuver_window(meta)
        vertical_marks(axes[0], d, win, gap_onset(d), label=(mode == runs[0][0]))
        axes[0].plot(d["t"], d["att_err_deg"], lw=1.4, label="%s" % mode)
        axes[1].plot(d["t"], np.rad2deg(d["d_y"]), lw=1.3, label="%s gap d_y" % mode)
    axes[0].set_ylabel("attitude error [deg]")
    axes[0].set_yscale("log")
    axes[0].legend(fontsize=8, loc="best")
    axes[0].grid(alpha=0.3)
    axes[0].set_title("%s (%s): attitude tracking, B0 vs B1" % (case, plant))
    axes[1].set_ylabel("pitch gap $d_y$ [N m]")
    axes[1].axhline(0, color="k", lw=0.6)
    axes[1].grid(alpha=0.3)
    axes[1].legend(fontsize=8)
    axes[1].set_xlabel("t [s]")
    fig.tight_layout()
    fig.savefig(FIG / ("%s_%s_attitude.png" % (case, plant)), dpi=140)
    plt.close(fig)

    # --- torque / surplus / gap / bounds for each mode ---------------
    for mode, d, meta in runs:
        win = maneuver_window(meta)
        fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
        for j, ax in enumerate(axes):
            ax.fill_between(d["t"], d["lo_%s" % "xyz"[j]], d["hi_%s" % "xyz"[j]],
                            color="0.8", label="predicted aero box")
            ax.plot(d["t"], d["tau_nom_%s" % "xyz"[j]], "b-", lw=1.0,
                    label=r"$\tau_{nom}$")
            ax.plot(d["t"], d["tauc_cmd_%s" % "xyz"[j]], "g-", lw=1.2,
                    label=r"$\tau_a$ command")
            ax.plot(d["t"], d["cmg_%s" % "xyz"[j]], "m-", lw=1.2,
                    label=r"$\tau_{cmg}=-A\nu$")
            ax.plot(d["t"], d["d_%s" % "xyz"[j]], "r--", lw=1.2, label="gap $d$")
            vertical_marks(ax, d, win, gap_onset(d), label=(j == 0))
            ax.set_ylabel(AX[j] + " [N m]")
            ax.grid(alpha=0.3)
            ax.set_yscale("symlog", linthresh=1.0)
        axes[0].legend(fontsize=7, ncol=2, loc="upper right")
        axes[-1].set_xlabel("t [s]")
        fig.suptitle("%s (%s, %s): aero torque, its bounds, CMG torque and gap"
                     % (case, plant, mode))
        fig.tight_layout()
        fig.savefig(FIG / ("%s_%s_%s_torque.png" % (case, plant, mode)), dpi=140)
        plt.close(fig)

        # --- configuration / gimbal rates / momentum ------------------
        fig, axes = plt.subplots(4, 1, figsize=(9, 9), sharex=True)
        for i in range(4):
            axes[0].plot(d["t"], np.rad2deg(d["g%d" % (i + 1)]), lw=1.1,
                         label=r"$\gamma_%d$" % (i + 1))
            axes[1].plot(d["t"], d["nu%d" % (i + 1)], lw=1.0,
                         label=r"$\nu_%d$" % (i + 1))
        axes[0].set_ylabel(r"$\gamma$ [deg]")
        axes[1].set_ylabel(r"$\nu$ [rad/s]")
        axes[0].legend(fontsize=7, ncol=4)
        axes[1].legend(fontsize=7, ncol=4)
        axes[2].plot(d["t"], d["H_norm"], lw=1.3, label=r"$|H|$")
        axes[2].set_ylabel(r"$|H|$ [N m s]")
        axes[2].legend(fontsize=8)
        axes[3].plot(d["t"], d["sigma_min"], lw=1.2)
        axes[3].set_ylabel(r"$\sigma_{min}(A)$")
        axes[3].set_xlabel("t [s]")
        for ax in axes:
            vertical_marks(ax, d, win, gap_onset(d), label=(ax is axes[0]))
            ax.grid(alpha=0.3)
        axes[2].set_yscale("symlog", linthresh=1.0)
        fig.suptitle("%s (%s, %s): CMG configuration, rates, momentum"
                     % (case, plant, mode))
        fig.tight_layout()
        fig.savefig(FIG / ("%s_%s_%s_cmg.png" % (case, plant, mode)), dpi=140)
        plt.close(fig)


def plot_migration(case, plant, runs):
    """Mechanism figure: gimbal deviation and CMG torque vs the future gap."""
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 1, figsize=(9, 6.5), sharex=True)
    for mode, d, meta in runs:
        win = maneuver_window(meta)
        g0 = np.array([d["g%d" % (i + 1)][0] for i in range(4)])
        dev = np.max(np.abs(np.column_stack([d["g%d" % (i + 1)] for i in range(4)])
                            - g0[None, :]), axis=1)
        axes[0].plot(d["t"], np.rad2deg(dev), lw=1.4, label="%s" % mode)
        axes[1].plot(d["t"], d["d_y"], lw=1.2, label="%s: gap $d_y$" % mode)
        vertical_marks(axes[0], d, win, gap_onset(d), label=(mode == runs[0][0]))
        vertical_marks(axes[1], d, win, gap_onset(d), label=False)
    axes[0].set_ylabel("max |gamma - gamma0| [deg]")
    axes[0].set_title("%s (%s): configuration migration vs the predicted gap"
                      % (case, plant))
    axes[0].legend(fontsize=8)
    axes[1].set_ylabel("pitch gap $d_y$ [N m]")
    axes[1].axhline(0, color="k", lw=0.6)
    axes[1].legend(fontsize=8)
    axes[1].set_xlabel("t [s]")
    for ax in axes:
        ax.grid(alpha=0.3)
    fig.tight_layout()
    fig.savefig(FIG / ("%s_%s_migration.png" % (case, plant)), dpi=140)
    plt.close(fig)


def plot_summary(rows):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    cases = sorted({r["case"] for r in rows})
    modes = sorted({r["mode"] for r in rows})
    fig, axes = plt.subplots(1, 3, figsize=(12, 3.8))
    w = 0.8 / max(len(modes), 1)
    for i, mode in enumerate(modes):
        vals, leads, fb = [], [], []
        for c in cases:
            hit = [r for r in rows if r["case"] == c and r["mode"] == mode]
            vals.append(hit[0]["rmse_man"] if hit else np.nan)
            leads.append(hit[0]["mig_lead"] if hit else np.nan)
            fb.append(hit[0]["fallback"] if hit else np.nan)
        xs = np.arange(len(cases)) + i * w
        axes[0].bar(xs, vals, width=w, label=mode)
        axes[1].bar(xs, leads, width=w, label=mode)
        axes[2].bar(xs, fb, width=w, label=mode)
    for ax, title in zip(axes, ["manoeuvre RMSE [deg]", "migration lead [s]",
                                "NLP fallback fraction"]):
        ax.set_xticks(np.arange(len(cases)) + 0.4 - w / 2)
        ax.set_xticklabels(cases)
        ax.set_title(title)
        ax.grid(alpha=0.3, axis="y")
    axes[0].legend(fontsize=8)
    fig.tight_layout()
    fig.savefig(FIG / "summary_matrix.png", dpi=140)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cases", default=None, help="comma separated, default all")
    ap.add_argument("--plant", default=None)
    ap.add_argument("--root", default="results/torque_gap")
    ap.add_argument("--fig-root", default="figures/torque_gap")
    args = ap.parse_args()
    global RES, FIG
    RES = (ROOT / args.root) if not Path(args.root).is_absolute() else Path(args.root)
    FIG = (ROOT / args.fig_root) if not Path(args.fig_root).is_absolute() \
        else Path(args.fig_root)
    FIG.mkdir(parents=True, exist_ok=True)
    keep = {c.strip().upper() for c in args.cases.split(",")} if args.cases else None

    groups = {}
    summary_rows = []
    for d in sorted(RES.iterdir()):
        if not d.is_dir() or not (d / "timeseries.csv").exists():
            continue
        meta = meta_of(d)
        exp = meta.get("experiment", {})
        case = meta.get("case", "")
        plant = exp.get("plant", "P0")
        mode = exp.get("mode") or "ctl"
        parts = d.name.split("_")
        tag = "_".join(parts[3:]) if len(parts) > 3 else ""
        if keep and case.upper() not in keep:
            continue
        if args.plant and plant.upper() != args.plant.upper():
            continue
        data = load(d)
        groups.setdefault((case + ("_" + tag if tag else ""), plant), []).append(
            (mode, data, meta))
        metrics = json.loads((d / "metrics.json").read_text(encoding="utf-8"))
        summary_rows.append(dict(
            case=case + ("_" + tag if tag else ""), mode=mode, plant=plant,
            rmse_man=metrics["rmse_deg"]["maneuver"] or np.nan,
            rmse_all=metrics["rmse_deg"]["all"],
            mig_lead=metrics.get("migration_lead_s") or np.nan,
            fallback=metrics.get("fallback_ratio", np.nan)))

    for (case, plant), runs in sorted(groups.items()):
        runs.sort(key=lambda r: r[0])
        plot_case(case, plant, runs)
        plot_migration(case, plant, runs)
        print("plotted", case, plant, [r[0] for r in runs])
    if summary_rows:
        plot_summary(summary_rows)
        print("plotted summary_matrix.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
