#!/usr/bin/env python
"""Compare complete A/B experiment roots that used different MPC backends.

Each root is a full 34-run matrix produced by scripts/run_no_cmg_matrix.py:
    baseline   results/torque_gap_no_cmg_comparison        IPOPT, original settings
    fast       results/torque_gap_no_cmg_comparison_fast   IPOPT, Tier-1 profile
    rti        results/torque_gap_no_cmg_comparison_rti    HPIPM RTI, 12 GN QPs

Outputs
    results/torque_gap_mpc_speed/backends.csv / backends.md
    figures/torque_gap_mpc_speed/backend_speed.png
    figures/torque_gap_mpc_speed/backend_timeseries_<scenario>.png

Only runs that exist in a root are reported; nothing is interpolated or
substituted, and P0/P1 are never mixed.

python scripts/compare_mpc_backends.py
"""
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

OUT = ROOT / "results" / "torque_gap_mpc_speed"
FIG = ROOT / "figures" / "torque_gap_mpc_speed"
ROOTS = [
    ("baseline (IPOPT, original settings)", "results/torque_gap_no_cmg_comparison"),
    ("fast (IPOPT, Tier 1)", "results/torque_gap_no_cmg_comparison_fast"),
    ("rti (HPIPM, 12 GN QPs)", "results/torque_gap_no_cmg_comparison_rti"),
    ("rti2 (HPIPM + trust regions)", "results/torque_gap_no_cmg_comparison_rti2"),
]
SCEN = ["S00_pitch30", "S01_pitch_minus30", "S02_pitch10", "S03_pitch45",
        "S04_roll30", "S05_yaw30", "S06_pitch30_roll20", "S07_sequential"]
ARMS = ["no_cmg", "with_cmg"]

COLS = ["backend", "phase", "scenario", "arm", "seed", "rmse_man_deg", "rmse_all_deg",
        "max_err_man_deg", "cpu_mean_s", "cpu_p95_s", "cpu_max_s", "iters_mean",
        "ms_per_iter", "fallback_ratio", "accepted_ratio", "realtime_met",
        "max_violation", "tau_rms_x", "tau_rms_y", "tau_rms_z"]


def load(root):
    out = {}
    for phase in ("P0", "P1"):
        for d in sorted((ROOT / root / phase).glob("*")):
            mp = d / "metrics.json"
            if not mp.exists():
                continue
            m = json.loads(mp.read_text(encoding="utf-8"))
            meta = json.loads((d / "meta.json").read_text(encoding="utf-8"))
            st = (m.get("stats") or {}).get("combined") or {}
            key = (phase, meta.get("scenario"), meta.get("arm"),
                   int(m.get("seed", 0)), root)
            out[(d.parent.name, meta.get("scenario"), meta.get("arm"),
                 int(m.get("seed", 0)))] = dict(
                rmse_man=m["rmse_deg"].get("combined"),
                rmse_all=m["rmse_deg"].get("all"),
                max_err=st.get("max_err_deg"),
                cpu_mean=m["cpu_s"]["mean"], cpu_p95=m["cpu_s"]["p95"],
                cpu_max=m["cpu_s"]["max"],
                iters=m["iterations"]["mean"],
                ms_iter=m.get("ms_per_iteration"),
                fb=m["fallback_ratio"], acc=m.get("accepted_ratio", 0.0),
                rt=m.get("realtime_met"), viol=m["max_violation"],
                tau=st.get("tau_rms"), dir=str(d))
    return out


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
    OUT.mkdir(parents=True, exist_ok=True)
    data = [(label, load(path)) for label, path in ROOTS]
    rows = []
    for label, d in data:
        back = label.split(" ")[0]
        for (phase, sc, arm, seed), v in sorted(d.items()):
            row = dict(backend=back, phase=phase, scenario=sc, arm=arm, seed=seed)
            row.update(rmse_man_deg=v["rmse_man"], rmse_all_deg=v["rmse_all"],
                       max_err_man_deg=v["max_err"], cpu_mean_s=v["cpu_mean"],
                       cpu_p95_s=v["cpu_p95"], cpu_max_s=v["cpu_max"],
                       iters_mean=v["iters"], ms_per_iter=v["ms_iter"],
                       fallback_ratio=v["fb"], accepted_ratio=v["acc"],
                       realtime_met=v["rt"], max_violation=v["viol"])
            if v["tau"]:
                row.update(tau_rms_x=v["tau"][0], tau_rms_y=v["tau"][1],
                           tau_rms_z=v["tau"][2])
            rows.append(row)
    with open(OUT / "backends.csv", "w", encoding="utf-8", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=COLS, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)

    lines = ["# MPC backend comparison (same experiment, three backends)", "",
             "Each column set is the fully re-run 34-run A/B matrix with one MPC "
             "backend, so both arms, all scenarios and all seeds use the same "
             "solver.", "",
             "| backend | phase | scenario/arm | seed | manoeuvre RMSE [deg] | "
             "mean [ms] | p95 [ms] | max [ms] | fallback | real-time |",
             "|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        lines.append("| %s | %s | %s/%s | %d | %s | %s | %s | %s | %s | %s |" % (
            r["backend"], r["phase"], r["scenario"], r["arm"], r["seed"],
            fmt(r["rmse_man_deg"], 4), fmt(1000 * r["cpu_mean_s"], 1),
            fmt(1000 * r["cpu_p95_s"], 1), fmt(1000 * r["cpu_max_s"], 1),
            fmt(r["fallback_ratio"]), fmt(r["realtime_met"])))

    # summary: mean over scenarios, per backend/arm
    lines += ["", "## Aggregate over the 8 P0 scenarios (seed 0)", "",
              "| backend | arm | mean RMSE [deg] | mean step [ms] | p95 step [ms] | "
              "max step [ms] | steps within 0.2 s | fallback |",
              "|---|---|---|---|---|---|---|---|"]
    for label, d in data:
        back = label.split(" ")[0]
        for arm in ARMS:
            sel = [v for (phase, sc, a, seed), v in d.items()
                   if phase == "P0" and a == arm and seed == 0 and sc in SCEN]
            if not sel:
                continue
            rm = float(np.mean([v["rmse_man"] for v in sel]))
            cm = float(np.mean([v["cpu_mean"] for v in sel]))
            p95 = float(np.mean([v["cpu_p95"] for v in sel]))
            mx = float(np.max([v["cpu_max"] for v in sel]))
            within = sum(1 for v in sel if v["cpu_p95"] <= 0.2)
            fb = float(np.mean([v["fb"] for v in sel]))
            lines.append("| %s | %s | %.4f | %.1f | %.1f | %.1f | %d/%d | %.3f |" % (
                back, arm, rm, 1000 * cm, 1000 * p95, 1000 * mx, within, len(sel), fb))
    (OUT / "backends.md").write_text("\n".join(lines) + "\n", encoding="utf-8")

    # figures
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        plt.rcParams.update({
            "font.family": "sans-serif",
            "font.sans-serif": ["Microsoft YaHei", "SimHei", "DejaVu Sans"],
            "axes.unicode_minus": False, "font.size": 8.5, "axes.labelsize": 8.5,
            "axes.titlesize": 9.5, "legend.fontsize": 7.5,
        })
        FIG.mkdir(parents=True, exist_ok=True)
        backs = [b.split(" ")[0] for b, _ in ROOTS]
        colors = {"baseline": "C3", "fast": "C1", "rti": "C0", "rti2": "C2"}
        for arm in ARMS:
            fig, axs = plt.subplots(1, 2, figsize=(12.5, 3.8))
            w = 0.26
            xp = np.arange(len(SCEN))
            for j, back in enumerate(backs):
                d = dict(data)[[l for l, p in ROOTS if l.startswith(back)][0]]
                mean = [1000 * d.get(("P0", sc, arm, 0), {}).get("cpu_mean", np.nan)
                        for sc in SCEN]
                p95 = [1000 * d.get(("P0", sc, arm, 0), {}).get("cpu_p95", np.nan)
                       for sc in SCEN]
                axs[0].bar(xp + (j - 1) * w, mean, w, label=back,
                           color=colors.get(back))
                axs[1].bar(xp + (j - 1) * w, p95, w, label=back,
                           color=colors.get(back))
            for a, t in ((axs[0], "control step, mean"), (axs[1], "control step, P95")):
                a.axhline(200.0, color="k", ls="--", lw=0.9, label="0.2 s period")
                a.set_yscale("log")
                a.set_ylabel("solve time [ms]")
                a.set_title(t)
                a.set_xticks(xp)
                a.set_xticklabels([s.split("_")[0] for s in SCEN])
                a.grid(alpha=0.3, axis="y")
                a.legend()
            fig.suptitle("MPC solve time per control step — arm = %s" % arm)
            fig.tight_layout()
            fig.savefig(FIG / ("backend_speed_%s.png" % arm), dpi=150)
            plt.close(fig)

        # RMSE vs time scatter
        fig, ax = plt.subplots(figsize=(7.0, 4.2))
        for label, d in data:
            back = label.split(" ")[0]
            for arm, marker in (("with_cmg", "o"), ("no_cmg", "s")):
                xs, ys = [], []
                for sc in SCEN:
                    v = d.get(("P0", sc, arm, 0))
                    if v:
                        xs.append(1000 * v["cpu_mean"])
                        ys.append(v["rmse_man"])
                ax.scatter(xs, ys, marker=marker, s=36, label="%s / %s" % (back, arm),
                           color=colors.get(back), alpha=0.85)
        ax.set_xscale("log")
        ax.set_xlabel("mean solve time per control step [ms]")
        ax.set_ylabel("manoeuvre-window RMSE [deg]")
        ax.axvline(200.0, color="k", ls="--", lw=0.9)
        ax.set_title("Tracking quality versus solve time (P0, seed 0)")
        ax.grid(alpha=0.3)
        ax.legend(ncol=2)
        fig.tight_layout()
        fig.savefig(FIG / "backend_rmse_vs_time.png", dpi=150)
        plt.close(fig)

        # per-step solve time for one scenario, all three backends
        for sc in ("S00_pitch30", "S05_yaw30"):
            fig, ax = plt.subplots(figsize=(10.0, 3.4))
            for label, path in ROOTS:
                back = label.split(" ")[0]
                d = ROOT / path / "P0" / ("%s_with_cmg" % sc)
                ts = d / "timeseries.csv"
                if not ts.exists():
                    continue
                t, cpu = [], []
                with open(ts, encoding="utf-8") as fh:
                    for r in csv.DictReader(fh):
                        t.append(float(r["t_next"]))
                        cpu.append(float(r["cpu_s"]))
                ax.plot(t, np.array(cpu) * 1000.0, lw=0.9, label=back,
                        color=colors.get(back))
            ax.axhline(200.0, color="k", ls="--", lw=0.9, label="0.2 s period")
            ax.set_yscale("log")
            ax.set_xlabel("t [s]")
            ax.set_ylabel("solve time [ms]")
            ax.set_title("%s / with_cmg — per-step solve time" % sc)
            ax.grid(alpha=0.3)
            ax.legend()
            fig.tight_layout()
            fig.savefig(FIG / ("backend_timeseries_%s.png" % sc), dpi=150)
            plt.close(fig)
    except Exception as exc:                                   # noqa: BLE001
        print("figure step skipped:", exc)

    print("wrote", OUT / "backends.csv")
    print("wrote", OUT / "backends.md")
    return 0


if __name__ == "__main__":
    sys.exit(main())
