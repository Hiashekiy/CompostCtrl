"""Reproduce the comparison study end to end.

    python run_all.py

Runs C0-C4, writes results/*.csv and results/comparison.png, and prints a
summary table.
"""
from __future__ import annotations

import os
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:                    # repository root, for nearspace/legacy
    sys.path.insert(0, _ROOT)

import csv
import os
import time

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from nearspace.atmosphere_factory import make_atmosphere
from nearspace.config import load_config
from legacy.controllers import AllocationController, UnifiedMPCController
from legacy.model import ModelParams
from legacy.simulate import run_case
from nearspace.trajectory import SkipTrajectory

ROOT = _ROOT
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "results")
CACHE = os.path.join(ROOT, "results", "_cache")    # shared NRLMSIS-2.0 cache
CFG = os.path.join(HERE, "config.yaml")            # frozen C0-C4 / T1-T3 config


def main():
    os.makedirs(OUT, exist_ok=True)
    cfg = load_config(CFG)
    atm = make_atmosphere(cfg, cache_dir=CACHE)
    traj = SkipTrajectory(cfg.trajectory, atm)

    nominal = ModelParams.from_config(cfg)
    ideal = ModelParams.from_config(cfg, ideal=True)
    surface_only = ModelParams.from_config(cfg, wheel_on=False)
    mismatch = ModelParams.from_config(cfg, rho_scale=0.8, cm_scale=0.9)

    cases = [
        ("C0 ideal actuators (MPC)", nominal, ideal, UnifiedMPCController(ideal, cfg)),
        ("C1 surface only", surface_only, surface_only, UnifiedMPCController(surface_only, cfg)),
        ("C2 torque allocation", nominal, nominal, AllocationController(nominal, cfg)),
        ("C3 unified MPC", nominal, nominal, UnifiedMPCController(nominal, cfg)),
        ("C4 unified MPC, mismatch", mismatch, nominal, UnifiedMPCController(nominal, cfg)),
    ]

    results = {}
    rows = []
    for label, plant_prm, ctrl_prm, controller in cases:
        t0 = time.perf_counter()
        res = run_case(label, cfg, traj, plant_prm, controller)
        wall = time.perf_counter() - t0
        results[label] = res
        m = dict(res["metrics"])
        m["wall_s"] = wall
        rows.append(m)
        print("%-26s RMSE=%7.4f deg  max=%7.3f deg  max|h|=%6.1f  sat=%5.1f%%  "
              "eta_min=%8.2f  solve=%6.1f ms  wall=%5.1f s"
              % (label, m["rmse_deg"], m["max_err_deg"], m["max_h"], 100 * m["sat_fraction"],
                 m["eta_min"], m["solve_ms_mean"], wall))

        with open(os.path.join(OUT, label.split()[0] + ".csv"), "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["t", "theta", "q", "delta", "M_rw", "h_w", "theta_ref", "err_deg",
                        "delta_c", "M_rw_c", "eta"])
            for i, tt in enumerate(res["t"]):
                x = res["x"][i]
                u = res["u"][i]
                w.writerow([tt, x[0], x[1], x[2], x[3], x[4], traj.theta_ref(tt),
                            np.rad2deg(res["err"][i]), u[0], u[1], res["eta"][i]])

    with open(os.path.join(OUT, "metrics.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        for r in rows:
            w.writerow(r)

    # ---- figure ----
    fig, ax = plt.subplots(3, 2, figsize=(13, 11))
    alt = np.array([traj.h(t) / 1000.0 for t in results["C3 unified MPC"]["t"]])
    ax[0, 0].plot(results["C3 unified MPC"]["t"], alt, "k")
    ax[0, 0].set_ylabel("altitude [km]"); ax[0, 0].set_xlabel("t [s]"); ax[0, 0].grid(alpha=.3)
    ax[0, 0].set_title("skip trajectory")

    for label, res in results.items():
        t = res["t"]
        ax[0, 1].plot(t, np.rad2deg(res["err"]), label=label)
        ax[1, 0].plot(t, np.rad2deg(res["u"][:, 0]), label=label)
        ax[1, 1].plot(t, res["u"][:, 1], label=label)
        ax[2, 0].plot(t, res["h"], label=label)
        ax[2, 1].plot(t, res["eta"], label=label)
    for a, (yl, ti) in zip(ax.ravel()[1:], [
            ("theta error [deg]", "attitude tracking error"),
            ("delta [deg]", "surface deflection"),
            ("M_rw [N m]", "wheel torque"),
            ("h_w [N m s]", "wheel angular momentum"),
            ("eta = Gamma/M_man [-]", "authority margin (diagnostic)")]):
        a.set_ylabel(yl); a.set_xlabel("t [s]"); a.grid(alpha=.3); a.set_title(ti)
        a.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "comparison.png"), dpi=130)
    print("\nwrote", os.path.join(OUT, "comparison.png"))


if __name__ == "__main__":
    main()
