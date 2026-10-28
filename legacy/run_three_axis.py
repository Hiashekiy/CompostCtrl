"""Three-axis comparison study.

    python run_three_axis.py
"""
from __future__ import annotations

import os
import sys

_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _ROOT not in sys.path:                    # repository root, for nearspace/legacy
    sys.path.insert(0, _ROOT)

import csv
import os
import sys
import time

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from nearspace.atmosphere_factory import make_atmosphere
from nearspace.config import load_config
from legacy.controllers3 import UnifiedMPC3Controller
from nearspace.model6 import VehicleParams
from legacy.simulate3 import run_case3
from legacy.trajectory3_legacy import Trajectory3D

ROOT = _ROOT
HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "results")
CACHE = os.path.join(ROOT, "results", "_cache")    # shared NRLMSIS-2.0 cache
CFG = os.path.join(HERE, "config.yaml")            # frozen C0-C4 / T1-T3 config


def main():
    os.makedirs(OUT, exist_ok=True)
    cfg = load_config(CFG)
    backend = sys.argv[1] if len(sys.argv) > 1 else getattr(cfg.environment, "backend", "exp")
    T = float(sys.argv[2]) if len(sys.argv) > 2 else None
    atm = make_atmosphere(cfg, cache_dir=CACHE, backend=backend)
    traj = Trajectory3D(cfg, atm)
    print("atmosphere backend:", backend)

    nominal = VehicleParams.from_config(cfg)
    surface_only = VehicleParams.from_config(cfg, wheel_on=False)

    cases = [
        ("T1 no wheels", surface_only, surface_only, UnifiedMPC3Controller(surface_only, cfg)),
        ("T3 unified MPC 3-axis", nominal, nominal, UnifiedMPC3Controller(nominal, cfg)),
    ]

    results, rows = {}, []
    for label, plant_prm, ctrl_prm, controller in cases:
        t0 = time.perf_counter()
        res = run_case3(label, cfg, traj, plant_prm, controller, T=T)
        wall = time.perf_counter() - t0
        results[label] = res
        m = dict(res["metrics"]); m["wall_s"] = wall
        rows.append(m)
        print("%-22s RMSE=%7.4f deg max=%7.3f deg (roll %.3f pitch %.3f yaw %.3f) "
              "max|h|=%6.1f sat=%5.1f%% solve=%6.1f ms fails=%3d wall=%5.1f s"
              % (label, m["rmse_deg"], m["max_err_deg"], m["roll_rmse_deg"],
                 m["pitch_rmse_deg"], m["yaw_rmse_deg"], m["max_h"], 100 * m["sat_fraction"],
                 m["solve_ms_mean"], m["n_qp_fail"], wall))
        with open(os.path.join(OUT, label.split()[0] + "_3axis.csv"), "w", newline="") as fh:
            w = csv.writer(fh)
            w.writerow(["t", "qw", "qx", "qy", "qz", "p", "q", "r", "de", "da", "dr",
                        "tau1", "tau2", "tau3", "h1", "h2", "h3",
                        "de_c", "da_c", "dr_c", "tauc1", "tauc2", "tauc3",
                        "err_roll", "err_pitch", "err_yaw", "err_norm_deg"])
            for i, tt in enumerate(res["t"]):
                w.writerow([tt] + list(res["x"][i]) + list(res["u"][i])
                           + list(res["err"][i]) + [np.rad2deg(np.linalg.norm(res["err"][i]))])

    with open(os.path.join(OUT, "metrics_3axis.csv"), "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        for r in rows:
            w.writerow(r)

    fig, ax = plt.subplots(3, 2, figsize=(13, 11))
    alt = np.array([traj.h(t) / 1000.0 for t in results["T3 unified MPC 3-axis"]["t"]])
    ax[0, 0].plot(results["T3 unified MPC 3-axis"]["t"], alt, "k")
    ax[0, 0].set_title("skip trajectory"); ax[0, 0].set_ylabel("altitude [km]")
    for label, res in results.items():
        t = res["t"]
        ax[0, 1].plot(t, np.rad2deg(np.linalg.norm(res["err"], axis=1)), label=label)
        ax[1, 0].plot(t, np.rad2deg(res["err"][:, 0]), label=label + " roll")
        ax[1, 0].plot(t, np.rad2deg(res["err"][:, 1]), "--", label=label + " pitch")
        ax[1, 1].plot(t, np.linalg.norm(res["h"], axis=1), label=label)
        ax[2, 0].plot(t, np.rad2deg(res["u"][:, 0:3]))
        ax[2, 1].plot(t, res["x"][:, 10:13])
    for a, (yl, ti) in zip(ax.ravel()[1:], [
            ("|attitude error| [deg]", "three-axis attitude error"),
            ("error [deg]", "roll / pitch error"),
            ("||h_w|| [N m s]", "wheel cluster momentum"),
            ("delta_e, delta_a, delta_r [deg]", "surface deflections"),
            ("tau_1..3 [N m]", "wheel motor torques")]):
        a.set_ylabel(yl); a.set_xlabel("t [s]"); a.grid(alpha=.3); a.set_title(ti); a.legend(fontsize=7)
    fig.tight_layout()
    fig.savefig(os.path.join(OUT, "comparison_3axis.png"), dpi=130)
    print("\nwrote", os.path.join(OUT, "comparison_3axis.png"))


if __name__ == "__main__":
    main()
