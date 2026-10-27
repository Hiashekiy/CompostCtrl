"""Aero-surface + CMG comparison (realistic CMG cluster).

    python run_cmg.py <backend> <T> <case>

case: aero | cmg   (aero = CMG disabled, cmg = unified MPC with the CMG cluster)
"""
from __future__ import annotations

import csv
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from nearspace.atmosphere_factory import make_atmosphere
from nearspace.cmg_control import LTVMPCCMG2
from nearspace.config import load_config
from nearspace.model6c import VehicleCMGParams, make_cluster, plant_step6c
from nearspace.trajectory3 import Trajectory3D

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "results")


def run(name, cfg, traj, prm, cl, mpc, T):
    dtc = float(cfg.simulation.dt_ctrl)
    dtp = float(cfg.simulation.dt_plant)
    nsub = max(1, int(round(dtc / dtp)))
    de = -prm.veh.Cm_alpha / prm.veh.Cm_de * (traj.pitch_ref(0.0) - traj.gamma(0.0))
    x = np.concatenate([traj.q_ref(0.0), np.zeros(3), [de, 0, 0], np.zeros(4), np.zeros(4)])
    t = 0.0
    ts, xs, us, errs, sig, solvet, status = [], [], [], [], [], [], []
    while t < T - 1e-9:
        u, info = mpc.solve(x, t, traj)
        for _ in range(nsub):
            p = traj.p_array(t, prm.veh)
            x, _ = plant_step6c(x, u, p, dtp, prm, cl, substeps=1)
            t += dtp
        ts.append(t); xs.append(x.copy()); us.append(np.array(u, float))
        errs.append(mpc._att_error(traj.q_ref(t), x[0:4]))
        sig.append(info.get("min_sigma", np.nan))
        solvet.append(info.get("solve_time", 0.0))
        status.append(info.get("qp_status", ""))
    ts = np.array(ts); xs = np.array(xs); us = np.array(us); errs = np.array(errs)
    H = np.array([np.linalg.norm(cl.momentum(xi[10:14])) for xi in xs])
    gdot = xs[:, 14:18]
    m = dict(name=name,
             rmse_deg=float(np.rad2deg(np.sqrt(np.mean(np.sum(errs ** 2, axis=1))))),
             max_err_deg=float(np.rad2deg(np.max(np.linalg.norm(errs, axis=1)))),
             roll_rmse_deg=float(np.rad2deg(np.sqrt(np.mean(errs[:, 0] ** 2)))),
             pitch_rmse_deg=float(np.rad2deg(np.sqrt(np.mean(errs[:, 1] ** 2)))),
             yaw_rmse_deg=float(np.rad2deg(np.sqrt(np.mean(errs[:, 2] ** 2)))),
             max_H_Nms=float(np.max(H)), H_limit_Nms=float(cl.prm.h_total),
             min_sigma_Nms=float(np.nanmin(sig)) if np.any(np.isfinite(sig)) else float("nan"),
             sigma_floor_Nms=float(prm.cmg.sigma_floor),
             int_delta2=float(np.sum(us[:, 0:3] ** 2) * dtc),
             max_gdot_rad_s=float(np.max(np.abs(gdot))),
             gdot_limit_rad_s=float(prm.cmg.gdot_max),
             solve_ms_mean=float(1e3 * np.mean(solvet)),
             n_qp_fail=int(sum(1 for s in status if s and "solved" not in s and "failsafe" not in s)),
             n_failsafe=int(sum(1 for s in status if "failsafe" in s)))
    return dict(t=ts, x=xs, u=us, err=errs, H=H, metrics=m)


def main():
    backend = sys.argv[1] if len(sys.argv) > 1 else "msis2"
    T = float(sys.argv[2]) if len(sys.argv) > 2 else 140.0
    case = sys.argv[3] if len(sys.argv) > 3 else "cmg"
    os.makedirs(OUT, exist_ok=True)
    cfg = load_config(os.path.join(HERE, "config.yaml"))
    atm = make_atmosphere(cfg, cache_dir=OUT, backend=backend)
    traj = Trajectory3D(cfg, atm)
    prm = VehicleCMGParams.from_config(cfg)
    cl = make_cluster(prm)
    if case == "aero":
        cfg.mpc_cmg.M_cmg_box = 1e-6
        label = "A aero surfaces only"
    else:
        label = "B aero + CMG (unified MPC)"
    mpc = LTVMPCCMG2(prm, cfg, cl)
    res = run(label, cfg, traj, prm, cl, mpc, T)
    m = res["metrics"]
    print("%-26s RMSE=%7.4f deg max=%7.3f (roll %.3f pitch %.3f yaw %.3f) "
          "max|H|=%6.1f/%.0f sigma_min=%5.1f/%.1f gdot_max=%.2f/%.2f solve=%5.0fms "
          "qp_fail=%d failsafe=%d"
          % (label, m["rmse_deg"], m["max_err_deg"], m["roll_rmse_deg"], m["pitch_rmse_deg"],
             m["yaw_rmse_deg"], m["max_H_Nms"], m["H_limit_Nms"], m["min_sigma_Nms"],
             m["sigma_floor_Nms"], m["max_gdot_rad_s"], m["gdot_limit_rad_s"],
             m["solve_ms_mean"], m["n_qp_fail"], m["n_failsafe"]))
    tag = case
    with open(os.path.join(OUT, "cmg_%s.csv" % tag), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t", "qw", "qx", "qy", "qz", "p", "q", "r", "de", "da", "dr",
                    "g1", "g2", "g3", "g4", "gd1", "gd2", "gd3", "gd4",
                    "de_c", "da_c", "dr_c", "err_roll", "err_pitch", "err_yaw", "H_cmg"])
        for i, tt in enumerate(res["t"]):
            w.writerow([tt] + list(res["x"][i]) + list(res["u"][i, 0:3])
                       + list(res["err"][i]) + [res["H"][i]])
    with open(os.path.join(OUT, "cmg_metrics_%s.csv" % tag), "w", newline="") as fh:
        w = csv.writer(fh)
        for k, v in m.items():
            w.writerow([k, v])
    print("wrote", os.path.join(OUT, "cmg_%s.csv" % tag))


if __name__ == "__main__":
    main()
