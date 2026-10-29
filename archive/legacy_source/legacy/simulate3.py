"""Closed-loop three-axis simulation and metrics."""
from __future__ import annotations

import numpy as np

from nearspace.model6 import VehicleParams, plant_step3


def run_case3(name, cfg, traj, plant_prm: VehicleParams, controller, T=None, x0=None):
    dtp = float(cfg.simulation.dt_plant)
    dtc = float(cfg.simulation.dt_ctrl)
    nsub = max(1, int(round(dtc / dtp)))
    T = float(cfg.simulation.T if T is None else T)
    if x0 is None:
        de = -plant_prm.Cm_alpha / plant_prm.Cm_de * (traj.pitch_ref(0.0) - traj.gamma(0.0))
        x0 = np.concatenate([traj.q_ref(0.0), np.zeros(3), [de, 0.0, 0.0],
                             np.zeros(3), np.zeros(3)])

    x = np.array(x0, dtype=float)
    t = 0.0
    ts, xs, us, errs, solve_t, status = [], [], [], [], [], []
    while t < T - 1e-9:
        u, info = controller.act(x, t, traj)
        for _ in range(nsub):
            p = traj.p_array(t, plant_prm)
            x, _ = plant_step3(x, u, p, dtp, plant_prm, substeps=1)
            t += dtp
        e = controller.mpc._attitude_error(traj.q_ref(t), x[0:4]) if hasattr(controller, "mpc") \
            else 2.0 * np.zeros(3)
        ts.append(t); xs.append(x.copy()); us.append(np.array(u, float))
        errs.append(e); solve_t.append(info.get("solve_time", 0.0))
        status.append(info.get("qp_status", ""))

    ts = np.array(ts); xs = np.array(xs); us = np.array(us); errs = np.array(errs)
    err_norm = np.rad2deg(np.linalg.norm(errs, axis=1))
    h = xs[:, 13:16]
    metrics = dict(
        name=name,
        rmse_deg=float(np.sqrt(np.mean(np.sum(errs ** 2, axis=1))) * 180.0 / np.pi),
        max_err_deg=float(np.max(err_norm)),
        roll_rmse_deg=float(np.rad2deg(np.sqrt(np.mean(errs[:, 0] ** 2)))),
        pitch_rmse_deg=float(np.rad2deg(np.sqrt(np.mean(errs[:, 1] ** 2)))),
        yaw_rmse_deg=float(np.rad2deg(np.sqrt(np.mean(errs[:, 2] ** 2)))),
        int_delta2=float(np.sum(us[:, 0:3] ** 2) * dtc),
        int_tau2=float(np.sum(us[:, 3:6] ** 2) * dtc),
        max_h=float(np.max(np.abs(h))),
        h_max=float(plant_prm.h_max),
        sat_fraction=float(np.mean(np.any(np.abs(h) >= 0.995 * plant_prm.h_max, axis=1))),
        solve_ms_mean=float(1e3 * np.mean(solve_t)),
        solve_ms_max=float(1e3 * np.max(solve_t)),
        n_qp_fail=int(sum(1 for s in status if s and "solved" not in s)),
    )
    return dict(t=ts, x=xs, u=us, err=errs, h=h, metrics=metrics)
