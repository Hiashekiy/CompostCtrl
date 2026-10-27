"""Closed-loop simulation runner and performance metrics."""
from __future__ import annotations

import numpy as np

from legacy.model import ModelParams, plant_step


def run_case(name, cfg, traj, plant_prm: ModelParams, controller, T=None, x0=None):
    dtp = float(cfg.simulation.dt_plant)
    dtc = float(cfg.simulation.dt_ctrl)
    nsub = max(1, int(round(dtc / dtp)))
    T = float(cfg.simulation.T if T is None else T)
    if x0 is None:
        x0 = np.array([traj.theta_ref(0.0), 0.0, 0.0, 0.0, 0.0])

    x = np.array(x0, dtype=float)
    t = 0.0
    ts, xs, us, etas, hs, errs, solve_times, statuses, sats = ([] for _ in range(9))
    while t < T - 1e-9:
        u, info = controller.act(x, t, traj, cfg.disturbance)
        for _ in range(nsub):
            p = traj.p_array(t, cfg.disturbance)
            x, _ = plant_step(x, u, p, dtp, plant_prm, substeps=1)
            t += dtp
        ts.append(t)
        xs.append(x.copy())
        us.append(np.array(u, dtype=float))
        errs.append(x[0] - traj.theta_ref(t))
        hs.append(x[4])
        etas.append(info.get("eta", np.nan))
        solve_times.append(info.get("solve_time", 0.0))
        statuses.append(info.get("qp_status", ""))
        sats.append(float(abs(x[4]) >= 0.995 * plant_prm.h_max))

    ts = np.array(ts)
    xs = np.array(xs)
    us = np.array(us)
    errs = np.array(errs)
    h = xs[:, 4]
    metrics = dict(
        name=name,
        rmse_deg=float(np.rad2deg(np.sqrt(np.mean(errs ** 2)))),
        max_err_deg=float(np.rad2deg(np.max(np.abs(errs)))),
        int_delta2=float(np.sum(us[:, 0] ** 2) * dtc),
        int_mrw2=float(np.sum(us[:, 1] ** 2) * dtc),
        max_h=float(np.max(np.abs(h))),
        h_max=float(plant_prm.h_max),
        sat_fraction=float(np.mean(sats)),
        eta_min=float(np.nanmin(etas)) if np.any(np.isfinite(etas)) else np.nan,
        solve_ms_mean=float(1e3 * np.mean(solve_times)),
        solve_ms_max=float(1e3 * np.max(solve_times)),
        n_qp_fail=int(sum(1 for s in statuses
                           if s and s != "allocation" and "solved" not in s)),
    )
    return dict(t=ts, x=xs, u=us, err=errs, eta=np.array(etas), h=h, metrics=metrics)
