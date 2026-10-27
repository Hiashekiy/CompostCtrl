"""Closed-loop run of the aero-surface / aero+CMG comparison.

One call of :func:`run` is one experiment case for one actuator mode.  The only
thing that differs between mode ``surface_only`` (A) and mode ``surface_cmg``
(B) is the actuator block; environment, initial state, reference, truth model,
integrator and time steps are identical by construction (implementation plan
§3.3).
"""
from __future__ import annotations

import platform
import subprocess
import sys
import time

import numpy as np

from . import __version__
from .atmosphere_factory import make_atmosphere
from .cmg_control import LTVMPCCMG2
from .config import dump
from .metrics import SCORING, compute
from .model6 import quat_geodesic, quat_to_euler, wrap_pi
from .model6c import VehicleCMGParams, make_cluster, plant_step6c
from .trajectory3 import Trajectory3D

MODES = ("surface_only", "surface_cmg")


# ---------------------------------------------------------------------------
# build
# ---------------------------------------------------------------------------
def actuator_mode(cfg) -> str:
    from .scenarios import actuator_mode as _m
    return _m(cfg)


def build(cfg, cache_dir=None):
    """Atmosphere, reference trajectory, truth-model params and cluster."""
    mode = actuator_mode(cfg)
    atm = make_atmosphere(cfg, cache_dir=cache_dir,
                          backend=getattr(cfg.environment, "backend", "msis2"))
    traj = Trajectory3D(cfg, atm)
    prm = VehicleCMGParams.from_config(cfg, cmg_on=(mode == "surface_cmg"))
    cl = make_cluster(prm)
    return dict(atm=atm, traj=traj, prm=prm, cluster=cl, mode=mode)


def _initial_state(traj, prm):
    v = prm.veh
    de = -v.Cm_alpha / v.Cm_de * (traj.pitch_ref(0.0) - traj.gamma(0.0))
    return np.concatenate([traj.q_ref(0.0), np.zeros(3), [de, 0.0, 0.0],
                           np.zeros(prm.cmg.n), np.zeros(prm.cmg.n)])


# ---------------------------------------------------------------------------
# one closed-loop run
# ---------------------------------------------------------------------------
def run(cfg, cache_dir=None, ctrl=None, progress=False):
    b = build(cfg, cache_dir=cache_dir)
    traj, prm, cl, mode = b["traj"], b["prm"], b["cluster"], b["mode"]
    mpc = ctrl if ctrl is not None else LTVMPCCMG2(prm, cfg, cl)

    T = float(cfg.simulation.T)
    dtc = float(cfg.simulation.dt_ctrl)
    dtp = float(cfg.simulation.dt_plant)
    nsub = max(1, int(round(dtc / dtp)))
    dmax_deg = np.rad2deg(prm.veh.delta_max)

    x = _initial_state(traj, prm)
    t = 0.0
    wall0 = time.perf_counter()
    rec = {k: [] for k in (
        "t", "h", "V", "rho", "qinf", "phi_ref", "theta_ref", "psi_ref",
        "phi", "theta", "psi", "err_roll_deg", "err_pitch_deg", "err_yaw_deg",
        "err_norm_deg", "omega", "delta_cmd_deg", "delta_deg", "delta_dot_deg_s",
        "M_aero", "gimbal_deg", "gdot", "H_cmg", "H_norm", "M_cmg", "sigma",
        "qp_ok", "qp_fail", "fallback", "qp_iter", "qp_converged", "viol",
        "solve_time_ms", "M_dist", "maneuver_active", "maneuver_phase")}

    def rec_row(tau, u, info):
        eul = quat_to_euler(x[0:4])
        eref = traj.euler_ref(tau)
        d = np.array([wrap_pi(eul[i] - eref[i]) for i in range(3)])
        M_cmg = cl.torque(x[10:14], x[14:18]) if prm.cmg_on else np.zeros(3)
        A = cl.jacobian(x[10:14])
        H = cl.momentum(x[10:14]) if prm.cmg_on else np.zeros(3)
        from .model6c import aero_moment6c
        Ma = aero_moment6c(x, traj.p_array(tau, prm.veh), prm)
        # the controller reports fallback explicitly; do NOT infer it from the
        # OSQP status string (an "inaccurate" solve is still a usable command)
        status_fail = int(info.get("fallback", 0))
        rec["t"].append(tau)
        rec["h"].append(traj.h(tau) / 1000.0)
        rec["V"].append(traj.V(tau))
        rec["rho"].append(traj.rho(tau))
        rec["qinf"].append(traj.qinf(tau))
        rec["phi_ref"].append(np.rad2deg(eref[0]))
        rec["theta_ref"].append(np.rad2deg(eref[1]))
        rec["psi_ref"].append(np.rad2deg(eref[2]))
        rec["phi"].append(np.rad2deg(eul[0]))
        rec["theta"].append(np.rad2deg(eul[1]))
        rec["psi"].append(np.rad2deg(eul[2]))
        rec["err_roll_deg"].append(np.rad2deg(d[0]))
        rec["err_pitch_deg"].append(np.rad2deg(d[1]))
        rec["err_yaw_deg"].append(np.rad2deg(d[2]))
        rec["err_norm_deg"].append(np.rad2deg(quat_geodesic(traj.q_ref(tau), x[0:4])))
        rec["omega"].append(x[4:7].copy())
        rec["delta_cmd_deg"].append(np.rad2deg(u[0:3]))
        rec["delta_deg"].append(np.rad2deg(x[7:10]))
        rec["delta_dot_deg_s"].append(np.rad2deg((u[0:3] - x[7:10]) / prm.veh.tau_s))
        rec["M_aero"].append(Ma)
        rec["gimbal_deg"].append(np.rad2deg(x[10:14]))
        rec["gdot"].append(x[14:18].copy())
        rec["H_cmg"].append(H.copy())
        rec["H_norm"].append(float(np.linalg.norm(H)))
        rec["M_cmg"].append(M_cmg)
        rec["sigma"].append(float(cl.sigma_min(A)) if prm.cmg_on else float(cl.prm.home_sigma_min))
        rec["qp_ok"].append(1 - status_fail)
        rec["qp_fail"].append(status_fail)
        rec["fallback"].append(status_fail)
        rec["qp_iter"].append(float(info.get("qp_iter", 0)))
        rec["qp_converged"].append(
            float(str(info.get("qp_status", "")).lower().startswith("solved")))
        rec["viol"].append(float(info.get("viol", 0.0)))
        rec["solve_time_ms"].append(1e3 * float(info.get("solve_time", 0.0)))
        rec["M_dist"].append(traj.disturbance(tau, cfg, prm.veh).copy())
        rec["maneuver_active"].append(int(traj.maneuver.phase(tau) == 1))
        rec["maneuver_phase"].append(traj.maneuver.phase(tau))

    x0 = x.copy()
    rec_row(0.0, np.zeros(7), {})
    k = 0
    while t < T - 1e-9:
        u, info = mpc.solve(x, t, traj)
        for _ in range(nsub):
            p = traj.p_array(t, prm.veh)
            x, _ = plant_step6c(x, u, p, dtp, prm, cl, substeps=1)
            t += dtp
        # the command held during [t-dtc, t]
        rec_row(t, u, info)
        k += 1
        if progress and k % 100 == 0:
            print("  t=%6.1f s  err=%.3f deg  |H|=%.1f  fail=%d"
                  % (t, rec["err_norm_deg"][-1], rec["H_norm"][-1],
                     int(np.sum(rec["qp_fail"]))), flush=True)

    wall = time.perf_counter() - wall0
    out = _pack(rec)
    out["wall_time_s"] = wall
    out["mode"] = mode
    out["x0"] = x0
    out["x_end"] = x
    out["windows"] = traj.windows(T)
    out["maneuver_delta_deg"] = float(np.rad2deg(traj.maneuver.delta))
    out["maneuver_axis"] = traj.maneuver.axis
    out["metrics"] = compute(out, out["windows"], dmax_deg)
    out["scoring"] = dict(SCORING)
    return out


def _pack(rec):
    arr = {}
    for k, v in rec.items():
        if v and isinstance(v[0], (list, np.ndarray)):
            arr[k] = np.array(v, dtype=float)
        else:
            arr[k] = np.array(v, dtype=float)
    return arr


# ---------------------------------------------------------------------------
# metadata
# ---------------------------------------------------------------------------
def _git_commit() -> str:
    try:
        return subprocess.check_output(["git", "rev-parse", "HEAD"],
                                       cwd=__file__.rsplit("nearspace", 1)[0],
                                       stderr=subprocess.DEVNULL).decode().strip()
    except Exception:
        return "no-git-repository"


def metadata(cfg, out, case, tag, extra=None) -> dict:
    prm_mode = out["mode"]
    labels = ["q0", "q1", "q2", "q3", "p", "q", "r", "de", "da", "dr",
              "g1", "g2", "g3", "g4", "gd1", "gd2", "gd3", "gd4"]
    meta = dict(
        case=case, tag=tag, actuator_mode=prm_mode,
        code_commit=_git_commit(), package_version=__version__,
        python=sys.version.split()[0], platform=platform.platform(),
        finished_utc=time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
        wall_time_s=out["wall_time_s"],
        scoring=out["scoring"],
        simulation=dump(cfg.simulation),
        windows=dict((k, list(v)) for k, v in out["windows"].items()),
        initial_state=dict(zip(labels, [float(v) for v in out["x0"]])),
        final_state=dict(zip(labels, [float(v) for v in out["x_end"]])),
        aborted=not bool(out["metrics"].get("numeric_ok", True)),
        config=dump(cfg),
    )
    m = out["metrics"]
    meta["failed_or_degraded"] = dict(
        qp_fail_count=m["qp_fail_count"], fallback_count=m["fallback_count"],
        max_constraint_violation=m["max_constraint_violation"],
        maneuver_completed=m.get("maneuver_completed"))
    if extra:
        meta.update(extra)
    return meta

