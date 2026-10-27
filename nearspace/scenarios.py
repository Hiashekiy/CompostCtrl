"""Scenario resolution and actuator-capability bookkeeping.

Two jobs:

* :func:`resolve` turns the human-readable ``scenario:`` block of an experiment
  YAML into the parameters the model actually uses (the skip profile amplitude
  is derived from the requested vertex altitude, so A and B cannot disagree).
* :func:`cmg_spec` and :func:`effective_weights` produce the numbers that MUST
  be printed at start-up (implementation plan §2.2 and §7.1): the difference
  between hardware limits and controller budgets, and the weights that are
  actually in force after the config overlay has been applied.
"""
from __future__ import annotations

import numpy as np


# ---------------------------------------------------------------------------
# scenario
# ---------------------------------------------------------------------------
MODES = ("surface_only", "surface_cmg")


def actuator_mode(cfg) -> str:
    """Resolve ``actuators.mode`` and apply the structural consequences.

    Mode A zeroes the CMG torque box so the optimiser has no CMG authority at
    all; the truth model removes the CMG separately through
    ``VehicleCMGParams.cmg_on``.
    """
    a = getattr(cfg, "actuators", None)
    mode = str(getattr(a, "mode", "surface_cmg")).lower()
    if mode not in MODES:
        raise ValueError("actuators.mode must be one of %s, got %r" % (MODES, mode))
    if mode == "surface_only":
        cfg.mpc_cmg.M_cmg_box = 0.0
    return mode


def resolve(cfg):
    """Apply the ``scenario:`` block in place and return the config."""
    sc = getattr(cfg, "scenario", None)
    vertex_km = float(getattr(sc, "vertex_km", 0.0)) if sc is not None else 0.0
    if vertex_km > 0.0:
        h0 = float(cfg.trajectory.h0)
        cfg.trajectory.amp = vertex_km * 1000.0 - h0
    T = getattr(sc, "T", None) if sc is not None else None
    if T is not None:
        cfg.simulation.T = float(T)
    actuator_mode(cfg)
    return cfg


def case_name(cfg) -> str:
    sc = getattr(cfg, "scenario", None)
    return str(getattr(sc, "name", "case")) if sc is not None else "case"


def vertex_time(cfg) -> float:
    """Time of the altitude vertex of the cosine skip profile."""
    return 0.5 * float(cfg.trajectory.period)


# ---------------------------------------------------------------------------
# effective MPC weights  (§7.1: print what is really in force)
# ---------------------------------------------------------------------------
def effective_weights(cfg) -> dict:
    m = cfg.mpc_cmg
    horizon = [(int(h.n), float(h.dt)) for h in m.horizon]
    w = dict(
        q_att=float(m.q_att), q_rate=float(m.q_rate),
        r_delta=float(m.r_delta), r_gdot=float(getattr(m, "r_gdot", 0.0)),
        r_cmg_transient=float(getattr(m, "r_cmg_transient", 0.01)),
        r_cmg_trim=float(getattr(m, "r_cmg_trim", 0.5)),
        e_thresh_deg=float(getattr(m, "e_thresh_deg", 0.5)),
        w_gamma=float(getattr(m, "w_gamma", 5.0)),
        w_H=float(getattr(m, "w_H", 1e-3)),
        s_delta=float(m.s_delta),
        rho_sing=float(getattr(m, "rho_sing", 1e4)),
        terminal_scale=float(m.terminal_scale),
        n_sqp=int(m.n_sqp), sigma_margin=float(getattr(m, "sigma_margin", 1.15)),
        torque_margin=float(getattr(m, "torque_margin", 0.8)),
        horizon=horizon,
    )
    w["adaptive_scheme_active"] = bool(abs(w["r_cmg_transient"] - w["r_cmg_trim"]) > 0.0)
    return w


def controller_budgets(cfg, prm) -> dict:
    m = cfg.mpc_cmg
    c = prm.cmg
    return dict(
        mode=("surface_cmg" if prm.cmg_on else "surface_only"),
        M_cmg_box_Nm=float(getattr(m, "M_cmg_box", 61.0)),
        H_lim_frac=float(getattr(m, "H_lim_frac", 0.3)),
        H_lim_Nms=float(getattr(m, "H_lim_frac", 0.3)) * c.h_total,
        H_planner_max_Nms=float(getattr(m, "H_planner_max", 50.0)),
        q_unload_min_Pa=float(getattr(m, "q_unload_min", 40.0)),
        unload_cap=float(getattr(m, "unload_cap", 0.6)),
        sigma_floor_Nms=c.sigma_floor,
        qp_viol_tol=float(getattr(m, "qp_viol_tol", 1e-3)),
    )


# ---------------------------------------------------------------------------
# CMG hardware capability  (§2.2 / §7.3)
# ---------------------------------------------------------------------------
def _fib_sphere(n):
    i = np.arange(n) + 0.5
    phi = np.arccos(1.0 - 2.0 * i / n)
    theta = np.pi * (1.0 + 5.0 ** 0.5) * i
    return np.column_stack([np.cos(theta) * np.sin(phi),
                            np.sin(theta) * np.sin(phi),
                            np.cos(phi)])


def _guaranteed_torque(cluster, n_dir=200000) -> float:
    """min over unit directions of the zonotope support  sum_i |u.A_i| * gdot_max.

    The reachable torque set is the Minkowski sum of the four segments
    +-A[:,i]*gdot_max, so its support function along u is sum_i |u.A_i|*gdot_max;
    the guaranteed (all-direction) torque is the minimum of that over the
    unit sphere.  Sampled on a Fibonacci sphere.
    """
    A = cluster.jacobian(np.zeros(cluster.prm.n))
    v = _fib_sphere(n_dir)
    h = np.abs(v @ A).sum(axis=1)
    return float(h.min() * cluster.prm.gdot_max)


def _axis_momentum(cluster, axis=1):
    """Exact min/max of H[axis] over the gimbal box.

    H[axis] = sum_i h_i(gamma_i)[axis] is separable in the gimbal angles, so the
    box extrema are the sums of the per-unit extrema (no grid search needed).
    """
    prm = cluster.prm
    g = np.linspace(-0.5 * np.pi, 0.5 * np.pi, 4001)
    lo = hi = 0.0
    for i in range(prm.n):
        f = prm.h0 * (cluster.s0[axis, i] * np.cos(g)
                      + cluster.gxs0[axis, i] * np.sin(g))
        lo += float(f.min())
        hi += float(f.max())
    return lo, hi


def cmg_spec(cfg, prm) -> dict:
    """Hardware/model limits, kept strictly apart from controller budgets."""
    from .cmg import CMGCluster
    cl = CMGCluster(prm.cmg)
    c = prm.cmg
    hmin, hmax = _axis_momentum(cl, 1)
    usable = min(hmax, -hmin)
    return dict(
        units=int(c.n), h0_Nms=float(c.h0), cluster_momentum_Nms=float(c.h_total),
        gimbal_rate_max_rad_s=float(c.gdot_max),
        gimbal_accel_max_rad_s2=float(c.gddot_max),
        unit_peak_torque_Nm=float(c.torque_per_unit),
        guaranteed_torque_Nm=_guaranteed_torque(cl),
        home_sigma_min_Nms=float(c.home_sigma_min),
        sigma_floor_Nms=float(c.sigma_floor),
        single_axis_momentum_Nms=float(usable),
        single_axis_momentum_range=(hmin, hmax),
        cluster_mass_kg=float(c.n * c.mass_per_unit),
        cluster_power_W=float(c.n * c.power_per_unit),
    )


def aero_authority(cfg, prm, traj, t) -> dict:
    """Max aero control moment on each axis at time t (|M| at full deflection)."""
    v = prm.veh
    Q = traj.qinf(t)
    caps = Q * v.S * np.array([v.b * abs(v.Cl_da), v.cbar * abs(v.Cm_de), v.b * abs(v.Cn_dr)])
    return dict(h_km=traj.h(t) / 1000.0, q_inf_Pa=float(Q),
                M_aero_max_Nm=[float(v_) for v_ in caps])


# ---------------------------------------------------------------------------
# manoeuvre feasibility  (§5.2: check before running)
# ---------------------------------------------------------------------------
def maneuver_demand(prm, delta_deg, Tm) -> dict:
    """Peak rate / acceleration / torque / angular impulse of a quintic step."""
    from .trajectory3 import QUINTIC_PEAK_ACC, QUINTIC_PEAK_RATE
    v = prm.veh
    d = np.deg2rad(float(delta_deg))
    return dict(
        delta_deg=float(delta_deg), Tm_s=float(Tm),
        peak_rate_rad_s=QUINTIC_PEAK_RATE * d / Tm,
        peak_accel_rad_s2=QUINTIC_PEAK_ACC * d / Tm ** 2,
        peak_pitch_torque_Nm=QUINTIC_PEAK_ACC * d / Tm ** 2 * v.Jy,
        impulse_half_Nms=QUINTIC_PEAK_RATE * d / Tm * v.Jy,
    )


def maneuver_feasibility(cfg, prm, traj, t0, delta_deg, Tm) -> dict:
    """Compare the quintic demand with what each mode can deliver at time t0."""
    from .model6c import make_cluster
    dem = maneuver_demand(prm, delta_deg, Tm)
    aero = aero_authority(cfg, prm, traj, t0)
    M_aero = float(aero["M_aero_max_Nm"][1])
    cl = make_cluster(prm)
    M_cmg = float(getattr(cfg.mpc_cmg, "M_cmg_box", 61.0)) * float(
        getattr(cfg.mpc_cmg, "torque_margin", 0.8))
    H_budget = float(getattr(cfg.mpc_cmg, "H_lim_frac", 0.3)) * prm.cmg.h_total
    half = 0.5 * dem["Tm_s"]
    imp_aero = M_aero * half
    imp_cmg = H_budget
    dem_imp = dem["impulse_half_Nms"]

    def verdict(M_avail, imp_avail):
        if dem["peak_pitch_torque_Nm"] > 1.15 * M_avail:
            return "infeasible"
        if dem_imp > 1.15 * imp_avail:
            return "infeasible"
        if dem_imp > 0.85 * imp_avail or dem["peak_pitch_torque_Nm"] > 0.85 * M_avail:
            return "marginal"
        return "feasible"

    return dict(
        demand=dem, aero=aero,
        mode_A=dict(peak_torque_Nm=M_aero, impulse_capacity_Nms=imp_aero,
                    verdict=verdict(M_aero, imp_aero)),
        mode_B=dict(peak_torque_Nm=M_aero + M_cmg,
                    impulse_capacity_Nms=imp_aero + imp_cmg,
                    verdict=verdict(M_aero + M_cmg, imp_aero + imp_cmg)),
        note="impulse capacity = max torque x half the manoeuvre time (aero) "
             "plus the controller momentum budget (CMG)",
    )
