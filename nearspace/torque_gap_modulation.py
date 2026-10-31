"""Predictive torque-gap modulation: nominal demand, surplus and gap.

For every prediction node k the nominal *total external control moment* demanded
by the attitude reference is estimated from the (nominal) rigid-body dynamics

    tau_nom[k] = J omega_dot_ref[k]
                 + omega_ref[k] x (J omega_ref[k] + H_nom)
                 - tau_d,nom[k]

`omega_ref` is the body rate corresponding to `q_ref` (from the trajectory's own
quaternion derivative, not an Euler-angle derivative), and `omega_dot_ref` its
time derivative.  `H_nom` is either the cluster momentum frozen at the current
configuration or zero (explicit configuration choice, identical in every arm).
`tau_d,nom` is the modelled disturbance torque at the node.

`tau_nom` is only used to extract surplus/gap information; it never prescribes a
split between the aerodynamic surfaces and the CMG.

    R+[k] = tau_max[k] - tau_nom[k]      (surplus towards the positive direction)
    R-[k] = tau_nom[k] - tau_min[k]      (surplus towards the negative direction)
    d[k]  = tau_nom[k] - clip(tau_nom[k], tau_min[k], tau_max[k])

A *negative* R- means the lower aero bound has already been exceeded; in that
case d[k] < 0 and the CMG has to supply -d[k] along that negative axis.  d has
the sign of the missing useful torque, R does not.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .model6 import quat_conj, quat_mul, quat_normalize


# --------------------------------------------------------------------------
# reference derivatives
# --------------------------------------------------------------------------
def omega_dot_ref(traj, t, eps=0.05):
    """d(omega_ref)/dt by central difference of the reference body rate."""
    wp = np.asarray(traj.omega_ref(t + eps), float)
    wm = np.asarray(traj.omega_ref(t - eps), float)
    return (wp - wm) / (2.0 * eps)


def q_dot_ref(traj, t, eps=2e-3):
    dq = quat_mul(quat_conj(quat_normalize(traj.q_ref(t - 0.5 * eps))),
                  quat_normalize(traj.q_ref(t + 0.5 * eps)))
    if dq[0] < 0.0:
        dq = -dq
    return dq / eps


# --------------------------------------------------------------------------
# nominal demand + modulation
# --------------------------------------------------------------------------
@dataclass
class ModulationConfig:
    h_nom: str = "freeze"     # freeze | zero
    tau_d: str = "nominal"    # nominal | zero
    gap_tol: float = 1e-6     # |d| below this is treated as "no gap" [N m]

    @classmethod
    def from_config(cls, cfg):
        m = getattr(getattr(cfg, "torque_gap", None), "modulation", None)
        if m is None:
            return cls()
        return cls(h_nom=str(getattr(m, "h_nom", "freeze")),
                   tau_d=str(getattr(m, "tau_d", "nominal")),
                   gap_tol=float(getattr(m, "gap_tol", 1e-6)))


def nominal_demand(traj, t, prm, H_nom, tau_d):
    """Nominal total control moment [N m] demanded by the attitude reference."""
    w = np.asarray(traj.omega_ref(t), float)
    wdot = omega_dot_ref(traj, t)
    return prm.J @ wdot + np.cross(w, prm.J @ w + np.asarray(H_nom, float)) - np.asarray(tau_d, float)


def modulation(tau_nom, tau_min, tau_max):
    """Surplus / gap vectors (all N m, same shape as the box)."""
    tau_nom = np.asarray(tau_nom, float)
    tau_min = np.asarray(tau_min, float)
    tau_max = np.asarray(tau_max, float)
    r_plus = tau_max - tau_nom
    r_minus = tau_nom - tau_min
    d = tau_nom - np.clip(tau_nom, tau_min, tau_max)
    return dict(R_plus=r_plus, R_minus=r_minus, d=d)


def gap_onset_time(times, d, tol=1e-6):
    """First time at which the gap becomes non-zero (None when there is none)."""
    mag = np.max(np.abs(np.atleast_2d(d)), axis=-1)
    idx = np.flatnonzero(mag > tol)
    if idx.size == 0:
        return None
    return float(np.asarray(times, float)[idx[0]])


def gap_by_axis_onset(times, d, tol=1e-6):
    """Per-axis first non-zero gap time (3 entries, None where there is none)."""
    d = np.atleast_2d(d)
    out = []
    for j in range(d.shape[1]):
        idx = np.flatnonzero(np.abs(d[:, j]) > tol)
        out.append(float(np.asarray(times, float)[idx[0]]) if idx.size else None)
    return out


def capability_residual(gamma, d, cluster, v_max):
    """E_k(gamma) = min_{|v_i| <= v_max} || -A(gamma) v - d ||^2 (exact inner solve).

    The minimisation is a 4-variable box-constrained least squares, solved exactly
    by bounded-variable LSQ.  Used for offline verification / diagnostics only:
    the optimiser carries `v` as an explicit decision variable instead of nesting
    this non-smooth inner minimum in the NLP.
    """
    from scipy.optimize import lsq_linear

    A = cluster.jacobian(np.asarray(gamma, float))
    d = np.asarray(d, float)
    if not np.any(np.abs(d) > 0.0):
        return 0.0, np.zeros(A.shape[1])
    res = lsq_linear(A, -d, bounds=(-float(v_max), float(v_max)), method="bvls",
                     tol=1e-12, max_iter=500)
    r = -A @ res.x - d
    return float(r @ r), res.x
