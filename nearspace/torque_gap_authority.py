"""Predicted three-axis aerodynamic torque authority (time-varying box).

The optimiser is given, at every prediction node, a per-axis interval for the
*total* aerodynamic moment

    tau_min[k] <= tau_a[k] <= tau_max[k]                                    (3 axes)

and nothing else about the surfaces: no deflection, no servo dynamics.

Total aerodynamic moment (identical definition to nearspace.model6.aero_moment,
which is what the 18-state truth model uses):

    tau_aero = Q S [ b Cl , cbar Cm , b Cn ]
    Cl = Cl_beta beta + Cl_p p bhat + Cl_r r bhat + Cl_da da + Cl_dr dr
    Cm = Cm0 + Cm_alpha alpha + Cm_q q chat + Cm_de de
    Cn = Cn_beta beta + Cn_p p bhat + Cn_r r bhat + Cn_da da + Cn_dr dr

Split into a state-dependent part (natural + rate damping) evaluated at the
*nominal/reference* flight state and a linear surface part

    tau_aero = tau_nat(Q, alpha, beta, omega_ref) + M_delta(Q) delta ,
    M_delta  = Q S [[b Cl_da, 0, b Cl_dr],
                    [0, cbar Cm_de, 0],
                    [b Cn_da, 0, b Cn_dr]] ,   |delta_i| <= delta_max_i

Because Cl_dr and Cn_da couple the roll and yaw rows, the three axes are NOT
independently realisable.  The box is therefore built from the exact per-axis
range of the reachable set {M_delta delta : |delta| <= delta_max} (which is the
axis-aligned hull of that set), and a conservative uniform scale is calibrated
offline so that random interior samples of the box are realisable by a bounded
least-squares deflection solve at the required rate.  Both the final scale and
the sample pass fraction are written to the run metadata.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.optimize import lsq_linear

from .model6 import aero_coeffs, flow_angles


# --------------------------------------------------------------------------
# aerodynamic moment decomposition
# --------------------------------------------------------------------------
def surface_matrix(Q, veh):
    """M_delta (3x3): the linear map from delta = [de, da, dr] to the aero moment.

    Column order follows the state layout of nearspace.model6 / model6c, i.e.
    delta = [elevator, aileron, rudder]; row order is [roll, pitch, yaw].
    """
    M = np.zeros((3, 3))
    M[0, 1] = Q * veh.S * veh.b * veh.Cl_da
    M[0, 2] = Q * veh.S * veh.b * veh.Cl_dr
    M[1, 0] = Q * veh.S * veh.cbar * veh.Cm_de
    M[2, 1] = Q * veh.S * veh.b * veh.Cn_da
    M[2, 2] = Q * veh.S * veh.b * veh.Cn_dr
    return M


def natural_moment_v(Q, alpha, beta, omega, V, veh):
    """Aero moment at zero deflection; V [m/s] sets the non-dimensional rates."""
    V = max(float(V), 1.0)
    chat = veh.cbar / (2.0 * V)
    bhat = veh.b / (2.0 * V)
    p_rate, q_rate, r_rate = (float(omega[0]), float(omega[1]), float(omega[2]))
    Cl = veh.Cl_beta * beta + veh.Cl_p * p_rate * bhat + veh.Cl_r * r_rate * bhat
    Cm = veh.Cm0 + veh.Cm_alpha * alpha + veh.Cm_q * q_rate * chat
    Cn = veh.Cn_beta * beta + veh.Cn_p * p_rate * bhat + veh.Cn_r * r_rate * bhat
    return Q * veh.S * np.array([veh.b * Cl, veh.cbar * Cm, veh.b * Cn])


def aero_moment_of_deflection(Q, alpha, beta, omega, V, delta, veh):
    """Total aerodynamic moment for a given deflection (same口径 as model6c)."""
    nat = natural_moment_v(Q, alpha, beta, omega, V, veh)
    return nat + surface_matrix(Q, veh) @ np.asarray(delta, float)


def axis_hull(Q, veh):
    """Per-axis half width of {M_delta delta : |delta_i| <= delta_max_i}."""
    M = surface_matrix(Q, veh)
    dmax = np.asarray(veh.delta_max, float)
    return np.abs(M) @ dmax


# --------------------------------------------------------------------------
# deflection allocation (used by the P1 truth plant and by the feasibility test)
# --------------------------------------------------------------------------
def allocate_deflection(tau_residual, Q, veh, delta_prev=None, ddelta_max=None):
    """Bounded least squares: min ||M delta - tau_residual||, |delta| <= delta_max.

    Q may be a scalar or a 3x3 matrix; the latter is used when the caller already
    assembled M_delta (cheaper in inner loops).
    """
    M = Q if np.ndim(Q) == 2 else surface_matrix(Q, veh)
    dmax = np.asarray(veh.delta_max, float)
    lo, hi = -dmax.copy(), dmax.copy()
    if delta_prev is not None:
        prev = np.asarray(delta_prev, float)
        span = np.asarray(ddelta_max, float) if ddelta_max is not None else dmax
        lo = np.maximum(lo, prev - span)
        hi = np.minimum(hi, prev + span)
    res = lsq_linear(M, np.asarray(tau_residual, float), bounds=(lo, hi),
                     method="bvls", tol=1e-10, max_iter=400)
    d = np.clip(res.x, lo, hi)
    return d, M @ d - np.asarray(tau_residual, float)


def realizable_fraction(Q, veh, scale, n_samples=1000, seed=0, tol_frac=1e-3):
    """Fraction of uniformly sampled box targets reachable by a bounded LSQ solve."""
    rng = np.random.default_rng(seed)
    half = float(scale) * axis_hull(Q, veh)
    half = np.maximum(half, 1e-12)
    targets = (rng.uniform(-1.0, 1.0, size=(n_samples, 3))) * half[None, :]
    M = surface_matrix(Q, veh)
    ok = 0
    worst = 0.0
    for tgt in targets:
        _, res = allocate_deflection(tgt, M, veh)
        rel = float(np.max(np.abs(res) / half))
        worst = max(worst, rel)
        if rel <= tol_frac:
            ok += 1
    return ok / float(n_samples), worst


def calibrate_scale(veh, Q_samples, require_frac=0.99, n_samples=1000, seed=0,
                    tol_frac=1e-3, min_scale=0.2, step=0.95):
    """Largest uniform scale whose box is realisable at `require_frac`.

    Returns (scale, diagnostics list).  The scale is applied to the per-axis half
    widths of the reachable-set hull, which makes the box strictly inside the set.
    """
    scale = 1.0
    diag = []
    while scale >= min_scale:
        fracs = []
        for Q in Q_samples:
            frac, worst = realizable_fraction(Q, veh, scale, n_samples=n_samples,
                                              seed=seed, tol_frac=tol_frac)
            fracs.append((float(Q), frac, worst))
        fmin = min(f for _, f, _ in fracs)
        diag.append(dict(scale=float(scale), min_fraction=float(fmin),
                         per_Q=[dict(Q=q, fraction=f, worst_rel=w) for q, f, w in fracs]))
        if fmin >= require_frac:
            return float(scale), diag
        scale *= step
    return float(min_scale), diag


# --------------------------------------------------------------------------
# time-varying box along the predicted trajectory
# --------------------------------------------------------------------------
@dataclass
class AuthorityConfig:
    mode: str = "timevarying"        # timevarying | fixed
    fixed_h_km: float = 100.0        # used by mode == 'fixed' (case C5)
    scale: float = 0.9               # conservative scale calibrated offline
    calibrate: bool = True
    require_frac: float = 0.99
    n_samples: int = 1000
    tol_frac: float = 1e-3

    @classmethod
    def from_config(cls, cfg):
        # the block lives under cfg.torque_gap.authority (cfg.authority does not
        # exist; reading it silently kept the defaults and dropped case C5)
        a = getattr(getattr(cfg, "torque_gap", None), "authority", None)
        if a is None:
            return cls()

        def g(name, default):
            return getattr(a, name, default)

        return cls(mode=str(g("mode", "timevarying")),
                   fixed_h_km=float(g("fixed_h_km", 100.0)),
                   scale=float(g("scale", 0.9)),
                   calibrate=bool(g("calibrate", True)),
                   require_frac=float(g("require_frac", 0.99)),
                   n_samples=int(g("n_samples", 1000)),
                   tol_frac=float(g("tol_frac", 1e-3)))


def build_boxes(times, cond, veh, auth: AuthorityConfig, rho_override=None):
    """Per-node aerodynamic torque box.

    Parameters
    ----------
    times : (K,) node times [s]
    cond  : dict with arrays 'rho'(K,), 'V'(K,), 'V_ned'(K,3), 'q_ref'(K,4),
            'omega_ref'(K,3) -- all evaluated on the *reference* trajectory
    rho_override : scalar
            Freeze the density (hence the authority) at one flight condition for
            the whole horizon; this is the fixed-boundary control arm (case C5).
            Airspeed still varies along the prediction window.
    """
    times = np.asarray(times, float)
    K = times.size
    tau_nat = np.zeros((K, 3))
    lo = np.zeros((K, 3))
    hi = np.zeros((K, 3))
    Qs = np.zeros(K)
    for k in range(K):
        rho = float(rho_override) if rho_override is not None else float(cond["rho"][k])
        Q = 0.5 * rho * float(cond["V"][k]) ** 2
        Qs[k] = Q
    for k in range(K):
        Q = Qs[k]
        V = max(float(cond["V"][k]), 1.0)
        rho = float(rho_override) if rho_override is not None else float(cond["rho"][k])
        p = np.concatenate([[rho], np.asarray(cond["V_ned"][k], float), np.zeros(3)])
        xprobe = np.concatenate([np.asarray(cond["q_ref"][k], float),
                                 np.asarray(cond["omega_ref"][k], float), np.zeros(9)])
        alpha, beta, _ = flow_angles(xprobe, p)
        nat = natural_moment_v(Q, alpha, beta, cond["omega_ref"][k], V, veh)
        half = float(auth.scale) * axis_hull(Q, veh)
        tau_nat[k] = nat
        lo[k] = nat - half
        hi[k] = nat + half
    return dict(tau_min=lo, tau_max=hi, tau_nat=tau_nat, Q=Qs)


def clip_to_box(tau, lo, hi):
    return np.clip(np.asarray(tau, float), np.asarray(lo, float), np.asarray(hi, float))
