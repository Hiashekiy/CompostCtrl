"""Aerodynamic / flight-condition diagnostics used by the S08 experiments.

This module only *observes* the frozen model.  It never enters a dynamics,
constraint or cost function, so adding it cannot change any simulation result.

Where each quantity actually comes from (S08 code audit §4):

    dynamic pressure  qbar = 0.5 * rho * |V_ned|^2
                      rho from the atmosphere backend along the analytic skip
                      profile, V_ned from the same profile (nearspace.trajectory3);
                      identical to the Q used by nearspace.model6.qinf and by the
                      aerodynamic torque box of nearspace.torque_gap_authority.
    angle of attack    alpha = atan2(w_b, u_b) with V_b = R(q)^T V_ned, i.e. the
                      actual body x axis against the actual relative wind, not
                      theta_ref.  nearspace.model6.flow_angles.
    aero coefficients  Cm = Cm0 + Cm_alpha alpha + Cm_q q_chat + Cm_de delta_e
                      (nearspace.model6.aero_coeffs).  CL and CD are NOT part of
                      this model: it is an attitude-only (rotational) model with
                      no aerodynamic force, so there is no lift or drag anywhere
                      in the simulation.
    Mach, Knudsen      from the atmosphere backend (speed of sound / mean free
                      path); they are recorded for the audit but the aero
                      coefficients do not depend on them.
"""
from __future__ import annotations

import numpy as np

from .model6 import flow_angles
from .torque_gap_authority import axis_hull, natural_moment_v


def flow_diagnostics(q, rho, V_ned, h_m, atm=None, cbar=None):
    """(alpha, beta, Mach, Knudsen, qbar) at one attitude / flight condition."""
    q = np.asarray(q, float)
    V_ned = np.asarray(V_ned, float)
    V = float(np.linalg.norm(V_ned))
    xprobe = np.concatenate([q, np.zeros(3)])
    p = np.concatenate([[float(rho)], V_ned, np.zeros(3)])
    alpha, beta, _ = flow_angles(xprobe, p)
    qbar = 0.5 * float(rho) * V ** 2
    mach = float("nan")
    kn = float("nan")
    if atm is not None:
        a = float(atm.a(float(h_m)))
        mach = V / a if a > 0.0 else float("nan")
        if cbar is not None:
            kn = float(atm.knudsen(float(h_m), float(cbar)))
    return dict(alpha_deg=float(np.rad2deg(alpha)), beta_deg=float(np.rad2deg(beta)),
                mach=mach, knudsen=kn, qbar_pa=qbar, V_rel=V)


def aero_moment_diagnostics(q, omega, delta, rho, V_ned, veh):
    """Natural and (if a deflection exists) total aerodynamic moment, and Cm."""
    V = max(float(np.linalg.norm(np.asarray(V_ned, float))), 1.0)
    Q = 0.5 * float(rho) * V ** 2
    xprobe = np.concatenate([np.asarray(q, float), np.asarray(omega, float)])
    p = np.concatenate([[float(rho)], np.asarray(V_ned, float), np.zeros(3)])
    alpha, beta, _ = flow_angles(xprobe, p)
    nat = natural_moment_v(Q, alpha, beta, omega, V, veh)
    chat = veh.cbar / (2.0 * V)
    Cm_nat = (veh.Cm0 + veh.Cm_alpha * alpha
              + veh.Cm_q * float(omega[1]) * chat)
    out = dict(alpha_deg=float(np.rad2deg(alpha)), tau_nat=nat, Cm_nat=float(Cm_nat))
    if delta is not None:
        de = float(np.asarray(delta, float)[0])
        out["Cm"] = float(Cm_nat + veh.Cm_de * de)
        out["delta_e_rad"] = de
    else:
        out["Cm"] = None
        out["delta_e_rad"] = None
    return out


def surface_capability(Q, veh):
    """Per-axis half width [N m] of the deflection-reachable aero moment set."""
    return axis_hull(float(Q), veh)


def surface_utilisation(tau_cmd, tau_nat, half):
    """Required control moment and its utilisation of the available half width.

    ``tau_req`` is what the controller asks the surfaces to add on top of the
    natural (zero-deflection) moment; ``util`` = |tau_req| / half, so 1.0 means
    the demand exactly fills the available aerodynamic control moment.  The
    absolute margin is the remaining moment [N m].
    """
    tau_cmd = np.asarray(tau_cmd, float)
    tau_nat = np.asarray(tau_nat, float)
    half = np.asarray(half, float)
    req = tau_cmd - tau_nat
    safe = np.maximum(half, 1e-9)
    util = np.abs(req) / safe
    return dict(tau_req=req, util=util, margin=half - np.abs(req))
