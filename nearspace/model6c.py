"""Three-axis vehicle model with a 4-unit single-gimbal CMG cluster.

State  x = [q_bn(4), omega(3), delta_aero(3), gamma(4), gamma_dot(4)]   (18)
Input  u = [delta_c(3), gamma_dot_c(4)]                                 (7)
Param  p = [rho, V_ned(3), M_d(3)]                                      (7)

    q_dot        = 0.5 q (x) [0, omega]
    J omega_dot  = -omega x (J omega + H_cmg(gamma)) - A(gamma) gamma_dot
                   + M_aero + M_d
    delta_dot    = (delta_c - delta)/tau_s
    gamma_dot    = gamma_dot                       (gimbal angle kinematics)
    d(gamma_dot) = (gamma_dot_c - gamma_dot)/tau_g

The CMG cluster output torque is algebraic: M_cmg = -A(gamma) gamma_dot, with
A = dh/dgamma (3x4) from nearspace.cmg. Rotor speed is regulated, so h0 is
constant. The controller commands gimbal RATES directly, i.e. the MPC itself is
the steering law.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.linalg import expm

from .cmg import CMGCluster, CMGParams
from .model6 import VehicleParams, aero_moment, flow_angles, quat_mul, quat_normalize

NX, NU = 18, 7


@dataclass
class VehicleCMGParams:
    veh: VehicleParams
    cmg: CMGParams
    sigma_floor: float = 0.30      # fraction of the home-config sigma_min
    # Actuator mode.  cmg_on=False is the "A: aerodynamic surfaces only"
    # configuration: the gimbals are FROZEN at the home configuration, no CMG
    # torque reaches the body and no CMG momentum enters the gyroscopic term.
    # This is switched in the truth model, not only in the optimiser, so case A
    # really is a vehicle without a CMG and not a controller that declines to
    # use one (implementation plan §3.1).
    cmg_on: bool = True

    @classmethod
    def from_config(cls, cfg, **over):
        veh = over.pop("veh", None) or VehicleParams.from_config(cfg)
        cmg = over.pop("cmg", None) or CMGParams.from_config(cfg)
        return cls(veh=veh, cmg=cmg, **over)

    @property
    def J(self):
        v = self.veh
        return np.diag([v.Jx, v.Jy, v.Jz])

    @property
    def Jinv(self):
        return np.diag([1.0 / self.veh.Jx, 1.0 / self.veh.Jy, 1.0 / self.veh.Jz])


def make_cluster(prm: VehicleCMGParams) -> CMGCluster:
    return CMGCluster(prm.cmg)


# --------------------------------------------------------------------------
# nonlinear model
# --------------------------------------------------------------------------
def cmg_momentum(x, cluster: CMGCluster):
    return cluster.momentum(x[10:14])


def cmg_torque(x, cluster: CMGCluster):
    return cluster.torque(x[10:14], x[14:18])


def aero_moment6c(x, p, prm: VehicleCMGParams):
    """Aero moment from the 3-axis aero model (same coefficients as model6)."""
    from .model6 import aero_coeffs
    v = prm.veh
    xa = np.concatenate([x[0:7], x[7:10], np.zeros(3), np.zeros(3)])
    Q = 0.5 * p[0] * float(np.dot(p[1:4], p[1:4]))
    Cl, Cm, Cn = aero_coeffs(xa, p, v)
    return Q * v.S * np.array([v.b * Cl, v.cbar * Cm, v.b * Cn])


def dynamics6c(x, u, p, prm: VehicleCMGParams, cluster: CMGCluster):
    v = prm.veh
    q = x[0:4]
    w = x[4:7]
    gamma = x[10:14]
    gdot = x[14:18]
    dx = np.empty(NX)
    dx[0:4] = 0.5 * quat_mul(q, np.array([0.0, w[0], w[1], w[2]]))
    if prm.cmg_on:
        Hc = cluster.momentum(gamma)
        Mc = cluster.torque(gamma, gdot)
        dx[10:14] = gdot
        dx[14:18] = (u[3:7] - gdot) / prm.cmg.tau_g
    else:
        Hc = np.zeros(3)
        Mc = np.zeros(3)
        dx[10:14] = 0.0
        dx[14:18] = 0.0
    Mtot = aero_moment6c(x, p, prm) + p[4:7] + Mc
    dx[4:7] = prm.Jinv @ (-np.cross(w, prm.J @ w + Hc) + Mtot)
    dx[7:10] = (u[0:3] - x[7:10]) / v.tau_s
    return dx


def jacobian6c(x, u, p, prm: VehicleCMGParams, cluster: CMGCluster):
    A = np.zeros((NX, NX))
    B = np.zeros((NX, NU))
    for i in range(NX):
        h = 1e-6 * max(1.0, abs(x[i]))
        xp = x.copy(); xp[i] += h
        xm = x.copy(); xm[i] -= h
        A[:, i] = (dynamics6c(xp, u, p, prm, cluster)
                   - dynamics6c(xm, u, p, prm, cluster)) / (2.0 * h)
    for j in range(NU):
        h = 1e-6 * max(1.0, abs(u[j]))
        up = u.copy(); up[j] += h
        um = u.copy(); um[j] -= h
        B[:, j] = (dynamics6c(x, up, p, prm, cluster)
                   - dynamics6c(x, um, p, prm, cluster)) / (2.0 * h)
    return A, B


def discretize6c(x, u, p, prm: VehicleCMGParams, cluster: CMGCluster, dt):
    A, B = jacobian6c(x, u, p, prm, cluster)
    f0 = dynamics6c(x, u, p, prm, cluster) - A @ x - B @ u
    M = np.zeros((NX + 1 + NU, NX + 1 + NU))
    M[:NX, :NX] = A
    M[:NX, NX] = f0
    M[:NX, NX + 1:] = B
    E = expm(M * dt)
    return E[:NX, :NX], E[:NX, NX + 1:], E[:NX, NX]


def limit_commands6c(x, u, prm: VehicleCMGParams):
    v, c = prm.veh, prm.cmg
    u = np.array(u, dtype=float).reshape(NU)
    lo = np.maximum(-v.delta_max, x[7:10] - v.tau_s * v.delta_dot_max)
    hi = np.minimum(v.delta_max, x[7:10] + v.tau_s * v.delta_dot_max)
    u[0:3] = np.clip(u[0:3], lo, hi)
    u[3:7] = np.clip(u[3:7], -c.gdot_max, c.gdot_max)
    if not prm.cmg_on:
        u[3:7] = 0.0
    return u


def plant_step6c(x, u_cmd, p, dt, prm: VehicleCMGParams, cluster: CMGCluster, substeps=1):
    x = np.array(x, dtype=float)
    u = limit_commands6c(x, u_cmd, prm)
    h = dt / float(substeps)
    for _ in range(substeps):
        k1 = dynamics6c(x, u, p, prm, cluster)
        k2 = dynamics6c(x + 0.5 * h * k1, u, p, prm, cluster)
        k3 = dynamics6c(x + 0.5 * h * k2, u, p, prm, cluster)
        k4 = dynamics6c(x + h * k3, u, p, prm, cluster)
        x = x + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
    x[0:4] = quat_normalize(x[0:4])
    x[7:10] = np.clip(x[7:10], -prm.veh.delta_max, prm.veh.delta_max)
    x[14:18] = np.clip(x[14:18], -prm.cmg.gdot_max, prm.cmg.gdot_max)
    return x, u
