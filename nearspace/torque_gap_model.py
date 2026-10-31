"""Predicted torque-gap joint attitude/CMG model -- P0 virtual-torque plant.

State  x = [q(4), omega(3), gamma(4)]                                 (11)
Input  u = [tau_a(3), nu(4)]                                          (7)

    q_dot        = 0.5 q (x) [0, omega]
    J omega_dot  = tau_a - A(gamma) nu + tau_d - omega x (J omega + H(gamma))
    gamma_dot    = nu

`tau_a` is the *total* three-axis aerodynamic moment [N m] produced by the
aerodynamic surfaces (natural + damping + surface terms, see
nearspace.torque_gap_authority).  It is not a deflection command.  `nu` is the
CMG gimbal rate [rad/s].  The CMG cluster geometry, momentum H(gamma) and
Jacobian A(gamma) are the ones from nearspace.cmg (4-unit pyramid, A = dh/dgamma,
M_cmg = -A nu), so the truth plant and the optimiser share one geometry.

This module is the numpy (truth / validation) side.  The CasADi graph used by the
optimiser is assembled in nearspace.torque_gap_ocp from `cluster_constants()` so
that exactly the same closed-form geometry is used in both.

Conventions (identical to nearspace.model6 / model6c):
    * quaternion scalar-first Hamilton, q maps body -> NED,
    * omega = [p, q, r] body rates, J diagonal body inertia,
    * gamma in rad, nu in rad/s, torques in N m.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from .cmg import CMGCluster, CMGParams
from .model6 import (VehicleParams, quat_conj, quat_mul, quat_normalize,
                     quat_to_R, euler_to_quat, flow_angles)

NX_TG, NU_TG = 11, 7


# --------------------------------------------------------------------------
# packed state helpers
# --------------------------------------------------------------------------
def pack_state(q, omega, gamma):
    return np.concatenate([np.asarray(q, float).ravel()[:4],
                           np.asarray(omega, float).ravel()[:3],
                           np.asarray(gamma, float).ravel()[:4]])


def split_state(x):
    x = np.asarray(x, float)
    return x[0:4].copy(), x[4:7].copy(), x[7:11].copy()


def pack_input(tau_a, nu):
    return np.concatenate([np.asarray(tau_a, float).ravel()[:3],
                           np.asarray(nu, float).ravel()[:4]])


def split_input(u):
    u = np.asarray(u, float)
    return u[0:3].copy(), u[3:7].copy()


# --------------------------------------------------------------------------
# parameters
# --------------------------------------------------------------------------
@dataclass
class TorquePlantParams:
    """Vehicle + CMG parameters of the 11-state model (all SI)."""

    veh: VehicleParams
    cmg: CMGParams

    @classmethod
    def from_config(cls, cfg):
        return cls(veh=VehicleParams.from_config(cfg), cmg=CMGParams.from_config(cfg))

    @property
    def J(self):
        v = self.veh
        return np.diag([v.Jx, v.Jy, v.Jz])

    @property
    def Jinv(self):
        v = self.veh
        return np.diag([1.0 / v.Jx, 1.0 / v.Jy, 1.0 / v.Jz])

    def cluster(self) -> CMGCluster:
        return CMGCluster(self.cmg)

    def cluster_constants(self) -> dict:
        """Closed-form pyramid geometry, identical to CMGCluster.__init__."""
        cl = CMGCluster(self.cmg)
        return dict(h0=float(self.cmg.h0), g_axis=cl.g_axis.copy(),
                    s0=cl.s0.copy(), gxs0=cl.gxs0.copy())


# --------------------------------------------------------------------------
# dynamics
# --------------------------------------------------------------------------
def dynamics_tg(x, u, tau_d, prm: TorquePlantParams, cluster: CMGCluster):
    """Time derivative of the 11 states with u = [tau_a(3), nu(4)]."""
    q, w, gamma = split_state(x)
    tau_a, nu = split_input(u)
    H = cluster.momentum(gamma)
    A = cluster.jacobian(gamma)
    dx = np.empty(NX_TG)
    dx[0:4] = 0.5 * quat_mul(q, np.array([0.0, w[0], w[1], w[2]]))
    dx[4:7] = prm.Jinv @ (tau_a - A @ nu + tau_d - np.cross(w, prm.J @ w + H))
    dx[7:11] = nu
    return dx


def rk4_step_tg(x, u, tau_d, dt, prm: TorquePlantParams, cluster: CMGCluster,
                normalize=True):
    """Classic RK4 step of the 11-state model (quaternion renormalised)."""
    k1 = dynamics_tg(x, u, tau_d, prm, cluster)
    k2 = dynamics_tg(x + 0.5 * dt * k1, u, tau_d, prm, cluster)
    k3 = dynamics_tg(x + 0.5 * dt * k2, u, tau_d, prm, cluster)
    k4 = dynamics_tg(x + dt * k3, u, tau_d, prm, cluster)
    xn = x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
    if normalize:
        xn[0:4] = quat_normalize(xn[0:4])
    return xn


def plant_step_tg(x, u_cmd, tau_d, dt, prm: TorquePlantParams, cluster: CMGCluster,
                  substeps=4, gamma_lim=None):
    """Advance the P0 truth plant one control interval with held input.

    The commanded input is first clipped to what the actuators can deliver:
    `tau_a` to the (time-varying) aerodynamic box is the caller's job because the
    box is time dependent; here only the CMG rate / gimbal travel limits apply.
    """
    x = np.asarray(x, float).copy()
    u = np.asarray(u_cmd, float).copy()
    u[3:7] = np.clip(u[3:7], -prm.cmg.gdot_max, prm.cmg.gdot_max)
    h = dt / float(max(int(substeps), 1))
    for i in range(int(max(substeps, 1))):
        x = rk4_step_tg(x, u, tau_d if np.ndim(tau_d) == 1 else tau_d[i], h, prm, cluster)
    x[7:11] = np.clip(x[7:11], -prm.cmg.gdot_max, prm.cmg.gdot_max)
    if gamma_lim is not None:
        x[7:11] = np.clip(x[7:11], gamma_lim[0], gamma_lim[1])
    return x, u


# --------------------------------------------------------------------------
# attitude error / reference helpers
# --------------------------------------------------------------------------
def quat_error(q_ref, q):
    """Error quaternion q_ref^{-1} (x) q (short arc, scalar part >= 0)."""
    qe = quat_mul(quat_conj(quat_normalize(np.asarray(q_ref, float))),
                  quat_normalize(np.asarray(q, float)))
    return -qe if qe[0] < 0.0 else qe


def attitude_error_vector(q_ref, q):
    """e_R in R^3 (rad, short arc): 2 * vector part of the error quaternion."""
    return 2.0 * quat_error(q_ref, q)[1:4]


def attitude_angle(q_ref, q):
    qe = quat_error(q_ref, q)
    return 2.0 * float(np.arccos(min(1.0, abs(qe[0]))))


def flow_state(q, omega, rho, V_ned, prm: TorquePlantParams):
    """(alpha, beta, Q) of the reference/nominal flight condition."""
    xprobe = np.concatenate([np.asarray(q, float), np.asarray(omega, float), np.zeros(9)])
    p = np.concatenate([[rho], np.asarray(V_ned, float), np.zeros(3)])
    alpha, beta, V = flow_angles(xprobe, p)
    Q = 0.5 * float(rho) * float(np.dot(V_ned, V_ned))
    return alpha, beta, Q


def cross_matrix(v):
    v = np.asarray(v, float)
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])
