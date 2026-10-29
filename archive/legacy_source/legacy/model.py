"""Unified pitch-axis model: aerodynamics + servo + reaction wheel + rigid body.

State  x = [theta, q, delta, M_rw, h_w]      (rad, rad/s, rad, N m, N m s)
Input  u = [delta_c, M_rw_c]                 (rad, N m)
Param  p = [rho, V, gamma, M_d]              (kg/m^3, m/s, rad, N m)

Equations
    theta_dot = q
    Iy q_dot  = M_a + M_rw + M_d
    delta_dot = (delta_c - delta)/tau_s
    M_rw_dot  = (M_rw_c - M_rw)/tau_w
    h_w_dot   = -M_rw                    (from pitch angular-momentum balance)

Actuator envelope (key modelling point):
    |M_rw| <= min(M_stall, P_max J_w / |h_w|)
i.e. stored momentum and deliverable torque trade off through drive power.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.linalg import expm


@dataclass
class ModelParams:
    # vehicle / aero
    Iy: float = 8500.0
    S: float = 4.2
    cbar: float = 3.2
    Cm0: float = 0.0
    Cm_alpha: float = -0.35
    Cm_q: float = -12.0
    Cm_delta: float = 0.45
    # servo
    delta_max: float = np.deg2rad(25.0)
    delta_dot_max: float = np.deg2rad(60.0)
    tau_s: float = 0.05
    # reaction wheel
    Jw: float = 0.35
    Omega_max: float = 5000.0 * 2.0 * np.pi / 60.0
    M_stall: float = 60.0
    P_max: float = 3000.0
    tau_w: float = 0.03
    M_cmd_max: float = 60.0
    dM_cmd_max: float = 2000.0
    # disturbance
    Md0: float = 20.0
    Md_amp: float = 15.0
    Md_freq: float = 0.05
    # switches / options
    wheel_on: bool = True
    ideal: bool = False
    rho_scale: float = 1.0
    cm_scale: float = 1.0

    @property
    def h_max(self) -> float:
        return self.Jw * self.Omega_max

    @classmethod
    def from_config(cls, cfg, **overrides) -> "ModelParams":
        veh, srv, whl, dis = cfg.vehicle, cfg.servo, cfg.wheel, cfg.disturbance
        kw = dict(
            Iy=float(veh.Iy), S=float(veh.S), cbar=float(veh.cbar),
            Cm0=float(veh.Cm0), Cm_alpha=float(veh.Cm_alpha),
            Cm_q=float(veh.Cm_q), Cm_delta=float(veh.Cm_delta),
            delta_max=np.deg2rad(float(srv.delta_max_deg)),
            delta_dot_max=np.deg2rad(float(srv.delta_dot_max_deg_s)),
            tau_s=float(srv.tau_s),
            Jw=float(whl.Jw),
            Omega_max=float(whl.Omega_max_rpm) * 2.0 * np.pi / 60.0,
            M_stall=float(whl.M_stall), P_max=float(whl.P_max),
            tau_w=float(whl.tau_w), M_cmd_max=float(whl.M_cmd_max),
            dM_cmd_max=float(whl.dM_cmd_max),
            Md0=float(dis.Md0), Md_amp=float(dis.Md_amp), Md_freq=float(dis.Md_freq),
        )
        kw.update(overrides)
        prm = cls(**kw)
        if prm.ideal:
            # ideal actuators: negligible lag only; all physical limits kept
            prm.tau_s = 0.01
            prm.tau_w = 0.01
        return prm


# --------------------------------------------------------------------------
# nonlinear model
# --------------------------------------------------------------------------
def aero_Q(p, prm: ModelParams) -> float:
    return 0.5 * prm.rho_scale * p[0] * p[1] ** 2 * prm.S * prm.cbar


def aero_moment(x, p, prm: ModelParams) -> float:
    Q = aero_Q(p, prm)
    alpha = x[0] - p[2]
    Cm = prm.cm_scale * (prm.Cm0
                         + prm.Cm_alpha * alpha
                         + prm.Cm_q * x[1] * prm.cbar / (2.0 * p[1])
                         + prm.Cm_delta * x[2])
    return Q * Cm


def dynamics(x, u, p, prm: ModelParams) -> np.ndarray:
    Ma = aero_moment(x, p, prm)
    M_rw = x[3] if prm.wheel_on else 0.0
    dx = np.empty(5)
    dx[0] = x[1]
    dx[1] = (Ma + M_rw + p[3]) / prm.Iy
    dx[2] = (u[0] - x[2]) / prm.tau_s
    if prm.wheel_on:
        dx[3] = (u[1] - x[3]) / prm.tau_w
        dx[4] = -x[3]
    else:
        dx[3] = 0.0
        dx[4] = 0.0
    return dx


def jacobian(x, p, prm: ModelParams):
    """Continuous-time Jacobians A = df/dx, B = df/du (B is constant)."""
    Q = aero_Q(p, prm)
    V = max(p[1], 1.0)
    A = np.zeros((5, 5))
    B = np.zeros((5, 2))
    A[0, 1] = 1.0
    A[1, 0] = Q * prm.cm_scale * prm.Cm_alpha / prm.Iy
    A[1, 1] = Q * prm.cm_scale * prm.Cm_q * prm.cbar / (2.0 * V) / prm.Iy
    A[1, 2] = Q * prm.cm_scale * prm.Cm_delta / prm.Iy
    A[2, 2] = -1.0 / prm.tau_s
    B[2, 0] = 1.0 / prm.tau_s
    if prm.wheel_on:
        A[1, 3] = 1.0 / prm.Iy
        A[3, 3] = -1.0 / prm.tau_w
        A[4, 3] = -1.0
        B[1, 1] = 1.0 / prm.Iy
        B[3, 1] = 1.0 / prm.tau_w
    return A, B


def discretize(x, u, p, prm: ModelParams, dt: float):
    """Exact ZOH discretisation of the affine model around (x,u,p)."""
    A, B = jacobian(x, p, prm)
    f0 = dynamics(x, u, p, prm) - A @ x - B @ u
    M = np.zeros((8, 8))
    M[:5, :5] = A
    M[:5, 5] = f0
    M[:5, 6:8] = B
    E = expm(M * dt)
    return E[:5, :5], E[:5, 6:8], E[:5, 5]


def wheel_avail(h_w: float, prm: ModelParams) -> float:
    """Usable wheel torque magnitude at stored momentum h_w."""
    return float(min(prm.M_stall, prm.P_max * prm.Jw / max(abs(h_w), 1e-6)))


def limit_commands(x, u, prm: ModelParams) -> np.ndarray:
    """Physical command box: servo position/rate reachability and motor command."""
    u = np.array(u, dtype=float).reshape(2)
    lo = max(-prm.delta_max, x[2] - prm.tau_s * prm.delta_dot_max)
    hi = min(prm.delta_max, x[2] + prm.tau_s * prm.delta_dot_max)
    u[0] = min(max(u[0], lo), hi)
    if prm.wheel_on:
        u[1] = min(max(u[1], -prm.M_cmd_max), prm.M_cmd_max)
    else:
        u[1] = 0.0
    return u


def plant_step(x, u_cmd, p, dt: float, prm: ModelParams, substeps: int = 1):
    """RK4 integration of the truth model plus saturation effects."""
    x = np.array(x, dtype=float)
    u = limit_commands(x, u_cmd, prm)
    h = dt / float(substeps)
    for _ in range(substeps):
        k1 = dynamics(x, u, p, prm)
        k2 = dynamics(x + 0.5 * h * k1, u, p, prm)
        k3 = dynamics(x + 0.5 * h * k2, u, p, prm)
        k4 = dynamics(x + h * k3, u, p, prm)
        x = x + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
    x[2] = float(np.clip(x[2], -prm.delta_max, prm.delta_max))
    if prm.wheel_on:
        av = wheel_avail(x[4], prm)
        x[3] = float(np.clip(x[3], -av, av))
        x[4] = float(np.clip(x[4], -prm.h_max, prm.h_max))
        if abs(x[4]) >= prm.h_max - 1e-9 and np.sign(x[3]) == -np.sign(x[4]):
            x[3] = 0.0
    else:
        x[3] = 0.0
        x[4] = 0.0
    return x, u


def rollout(x0, u_seq, p_seq, dts, prm: ModelParams, substep: float = 0.01):
    """Nonlinear rollout used for MPC initialisation / diagnostics."""
    x = np.array(x0, dtype=float)
    xs = [x.copy()]
    for u, p, dt in zip(u_seq, p_seq, dts):
        n = max(1, int(np.ceil(dt / substep)))
        for _ in range(n):
            x, _ = plant_step(x, u, p, dt / n, prm, substeps=1)
        xs.append(x.copy())
    return np.array(xs)
