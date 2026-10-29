"""Three-axis (6-DOF attitude) model of the near-space vehicle.

State  x = [q_bn (4), omega (3), delta (3), tau (3), h_w (3)]        (16)
Input  u = [delta_c (3), tau_c (3)]                                 (6)
Param  p = [rho, V_n(3), M_d(3)]                                    (7)

Frames / conventions
    * NED inertial frame: x north, y east, z down.
    * Body frame: x forward, y right, z down.
    * q_bn = [w, x, y, z] rotates body vectors into NED (v_n = R(q) v_b).
    * omega = [p, q, r] body rates, delta = [de, da, dr], tau = [t1, t2, t3]
      motor torques on three orthogonal rotors, h_w = J_w * Omega.

Dynamics
    q_dot   = 0.5 * q (x) [0, omega]
    J omega_dot = -omega x (J omega + h_w) - tau + M_aero + M_d
    delta_dot   = (delta_c - delta) / tau_s
    tau_dot     = (tau_c - tau) / tau_w
    h_w_dot     = tau

Wheel envelope (per axis):  |tau_i| <= min(M_stall, P_max J_w / |h_i|)
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np
from scipy.linalg import expm

NX, NU = 16, 6


# --------------------------------------------------------------------------
# quaternion helpers (scalar-first, Hamilton)
# --------------------------------------------------------------------------
def quat_mul(a, b):
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.array([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ])


def quat_conj(q):
    return np.array([q[0], -q[1], -q[2], -q[3]])


def quat_normalize(q):
    n = np.linalg.norm(q)
    return q / n if n > 1e-12 else np.array([1.0, 0.0, 0.0, 0.0])


def quat_to_R(q):
    """Rotation matrix body -> NED."""
    w, x, y, z = q
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])


def euler_to_quat(roll, pitch, yaw):
    """ZYX (yaw-pitch-roll) Euler angles -> quaternion body->NED."""
    cr, sr = np.cos(0.5 * roll), np.sin(0.5 * roll)
    cp, sp = np.cos(0.5 * pitch), np.sin(0.5 * pitch)
    cy, sy = np.cos(0.5 * yaw), np.sin(0.5 * yaw)
    return np.array([
        cr * cp * cy + sr * sp * sy,
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
    ])


def quat_to_euler(q):
    """Quaternion body->NED -> ZYX (yaw-pitch-roll) Euler angles."""
    w, x, y, z = quat_normalize(q)
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    s = np.clip(2.0 * (w * y - z * x), -1.0, 1.0)
    pitch = np.arcsin(s)
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.array([roll, pitch, yaw])


def quat_geodesic(qa, qb):
    """Angle (rad) between two attitudes: 2*acos|qa.qb|."""
    d = float(np.dot(quat_normalize(qa), quat_normalize(qb)))
    return 2.0 * np.arccos(min(1.0, abs(d)))


def wrap_pi(a):
    return float((a + np.pi) % (2.0 * np.pi) - np.pi)


def cross_matrix(v):
    return np.array([[0.0, -v[2], v[1]],
                     [v[2], 0.0, -v[0]],
                     [-v[1], v[0], 0.0]])


# --------------------------------------------------------------------------
# parameters
# --------------------------------------------------------------------------
@dataclass
class VehicleParams:
    # mass / inertia
    mass: float = 1200.0
    Jx: float = 1200.0
    Jy: float = 8500.0
    Jz: float = 9000.0
    # aero reference geometry
    S: float = 4.2
    b: float = 2.6
    cbar: float = 3.2
    # longitudinal coefficients
    Cm0: float = 0.0
    Cm_alpha: float = -0.35
    Cm_q: float = -12.0
    Cm_de: float = 0.45
    # lateral / directional coefficients
    Cl_beta: float = -0.08
    Cl_p: float = -0.40
    Cl_r: float = 0.10
    Cl_da: float = 0.12
    Cl_dr: float = 0.02
    Cn_beta: float = 0.12
    Cn_p: float = -0.03
    Cn_r: float = -0.15
    Cn_da: float = 0.01
    Cn_dr: float = -0.10
    # servo (elevator, aileron, rudder)
    delta_max: np.ndarray = field(default_factory=lambda: np.deg2rad([25.0, 20.0, 20.0]))
    delta_dot_max: np.ndarray = field(default_factory=lambda: np.deg2rad([60.0, 60.0, 60.0]))
    tau_s: float = 0.05
    # wheel cluster
    Jw: float = 0.7
    Omega_max: float = 5000.0 * 2.0 * np.pi / 60.0
    M_stall: float = 60.0
    P_max: float = 12000.0
    tau_w: float = 0.03
    tau_cmd_max: float = 60.0
    dtau_cmd_max: float = 2000.0
    wheel_on: bool = True
    ideal: bool = False
    # disturbance
    Md0: np.ndarray = field(default_factory=lambda: np.array([5.0, 20.0, 8.0]))
    Md_amp: np.ndarray = field(default_factory=lambda: np.array([4.0, 15.0, 6.0]))
    Md_freq: np.ndarray = field(default_factory=lambda: np.array([0.03, 0.05, 0.04]))

    @property
    def h_max(self):
        return self.Jw * self.Omega_max

    @property
    def Jinv(self):
        return np.diag([1.0 / self.Jx, 1.0 / self.Jy, 1.0 / self.Jz])

    def copy(self, **over):
        import copy as _copy
        new = _copy.deepcopy(self)
        for k, v in over.items():
            setattr(new, k, v)
        if new.ideal:
            new.tau_s = 0.01
            new.tau_w = 0.01
        return new


    @classmethod
    def from_config(cls, cfg, **over):
        v = getattr(cfg, "vehicle3", None)
        if v is None:
            return cls(**over)
        kw = dict(
            mass=float(v.mass), Jx=float(v.Jx), Jy=float(v.Jy), Jz=float(v.Jz),
            S=float(v.S), b=float(v.b), cbar=float(v.cbar),
            Cm0=float(getattr(v, "Cm0", 0.0)), Cm_alpha=float(v.Cm_alpha),
            Cm_q=float(v.Cm_q), Cm_de=float(v.Cm_de),
            Cl_beta=float(v.Cl_beta), Cl_p=float(v.Cl_p), Cl_r=float(v.Cl_r),
            Cl_da=float(v.Cl_da), Cl_dr=float(v.Cl_dr),
            Cn_beta=float(v.Cn_beta), Cn_p=float(v.Cn_p), Cn_r=float(v.Cn_r),
            Cn_da=float(v.Cn_da), Cn_dr=float(v.Cn_dr),
            delta_max=np.deg2rad(np.array(v.delta_max_deg, dtype=float)),
            delta_dot_max=np.deg2rad(np.array(v.delta_dot_max_deg_s, dtype=float)),
            Jw=float(v.Jw), Omega_max=float(v.Omega_max_rpm) * 2.0 * np.pi / 60.0,
            M_stall=float(v.M_stall), P_max=float(v.P_max),
            Md0=np.array(v.Md0, dtype=float), Md_amp=np.array(v.Md_amp, dtype=float),
            Md_freq=np.array(v.Md_freq, dtype=float),
        )
        kw.update(over)
        prm = cls(**kw)
        if prm.ideal:
            prm.tau_s = 0.01
            prm.tau_w = 0.01
        return prm


# --------------------------------------------------------------------------
# nonlinear model
# --------------------------------------------------------------------------
def qinf(p, prm):
    return 0.5 * p[0] * float(np.dot(p[1:4], p[1:4]))


def flow_angles(x, p):
    """(alpha, beta, V) from the body-frame velocity."""
    Vn = p[1:4]
    V = float(np.linalg.norm(Vn))
    if V < 1.0:
        return 0.0, 0.0, 1.0
    Vb = quat_to_R(x[0:4]).T @ Vn
    u, v, w = Vb
    alpha = float(np.arctan2(w, max(u, 1e-6)))
    beta = float(np.arcsin(np.clip(v / V, -1.0, 1.0)))
    return alpha, beta, V


def aero_coeffs(x, p, prm):
    alpha, beta, V = flow_angles(x, p)
    p_rate, q_rate, r_rate = x[4], x[5], x[6]
    de, da, dr = x[7], x[8], x[9]
    chat = prm.cbar / (2.0 * max(V, 1.0))
    bhat = prm.b / (2.0 * max(V, 1.0))
    Cl = (prm.Cl_beta * beta + prm.Cl_p * p_rate * bhat + prm.Cl_r * r_rate * bhat
          + prm.Cl_da * da + prm.Cl_dr * dr)
    Cm = (prm.Cm0 + prm.Cm_alpha * alpha + prm.Cm_q * q_rate * chat + prm.Cm_de * de)
    Cn = (prm.Cn_beta * beta + prm.Cn_p * p_rate * bhat + prm.Cn_r * r_rate * bhat
          + prm.Cn_da * da + prm.Cn_dr * dr)
    return Cl, Cm, Cn


def aero_moment(x, p, prm):
    Q = qinf(p, prm)
    Cl, Cm, Cn = aero_coeffs(x, p, prm)
    return Q * prm.S * np.array([prm.b * Cl, prm.cbar * Cm, prm.b * Cn])


def dynamics3(x, u, p, prm):
    q = x[0:4]
    w = x[4:7]
    delta = x[7:10]
    tau = x[10:13]
    hw = x[13:16]

    dx = np.empty(NX)
    dx[0:4] = 0.5 * quat_mul(q, np.array([0.0, w[0], w[1], w[2]]))
    if prm.wheel_on:
        Mtot = aero_moment(x, p, prm) + p[4:7] - tau
        dx[4:7] = prm.Jinv @ (-np.cross(w, np.diag([prm.Jx, prm.Jy, prm.Jz]) @ w + hw) + Mtot)
        dx[10:13] = (u[3:6] - tau) / prm.tau_w
        dx[13:16] = tau
    else:
        Mtot = aero_moment(x, p, prm) + p[4:7]
        dx[4:7] = prm.Jinv @ (-np.cross(w, np.diag([prm.Jx, prm.Jy, prm.Jz]) @ w) + Mtot)
        dx[10:13] = 0.0
        dx[13:16] = 0.0
    dx[7:10] = (u[0:3] - delta) / prm.tau_s
    return dx


def wheel_avail3(h, prm):
    return np.minimum(prm.M_stall, prm.P_max * prm.Jw / np.maximum(np.abs(h), 1e-6))


def jacobian3(x, u, p, prm):
    """Central-difference Jacobians (16x16, 16x6). Cheap enough at this size."""
    A = np.zeros((NX, NX))
    B = np.zeros((NX, NU))
    for i in range(NX):
        hx = 1e-6 * max(1.0, abs(x[i]))
        xp = x.copy(); xp[i] += hx
        xm = x.copy(); xm[i] -= hx
        A[:, i] = (dynamics3(xp, u, p, prm) - dynamics3(xm, u, p, prm)) / (2.0 * hx)
    for j in range(NU):
        hu = 1e-6 * max(1.0, abs(u[j]))
        up = u.copy(); up[j] += hu
        um = u.copy(); um[j] -= hu
        B[:, j] = (dynamics3(x, up, p, prm) - dynamics3(x, um, p, prm)) / (2.0 * hu)
    return A, B


def discretize3(x, u, p, prm, dt):
    A, B = jacobian3(x, u, p, prm)
    f0 = dynamics3(x, u, p, prm) - A @ x - B @ u
    M = np.zeros((NX + 1 + NU, NX + 1 + NU))
    M[:NX, :NX] = A
    M[:NX, NX] = f0
    M[:NX, NX + 1:] = B
    E = expm(M * dt)
    return E[:NX, :NX], E[:NX, NX + 1:], E[:NX, NX]


def limit_commands3(x, u, prm):
    u = np.array(u, dtype=float).reshape(NU)
    lo = np.maximum(-prm.delta_max, x[7:10] - prm.tau_s * prm.delta_dot_max)
    hi = np.minimum(prm.delta_max, x[7:10] + prm.tau_s * prm.delta_dot_max)
    u[0:3] = np.clip(u[0:3], lo, hi)
    if prm.wheel_on:
        u[3:6] = np.clip(u[3:6], -prm.tau_cmd_max, prm.tau_cmd_max)
    else:
        u[3:6] = 0.0
    return u


def plant_step3(x, u_cmd, p, dt, prm, substeps=1):
    x = np.array(x, dtype=float)
    u = limit_commands3(x, u_cmd, prm)
    h = dt / float(substeps)
    for _ in range(substeps):
        k1 = dynamics3(x, u, p, prm)
        k2 = dynamics3(x + 0.5 * h * k1, u, p, prm)
        k3 = dynamics3(x + 0.5 * h * k2, u, p, prm)
        k4 = dynamics3(x + h * k3, u, p, prm)
        x = x + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
    x[0:4] = quat_normalize(x[0:4])
    x[7:10] = np.clip(x[7:10], -prm.delta_max, prm.delta_max)
    if prm.wheel_on:
        av = wheel_avail3(x[13:16], prm)
        x[10:13] = np.clip(x[10:13], -av, av)
        x[13:16] = np.clip(x[13:16], -prm.h_max, prm.h_max)
    else:
        x[10:13] = 0.0
        x[13:16] = 0.0
    return x, u
