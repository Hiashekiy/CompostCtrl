"""真值 plant（指导实施文件 §3.3/§3.4）：18 状态、RK4、机械限制全部生效。

状态 x = [q(4), omega(3), delta(3), gamma(4), gamma_dot(4)]
输入 u = [delta_c(3), gamma_dot_c(4)]
环境 p = FlightEnv(rho, V_ned, M_dist)

    q_dot        = 0.5 q (x) [0, omega]
    J omega_dot  = -omega x (J omega + H_cmg(gamma)) - A(gamma) gamma_dot
                   + M_aero(delta) + M_dist
    delta_dot    = (delta_c - delta)/tau_s
    gamma_dot_dot= sat((gamma_dot_c - gamma_dot)/tau_g, +-gddot_max)
    gamma_dot    = gamma_dot

每一步之后强制裁剪：|delta| <= delta_max、|gamma_dot| <= gdot_max、|gamma| <= travel。
分配器与 plant 共用同一个 predicted_torque()，因此"预测 == 真实"是结构性的，
而不是两套公式碰巧接近。
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .aerodynamic_surface import (
    AeroSpec,
    aero_moment,
    clamp_delta,
    limit_delta_command,
)
from .cmg_pyramid import CMGSpec, PyramidCMG
from .rigid_body import Inertia, quat_mul, quat_normalize

NX, NU = 18, 7


@dataclass
class FlightEnv:
    """飞行状态与扰动（每个控制周期由参考轨迹给出）。"""

    rho: float = 5.34e-7
    V_ned: np.ndarray = field(default_factory=lambda: np.array([4000.0, 0.0, 0.0]))
    M_dist: np.ndarray = field(default_factory=lambda: np.zeros(3))


@dataclass
class PlantSpec:
    inertia: Inertia = field(default_factory=Inertia)
    aero: AeroSpec = field(default_factory=AeroSpec)
    cmg: CMGSpec = field(default_factory=CMGSpec)
    _cluster: "PyramidCMG | None" = field(default=None, init=False, repr=False, compare=False)

    @property
    def cluster(self) -> "PyramidCMG":
        """缓存的 CMG 模型（每周期要调用上千次，重复构造会拖慢 5–10 倍）。"""
        if self._cluster is None:
            self._cluster = PyramidCMG(self.cmg)
        return self._cluster


def unpack(x):
    return x[0:4], x[4:7], x[7:10], x[10:14], x[14:18]


def predicted_torque(x, delta, gamma, gdot, env: FlightEnv, spec: PlantSpec) -> np.ndarray:
    """给定被执行的实际偏角/框架角/框架角速度时的总力矩（分配器与 plant 共用）。"""
    _, Mc = spec.cluster.momentum_and_torque(gamma, gdot)
    return (aero_moment(x[0:4], x[4:7], delta, env.V_ned, env.rho, spec.aero) + Mc)


def torque_terms(x, env: FlightEnv, spec: PlantSpec) -> dict:
    """真实动力学里实际起作用的分项力矩（用于日志三路对照）。"""
    _, omega, delta, gamma, gdot = unpack(x)
    H, Mc = spec.cluster.momentum_and_torque(gamma, gdot)
    Ma = aero_moment(x[0:4], omega, delta, env.V_ned, env.rho, spec.aero)
    return {"M_aero": Ma, "M_cmg": Mc, "M_cmg_momentum": H,
            "M_dist": np.asarray(env.M_dist, dtype=float), "M_total": Ma + Mc}


def dynamics(x, u, env: FlightEnv, spec: PlantSpec) -> np.ndarray:
    q, omega, delta, gamma, gdot = unpack(x)
    J = spec.inertia.J
    dx = np.empty(NX)
    dx[0:4] = 0.5 * quat_mul(q, np.array([0.0, omega[0], omega[1], omega[2]]))
    Hc, Mc = spec.cluster.momentum_and_torque(gamma, gdot)
    Mtot = (aero_moment(q, omega, delta, env.V_ned, env.rho, spec.aero)
            + Mc + np.asarray(env.M_dist, dtype=float))
    dx[4:7] = spec.inertia.Jinv @ (-np.cross(omega, J @ omega + Hc) + Mtot)
    dx[7:10] = (u[0:3] - delta) / spec.aero.tau_s
    dx[10:14] = gdot
    dx[14:18] = spec.cluster.gimbal_derivative(gdot, u[3:7])
    return dx


def limit_commands(x, u, spec: PlantSpec, dt: float) -> np.ndarray:
    """把指令投影到当前周期的可达集（舵面幅值/速率、框架角速度/加速度/行程）。"""
    delta, gamma, gdot = x[7:10], x[10:14], x[14:18]
    cmg = spec.cluster
    out = np.array(u, dtype=float).reshape(NU)
    out[0:3] = limit_delta_command(delta, out[0:3], spec.aero)
    out[3:7] = cmg.limit_gdot_command(gamma, gdot, out[3:7], dt)
    return out


def post_step(x, spec: PlantSpec) -> np.ndarray:
    out = np.array(x, dtype=float)
    out[0:4] = quat_normalize(out[0:4])
    out[7:10] = clamp_delta(out[7:10], spec.aero)
    g, gd = spec.cluster.clamp_state(out[10:14], out[14:18])
    out[10:14] = g
    out[14:18] = gd
    return out


def step(x, u_cmd, env: FlightEnv, spec: PlantSpec, dt: float, substeps: int = 1):
    """一个控制周期内的 RK4 积分，返回 (新状态, 实际生效的指令)。"""
    x = np.array(x, dtype=float)
    u = limit_commands(x, u_cmd, spec, dt)
    h = dt / float(substeps)
    for _ in range(substeps):
        k1 = dynamics(x, u, env, spec)
        k2 = dynamics(x + 0.5 * h * k1, u, env, spec)
        k3 = dynamics(x + 0.5 * h * k2, u, env, spec)
        k4 = dynamics(x + h * k3, u, env, spec)
        x = x + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
    return post_step(x, spec), u


def initial_state(x0: dict | None = None) -> np.ndarray:
    x = np.zeros(NX)
    x[0] = 1.0
    x0 = x0 or {}
    if "euler_deg" in x0:
        from .rigid_body import euler_to_quat

        r, p, y = np.deg2rad(np.asarray(x0["euler_deg"], dtype=float))
        x[0:4] = euler_to_quat(r, p, y)
    if "omega_deg_s" in x0:
        x[4:7] = np.deg2rad(np.asarray(x0["omega_deg_s"], dtype=float))
    if "delta_deg" in x0:
        x[7:10] = np.deg2rad(np.asarray(x0["delta_deg"], dtype=float))
    if "gamma_deg" in x0:
        x[10:14] = np.deg2rad(np.asarray(x0["gamma_deg"], dtype=float))
    if "gdot_deg_s" in x0:
        x[14:18] = np.deg2rad(np.asarray(x0["gdot_deg_s"], dtype=float))
    return x
