"""气动舵面（指导实施文件 §3.2）。

线性气动系数代理模型（阶段 1 核验并标注：与 alpha 无关、无失速、无马赫效应）：

    alpha = atan2(w, u), beta = asin(v/V)      （V_ned 转到机体系）
    Cl = Cl_beta beta + Cl_p p b/(2V) + Cl_r r b/(2V) + Cl_da da + Cl_dr dr
    Cm = Cm0 + Cm_alpha alpha + Cm_q q cbar/(2V) + Cm_de de
    Cn = Cn_beta beta + Cn_p p b/(2V) + Cn_r r b/(2V) + Cn_da da + Cn_dr dr
    M_aero = q_inf S [b Cl, cbar Cm, b Cn],  q_inf = rho V^2 / 2

执行器：一阶伺服 delta_dot = (delta_c - delta)/tau_s，指令裁剪到可达集
|delta_c - delta| <= tau_s * delta_dot_max，因此 |delta_dot| <= delta_dot_max 严格成立。
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .rigid_body import cross_matrix  # noqa: F401  (供外部一致的叉乘工具)


@dataclass
class AeroSpec:
    S: float = 4.2
    b: float = 2.6
    cbar: float = 3.2
    Cm0: float = 0.0
    Cm_alpha: float = -0.35
    Cm_q: float = -12.0
    Cm_de: float = 0.45
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
    delta_max_deg: tuple = (25.0, 20.0, 20.0)
    delta_dot_max_deg_s: tuple = (60.0, 60.0, 60.0)
    tau_s: float = 0.05

    @property
    def delta_max(self) -> np.ndarray:
        return self._delta_max

    @property
    def delta_dot_max(self) -> np.ndarray:
        return self._delta_dot_max

    def __post_init__(self):
        self._delta_max = np.deg2rad(np.asarray(self.delta_max_deg, dtype=float))
        self._delta_dot_max = np.deg2rad(np.asarray(self.delta_dot_max_deg_s, dtype=float))

    @classmethod
    def from_dict(cls, cfg: dict) -> "AeroSpec":
        out = {}
        for k in cls.__dataclass_fields__:
            if k in cfg:
                v = cfg[k]
                out[k] = tuple(v) if k in ("delta_max_deg", "delta_dot_max_deg_s") else v
        return cls(**out)


def flow_angles(q, omega, V_ned) -> tuple[float, float, float]:
    """由姿态与机体系速度得到 (alpha, beta, V)。V_ned 为惯性系速度矢量。"""
    Vn = np.asarray(V_ned, dtype=float)
    V = float(np.linalg.norm(Vn))
    if V < 1.0:
        return 0.0, 0.0, 1.0
    from .rigid_body import quat_to_R

    Vb = quat_to_R(q).T @ Vn
    u, v, w = Vb
    alpha = float(np.arctan2(w, max(u, 1e-6)))
    beta = float(np.arcsin(np.clip(v / V, -1.0, 1.0)))
    return alpha, beta, V


def q_inf(rho: float, V: float) -> float:
    return 0.5 * rho * V * V


def aero_coeffs(q, omega, delta, V_ned, rho, spec: AeroSpec):
    alpha, beta, V = flow_angles(q, omega, V_ned)
    p, qq, r = np.asarray(omega, dtype=float)
    de, da, dr = np.asarray(delta, dtype=float)
    chat = spec.cbar / (2.0 * max(V, 1.0))
    bhat = spec.b / (2.0 * max(V, 1.0))
    Cl = (spec.Cl_beta * beta + spec.Cl_p * p * bhat + spec.Cl_r * r * bhat
          + spec.Cl_da * da + spec.Cl_dr * dr)
    Cm = (spec.Cm0 + spec.Cm_alpha * alpha + spec.Cm_q * qq * chat + spec.Cm_de * de)
    Cn = (spec.Cn_beta * beta + spec.Cn_p * p * bhat + spec.Cn_r * r * bhat
          + spec.Cn_da * da + spec.Cn_dr * dr)
    return Cl, Cm, Cn


def aero_moment(q, omega, delta, V_ned, rho, spec: AeroSpec) -> np.ndarray:
    Vn = np.asarray(V_ned, dtype=float)
    V = float(np.linalg.norm(Vn))
    alpha, beta, V = flow_angles(q, omega, Vn)
    p, qq, r = np.asarray(omega, dtype=float)
    de, da, dr = np.asarray(delta, dtype=float)
    chat = spec.cbar / (2.0 * max(V, 1.0))
    bhat = spec.b / (2.0 * max(V, 1.0))
    Cl = (spec.Cl_beta * beta + spec.Cl_p * p * bhat + spec.Cl_r * r * bhat
          + spec.Cl_da * da + spec.Cl_dr * dr)
    Cm = (spec.Cm0 + spec.Cm_alpha * alpha + spec.Cm_q * qq * chat + spec.Cm_de * de)
    Cn = (spec.Cn_beta * beta + spec.Cn_p * p * bhat + spec.Cn_r * r * bhat
          + spec.Cn_da * da + spec.Cn_dr * dr)
    Q = 0.5 * rho * V * V
    return Q * spec.S * np.array([spec.b * Cl, spec.cbar * Cm, spec.b * Cn])


def aero_jacobian_delta(q, omega, V_ned, rho, spec: AeroSpec) -> np.ndarray:
    """dM_aero/d[de, da, dr]（3x3，冻结当前动压，供分配器使用）。"""
    V = float(np.linalg.norm(np.asarray(V_ned, dtype=float)))
    Q = q_inf(rho, V)
    B = np.zeros((3, 3))
    B[0, 1] = Q * spec.S * spec.b * spec.Cl_da
    B[0, 2] = Q * spec.S * spec.b * spec.Cl_dr
    B[1, 0] = Q * spec.S * spec.cbar * spec.Cm_de
    B[2, 1] = Q * spec.S * spec.b * spec.Cn_da
    B[2, 2] = Q * spec.S * spec.b * spec.Cn_dr
    return B


def limit_delta_command(delta, delta_cmd, spec: AeroSpec) -> np.ndarray:
    """把偏角指令裁剪到 |delta| 与 |delta_dot| 同时可达的集合内。"""
    d = np.asarray(delta, dtype=float)
    lo = np.maximum(-spec.delta_max, d - spec.tau_s * spec.delta_dot_max)
    hi = np.minimum(spec.delta_max, d + spec.tau_s * spec.delta_dot_max)
    return np.clip(np.asarray(delta_cmd, dtype=float), lo, hi)


def clamp_delta(delta, spec: AeroSpec) -> np.ndarray:
    return np.clip(np.asarray(delta, dtype=float), -spec.delta_max, spec.delta_max)
