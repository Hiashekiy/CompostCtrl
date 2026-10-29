"""参考姿态与飞行环境（指导实施文件 §4.1）。

机动参考用平滑五次多项式 r(s) = 10 s^3 - 15 s^4 + 6 s^5，端点速率与加速度为零；
参考姿态 = 配平姿态 + 机动偏置，参考角速度由参考姿态的中心差分给出（不把角度差当力矩）。
飞行环境（rho, V_ned, M_dist）在这里统一生成，A/B/C/D 四组必须完全一致。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..dynamics.atmosphere import TableAtmosphere
from ..dynamics.rigid_body import euler_to_quat, quat_conj, quat_mul, quat_normalize
from ..dynamics.vehicle_plant import FlightEnv

AXES = {"roll": 0, "pitch": 1, "yaw": 2}


def quintic(s: float) -> float:
    return 10.0 * s ** 3 - 15.0 * s ** 4 + 6.0 * s ** 5


@dataclass
class Maneuver:
    axis: str = "pitch"
    t0: float = 10.0
    duration: float = 20.0
    delta_deg: float = 3.0

    @property
    def t_end(self) -> float:
        return self.t0 + self.duration

    @property
    def enabled(self) -> bool:
        return self.axis in AXES and abs(self.delta_deg) > 0.0

    def value(self, t: float) -> float:
        if not self.enabled:
            return 0.0
        s = (t - self.t0) / max(self.duration, 1e-9)
        if s <= 0.0:
            return 0.0
        if s >= 1.0:
            return float(np.deg2rad(self.delta_deg))
        return float(np.deg2rad(self.delta_deg) * quintic(s))

    def peak_rate(self) -> float:
        return 1.875 * float(np.deg2rad(abs(self.delta_deg))) / max(self.duration, 1e-9)

    def peak_accel(self) -> float:
        return 5.773503 * float(np.deg2rad(abs(self.delta_deg))) / max(self.duration, 1e-9) ** 2

    @classmethod
    def from_dict(cls, cfg: dict) -> "Maneuver":
        return cls(**{k: v for k, v in cfg.items() if k in cls.__dataclass_fields__})


class Reference:
    def __init__(self, scenario: dict, atmosphere: TableAtmosphere | None = None):
        self.altitude_km = float(scenario.get("altitude_km", 100.0))
        self.V = float(scenario.get("V_m_s", 4000.0))
        self.heading_deg = float(scenario.get("heading_deg", 0.0))
        self.trim = np.deg2rad(np.asarray(scenario.get("trim_euler_deg", (0.0, 0.0, 0.0)),
                                          dtype=float))
        self.atm = atmosphere or TableAtmosphere()
        self.maneuver = Maneuver.from_dict(scenario.get("maneuver", {}) or {})
        self.dist = np.asarray(scenario.get("M_dist", (0.0, 0.0, 0.0)), dtype=float)
        self.dist_amp = np.asarray(scenario.get("M_dist_amp", (0.0, 0.0, 0.0)), dtype=float)
        self.dist_freq = np.asarray(scenario.get("M_dist_freq", (0.0, 0.0, 0.0)), dtype=float)

    # ---- 几何/环境 ------------------------------------------------------
    @property
    def h_m(self) -> float:
        return self.altitude_km * 1000.0

    def rho(self, t: float = 0.0) -> float:
        return self.atm.rho(self.h_m)

    def qinf(self, t: float = 0.0) -> float:
        return self.atm.qinf(self.h_m, self.V)

    def disturbance(self, t: float) -> np.ndarray:
        return self.dist + self.dist_amp * np.sin(2.0 * np.pi * self.dist_freq * t)

    def env(self, t: float) -> FlightEnv:
        g = np.deg2rad(self.heading_deg)
        V_ned = self.V * np.array([np.cos(g), np.sin(g), 0.0])
        return FlightEnv(rho=self.rho(t), V_ned=V_ned, M_dist=self.disturbance(t))

    # ---- 姿态参考 -------------------------------------------------------
    def euler_ref(self, t: float) -> np.ndarray:
        e = np.array(self.trim, dtype=float)
        e[AXES.get(self.maneuver.axis, 1)] += self.maneuver.value(t)
        return e

    def q_ref(self, t: float) -> np.ndarray:
        e = self.euler_ref(t)
        return euler_to_quat(e[0], e[1], e[2])

    def omega_ref(self, t: float, eps: float = 1e-3) -> np.ndarray:
        dq = quat_mul(quat_conj(self.q_ref(t - 0.5 * eps)), self.q_ref(t + 0.5 * eps))
        if dq[0] < 0.0:
            dq = -dq
        return 2.0 * dq[1:4] / eps
