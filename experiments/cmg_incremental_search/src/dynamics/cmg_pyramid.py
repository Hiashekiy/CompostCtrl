"""四单框架金字塔式 CMG（指导实施文件 §3.3），唯一几何/力矩映射实现。

    g_i      = [sin(b) cos(t_i), sin(b) sin(t_i), cos(b)]      框架轴
    s_i(0)   = [-sin(t_i), cos(t_i), 0]                        转子轴（gamma = 0）
    s_i(g)   = cos(g) s_i(0) + sin(g) (g_i x s_i(0))
    H(g)     = h0 * sum_i s_i(g)                               簇角动量
    A(g)     = dH/dg,  A[:, i] = h0 (g_i x s_i(g))
    M_cmg    = -A(g) gdot

机械限制（本实验的硬约束，来源见 configs/base.yaml 注释）：
    |gdot_i| <= gdot_max        rad/s
    |gddot_i| <= gddot_max      rad/s^2   （一阶框架环做饱和，plant 与 QP 都生效）
    |gamma_i| <= travel         有限行程 ±90°，不绕回
转子转速恒定（h0 不变），不含摩擦与转速环动态。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np


def _cross3(a, b):
    return np.vstack([a[1] * b[2] - a[2] * b[1],
                      a[2] * b[0] - a[0] * b[2],
                      a[0] * b[1] - a[1] * b[0]])


@dataclass
class CMGSpec:
    n: int = 4
    beta_deg: float = 54.7356
    h0: float = 50.0            # N m s 每台转子角动量
    gdot_max: float = 1.5       # rad/s
    gddot_max: float = 10.0     # rad/s^2
    tau_g: float = 0.03         # s 框架角速度环时间常数
    travel_deg: float = 90.0    # 有限机械行程 ±90°
    rotor_rpm: float = 6000.0
    sigma_floor_frac: float = 0.30   # 奇异地板占家位 sigma_min 的比例

    @property
    def home_sigma_min(self) -> float:
        """家位（gamma=0）的解析奇异值 h0*sqrt(2/3)（beta=arcsin(sqrt(2/3)) 时精确）。"""
        return float(self.h0 * np.sqrt(2.0 / 3.0))

    @property
    def sigma_floor(self) -> float:
        return float(self.sigma_floor_frac * self.home_sigma_min)

    @property
    def beta(self) -> float:
        return float(np.deg2rad(self.beta_deg))

    @property
    def travel(self) -> float:
        return float(np.deg2rad(self.travel_deg))

    @property
    def h_total(self) -> float:
        return self.n * self.h0

    @classmethod
    def from_dict(cls, cfg: dict) -> "CMGSpec":
        keys = {f for f in cls.__dataclass_fields__}
        return cls(**{k: v for k, v in cfg.items() if k in keys})


class PyramidCMG:
    def __init__(self, spec: CMGSpec):
        self.spec = spec
        n = spec.n
        th = 2.0 * np.pi * np.arange(n) / n
        sb, cb = np.sin(spec.beta), np.cos(spec.beta)
        self.g_axis = np.vstack([sb * np.cos(th), sb * np.sin(th), cb * np.ones(n)])
        self.s0 = np.vstack([-np.sin(th), np.cos(th), np.zeros(n)])
        self.gxs0 = _cross3(self.g_axis, self.s0)

    # ---- 运动学映射 -----------------------------------------------------
    def spin_axes(self, gamma) -> np.ndarray:
        g = np.asarray(gamma, dtype=float)
        return self.s0 * np.cos(g) + self.gxs0 * np.sin(g)

    def momentum(self, gamma) -> np.ndarray:
        return self.spec.h0 * self.spin_axes(gamma).sum(axis=1)

    def jacobian(self, gamma) -> np.ndarray:
        return self.spec.h0 * _cross3(self.g_axis, self.spin_axes(gamma))

    def torque(self, gamma, gdot) -> np.ndarray:
        return -self.jacobian(gamma) @ np.asarray(gdot, dtype=float)

    def momentum_and_torque(self, gamma, gdot):
        """一次算好 spin_axes，同时返回 (H, M)。热路径用，避免重复构造 (3,n) 数组。"""
        s = self.spin_axes(gamma)
        H = self.spec.h0 * s.sum(axis=1)
        A = self.spec.h0 * _cross3(self.g_axis, s)
        return H, -A @ np.asarray(gdot, dtype=float)

    # ---- 性能指标 -------------------------------------------------------
    @staticmethod
    def sigma_min(A) -> float:
        return float(np.linalg.svd(np.asarray(A, dtype=float), compute_uv=False)[-1])

    @staticmethod
    def det_AAt(A) -> float:
        A = np.asarray(A, dtype=float)
        return float(np.linalg.det(A @ A.T))

    def guaranteed_torque(self, gamma) -> float:
        """最坏方向的可交付力矩 sigma_min(A) * gdot_max。"""
        return self.sigma_min(self.jacobian(gamma)) * self.spec.gdot_max

    def peak_torque(self, gamma) -> float:
        """最优方向的峰值力矩 max_{|u|<=1} ||A u|| * gdot_max（顶点枚举，n=4）。"""
        A = self.jacobian(gamma)
        n = self.spec.n
        verts = np.array([[(-1) ** ((k >> i) & 1) for i in range(n)]
                          for k in range(2 ** n)], dtype=float)
        return float(np.max(np.linalg.norm(A @ verts.T, axis=0))) * 1.0

    def capability(self, gamma) -> dict:
        A = self.jacobian(gamma)
        sig = self.sigma_min(A)
        return {
            "sigma_min": sig,
            "guaranteed_torque": sig * self.spec.gdot_max,
            "peak_torque": self.peak_torque(gamma),
            "momentum_norm": float(np.linalg.norm(self.momentum(gamma))),
            "det_AAt": self.det_AAt(A),
        }

    # ---- 机械限制 -------------------------------------------------------
    def limit_gdot_command(self, gamma, gdot, gdot_cmd, dt):
        """当前周期的框架角速度指令可达集（行程、角速度、角加速度三重约束）。"""
        s = self.spec
        cmd = np.clip(np.asarray(gdot_cmd, dtype=float), -s.gdot_max, s.gdot_max)
        dg = s.gddot_max * dt
        cmd = np.clip(cmd, np.asarray(gdot, dtype=float) - dg, np.asarray(gdot, dtype=float) + dg)
        # 行程：一步之内不许越过 ±travel
        g = np.asarray(gamma, dtype=float)
        lo_travel = (-s.travel - g) / max(dt, 1e-9)
        hi_travel = (s.travel - g) / max(dt, 1e-9)
        cmd = np.clip(cmd, np.maximum(-s.gdot_max, lo_travel), np.minimum(s.gdot_max, hi_travel))
        return cmd

    def gimbal_derivative(self, gdot, gdot_cmd) -> np.ndarray:
        """一阶框架环，加速度饱和到 gddot_max（硬约束）。"""
        s = self.spec
        acc = (np.asarray(gdot_cmd, dtype=float) - np.asarray(gdot, dtype=float)) / s.tau_g
        return np.clip(acc, -s.gddot_max, s.gddot_max)

    def clamp_state(self, gamma, gdot):
        s = self.spec
        g = np.clip(np.asarray(gamma, dtype=float), -s.travel, s.travel)
        gd = np.clip(np.asarray(gdot, dtype=float), -s.gdot_max, s.gdot_max)
        at_hi = g >= s.travel - 1e-12
        at_lo = g <= -s.travel + 1e-12
        gd = np.where(at_hi & (gd > 0.0), 0.0, gd)
        gd = np.where(at_lo & (gd < 0.0), 0.0, gd)
        return g, gd
