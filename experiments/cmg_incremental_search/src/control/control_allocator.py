"""控制分配器（指导实施文件 §3.4/§8）。

第一版：冻结当前 CMG 映射矩阵 A(gamma) 与当前动压下的气动效率矩阵 B_delta 的局部 QP：

    min_{delta, gdot} || W_tau^0.5 (tau_req - B_delta delta + A gamma_dot) ||^2
                    + lambda_gamma ||gamma_dot - gamma_dot_ref||^2
                    + lambda_delta ||delta||^2

约束全部是**盒约束**（因此可以用有界最小二乘精确求解，不需要一般 QP 求解器）：

    |delta_i| <= delta_max_i
    |delta_i - delta_i(k)| <= tau_s delta_dot_max_i        （速率可达）
    |gamma_dot_i| <= gdot_max
    |gamma_dot_i - gamma_dot_i(k)| <= gddot_max dt_ctrl    （加速度可达，硬约束）
    |gamma_i + gamma_dot_i dt_ctrl| <= travel              （行程，不越界）

输出：舵面指令、框架角速度指令、预测实际力矩、力矩残差、可行性、耗时。
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np
from scipy.optimize import lsq_linear

from ..dynamics.aerodynamic_surface import aero_jacobian_delta
from ..dynamics.cmg_pyramid import PyramidCMG
from ..dynamics.vehicle_plant import FlightEnv, PlantSpec, predicted_torque, unpack


@dataclass
class AllocatorWeights:
    w_tau: tuple = (1.0, 1.0, 1.0)
    lambda_gamma: float = 2.0e-4
    lambda_delta: float = 5.0e-3
    feas_tol: float = 1.0e-3     # 力矩残差范数（N m）超过即判不可行

    @classmethod
    def from_dict(cls, cfg: dict) -> "AllocatorWeights":
        out = dict(cls().__dict__)
        out.update({k: (tuple(v) if isinstance(v, list) else v)
                    for k, v in cfg.items() if k in cls.__dataclass_fields__})
        return cls(**out)


@dataclass
class AllocationResult:
    delta_cmd: np.ndarray
    gdot_cmd: np.ndarray
    predicted_torque: np.ndarray
    residual: np.ndarray
    feasible: bool
    fallback: bool
    status: str
    solve_time_ms: float
    details: dict = field(default_factory=dict)

    @property
    def residual_norm(self) -> float:
        return float(np.linalg.norm(self.residual))


class ControlAllocator:
    def __init__(self, spec: PlantSpec, weights: AllocatorWeights | None = None):
        self.spec = spec
        self.w = weights or AllocatorWeights()
        self.cmg = PyramidCMG(spec.cmg)

    def bounds(self, x, dt: float):
        delta, gamma, gdot = x[7:10], x[10:14], x[14:18]
        a, c = self.spec.aero, self.spec.cmg
        d_lo = np.maximum(-a.delta_max, delta - a.tau_s * a.delta_dot_max)
        d_hi = np.minimum(a.delta_max, delta + a.tau_s * a.delta_dot_max)
        g_lo = np.maximum.reduce([
            -c.gdot_max * np.ones(c.n),
            gdot - c.gddot_max * dt,
            (-c.travel - gamma) / max(dt, 1e-9),
        ])
        g_hi = np.minimum.reduce([
            c.gdot_max * np.ones(c.n),
            gdot + c.gddot_max * dt,
            (c.travel - gamma) / max(dt, 1e-9),
        ])
        lo = np.concatenate([d_lo, g_lo])
        hi = np.concatenate([d_hi, g_hi])
        if not self.spec.cmg_on:          # A 组：CMG 结构上不存在，框架角速度恒为 0
            lo[3:] = 0.0
            hi[3:] = 0.0
        return np.minimum(lo, hi), np.maximum(lo, hi)

    def _design(self, x, env: FlightEnv):
        q, omega, delta, gamma, gdot = unpack(x)
        B_delta = aero_jacobian_delta(q, omega, env.V_ned, env.rho, self.spec.aero)
        B_gdot = -self.cmg.jacobian(gamma)          # M_cmg = -A gamma_dot
        # 与状态有关、与 delta 无关的气动配平项（alpha/beta/角速度贡献），
        # 必须作为常数项进入 QP，否则"预测力矩"会漏掉它（阶段 3 的 T4 检查）。
        from ..dynamics.aerodynamic_surface import aero_moment

        M_aero0 = aero_moment(q, omega, np.zeros(3), env.V_ned, env.rho, self.spec.aero)
        return M_aero0, B_delta, B_gdot

    def _solve(self, M_aero0, B_delta, B_gdot, tau_req, gdot_ref, lo, hi):
        w = np.sqrt(np.asarray(self.w.w_tau, dtype=float))
        lg = np.sqrt(self.w.lambda_gamma)
        ld = np.sqrt(self.w.lambda_delta)
        n, m = 3, 4
        M = np.zeros((3 + m + n, n + m))
        M[0:3, 0:n] = w[:, None] * B_delta
        M[0:3, n:] = w[:, None] * B_gdot
        M[3:3 + m, n:] = lg * np.eye(m)
        M[3 + m:, 0:n] = ld * np.eye(n)
        b = np.concatenate([w * (np.asarray(tau_req, dtype=float) - M_aero0),
                            lg * np.asarray(gdot_ref, dtype=float),
                            np.zeros(n)])
        # 被钉死的变量（例如 A 组框架角速度恒 0）先从问题里剔除，否则 bvls 要求严格下界
        free = (hi - lo) > 1e-12
        z = np.array(lo, dtype=float)
        if not np.any(free):
            return z[:n], z[n:], True, "all-fixed"
        sol = lsq_linear(M[:, free], b + M[:, ~free] @ z[~free],
                         bounds=(lo[free], hi[free]), method="bvls", tol=1e-10, max_iter=200)
        z[free] = np.asarray(sol.x, dtype=float)
        return z[:n], z[n:], bool(sol.success), str(sol.message)

    def allocate(self, x, tau_req, env: FlightEnv, gdot_ref=None, dt: float = 0.2) -> AllocationResult:
        t0 = time.perf_counter()
        delta, gamma, gdot = x[7:10], x[10:14], x[14:18]
        if gdot_ref is None:
            gdot_ref = np.zeros(self.spec.cmg.n)
        M_aero0, B_delta, B_gdot = self._design(x, env)
        lo, hi = self.bounds(x, dt)
        try:
            delta_c, gdot_c, ok, msg = self._solve(M_aero0, B_delta, B_gdot, tau_req,
                                                   gdot_ref, lo, hi)
        except Exception as exc:                      # 求解器异常：直接进安全回退
            delta_c = np.clip(np.zeros(3), lo[:3], hi[:3])
            gdot_c = np.clip(np.zeros(4), lo[3:], hi[3:])
            ok, msg = False, f"solver-error: {exc}"
        # 预测力矩用"本轮实际会被 plant 执行"的指令评估，保证与 plant 同源
        pred = M_aero0 + B_delta @ delta_c + B_gdot @ gdot_c
        resid = np.asarray(tau_req, dtype=float) - pred
        # 可行性看残差与数值有效性；求解器的 success 只作诊断（退化问题里 bvls 会报
        # success=False 但解是正确的，若把它当"不可行"会让 A 组每周期都误进回退）。
        finite = bool(np.all(np.isfinite(delta_c)) and np.all(np.isfinite(gdot_c)))
        feasible = finite and float(np.linalg.norm(resid)) <= self.w.feas_tol
        return AllocationResult(
            delta_cmd=delta_c, gdot_cmd=gdot_c, predicted_torque=pred, residual=resid,
            feasible=feasible, fallback=False, status=msg,
            solve_time_ms=1e3 * (time.perf_counter() - t0),
            details={"bounds_lo": lo, "bounds_hi": hi, "M_aero0": M_aero0},
        )

    def plant_consistent_torque(self, x, delta, gamma, gdot, env: FlightEnv) -> np.ndarray:
        return predicted_torque(x, delta, gamma, gdot, env, self.spec)
