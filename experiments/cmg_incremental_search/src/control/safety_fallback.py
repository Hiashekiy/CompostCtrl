"""安全回退（指导实施文件 §3.4/§7.3）：姿态安全优先于构型迁移。

触发条件：分配不可行、搜索无解/超时、或调用方显式要求。回退顺序：

1. 暂停构型迁移（框架角速度参考置零，只保留速率/加速度意义上的必要动作）；
2. 逐级缩小期望总力矩（1.0 → 0.5 → 0.25），再求解；
3. 仍不可行则退化为"舵面尽量出力 + 框架角速度归零"的解析解，并如实报告残差。

每次回退都记 reason，不允许静默吞掉。
"""

from __future__ import annotations

import numpy as np

from .control_allocator import AllocationResult, ControlAllocator
from ..dynamics.aerodynamic_surface import aero_jacobian_delta
from ..dynamics.vehicle_plant import FlightEnv, PlantSpec, unpack


class SafetyFallback:
    def __init__(self, spec: PlantSpec, allocator: ControlAllocator):
        self.spec = spec
        self.allocator = allocator

    def actuate(self, x, tau_req, env: FlightEnv, dt: float, reason: str) -> AllocationResult:
        x = np.asarray(x, dtype=float)
        tau_req = np.asarray(tau_req, dtype=float)
        zero_ref = np.zeros(self.spec.cmg.n)
        last: AllocationResult | None = None
        for scale in (1.0, 0.5, 0.25):
            res = self.allocator.allocate(x, scale * tau_req, env, zero_ref, dt)
            res.details["fallback_scale"] = scale
            if res.feasible:
                # 残差必须相对**原始**期望力矩报告，不能相对缩放后的请求，
                # 否则日志会掩盖"力矩不够"这一事实（见 docs 阶段 3 报告）。
                res.residual = tau_req - res.predicted_torque
                res.details["scaled_feasible"] = True
                res.fallback = True
                res.status = f"fallback({reason}, scale={scale})"
                return res
            last = res
        dres = self._surface_only(x, tau_req, env, dt)
        dres.fallback = True
        dres.status = f"fallback({reason}, surface-only)"
        dres.details["scaled_feasible"] = False
        dres.details["previous_residual_norm"] = last.residual_norm if last else float("nan")
        return dres

    def _surface_only(self, x, tau_req, env: FlightEnv, dt: float) -> AllocationResult:
        """框架角速度归零，用舵面按效率矩阵做最小范数解（不迭代、必定返回）。"""
        from ..dynamics.aerodynamic_surface import aero_moment

        q, omega, delta, gamma, gdot = unpack(x)
        B_delta = aero_jacobian_delta(q, omega, env.V_ned, env.rho, self.spec.aero)
        M_aero0 = aero_moment(q, omega, np.zeros(3), env.V_ned, env.rho, self.spec.aero)
        lo, hi = self.allocator.bounds(x, dt)
        try:
            d = np.linalg.lstsq(B_delta, tau_req - M_aero0, rcond=None)[0]
        except np.linalg.LinAlgError:
            d = np.zeros(3)
        d = np.clip(d, lo[:3], hi[:3])
        g_cmd = np.clip(np.zeros(self.spec.cmg.n), lo[3:], hi[3:])
        pred = M_aero0 + B_delta @ d
        resid = tau_req - pred
        return AllocationResult(
            delta_cmd=d, gdot_cmd=g_cmd, predicted_torque=pred, residual=resid,
            feasible=False, fallback=True, status="surface-only", solve_time_ms=0.0,
        )
