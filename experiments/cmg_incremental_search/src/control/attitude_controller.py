"""基础姿态控制器（指导实施文件 §3.4）：四元数 PD + 陀螺前馈，输出期望总力矩。

    e        = 2 dq_vec(q_ref^-1 ⊗ q)       （参考 -> 机体的旋转，指向"机体偏离参考"的方向）
    w_err    = omega - omega_ref
    tau_req  = -J (k_att e + k_rate w_err) + omega x (J omega + H_cmg)

**符号说明**：`e` 是从参考到机体的旋转矢量（"机体相对参考偏了 e"），因此纠偏力矩必须取
**负号**，`w_err` 同样取负号做阻尼。该符号在阶段 3 的闭环测试中被显式验证：写反时姿态
误差指数增长（峰值 179°），改正后误差单调收敛。

第一版刻意不做 MPC：它必须是最稳定、最易解释的基线，A/B/C/D 四组共用。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..dynamics.cmg_pyramid import PyramidCMG
from ..dynamics.rigid_body import quat_error
from ..dynamics.vehicle_plant import PlantSpec, unpack


@dataclass
class PDGains:
    k_att: float = 0.6      # 1/s^2 量纲：J * k_att * e 直接给力矩
    k_rate: float = 1.6     # 1/s
    tau_max: float = 1.0e3  # 期望总力矩饱和（保护 QP 数值，不改变物理）

    @classmethod
    def from_dict(cls, cfg: dict) -> "PDGains":
        return cls(**{k: v for k, v in cfg.items() if k in cls.__dataclass_fields__})


class AttitudeController:
    def __init__(self, spec: PlantSpec, gains: PDGains):
        self.spec = spec
        self.gains = gains
        self.cmg = PyramidCMG(spec.cmg)

    def torque_request(self, x, q_ref, omega_ref) -> np.ndarray:
        q, omega, _, gamma, _ = unpack(x)
        e = quat_error(q_ref, q)
        w_err = np.asarray(omega, dtype=float) - np.asarray(omega_ref, dtype=float)
        J = self.spec.inertia.J
        tau = -J @ (self.gains.k_att * e + self.gains.k_rate * w_err)
        H = self.cmg.momentum(gamma) if self.spec.cmg_on else np.zeros(3)
        tau = tau + np.cross(omega, J @ omega + H)
        n = float(np.linalg.norm(tau))
        if n > self.gains.tau_max:
            tau = tau * (self.gains.tau_max / n)
        return tau
