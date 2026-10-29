"""未来需求（指导实施文件 §6）。

每个控制周期根据已知未来参考、预测飞行状态和基础姿态控制器，生成

    R_k = { tau_req,k+j , h_k+j , q_ref,k+j }_{j=1..N}

* `tau_req` 由姿态控制律作用在**预测状态**上算出，**不是**把参考角度差当力矩；
* 预测状态用 plant 自身向前积分（B 组名义行为，`gamma_dot_ref = 0`）得到，
  这样"未来机动时姿态控制器会要多大力矩"是自洽的；
* 时间权重先归一化；已知机动窗口内权重高（`w_near`），远期衰减到 `w_far`。
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from ..control.attitude_controller import AttitudeController
from ..control.control_allocator import ControlAllocator
from ..dynamics.vehicle_plant import FlightEnv, PlantSpec, step as plant_step


@dataclass
class FutureDemand:
    t: np.ndarray                       # (N,)
    dt: float
    q_ref: np.ndarray                   # (N,4)
    omega_ref: np.ndarray               # (N,3)
    tau_req: np.ndarray                 # (N,3)
    x_pred: np.ndarray                  # (N,18) 预测状态（含 q, omega, delta, gamma, gdot）
    rho: np.ndarray                     # (N,)
    V: np.ndarray                       # (N,)
    weight: np.ndarray                  # (N,) 归一化（sum = 1）
    envs: list = field(default_factory=list)   # 每个时间片的 FlightEnv（含 V_ned / 扰动）
    high_window: tuple[int, int] | None = None
    meta: dict = field(default_factory=dict)

    @property
    def n(self) -> int:
        return int(self.t.size)

    def weighted(self, values: np.ndarray) -> float:
        """按时间权重求平均（values 形状 (N,) 或 (N,3) 会自动取范数）。"""
        v = np.asarray(values, dtype=float)
        if v.ndim > 1:
            v = np.linalg.norm(v, axis=1)
        return float(np.sum(self.weight * v))

    def from_index(self, j0: int) -> "FutureDemand":
        """把窗口前移到第 j0 个时间片（复用已算好的未来需求，权重重新归一化）。

        未来参考是已知的，因此同一个 R_k 可以在若干控制周期内复用；`j0` 就是已经
        走过的时间片数。这样每周期不必重算整条需求链（实测重算 ~40 ms）。
        """
        j0 = int(max(0, min(j0, self.n - 1)))
        if j0 == 0:
            return self
        w = np.array(self.weight[j0:], dtype=float, copy=True)
        w = w / max(w.sum(), 1e-12)
        high = self.high_window
        if high is not None:
            hi = (max(0, high[0] - j0), max(0, high[1] - j0))
            high = hi if hi[1] > 0 else None
        meta = dict(self.meta)
        meta["reused_from_index"] = j0
        meta["t0"] = float(self.t[j0])
        return FutureDemand(t=self.t[j0:], dt=self.dt, q_ref=self.q_ref[j0:],
                            omega_ref=self.omega_ref[j0:], tau_req=self.tau_req[j0:],
                            x_pred=self.x_pred[j0:], rho=self.rho[j0:], V=self.V[j0:],
                            weight=w, envs=self.envs[j0:], high_window=high, meta=meta)


def build_demand(x, t0: float, ref, ctrl: AttitudeController, alloc: ControlAllocator,
                 spec: PlantSpec, dt_search: float, horizon_s: float,
                 w_near: float = 1.0, w_far: float = 0.35,
                 predict_substeps: int = 5) -> FutureDemand:
    n = max(1, int(round(horizon_s / dt_search)))
    ts = t0 + dt_search * np.arange(n)

    q_ref = np.zeros((n, 4))
    omega_ref = np.zeros((n, 3))
    tau_req = np.zeros((n, 3))
    x_pred = np.zeros((n, 18))
    rho = np.zeros(n)
    V = np.zeros(n)

    xs = np.array(x, dtype=float)
    envs: list = []
    for j in range(n):
        env = ref.env(ts[j])
        envs.append(env)
        x_pred[j] = xs
        rho[j] = env.rho
        V[j] = float(np.linalg.norm(env.V_ned))
        q_ref[j] = ref.q_ref(ts[j])
        omega_ref[j] = ref.omega_ref(ts[j])
        tau_req[j] = ctrl.torque_request(xs, q_ref[j], omega_ref[j])
        res = alloc.allocate(xs, tau_req[j], env, np.zeros(spec.cmg.n), dt_search)
        u = np.concatenate([res.delta_cmd, res.gdot_cmd])
        xs, _ = plant_step(xs, u, env, spec, dt_search, max(1, predict_substeps))

    # 时间权重：机动窗口内为 w_near，窗口外向 w_far 线性过渡，再归一化
    w = np.full(n, float(w_far))
    mv = getattr(ref, "maneuver", None)
    high = None
    if mv is not None and getattr(mv, "enabled", False):
        lo = max(0, int(np.floor((mv.t0 - t0) / dt_search)))
        hi = min(n, int(np.ceil((mv.t_end - t0) / dt_search)) + 1)
        if hi > lo:
            w[lo:hi] = float(w_near)
            high = (lo, hi)
    if high is not None:                     # 窗口两侧线性过渡，避免阶跃权重
        lo, hi = high
        for j in range(lo):
            w[j] = w_far + (w_near - w_far) * (j / max(lo, 1))
        for j in range(hi, n):
            frac = (j - hi) / max(n - hi, 1)
            w[j] = w_near + (w_far - w_near) * frac
    w = w / max(w.sum(), 1e-12)

    return FutureDemand(t=ts, dt=float(dt_search), q_ref=q_ref, omega_ref=omega_ref,
                        tau_req=tau_req, x_pred=x_pred, rho=rho, V=V, weight=w, envs=envs,
                        high_window=high,
                        meta={"horizon_s": float(horizon_s), "n": n, "w_near": w_near,
                              "w_far": w_far, "t0": float(t0),
                              "tau_req_peak": float(np.max(np.linalg.norm(tau_req, axis=1)))})
