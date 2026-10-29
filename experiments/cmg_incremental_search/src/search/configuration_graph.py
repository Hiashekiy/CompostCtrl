"""构型离散图（指导实施文件 §5）。

节点 = 离散四框架角构型 `v_i ↔ gamma_i`，网格间距 `angular_step_deg`（第一版 10°），
图是**局部图**：只保留距当前真实构型 `local_radius_deg` 以内、且在机械行程 ±90° 内的节点。

邻接动作（第一版 §5.2）：保持 + 4 个框架角各正/反向移动一个网格步长，最多 9 个。
每条边必须满足：框架角速度（`|dgamma/dt| <= gdot_max`）、机械行程、以及
`|gamma_dot_ref - gamma_dot_now| <= gddot_max dt` 的角加速度可达性。

时间展开（§5.3）：搜索状态 `s = (v_i, j)`，边把 `(v_i, j)` 接到 `(v_l, j+1)`，
因此同一构型在机动前/中/后的代价可以不同。

图是**隐式**的（按需生成邻居），因此 4 维网格不会爆炸；节点键是整数索引元组。
"""

from __future__ import annotations

import itertools
from dataclasses import dataclass

import numpy as np

from ..dynamics.cmg_pyramid import PyramidCMG


@dataclass
class EdgeMechanics:
    feasible: bool
    reason: str
    gdot_ref: np.ndarray
    dgamma: np.ndarray
    travel_ok: bool
    rate_ok: bool
    accel_ok: bool


class ConfigurationGraph:
    def __init__(self, cmg: PyramidCMG, angular_step_deg: float = 10.0,
                 max_neighbors: int = 9, local_radius_deg: float = 60.0):
        self.cmg = cmg
        self.spec = cmg.spec
        self.step = float(np.deg2rad(angular_step_deg))
        self.max_neighbors = int(max_neighbors)
        self.radius = float(np.deg2rad(local_radius_deg))
        self.travel = float(self.spec.travel)
        self.max_step_index = int(np.floor(self.travel / self.step))

    # ---- 节点 -----------------------------------------------------------
    def quantize(self, gamma) -> tuple[int, ...]:
        g = np.asarray(gamma, dtype=float)
        idx = tuple(int(round(x / self.step)) for x in g)
        return tuple(int(np.clip(i, -self.max_step_index, self.max_step_index)) for i in idx)

    def node_gamma(self, idx: tuple[int, ...]) -> np.ndarray:
        return np.asarray(idx, dtype=float) * self.step

    def in_local_graph(self, idx: tuple[int, ...], center: tuple[int, ...]) -> bool:
        g = self.node_gamma(idx)
        gc = self.node_gamma(center)
        return bool(np.all(np.abs(g) <= self.travel + 1e-12)
                    and np.all(np.abs(g - gc) <= self.radius + 1e-12))

    def neighbors(self, idx: tuple[int, ...], center: tuple[int, ...]):
        """返回 [(next_idx, action_name, dgamma)]，按 max_neighbors 截断。"""
        out = [(idx, "hold", np.zeros(self.spec.n))]
        for i in range(self.spec.n):
            for sign, tag in ((+1, "+"), (-1, "-")):
                nxt = list(idx)
                nxt[i] += sign
                nxt = tuple(nxt)
                if not self.in_local_graph(nxt, center):
                    continue
                dg = self.node_gamma(nxt) - self.node_gamma(idx)
                out.append((nxt, f"g{i + 1}{tag}", dg))
        return out[: self.max_neighbors]

    # ---- 边 -------------------------------------------------------------
    def edge_mechanics(self, idx, idx_next, gdot_now, dt_search: float) -> EdgeMechanics:
        s = self.spec
        g = self.node_gamma(idx)
        gn = self.node_gamma(idx_next)
        dgamma = gn - g
        gdot_ref = dgamma / max(dt_search, 1e-9)
        travel_ok = bool(np.all(np.abs(gn) <= self.travel + 1e-12))
        rate_ok = bool(np.all(np.abs(gdot_ref) <= s.gdot_max + 1e-12))
        accel_ok = bool(np.all(
            np.abs(gdot_ref - np.asarray(gdot_now, dtype=float)) <= s.gddot_max * dt_search + 1e-12))
        reasons = []
        if not travel_ok:
            reasons.append("travel")
        if not rate_ok:
            reasons.append("gdot_max")
        if not accel_ok:
            reasons.append("gddot_max")
        return EdgeMechanics(feasible=not reasons, reason="+".join(reasons) or "ok",
                             gdot_ref=gdot_ref, dgamma=dgamma,
                             travel_ok=travel_ok, rate_ok=rate_ok, accel_ok=accel_ok)

    # ---- 诊断 -----------------------------------------------------------
    def describe(self) -> dict:
        return {
            "angular_step_deg": float(np.rad2deg(self.step)),
            "max_neighbors": self.max_neighbors,
            "local_radius_deg": float(np.rad2deg(self.radius)),
            "travel_deg": float(np.rad2deg(self.travel)),
            "max_step_index": self.max_step_index,
            "max_gamma_per_step_rad": float(self.step),
            "gdot_at_dt_0p5": float(self.step / 0.5),
        }

    def all_nodes_in_radius(self, center: tuple[int, ...]) -> list[tuple[int, ...]]:
        """仅供离线统计/绘图：局部图内的全部节点。"""
        rng = range(-self.max_step_index, self.max_step_index + 1)
        nodes = []
        for idx in itertools.product(rng, repeat=self.spec.n):
            if self.in_local_graph(idx, center):
                nodes.append(idx)
        return nodes
