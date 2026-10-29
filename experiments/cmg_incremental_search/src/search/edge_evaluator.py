"""边级可行性与代价（指导实施文件 §5.4）。

两级评估，兼顾"约束下的可实现性"与实时预算：

1. **筛选层（快，~10 µs/边）**：用执行器约束的**支撑函数**判断需求力矩能否被实现。
   对需求方向 τ̂ 有

       mu(gamma, tau_hat) = sum_i |A_i . tau_hat| * gdot_max  +  sum_j |B_j . tau_hat| * delta_max_j

   （`A` 为 CMG 雅可比，`B = dM_aero/ddelta`；两项可分离，故这是该方向上的精确上界）。
   缺口 `gap = max(0, ||tau_res|| - mu)`，其中 `tau_res = tau_req - M_aero0`。
   `mu` 是沿该方向的精确支撑值，因此 `gap` 是"该方向能拿到多少力矩"的严格度量；
   但它不保证 τ_res 的可加实际可达（非各向同性），所以只用于**筛选与代价**。
2. **验证层（精确，~0.1 ms/条）**：对 A* 最终路径的每条边，用控制分配 QP
   （与 plant 同一套模型）重新求解并复核残差；任何一条边复核失败，整条路径作废
   （`astar_replanner.BoundedAStar` 的 `verify_path`）。

代价（§5.4）：

    J = J_track + lambda_m J_move + lambda_s J_sing + lambda_f J_future

σ_min 只进 `J_sing`（风险项）；"能力不足"一律由 §5.4 要求的执行器约束评估给出，
不用静态奇异值替代。
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from ..control.control_allocator import ControlAllocator
from ..dynamics.aerodynamic_surface import aero_jacobian_delta, aero_moment
from ..dynamics.cmg_pyramid import PyramidCMG
from .configuration_graph import ConfigurationGraph, EdgeMechanics
from .future_demand import FutureDemand


@dataclass
class EdgeWeights:
    lambda_move: float = 1.0
    lambda_sing: float = 2.0
    lambda_future: float = 4.0
    tau_scale: float = 60.0        # N m，用于归一化力矩缺口（与保证力矩同量级）
    feasibility_tol: float = 1.0   # N m，缺口超过即判边不可行

    @classmethod
    def from_dict(cls, cfg: dict) -> "EdgeWeights":
        return cls(**{k: v for k, v in cfg.items() if k in cls.__dataclass_fields__})


@dataclass
class EdgeResult:
    feasible: bool
    reason: str
    cost: float
    terms: dict
    gdot_ref: np.ndarray
    residual_norm: float
    mechanics: EdgeMechanics
    support_mu: float = float("nan")
    verified: bool | None = None       # 验证层的结论（None = 只做了筛选层）

    def as_row(self) -> dict:
        return {"feasible": self.feasible, "reason": self.reason, "cost": self.cost,
                **{f"J_{k}": float(v) for k, v in self.terms.items()},
                "residual_norm": float(self.residual_norm),
                "support_mu": float(self.support_mu),
                "verified": self.verified,
                "gdot_ref_norm": float(np.linalg.norm(self.gdot_ref))}


class EdgeEvaluator:
    def __init__(self, cmg: PyramidCMG, allocator: ControlAllocator,
                 graph: ConfigurationGraph, weights: EdgeWeights, demand: FutureDemand):
        self.cmg = cmg
        self.alloc = allocator
        self.spec = allocator.spec
        self.graph = graph
        self.w = weights
        self.demand = demand
        self._node_future: dict[tuple[int, ...], float] = {}
        self._edge_cache: dict[tuple, EdgeResult] = {}
        self.sample_stride = max(1, demand.n // 3)

    # ---- 支撑函数（筛选层） ---------------------------------------------
    def support_mu(self, gamma, tau_hat, x_env) -> float:
        q, omega, V_ned, rho = x_env
        A = self.cmg.jacobian(gamma)
        B = aero_jacobian_delta(q, omega, V_ned, rho, self.spec.aero)
        gmax = self.cmg.spec.gdot_max
        dmax = self.spec.aero.delta_max
        return float(np.sum(np.abs(A.T @ tau_hat)) * gmax + np.sum(np.abs(B.T @ tau_hat) * dmax))

    def aero_trim(self, q, omega, V_ned, rho) -> np.ndarray:
        return aero_moment(q, omega, np.zeros(3), V_ned, rho, self.spec.aero)

    def _support_gap(self, gamma, tau_req, j: int) -> tuple[float, float]:
        env = self.demand.envs[j]
        x = self.demand.x_pred[j]
        tau_res = np.asarray(tau_req, dtype=float) - self.aero_trim(
            x[0:4], x[4:7], env.V_ned, env.rho)
        n = float(np.linalg.norm(tau_res))
        if n < 1e-9:
            return 0.0, 0.0
        mu = self.support_mu(gamma, tau_res / n, (x[0:4], x[4:7], env.V_ned, env.rho))
        return mu, max(0.0, n - mu)

    # ---- 节点未来能力（缓存） -------------------------------------------
    def node_future_gap(self, idx: tuple[int, ...]) -> float:
        key = tuple(idx)
        hit = self._node_future.get(key)
        if hit is not None:
            return hit
        gamma = self.graph.node_gamma(idx)
        total, wsum = 0.0, 0.0
        for j in range(0, self.demand.n, self.sample_stride):
            _mu, gap = self._support_gap(gamma, self.demand.tau_req[j], j)
            total += self.demand.weight[j] * (gap / self.w.tau_scale) ** 2
            wsum += self.demand.weight[j]
        val = total / max(wsum, 1e-12)
        self._node_future[key] = val
        return val

    def singularity_cost(self, gamma) -> float:
        sig = self.cmg.sigma_min(self.cmg.jacobian(gamma))
        floor = float(self.cmg.spec.sigma_floor)
        deficit = max(0.0, (floor - sig) / max(floor, 1e-9))
        return float(deficit ** 2)

    # ---- 边 -------------------------------------------------------------
    def evaluate(self, idx, idx_next, j: int, gdot_now) -> EdgeResult:
        key = (tuple(idx), tuple(idx_next), int(j))
        hit = self._edge_cache.get(key)
        if hit is not None:
            return hit
        mech = self.graph.edge_mechanics(idx, idx_next, gdot_now, self.demand.dt)
        gamma_next = self.graph.node_gamma(idx_next)
        sing = self.singularity_cost(gamma_next)
        move = float(np.linalg.norm(mech.dgamma) / self.graph.step
                     + np.linalg.norm(mech.gdot_ref) / max(self.cmg.spec.gdot_max, 1e-9))
        if not mech.feasible:
            out = EdgeResult(feasible=False, reason=mech.reason, cost=float("inf"),
                             terms={"track": float("inf"), "move": move,
                                    "sing": sing, "future": float("inf")},
                             gdot_ref=np.zeros(self.cmg.spec.n),
                             residual_norm=float("nan"), mechanics=mech)
            self._edge_cache[key] = out
            return out

        mu, gap = self._support_gap(gamma_next, self.demand.tau_req[j], j)
        track = (gap / self.w.tau_scale) ** 2
        future = self.node_future_gap(idx_next)
        cost = (self.demand.weight[j] * track
                + self.w.lambda_move * move
                + self.w.lambda_sing * sing
                + self.w.lambda_future * future)
        feasible = gap <= self.w.feasibility_tol
        out = EdgeResult(feasible=feasible,
                         reason="ok" if feasible else f"support_gap>{self.w.feasibility_tol:g}",
                         cost=float(cost),
                         terms={"track": track, "move": move, "sing": sing, "future": future},
                         gdot_ref=mech.gdot_ref, residual_norm=gap, mechanics=mech,
                         support_mu=mu)
        self._edge_cache[key] = out
        return out

    # ---- 验证层：用精确 QP 复核（路径级） --------------------------------
    def verify_edge(self, idx, idx_next, j: int, gdot_now) -> EdgeResult:
        """用与 plant 同一套模型的分配 QP 精确复核一条边。"""
        key = (tuple(idx), tuple(idx_next), int(j))
        res = self.evaluate(idx, idx_next, j, gdot_now)
        if not res.mechanics.feasible:
            return res
        gamma_next = self.graph.node_gamma(idx_next)
        env = self.demand.envs[j]
        x = self.demand.x_pred[j].copy()
        x[10:14] = gamma_next
        alloc = self.alloc.allocate(x, self.demand.tau_req[j], env, res.gdot_ref, self.demand.dt)
        out = EdgeResult(feasible=alloc.residual_norm <= self.w.feasibility_tol + 1e-9,
                         reason="ok" if alloc.residual_norm <= self.w.feasibility_tol + 1e-9
                         else f"qp_residual>{self.w.feasibility_tol:g}",
                         cost=res.cost, terms=res.terms, gdot_ref=res.gdot_ref,
                         residual_norm=alloc.residual_norm, mechanics=res.mechanics,
                         support_mu=res.support_mu, verified=True)
        self._edge_cache[key] = out
        return out
