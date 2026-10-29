"""有界 A*（指导实施文件 §7.1，C 组）。

* 在时间展开图上运行，状态 `(v_i, j)`；`max_expansions` 与 `max_search_time_ms`
  双重预算，命中任一即停（配置缺失时程序启动就失败，见 src/config.py）；
* 返回当前找到的最佳可行路径，**只执行第一步**，下一周期用真实新状态重新搜索；
* 启发函数默认取 0（等价于有界 Dijkstra）。**因为 h 不是剩余代价的可采纳下界，
  本实现不宣称 A* 的全局最优性**；目标是"有界时间内找到可行路径"（§7.1）。

状态文案：`ok` 找到可行路径；`timeout` 预算耗尽但可能已有候选；`no_path` 无可行边；
`budget` 节点展开耗尽。任何非 ok 状态调用方都不得直接执行旧决策（§7.3）。
"""

from __future__ import annotations

import heapq
import time
from dataclasses import dataclass, field

import numpy as np

from .configuration_graph import ConfigurationGraph
from .edge_evaluator import EdgeEvaluator, EdgeResult
from .future_demand import FutureDemand, build_demand


@dataclass
class SearchBudget:
    max_expansions: int = 400
    max_search_time_ms: float = 60.0

    @classmethod
    def from_dict(cls, cfg: dict) -> "SearchBudget":
        me = cfg.get("max_expansions")
        mt = cfg.get("max_search_time_ms")
        if me is None or mt is None:
            raise ValueError("search.max_expansions / max_search_time_ms 必填（禁止默认无限搜索）")
        return cls(int(me), float(mt))


@dataclass
class SearchResult:
    status: str
    gdot_ref: np.ndarray
    path: list = field(default_factory=list)
    cost: float = float("inf")
    expansions: int = 0
    time_ms: float = 0.0
    demand_ms: float = 0.0
    total_ms: float = 0.0
    diagnostics: dict = field(default_factory=dict)

    @property
    def info(self) -> dict:
        return {"status": self.status, "time_ms": self.time_ms, "nodes": self.expansions,
                "cost": self.cost, "path_len": len(self.path),
                "demand_ms": self.demand_ms, "total_ms": self.total_ms}


class BoundedAStar:
    def __init__(self, cmg, allocator, ctrl, spec, graph_cfg: dict, search_cfg: dict,
                 weights, budget: SearchBudget):
        self.cmg = cmg
        self.alloc = allocator
        self.ctrl = ctrl
        self.spec = spec
        self.graph = ConfigurationGraph(
            cmg,
            angular_step_deg=float(graph_cfg.get("angular_step_deg", 10.0)),
            max_neighbors=int(graph_cfg.get("max_neighbors", 9)),
            local_radius_deg=float(graph_cfg.get("local_radius_deg", 60.0)),
        )
        self.search_cfg = search_cfg
        self.weights = weights
        self.budget = budget
        self._demand_cache: tuple[float, np.ndarray, FutureDemand] | None = None
        self.demand_time_ms = 0.0

    # ---- 未来需求（可复用）----------------------------------------------
    def _get_demand(self, x, t, ref, dt_search: float, horizon: float) -> FutureDemand:
        """按 `demand_refresh_s` 节奏重算未来需求；中间周期复用并前移窗口。

        未来参考是已知量，重算整条链实测约 40–55 ms（n=60），因此不允许每周期都重算；
        复用条件：时间未超过刷新周期、窗口还剩 ≥10 片、机体姿态/角速度没有明显偏离。
        """
        from ..dynamics.rigid_body import quat_geodesic

        refresh = float(self.search_cfg.get("demand_refresh_s", 1.0))
        cache = self._demand_cache
        if cache is not None:
            cache_t, cache_x, cache_dem = cache
            j0 = int(round((t - cache_t) / dt_search))
            drift_deg = float(np.rad2deg(quat_geodesic(cache_x[0:4], x[0:4])))
            ok = (0 <= j0 < cache_dem.n - 10
                  and (t - cache_t) <= refresh + 1e-9
                  and drift_deg < 0.5)
            if ok:
                self.demand_time_ms = 0.0
                return cache_dem.from_index(j0)
        t1 = time.perf_counter()
        dem = build_demand(
            x, t, ref, self.ctrl, self.alloc, self.spec, dt_search, horizon,
            w_near=float(self.search_cfg.get("w_near", 1.0)),
            w_far=float(self.search_cfg.get("w_far", 0.35)),
            predict_substeps=int(self.search_cfg.get("predict_substeps", 2)),
        )
        self.demand_time_ms = 1e3 * (time.perf_counter() - t1)
        self._demand_cache = (float(t), np.array(x, dtype=float), dem)
        return dem

    # ---- 对外接口（与 runner 的 search 接口一致）-------------------------
    def step(self, x, t, ref, env, dt):
        """C 组入口：以"保持当前构型"作为初始可行解，在预算内寻找更优路径。

        为什么给 hold 种子：h=0 的有界搜索在能力受限工况下会在同一层铺开很多兄弟节点，
        60 ms 内常常完不成（阶段 7 实测 291/300 周期 timeout）。以 hold 路径作为 incumbent
        可以先保证"有可行解可执行"，再把剩余预算用于改进——这仍然是有界 A*，
        只是起点不是一个空解。D 组则用上一周期路径当 incumbent。
        """
        n_demand = None
        dt_search = float(self.search_cfg.get("step_s", 0.5))
        horizon = float(self.search_cfg.get("horizon_s", 30.0))
        demand = self._get_demand(x, t, ref, dt_search, horizon)
        n_demand = demand.n
        start = self.graph.quantize(x[10:14])
        seed = [start] * n_demand if n_demand > 0 else None
        res = self.plan(x, t, ref, demand=demand, incumbent=seed)
        return res.gdot_ref, res.info

    def _peek_n(self, t: float, ref) -> int:
        """窗口层数（用于构造 hold 种子）；与 build_demand 的 n 保持一致。"""
        dt_search = float(self.search_cfg.get("step_s", 0.5))
        horizon = float(self.search_cfg.get("horizon_s", 30.0))
        return max(1, int(round(horizon / dt_search)))

    # ---- 搜索 -----------------------------------------------------------
    def plan(self, x, t, ref, demand: FutureDemand | None = None,
             incumbent: list[tuple[int, ...]] | None = None,
             budget: SearchBudget | None = None,
             incumbent_verified: bool = False) -> SearchResult:
        """在图搜索。`incumbent` 是上一周期的路径（节点索引序列，与时间层对齐），
        给出时先复核它；只在预算内寻找**严格更优**的路径（D 组热启动，§7.2）。"""
        t_start = time.perf_counter()
        dt_search = float(self.search_cfg.get("step_s", 0.5))
        horizon = float(self.search_cfg.get("horizon_s", 30.0))
        budget = budget or self.budget
        if demand is None:
            demand = self._get_demand(x, t, ref, dt_search, horizon)
        demand_ms = self.demand_time_ms
        t0 = time.perf_counter()          # 预算只统计图搜索部分，需求生成单独记时
        ev = EdgeEvaluator(self.cmg, self.alloc, self.graph, self.weights, demand)
        n = demand.n
        start = self.graph.quantize(x[10:14])
        center = start
        gdot_now = np.asarray(x[14:18], dtype=float)

        # 起点若本身不在局部图内（例如超过半径），就地夹紧到网格
        g0 = self.graph.node_gamma(start)
        if not self.graph.in_local_graph(start, center):
            start = tuple(int(i) for i in np.clip(
                np.round(g0 / self.graph.step), -self.graph.max_step_index,
                self.graph.max_step_index))
            g0 = self.graph.node_gamma(start)

        # ---- 历史路径复核（热启动） -------------------------------------
        inc_cost, inc_valid, inc_rows = self._evaluate_incumbent(ev, incumbent, start, n,
                                                                 gdot_now, incumbent_verified)

        open_heap: list[tuple[float, int, tuple, int]] = []
        counter = 0
        heapq.heappush(open_heap, (0.0, counter, start, 0))
        best_cost = {start: 0.0}
        parent: dict[tuple[tuple, int], tuple[tuple, int] | None] = {(start, 0): None}
        edge_info: dict[tuple[tuple, int], EdgeResult | None] = {(start, 0): None}
        expanded = 0
        status = "no_path"
        goal_state = None

        while open_heap:
            if expanded >= budget.max_expansions:
                status = "budget"
                break
            if 1e3 * (time.perf_counter() - t0) > budget.max_search_time_ms:
                status = "timeout"
                break
            cost, _, idx, j = heapq.heappop(open_heap)
            if cost > best_cost.get((idx, j), float("inf")) + 1e-12:
                continue
            expanded += 1
            if j == n - 1:
                # h = 0，所以出堆即当前最优；只有优于历史路径才替换
                if inc_valid and cost >= inc_cost - 1e-12:
                    status = "warm"
                    break
                goal_state = (idx, j)
                status = "repaired" if inc_valid else "ok"
                break
            for idx_next, action, _dgamma in self.graph.neighbors(idx, center):
                edge = ev.evaluate(idx, idx_next, j, gdot_now)
                if not edge.feasible:
                    continue
                g = cost + edge.cost
                state = (idx_next, j + 1)
                if g < best_cost.get(state, float("inf")) - 1e-12:
                    best_cost[state] = g
                    parent[state] = (idx, j)
                    edge_info[state] = edge
                    counter += 1
                    heapq.heappush(open_heap, (g, counter, idx_next, j + 1))

        path: list[tuple[tuple, int]] = []
        edge_rows: list[dict] = []
        if status == "warm" and inc_valid:
            path = [(tuple(idx), int(j)) for j, idx in enumerate(incumbent or [])][:n]
            edge_rows = inc_rows
        elif goal_state is not None:
            s = goal_state
            while s is not None:
                path.append(s)
                s = parent.get(s)
            path.reverse()
            for s in path[1:]:
                e = edge_info.get(s)
                edge_rows.append(e.as_row() if e is not None else {})
        elif inc_valid:
            status = "warm"                       # 预算耗尽但历史路径仍可行 -> 复用它
            path = [(tuple(idx), int(j)) for j, idx in enumerate(incumbent or [])][:n]
            edge_rows = inc_rows

        gdot_ref = np.zeros(self.cmg.spec.n)
        verified = None
        verify_ms = 0.0
        if status in ("ok", "repaired", "warm") and len(path) >= 2:
            tv = time.perf_counter()
            verified = self._verify_path(ev, path, gdot_now)
            verify_ms = 1e3 * (time.perf_counter() - tv)
            if verified:
                first = edge_info.get(path[1])
                if status == "warm" or first is None:
                    gdot_ref = np.asarray(self.graph.node_gamma(path[1][0])
                                          - self.graph.node_gamma(path[0][0]), dtype=float)
                    gdot_ref = gdot_ref / max(demand.dt, 1e-9)
                else:
                    gdot_ref = np.asarray(first.gdot_ref, dtype=float)
            else:
                status = "unverified_path"        # 路径未通过精确复核 -> 不得执行（§7.3）
        time_ms = 1e3 * (time.perf_counter() - t0)
        total_ms = 1e3 * (time.perf_counter() - t_start)
        return SearchResult(
            status=status, gdot_ref=gdot_ref, path=path,
            cost=float(best_cost.get(goal_state, float("inf"))) if goal_state
            else (inc_cost if inc_valid else float("inf")),
            expansions=expanded, time_ms=time_ms, demand_ms=demand_ms, total_ms=total_ms,
            diagnostics={
                "start_index": list(start),
                "start_gamma_deg": np.rad2deg(g0).tolist(),
                "incumbent": {"provided": incumbent is not None, "valid": inc_valid,
                              "cost": inc_cost},
                "demand": demand.meta,
                "graph": self.graph.describe(),
                "edges": edge_rows,
                "verified": verified,
                "verify_ms": verify_ms,
                "open_states": len(open_heap),
                "frame_deg": {"sigma_min": float(self.cmg.sigma_min(
                    self.cmg.jacobian(g0)))},
            },
        )

    def _evaluate_incumbent(self, ev, incumbent, start, n, gdot_now, pre_verified=False):
        """复核历史路径：必须从**当前真实构型**出发、逐边通过筛选层，再给出代价。"""
        if not incumbent:
            return float("inf"), False, []
        inc = [tuple(idx) for idx in incumbent]
        if len(inc) != n or inc[0] != tuple(start):
            return float("inf"), False, []
        total = 0.0
        rows = []
        for j, (idx, idx_next) in enumerate(zip(inc[:-1], inc[1:])):
            e = ev.evaluate(idx, idx_next, j, gdot_now)
            if not e.feasible:
                return float("inf"), False, rows
            total += e.cost
            rows.append(e.as_row())
        if pre_verified:
            rows.append({"pre_verified": True})
        return float(total), True, rows

    def _verify_path(self, ev, path, gdot_now) -> bool:
        """用精确分配 QP 逐边复核；任何一条边不过，整条路径作废（§7.3）。"""
        for a, b in zip(path[:-1], path[1:]):
            (idx, _j), (idx_next, j_next) = a, b
            res = ev.verify_edge(idx, idx_next, j_next - 1, gdot_now)
            if not res.feasible:
                return False
        return True
