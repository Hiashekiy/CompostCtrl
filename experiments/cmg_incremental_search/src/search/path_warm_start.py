"""D 组：历史路径热启动与局部修复（指导实施文件 §7.2）。

每周期流程（与 §7.2 的 7 条一一对应）：

1. 移除已执行的第一步，把剩余路径**接续到当前真实构型**：当前量化构型若与路径下一
   节点不连续，先试一次单步迁移，连不上就判定"路径失效"（不硬接）；
2. 用当前周期的未来需求**重新验证**剩余路径（逐边筛选层 + 路径级精确 QP 复核）；
3. 更新受高度/动压/参考/当前状态影响的边代价——代价本来就是按时间层 `j` 与当周期
   `demand` 现算的，因此"更新"等价于用新 demand 重算，不存在陈旧代价；
4. 优先修复失效片段：把可用预算的 `local_repair_budget_ratio` 给"历史路径附近"的修复
   （实现为带 incumbent 的有界搜索，只接受严格更优的解），其余给其他区域探索
   （`exploration_budget_ratio`）；
5. 保留路径以外的有限探索预算：
   `exploration_budget_ratio` 段预算用于纯探索（不带 incumbent 的有界搜索，取更优者）；
6. 路径失效或修复失败时扩大搜索：先是修复预算，再是完整重搜（次数上限 `max_full_replans`）；
7. 只输出通过当前可行性检查的第一步——最终仍由 `_verify_path` 的精确 QP 把关。

所有失败的路径都**不会**被执行（§7.3）；`step()` 的返回值里带 `status`，runner 见到非
`ok/warm/repaired` 就停迁移并走安全回退。
"""

from __future__ import annotations

import time
from dataclasses import dataclass

import numpy as np

from .astar_replanner import BoundedAStar, SearchBudget, SearchResult
from .incremental_repair import ReanchorResult, reanchor


@dataclass
class WarmStartStats:
    reuse_hits: int = 0
    repairs: int = 0
    full_replans: int = 0
    invalid_paths: int = 0
    reuse_time_ms: float = 0.0
    replan_time_ms: float = 0.0

    def as_dict(self) -> dict:
        n = max(self.reuse_hits + self.repairs + self.full_replans, 1)
        return {
            "reuse_hits": self.reuse_hits,
            "repairs": self.repairs,
            "full_replans": self.full_replans,
            "invalid_paths": self.invalid_paths,
            "reuse_rate": self.reuse_hits / n,
            "reuse_time_ms_mean": self.reuse_time_ms / max(self.reuse_hits, 1),
            "replan_time_ms_mean": self.replan_time_ms / max(self.repairs + self.full_replans, 1),
        }


class WarmStartReplanner:
    def __init__(self, astar: BoundedAStar, search_cfg: dict, budget: SearchBudget):
        self.astar = astar
        self.cfg = search_cfg
        self.budget = budget
        self.last_path: list[tuple[int, ...]] | None = None
        self.last_time: float | None = None
        self.last_verified: bool = False
        self.stats = WarmStartStats()
        self.max_full_replans = int(search_cfg.get("max_full_replans", 3))
        self.repair_ratio = float(search_cfg.get("local_repair_budget_ratio", 0.8))
        self.explore_ratio = float(search_cfg.get("exploration_budget_ratio", 0.2))
        self.reuse_enabled = bool(search_cfg.get("reuse_path", True))

    # ---- runner 接口 ----------------------------------------------------
    def step(self, x, t, ref, env, dt):
        res = self.plan(x, t, ref)
        return res.gdot_ref, res.info

    # ---- 主流程 ---------------------------------------------------------
    def plan(self, x, t, ref) -> SearchResult:
        dt_search = float(self.cfg.get("step_s", 0.5))
        if not self.reuse_enabled or self.last_path is None or self.last_time is None:
            return self._full_replan(x, t, ref, reason="cold")

        # 1) 接续到当前真实构型（时间层按已走过的时间片平移）
        steps = int(round((t - self.last_time) / dt_search))
        demand = self.astar._get_demand(x, t, ref, dt_search,
                                       float(self.cfg.get("horizon_s", 30.0)))
        anchor = reanchor(self.astar.graph, self.last_path, steps, x[10:14], demand.n,
                          dt_search)
        if not anchor.ok:
            self.stats.invalid_paths += 1
            return self._full_replan(x, t, ref, reason=f"reanchor:{anchor.reason}",
                                     demand=demand)

        # 2+3) 复核并（必要时）局部修复，预算按 80/20 分配
        repair_budget = SearchBudget(
            max_expansions=max(1, int(self.budget.max_expansions * self.repair_ratio)),
            max_search_time_ms=max(0.1, self.budget.max_search_time_ms * self.repair_ratio))
        t_repair = time.perf_counter()
        res = self.astar.plan(x, t, ref, demand=demand, incumbent=anchor.path,
                              budget=repair_budget)
        repair_ms = 1e3 * (time.perf_counter() - t_repair)
        if res.status in ("warm", "repaired", "ok") and self._ok(res):
            if res.status == "warm":
                self.stats.reuse_hits += 1
                self.stats.reuse_time_ms += repair_ms
            else:
                self.stats.repairs += 1
                self.stats.replan_time_ms += repair_ms
            self._remember(res, t)
            res.diagnostics["warm_start"] = {"mode": res.status, "repair_ms": repair_ms,
                                             "anchor": anchor.as_dict()}
            return res
        self.stats.invalid_paths += 1

        # 4+5) 修复失败：把探索预算用于一次不带 incumbent 的有界搜索
        explore_budget = SearchBudget(
            max_expansions=max(1, int(self.budget.max_expansions * self.explore_ratio)),
            max_search_time_ms=max(0.1, self.budget.max_search_time_ms * self.explore_ratio))
        t_ex = time.perf_counter()
        res_ex = self.astar.plan(x, t, ref, demand=demand, budget=explore_budget)
        explore_ms = 1e3 * (time.perf_counter() - t_ex)
        if res_ex.status in ("ok", "repaired"):
            self.stats.repairs += 1
            self.stats.replan_time_ms += explore_ms
            self._remember(res_ex, t)
            res_ex.diagnostics["warm_start"] = {"mode": "explore", "repair_ms": repair_ms,
                                               "explore_ms": explore_ms,
                                               "anchor": anchor.as_dict()}
            return res_ex

        # 6) 完整重搜（次数受限）
        return self._full_replan(x, t, ref, reason=f"repair-failed:{res.status}",
                                 demand=demand)

    # ---- 内部 -----------------------------------------------------------
    def _full_replan(self, x, t, ref, reason: str, demand=None) -> SearchResult:
        if self.stats.full_replans >= self.max_full_replans and self.last_path is not None:
            # 已达重搜上限：明确返回失败，由 runner 停迁移 + 安全回退（§7.3）
            return SearchResult(status="no_path", gdot_ref=np.zeros(self.astar.cmg.spec.n),
                                diagnostics={"warm_start": {"mode": "replan-capped",
                                                            "reason": reason}})
        t0 = time.perf_counter()
        res = self.astar.plan(x, t, ref, demand=demand)
        self.stats.full_replans += 1
        self.stats.replan_time_ms += 1e3 * (time.perf_counter() - t0)
        if self._ok(res):
            self._remember(res, t)
        res.diagnostics["warm_start"] = {"mode": "full_replan", "reason": reason}
        return res

    @staticmethod
    def _ok(res: SearchResult) -> bool:
        return (res.status in ("ok", "warm", "repaired")
                and res.diagnostics.get("verified") is True
                and np.all(np.isfinite(res.gdot_ref)))

    def _remember(self, res: SearchResult, t: float) -> None:
        self.last_path = [tuple(idx) for idx, _j in res.path] if res.path else None
        self.last_time = float(t)
        self.last_verified = bool(res.diagnostics.get("verified"))
