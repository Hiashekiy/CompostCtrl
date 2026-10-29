"""局部修复的基础件（指导实施文件 §7.2 第 1–3 条）。

标准 D* Lite 不能原样处理滚动时间展开图（时间层每周期都要重排），因此这里只实现
**可验证**的两件事：

* `reanchor`：把上一周期路径去掉已执行的 `steps` 片、并把首节点换成当前真实构型的
  量化节点；若换首节点后第一步迁移在机械上不可达，则返回失败（不硬接）；
* `affected_edges`：给出受"当前状态/高度/动压/参考"变化影响的边索引，供调用方决定
  重新评估哪些片段（代价本身总是用当周期 demand 现算，因此这里只是**记账**）。
"""

from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np

from .configuration_graph import ConfigurationGraph


@dataclass
class ReanchorResult:
    ok: bool
    reason: str
    path: list[tuple[int, ...]] = field(default_factory=list)
    dropped_steps: int = 0
    reanchored: bool = False
    first_step_reachable: bool = True

    def as_dict(self) -> dict:
        return {"ok": self.ok, "reason": self.reason, "dropped_steps": self.dropped_steps,
                "reanchored": self.reanchored,
                "first_step_reachable": self.first_step_reachable}


def reanchor(graph: ConfigurationGraph, path: list[tuple[int, ...]], steps: int,
             gamma_now, horizon_n: int, dt_search: float) -> ReanchorResult:
    if not path:
        return ReanchorResult(False, "empty")
    steps = int(max(0, steps))
    tail = [tuple(idx) for idx in path[steps:]]
    if not tail:
        return ReanchorResult(False, "consumed", dropped_steps=steps)
    if horizon_n <= 0:
        return ReanchorResult(False, "empty-horizon")

    start = graph.quantize(gamma_now)
    reanchored = start != tail[0]
    if reanchored:
        # 当前构型与路径下一节点不连续：检查一步能否迁到路径的下一节点
        nxt = tail[0] if len(tail) > 1 else None
        if nxt is None:
            return ReanchorResult(False, "no-next-node", dropped_steps=steps)
        mech = graph.edge_mechanics(start, nxt, np.zeros(graph.spec.n), dt_search)
        if not mech.feasible:
            return ReanchorResult(False, f"cannot-join:{mech.reason}", dropped_steps=steps,
                                  reanchored=True, first_step_reachable=False)
        tail = [start] + tail
    else:
        tail = [start] + tail[1:]

    # 时间层必须正好覆盖当前窗口
    if len(tail) > horizon_n:
        tail = tail[:horizon_n]
    while len(tail) < horizon_n:
        tail.append(tail[-1])          # 窗口比历史路径长：末端保持构型
    return ReanchorResult(True, "ok", path=tail, dropped_steps=steps, reanchored=reanchored)


def affected_edges(prev_env: dict, now_env: dict, gamma_changed: bool, tol: float = 0.05) -> dict:
    """哪些量变化会改变边代价（记账用，不参与决策）。"""
    out = {"rho": False, "V": False, "reference": False, "state": gamma_changed}
    for key, name in (("rho", "rho"), ("V", "V")):
        a, b = float(prev_env.get(key, 0.0)), float(now_env.get(key, 0.0))
        denom = max(abs(a), 1e-12)
        out[name] = abs(b - a) / denom > tol
    out["reference"] = bool(prev_env.get("maneuver_phase", 0) != now_env.get("maneuver_phase", 0))
    return out
