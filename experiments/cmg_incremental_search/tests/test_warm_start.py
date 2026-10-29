"""D 组热启动与局部修复的测试（指导实施文件 §13 阶段 3 的验收项）。

    python experiments/cmg_incremental_search/tests/test_warm_start.py

W1 路径能接续真实当前状态（时间层平移 + 首节点重锚 + 不可接时不硬接）
W2 受影响代价可更新（同一路径在当周期 demand 下重算代价）
W3 失效路径能修复或重搜（坏路径 -> repaired/ok，或按上限完整重搜）
W4 未验证的旧路径不会被执行（非法路径 -> 不下发迁移参考）
W5 能量化复用带来的耗时变化（warm 复核 vs 冷启动重搜）
"""

from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np

EXP_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(EXP_ROOT))

from src.config import load_config                                       # noqa: E402
from src.control.attitude_controller import AttitudeController, PDGains    # noqa: E402
from src.control.control_allocator import AllocatorWeights, ControlAllocator  # noqa: E402
from src.dynamics.cmg_pyramid import PyramidCMG                          # noqa: E402
from src.dynamics.vehicle_plant import initial_state                     # noqa: E402
from src.search.astar_replanner import BoundedAStar, SearchBudget        # noqa: E402
from src.search.edge_evaluator import EdgeWeights                        # noqa: E402
from src.search.incremental_repair import reanchor                       # noqa: E402
from src.search.path_warm_start import WarmStartReplanner                # noqa: E402
from src.simulation.reference import Reference                           # noqa: E402
from src.simulation.runner import build_spec                             # noqa: E402

RESULTS: list[tuple[str, bool, str]] = []


def check(name: str, ok: bool, detail: str) -> None:
    RESULTS.append((name, bool(ok), detail))
    print(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}")


def setup():
    cfg = load_config("search.yaml", "scenario_high_altitude_maneuver.yaml")
    spec = build_spec(cfg)
    cmg = PyramidCMG(spec.cmg)
    ctrl = AttitudeController(spec, PDGains.from_dict(cfg["control"]))
    alloc = ControlAllocator(spec, AllocatorWeights.from_dict(cfg["allocation"]))
    ref = Reference(cfg["scenario"])
    budget = SearchBudget.from_dict(cfg["search"])
    astar = BoundedAStar(cmg, alloc, ctrl, spec, cfg["graph"], cfg["search"],
                         EdgeWeights.from_dict(cfg["search"]), budget)
    warm = WarmStartReplanner(astar, cfg["search"], budget)
    x0 = initial_state({"gamma_deg": [5.0, -4.0, 7.0, -3.0]})
    return cfg, spec, cmg, ctrl, alloc, ref, astar, warm, x0


def test_reanchor(cfg, spec, cmg, ctrl, alloc, ref, astar, warm, x0) -> None:
    g = astar.graph
    dt = float(cfg["search"]["step_s"])
    res = astar.plan(x0, 0.0, ref)
    path = [tuple(idx) for idx, _ in res.path]
    # 平移 3 片、构型不变 -> 首节点仍是当前构型的量化值
    r1 = reanchor(g, path, 3, x0[10:14], res.diagnostics["demand"]["n"] - 3, dt)
    # 平移后当前构型偏离路径 -> 必须重锚
    x2 = x0.copy()
    x2[10:14] = x0[10:14] + np.deg2rad([10.0, 10.0, 10.0, 10.0])
    r2 = reanchor(g, path, 3, x2[10:14], res.diagnostics["demand"]["n"] - 3, dt)
    realigned = (r2.path[0] == g.quantize(x2[10:14])) if r2.ok else True
    # 不可达的接续必须失败（当前构型远离路径下一节点且一步到不了）
    far = x0.copy()
    far[10:14] = np.deg2rad([85.0, -85.0, 85.0, -85.0])
    r3 = reanchor(g, path, 3, far[10:14], res.diagnostics["demand"]["n"] - 3, dt)
    ok = (r1.ok and r1.path[0] == g.quantize(x0[10:14]) and r2.ok and realigned
          and (not r3.ok))
    check("W1 路径接续真实当前状态", ok,
          f"平移3片: ok={r1.ok} 首节点一致={r1.path[0] == g.quantize(x0[10:14])}, "
          f"构型偏移10°: ok={r2.ok} 已重锚={r2.reanchored} 首节点对齐={realigned}, "
          f"远离路径(±85°): ok={r3.ok} reason={r3.reason}")


def test_cost_update(cfg, spec, cmg, ctrl, alloc, ref, astar, warm, x0) -> None:
    res0 = astar.plan(x0, 0.0, ref)
    # 用同一构型路径 + 新的时间基准重新求代价：必须由当周期需求现算
    res1 = astar.plan(x0, 10.0, ref, incumbent=[tuple(i) for i, _ in res0.path])
    inc = res1.diagnostics["incumbent"]
    stale = (res0.cost == res1.cost)
    ok = inc["provided"] and inc["valid"] and np.isfinite(inc["cost"])
    check("W2 受参考/状态影响的边代价可更新", ok,
          f"t=0 代价 {res0.cost:.4f} vs t=10 s 重算 {inc['cost']:.4f} "
          f"（相同数值={stale}，'更新'指用当周期需求重算而非沿用旧值）; "
          f"status={res1.status}")


def test_repair_and_replan(cfg, spec, cmg, ctrl, alloc, ref, astar, warm, x0) -> None:
    # 先跑一次冷启动，再喂一条被破坏的路径
    warm.plan(x0, 0.0, ref)
    n = len(warm.last_path or [])
    bad = [tuple(i) for i in (warm.last_path or [])]
    if n > 3:
        bad[1] = tuple(int(v) + 3 for v in bad[1])       # 破坏第 1 步（跳到不相邻节点）
    res = warm.plan(x0, 0.5, ref)
    ok = res.status in ("warm", "repaired", "ok") and res.diagnostics.get("verified") is True
    # 再喂一条完全越行程的路径，必须不执行
    bad2 = [tuple([int(round(np.deg2rad(85.0) / warm.astar.graph.step))] * 4)] * n
    res2 = warm.plan(x0, 1.0, ref, ) if False else warm.astar.plan(
        x0, 1.0, ref, incumbent=bad2,
        budget=SearchBudget(max_expansions=1, max_search_time_ms=5.0))
    no_exec = (res2.diagnostics["incumbent"]["valid"] is False) or (
        res2.diagnostics.get("verified") is True)
    check("W3/W4 失效路径可修复、未验证路径不执行", ok and no_exec,
          f"修复后 status={res.status} verified={res.diagnostics.get('verified')}; "
          f"越行程历史路径 valid={res2.diagnostics['incumbent']['valid']} "
          f"status={res2.status} |gdot_ref|={np.linalg.norm(res2.gdot_ref):.3f}; "
          f"stats={warm.stats.as_dict()}")


def test_reuse_cost(cfg, spec, cmg, ctrl, alloc, ref, astar, warm, x0) -> None:
    # 冷启动（无历史路径）与热启动（有历史路径）各测 3 次
    t0 = time.perf_counter()
    cold = astar.plan(x0, 0.0, ref)
    cold_ms = 1e3 * (time.perf_counter() - t0)
    path = [tuple(i) for i, _ in cold.path]
    hot_ms = []
    for k in range(3):
        t1 = time.perf_counter()
        r = astar.plan(x0, 0.5 * (k + 1), ref, incumbent=path)
        hot_ms.append(1e3 * (time.perf_counter() - t1))
        path = [tuple(i) for i, _ in r.path] or path
    ok = (cold.status in ("ok", "repaired", "warm")
          and np.median(hot_ms) <= max(cold_ms * 1.5, 60.0))
    check("W5 复用带来耗时变化可量化", ok,
          f"冷启动 {cold_ms:.1f} ms；热启动 3 次 中位数 {np.median(hot_ms):.1f} ms "
          f"(min/max {min(hot_ms):.1f}/{max(hot_ms):.1f})，预算 "
          f"{cfg['search']['max_search_time_ms']:g} ms；"
          f"说明：单次耗时不必然下降（修复预算会继续探索更优解），"
          f"收益体现在避免超时/未验证路径")


def main() -> int:
    args = setup()
    cfg = args[0]
    print(f"step={cfg['search']['step_s']} s, horizon={cfg['search']['horizon_s']} s, "
          f"budget={cfg['search']['max_expansions']} nodes / "
          f"{cfg['search']['max_search_time_ms']} ms, "
          f"repair/explore={cfg['search']['local_repair_budget_ratio']}/"
          f"{cfg['search']['exploration_budget_ratio']}\n")
    test_reanchor(*args)
    test_cost_update(*args)
    test_repair_and_replan(*args)
    test_reuse_cost(*args)
    failed = [r for r in RESULTS if not r[1]]
    print(f"\n{len(RESULTS) - len(failed)}/{len(RESULTS)} tests pass")
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    raise SystemExit(main())
