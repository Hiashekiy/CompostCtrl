"""构型图与有界 A* 的离线测试（指导实施文件 §13 阶段 2 的验收项）。

    python experiments/cmg_incremental_search/tests/test_search_graph.py

检查：
  S1 节点与边满足机械约束（行程、框架角速度、角加速度可达性、邻居数上限）
  S2 未来需求时间对齐正确（时刻/权重/力矩来自控制律而非角度差）
  S3 边代价与可行性可追踪（成本分解、失败原因）
  S4 A* 返回经复核的可行路径，且只取第一步
  S5 预算耗尽/超时可安全回退（status 非 ok 时输出零迁移参考）
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
from src.search.configuration_graph import ConfigurationGraph            # noqa: E402
from src.search.edge_evaluator import EdgeEvaluator, EdgeWeights         # noqa: E402
from src.search.future_demand import build_demand                        # noqa: E402
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
    x0 = initial_state({"gamma_deg": [5.0, -4.0, 7.0, -3.0], "euler_deg": [0.3, -0.4, 0.2]})
    return cfg, spec, cmg, ctrl, alloc, ref, x0


def test_graph_mechanics(cfg, spec, cmg, ctrl, alloc, ref, x0) -> None:
    g = ConfigurationGraph(cmg, angular_step_deg=cfg["graph"]["angular_step_deg"],
                           max_neighbors=cfg["graph"]["max_neighbors"],
                           local_radius_deg=cfg["graph"]["local_radius_deg"])
    start = g.quantize(x0[10:14])
    nbrs = g.neighbors(start, start)
    dt = float(cfg["search"]["step_s"])
    worst = {}
    ok = len(nbrs) <= g.max_neighbors and any(a == "hold" for _, a, _ in nbrs)
    for idx, action, _ in nbrs:
        mech = g.edge_mechanics(start, idx, x0[14:18], dt)
        gd = abs(mech.gdot_ref)
        worst["gdot_max"] = max(worst.get("gdot_max", 0.0), float(gd.max()))
        worst["dgamma_max_deg"] = max(worst.get("dgamma_max_deg", 0.0),
                                      float(np.rad2deg(np.abs(mech.dgamma).max())))
        worst["gamma_max_deg"] = max(worst.get("gamma_max_deg", 0.0),
                                     float(np.rad2deg(np.abs(g.node_gamma(idx)).max())))
        ok = ok and mech.feasible and bool(np.all(gd <= spec.cmg.gdot_max + 1e-12))
    check("S1 节点/边满足机械约束", ok,
          f"{len(nbrs)} 个动作（含保持）；|gdot_ref|max={worst['gdot_max']:.3f} rad/s "
          f"<= {spec.cmg.gdot_max}；|dgamma|max={worst['dgamma_max_deg']:.1f} deg/步；"
          f"|gamma|max={worst['gamma_max_deg']:.1f} deg <= {spec.cmg.travel_deg}")

    # 超行程的边必须被拒绝
    far = tuple(int(i) for i in np.round(np.deg2rad([95.0, 0.0, 0.0, 0.0]) / g.step))
    far_node = tuple(int(np.clip(i, -g.max_step_index, g.max_step_index)) for i in far)
    mech = g.edge_mechanics(start, far_node, x0[14:18], dt)
    check("S1b 越行程/超速的边被判不可行", (not mech.feasible) or
          bool(np.all(np.abs(g.node_gamma(far_node)) <= spec.cmg.travel + 1e-12)),
          f"目标构型 {np.rad2deg(g.node_gamma(far_node))} deg, reason={mech.reason}")


def test_demand_alignment(cfg, spec, cmg, ctrl, alloc, ref, x0) -> None:
    demand = build_demand(x0, 0.0, ref, ctrl, alloc, spec,
                          float(cfg["search"]["step_s"]), float(cfg["search"]["horizon_s"]),
                          float(cfg["search"]["w_near"]), float(cfg["search"]["w_far"]))
    dt = demand.dt
    t_ok = np.allclose(demand.t, dt * np.arange(demand.n))
    n_ok = demand.n == int(round(cfg["search"]["horizon_s"] / dt))
    w_ok = abs(float(demand.weight.sum()) - 1.0) < 1e-9
    # tau_req 必须等于控制律在预测状态上的输出（不是角度差直接当力矩）
    j = demand.n // 3
    tau_ref = ctrl.torque_request(demand.x_pred[j], demand.q_ref[j], demand.omega_ref[j])
    tau_ok = float(np.max(np.abs(tau_ref - demand.tau_req[j]))) < 1e-9
    # 关键机动窗口内权重应高于窗口之前（本场景 horizon 恰好覆盖到机动结束，无"远期"区段）
    win = demand.high_window
    w_ok2 = True
    detail_w = ""
    if win is not None:
        lo, hi = win
        before = float(np.mean(demand.weight[:lo])) if lo > 0 else float("nan")
        inside = float(np.mean(demand.weight[lo:hi]))
        w_ok2 = inside >= before - 1e-12
        detail_w = f"平均权重 窗口内={inside:.4f} vs 窗口前={before:.4f}"
    check("S2 未来需求时间对齐正确", t_ok and n_ok and w_ok and tau_ok and w_ok2,
          f"N={demand.n}, dt={dt}s, sum(w)={demand.weight.sum():.6f}, high_window={win}, "
          f"|tau_req-ctrl(x_pred)|max="
          f"{np.max(np.abs(tau_ref - demand.tau_req[j])):.2e} N m, {detail_w}, "
          f"tau_req峰值={demand.meta['tau_req_peak']:.1f} N m")


def test_edge_traceability(cfg, spec, cmg, ctrl, alloc, ref, x0) -> None:
    g = ConfigurationGraph(cmg, float(cfg["graph"]["angular_step_deg"]),
                           int(cfg["graph"]["max_neighbors"]),
                           float(cfg["graph"]["local_radius_deg"]))
    demand = build_demand(x0, 0.0, ref, ctrl, alloc, spec,
                          float(cfg["search"]["step_s"]), float(cfg["search"]["horizon_s"]),
                          float(cfg["search"]["w_near"]), float(cfg["search"]["w_far"]))
    ev = EdgeEvaluator(cmg, alloc, g, EdgeWeights.from_dict(cfg["search"]), demand)
    start = g.quantize(x0[10:14])
    rows = []
    for idx, _a, _dg in g.neighbors(start, start):
        r = ev.evaluate(start, idx, 0, x0[14:18])
        rows.append(r.as_row())
    keys = {"feasible", "reason", "cost", "J_track", "J_move", "J_sing", "J_future",
            "residual_norm", "gdot_ref_norm"}
    has_keys = all(keys <= set(r) for r in rows)
    finite = all(np.isfinite(r["cost"]) for r in rows if r["feasible"])
    reasons = {r["reason"] for r in rows}
    check("S3 边代价与可行性可追踪", has_keys and finite,
          f"{len(rows)} 条边；字段齐全={has_keys}；可行边代价有限={finite}；"
          f"示例 J_move={rows[1]['J_move']:.3f} J_sing={rows[1]['J_sing']:.3f} "
          f"J_future={rows[1]['J_future']:.3e} residual={rows[1]['residual_norm']:.3f} N m；"
          f"reasons={sorted(reasons)}")


def _astar(cfg, spec, cmg, ctrl, alloc, max_exp=400, max_ms=60.0):
    budget = SearchBudget(max_expansions=max_exp, max_search_time_ms=max_ms)
    return BoundedAStar(cmg, alloc, ctrl, spec, cfg["graph"], cfg["search"],
                        EdgeWeights.from_dict(cfg["search"]), budget)


def test_astar_path(cfg, spec, cmg, ctrl, alloc, ref, x0) -> None:
    astar = _astar(cfg, spec, cmg, ctrl, alloc)
    t0 = time.perf_counter()
    res = astar.plan(x0, 0.0, ref)
    wall = 1e3 * (time.perf_counter() - t0)
    ok = res.status == "ok" and len(res.path) > 2
    detail = (f"status={res.status}, expansions={res.expansions}, "
              f"图搜索 {res.time_ms:.1f} ms / 需求 {res.demand_ms:.1f} ms / "
              f"总计 {res.total_ms:.1f} ms (wall {wall:.1f} ms), cost={res.cost:.3f}, "
              f"path_len={len(res.path)}")
    if ok:
        edges = res.diagnostics["edges"]
        feas = all(e.get("feasible", False) for e in edges)
        rate = max(float(e.get("gdot_ref_norm", 0.0)) for e in edges)
        step_ok = float(np.linalg.norm(res.gdot_ref)) <= spec.cmg.gdot_max + 1e-9
        verified = res.diagnostics.get("verified")
        ok = ok and feas and rate <= spec.cmg.gdot_max + 1e-9 and step_ok and verified is True
        detail += (f"；筛选层全可行={feas}；|gdot_ref|max={rate:.3f} rad/s；"
                   f"精确复核={verified}（{res.diagnostics.get('verify_ms', 0):.1f} ms）；"
                   f"下发第一步 |gdot_ref|={np.linalg.norm(res.gdot_ref):.3f} rad/s")
    check("S4 A* 返回经复核的可行路径（只取第一步）", ok, detail)


def test_budget_fallback(cfg, spec, cmg, ctrl, alloc, ref, x0) -> None:
    a1 = _astar(cfg, spec, cmg, ctrl, alloc, max_exp=1, max_ms=60000.0)
    r1 = a1.plan(x0, 0.0, ref)
    a2 = _astar(cfg, spec, cmg, ctrl, alloc, max_exp=100000, max_ms=0.001)
    r2 = a2.plan(x0, 0.0, ref)
    ok = (r1.status in ("budget", "no_path", "timeout") and r2.status in ("timeout", "budget",
                                                                         "no_path")
          and np.allclose(r1.gdot_ref, 0.0) and np.allclose(r2.gdot_ref, 0.0))
    check("S5 预算耗尽/超时安全回退", ok,
          f"max_expansions=1 -> status={r1.status}, gdot_ref={r1.gdot_ref.tolist()}；"
          f"max_search_time_ms=0.001 -> status={r2.status}, "
          f"gdot_ref={np.round(r2.gdot_ref, 6).tolist()}")


def main() -> int:
    args = setup()
    cfg, spec = args[0], args[1]
    print(f"graph: step={cfg['graph']['angular_step_deg']} deg, "
          f"radius={cfg['graph']['local_radius_deg']} deg, "
          f"budget: {cfg['search']['max_expansions']} nodes / "
          f"{cfg['search']['max_search_time_ms']} ms\n")
    test_graph_mechanics(*args)
    test_demand_alignment(*args)
    test_edge_traceability(*args)
    test_astar_path(*args)
    test_budget_fallback(*args)
    failed = [r for r in RESULTS if not r[1]]
    print(f"\n{len(RESULTS) - len(failed)}/{len(RESULTS)} tests pass")
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    raise SystemExit(main())
