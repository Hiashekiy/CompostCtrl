"""阶段 8 实验矩阵：A/B/C/D + 五项消融 × 5 个初始构型。

    python experiments/cmg_incremental_search/scripts/run_matrix.py --jobs 1
    python experiments/cmg_incremental_search/scripts/run_matrix.py --only core

矩阵（每次运行 60 s，200 ms 控制周期，主场景 configs/scenario_main.yaml）：
  core      A / B / C / D
  ablation  C_nofuture（去未来需求项）/ C_nocapability（用静态 sigma_min 代替可实现性）
            / C_nosing（去奇异风险项）/ D_noreuse（关历史复用->退化为 C）
            / D_noexplore（关路径外探索）
初始构型 5 个（指导实施文件 §9："正式实验至少测试 5 个合理的初始 CMG 构型"）。
本实验的扰动是确定性的（无随机项），因此不需要配对随机种子；这点在最终报告里写明。

输出：results/metrics/matrix_runs.csv（每次运行一行）、matrix_<name>.json（明细）
"""

from __future__ import annotations

import argparse
import csv
import sys
import time
from pathlib import Path

import numpy as np

EXP_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(EXP_ROOT))

from src.config import load_config                                        # noqa: E402
from src.control.attitude_controller import AttitudeController, PDGains  # noqa: E402
from src.control.control_allocator import AllocatorWeights, ControlAllocator  # noqa: E402
from src.dynamics.cmg_pyramid import PyramidCMG                          # noqa: E402
from src.search.astar_replanner import BoundedAStar, SearchBudget        # noqa: E402
from src.search.edge_evaluator import EdgeWeights                        # noqa: E402
from src.search.path_warm_start import WarmStartReplanner                # noqa: E402
from src.simulation.logger import save_json                              # noqa: E402
from src.simulation.runner import build_spec, run_case                   # noqa: E402

METRICS = EXP_ROOT / "results" / "metrics"
RAW = EXP_ROOT / "results" / "raw"

# 初始 CMG 构型（deg）：家位 / 小幅 / 交替 / 较大 / 接近行程边缘
INITIAL_GAMMAS = {
    "g0_home": [0.0, 0.0, 0.0, 0.0],
    "g1_small": [5.0, -4.0, 7.0, -3.0],
    "g2_alt": [-10.0, 10.0, -10.0, 10.0],
    "g3_large": [20.0, -20.0, 20.0, -20.0],
    "g4_edge": [40.0, -40.0, 40.0, -40.0],
}

CASES = {
    "A": {"policy": "A", "overrides": {}},
    "B": {"policy": "B", "overrides": {}},
    "C": {"policy": "C", "overrides": {}},
    "D": {"policy": "D", "overrides": {}},
    "C_nofuture": {"policy": "C", "overrides": {"search.lambda_future": 0.0}},
    "C_nocapability": {"policy": "C", "overrides": {"search.future_mode": "sigma"}},
    "C_nosing": {"policy": "C", "overrides": {"search.lambda_sing": 0.0}},
    "D_noreuse": {"policy": "D", "overrides": {"search.reuse_path": False}},
    "D_noexplore": {"policy": "D", "overrides": {"search.exploration_budget_ratio": 0.0,
                                                 "search.local_repair_budget_ratio": 1.0}},
}
CORE = ["A", "B", "C", "D"]
ABLATIONS = ["C_nofuture", "C_nocapability", "C_nosing", "D_noreuse", "D_noexplore"]

FIELDS = ["case", "policy", "initial", "success", "att_rmse_deg", "att_peak_deg",
          "att_final_deg", "t_settle_s", "residual_rms_nm", "residual_max_nm",
          "infeasible_count", "fallback_count", "aero_sat_frac", "gamma_max_deg",
          "sigma_min_min", "H_norm_max", "gddot_peak_rad_s2", "ctrl_ms_mean",
          "ctrl_ms_max", "search_ms_mean", "search_ms_max", "search_nodes_mean",
          "reuse_rate", "full_replans"]


def apply_overrides(cfg: dict, overrides: dict) -> dict:
    for key, value in overrides.items():
        node = cfg
        parts = key.split(".")
        for p in parts[:-1]:
            node = node.setdefault(p, {})
        node[parts[-1]] = value
    return cfg


def build_searcher(cfg: dict, policy: str):
    spec = build_spec(cfg)
    cmg = PyramidCMG(spec.cmg)
    ctrl = AttitudeController(spec, PDGains.from_dict(cfg["control"]))
    alloc = ControlAllocator(spec, AllocatorWeights.from_dict(cfg["allocation"]))
    budget = SearchBudget.from_dict(cfg["search"])
    astar = BoundedAStar(cmg, alloc, ctrl, spec, cfg["graph"], cfg["search"],
                         EdgeWeights.from_dict(cfg["search"]), budget)
    return WarmStartReplanner(astar, cfg["search"], budget) if policy == "D" else astar


def run_one(case: str, initial: str) -> dict:
    spec = CASES[case]
    cfg = load_config("search.yaml", "scenario_main.yaml")
    cfg = apply_overrides(cfg, spec["overrides"])
    cfg["policy"] = spec["policy"]
    if spec["policy"] == "A":
        cfg["actuators"] = {"cmg_on": False}
    name = f"{case}__{initial}"
    search = build_searcher(cfg, spec["policy"]) if spec["policy"] in "CD" else None
    x0 = {"gamma_deg": INITIAL_GAMMAS[initial]}
    res = run_case(cfg, name=name, search=search, x0=x0)
    m = res.metrics
    arr = res.log.arrays()
    extra = {}
    if search is not None and "search_status" in arr:
        st = np.asarray(arr["search_status"])
        extra["search_status_counts"] = {s: int(np.sum(st == s)) for s in set(st.tolist())}
    if hasattr(search, "stats"):
        extra["warm_start"] = search.stats.as_dict()
    res.log.save(RAW / name)
    save_json(m, METRICS / f"{name}.json")
    save_json(extra, METRICS / f"{name}_search.json")
    ws = extra.get("warm_start", {})
    return {
        "case": case, "policy": spec["policy"], "initial": initial,
        "success": bool(m["success"]), "att_rmse_deg": m["att_rmse_deg"],
        "att_peak_deg": m["att_peak_deg"], "att_final_deg": m["att_final_deg"],
        "t_settle_s": m["t_settle_s"], "residual_rms_nm": m["residual_rms_nm"],
        "residual_max_nm": m["residual_max_nm"], "infeasible_count": m["infeasible_count"],
        "fallback_count": m["fallback_count"], "aero_sat_frac": m["aero_sat_frac"],
        "gamma_max_deg": m["gamma_max_deg"], "sigma_min_min": m["sigma_min_min"],
        "H_norm_max": m["H_norm_max"], "gddot_peak_rad_s2": m["gddot_peak_rad_s2"],
        "ctrl_ms_mean": m["ctrl_ms_mean"], "ctrl_ms_max": m["ctrl_ms_max"],
        "search_ms_mean": m["search_ms_mean"], "search_ms_max": m["search_ms_max"],
        "search_nodes_mean": m.get("search_nodes_mean", 0.0),
        "reuse_rate": ws.get("reuse_rate", 0.0),
        "full_replans": ws.get("full_replans", 0),
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--only", default="all", choices=["all", "core", "ablation"])
    ap.add_argument("--tag", default="matrix_runs")
    args = ap.parse_args()

    cases = CORE if args.only == "core" else ABLATIONS if args.only == "ablation" else \
        CORE + ABLATIONS
    rows: list[dict] = []
    t_start = time.perf_counter()
    for case in cases:
        for initial in INITIAL_GAMMAS:
            t0 = time.perf_counter()
            row = run_one(case, initial)
            rows.append(row)
            print(f"{case:14s} {initial:9s} rmse={row['att_rmse_deg']:6.3f} "
                  f"peak={row['att_peak_deg']:6.3f} resid_max={row['residual_max_nm']:6.1f} "
                  f"fb={row['fallback_count']:3d} sigma={row['sigma_min_min']:6.2f} "
                  f"ctrl_max={row['ctrl_ms_max']:6.1f} ms "
                  f"[{time.perf_counter() - t0:5.1f} s]", flush=True)

    METRICS.mkdir(parents=True, exist_ok=True)
    out = METRICS / f"{args.tag}.csv"
    with out.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    save_json({"rows": rows, "cases": cases, "initial_gammas": INITIAL_GAMMAS,
               "elapsed_s": time.perf_counter() - t_start}, METRICS / f"{args.tag}.json")
    print(f"\n{len(rows)} runs in {time.perf_counter() - t_start:.0f} s -> {out}")
    return 0


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    raise SystemExit(main())
