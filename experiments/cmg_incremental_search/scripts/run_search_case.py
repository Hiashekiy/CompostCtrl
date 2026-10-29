"""运行单场景闭环：B（无搜索）/ C（每周期有界 A*）/ D（热启动与局部修复）/ A（仅舵面）。

    python experiments/cmg_incremental_search/scripts/run_search_case.py --policy C
    python experiments/cmg_incremental_search/scripts/run_search_case.py --policy D --name d3
    python experiments/cmg_incremental_search/scripts/run_search_case.py --policy A \
        --set scenario.maneuver.delta_deg=8.0

输出：results/raw/<name>/{timeseries.csv,npz}、results/metrics/<name>.json、
      results/metrics/<name>_config_snapshot.yaml、（C/D）results/metrics/<name>_search.json
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import yaml

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

RAW = EXP_ROOT / "results" / "raw"
METRICS = EXP_ROOT / "results" / "metrics"


def apply_overrides(cfg: dict, overrides: list[str]) -> dict:
    for item in overrides:
        key, _, value = item.partition("=")
        node = cfg
        parts = key.split(".")
        for p in parts[:-1]:
            node = node.setdefault(p, {})
        node[parts[-1]] = yaml.safe_load(value)
    return cfg


def build_searcher(cfg: dict, policy: str):
    spec = build_spec(cfg)
    cmg = PyramidCMG(spec.cmg)
    ctrl = AttitudeController(spec, PDGains.from_dict(cfg["control"]))
    alloc = ControlAllocator(spec, AllocatorWeights.from_dict(cfg["allocation"]))
    budget = SearchBudget.from_dict(cfg["search"])
    astar = BoundedAStar(cmg, alloc, ctrl, spec, cfg["graph"], cfg["search"],
                         EdgeWeights.from_dict(cfg["search"]), budget)
    if policy == "D":
        return WarmStartReplanner(astar, cfg["search"], budget)
    return astar


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--policy", default="C", choices=list("ABCD"))
    ap.add_argument("--scenario", default="scenario_high_altitude_maneuver.yaml",
                    help="场景配置（例如 scenario_main.yaml）")
    ap.add_argument("--name", default=None)
    ap.add_argument("--set", dest="overrides", action="append", default=[])
    args = ap.parse_args()

    cfg = load_config("search.yaml", args.scenario)
    cfg = apply_overrides(cfg, args.overrides)
    cfg["policy"] = args.policy
    if args.policy == "A":
        cfg["actuators"] = {"cmg_on": False}
    name = args.name or f"policy_{args.policy}"

    search = build_searcher(cfg, args.policy) if args.policy in "CD" else None
    res = run_case(cfg, name=name, search=search)
    paths = res.log.save(RAW / name)
    save_json(res.metrics, METRICS / f"{name}.json")
    save_json(res.meta, METRICS / f"{name}_meta.json")
    (METRICS / f"{name}_config_snapshot.yaml").write_text(
        yaml.safe_dump(cfg, allow_unicode=True, sort_keys=True), encoding="utf-8", newline="\n")

    arr = res.log.arrays()
    extra = {}
    if search is not None and "search_status" in arr:
        statuses = np.asarray(arr["search_status"])
        extra["search_status_counts"] = {s: int(np.sum(statuses == s))
                                        for s in set(statuses.tolist())}
        mk = "search_time_ms" if "search_time_ms" in arr else None
        if mk:
            extra["search_ms_mean"] = float(np.mean(arr[mk]))
            extra["search_ms_max"] = float(np.max(arr[mk]))
        extra["search_nodes_mean"] = float(np.mean(arr["search_nodes"]))
        extra["search_nodes_max"] = float(np.max(arr["search_nodes"]))
    if hasattr(search, "stats"):
        extra["warm_start"] = search.stats.as_dict()
    if extra:
        save_json(extra, METRICS / f"{name}_search.json")

    m = res.metrics
    print(f"[{name}] policy={args.policy} att rmse={m['att_rmse_deg']:.3f} deg, "
          f"peak={m['att_peak_deg']:.3f}, final={m['att_final_deg']:.3f}, "
          f"settle={m['t_settle_s']} s, success={m['success']}")
    print(f"  residual rms/max={m['residual_rms_nm']:.3f}/{m['residual_max_nm']:.3f} N m, "
          f"request_infeasible={m['infeasible_count']}, fallback={m['fallback_count']}, "
          f"timeout={m['timeout_count']}")
    print(f"  gamma max={m['gamma_max_deg']:.1f} deg, sigma_min={m['sigma_min_min']:.2f}, "
          f"H_max={m['H_norm_max']:.1f} N m s")
    print(f"  ctrl ms mean/max={m['ctrl_ms_mean']:.2f}/{m['ctrl_ms_max']:.2f}, "
          f"search ms mean/max={m['search_ms_mean']:.2f}/{m['search_ms_max']:.2f}")
    if extra:
        print(f"  {extra}")
    print(f"  -> {paths['csv']}")
    return 0


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    raise SystemExit(main())
