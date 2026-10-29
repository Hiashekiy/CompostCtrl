"""运行基础闭环（B 组形态：无构型迁移参考）并落盘原始时序与指标。

    python experiments/cmg_incremental_search/scripts/run_base_case.py
    python experiments/cmg_incremental_search/scripts/run_base_case.py --name my_case \\
        --set scenario.maneuver.delta_deg=5.0

输出：
    results/raw/<name>/timeseries.csv, timeseries.npz
    results/metrics/<name>.json
    results/metrics/<name>_config.yaml   （配置快照，指导实施文件 §4.3）
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import yaml

EXP_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(EXP_ROOT))

from src.config import EXP_ROOT as _  # noqa: E402,F401
from src.config import load_config                                  # noqa: E402
from src.simulation.logger import save_json                          # noqa: E402
from src.simulation.runner import run_case                           # noqa: E402

RAW = EXP_ROOT / "results" / "raw"
METRICS = EXP_ROOT / "results" / "metrics"


def apply_overrides(cfg: dict, overrides: list[str]) -> dict:
    for item in overrides:
        key, _, value = item.partition("=")
        node = cfg
        parts = key.split(".")
        for p in parts[:-1]:
            node = node.setdefault(p, {})
        raw = yaml.safe_load(value)
        node[parts[-1]] = raw
    return cfg


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--name", default="base_B")
    ap.add_argument("--set", dest="overrides", action="append", default=[],
                    help="例如 scenario.maneuver.delta_deg=5.0")
    args = ap.parse_args()

    cfg = load_config("search.yaml", "scenario_high_altitude_maneuver.yaml")
    cfg = apply_overrides(cfg, args.overrides)

    res = run_case(cfg, name=args.name)
    paths = res.log.save(RAW / args.name)
    save_json(res.metrics, METRICS / f"{args.name}.json")
    save_json(res.meta, METRICS / f"{args.name}_meta.json")
    (METRICS / f"{args.name}_config_snapshot.yaml").write_text(
        yaml.safe_dump(cfg, allow_unicode=True, sort_keys=True), encoding="utf-8", newline="\n")

    m = res.metrics
    print(f"[{args.name}] att rmse={m['att_rmse_deg']:.3f} deg, peak={m['att_peak_deg']:.3f}, "
          f"final={m['att_final_deg']:.3f}, settle={m['t_settle_s']} s, "
          f"success={m['success']}")
    print(f"  residual rms/max = {m['residual_rms_nm']:.3f}/{m['residual_max_nm']:.3f} N m, "
          f"request_infeasible={m['infeasible_count']}, fallback={m['fallback_count']}")
    print(f"  gdot peak={m['gdot_peak_rad_s']:.3f} rad/s, gddot peak={m['gddot_peak_rad_s2']:.2f} "
          f"rad/s^2, gamma max={m['gamma_max_deg']:.1f} deg, sigma_min={m['sigma_min_min']:.2f}")
    print(f"  alloc ms mean/max = {m['alloc_ms_mean']:.2f}/{m['alloc_ms_max']:.2f}, "
          f"ctrl ms mean/max = {m['ctrl_ms_mean']:.2f}/{m['ctrl_ms_max']:.2f}")
    print(f"  -> {paths['csv']}")
    return 0


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    raise SystemExit(main())
