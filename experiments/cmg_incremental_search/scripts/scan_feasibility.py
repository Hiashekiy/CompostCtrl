"""可行性扫描（指导实施文件 §4.2）：找出"A 明显困难、B 仍可行"的任务区域。

按 §4.2 的顺序：
 1. 先跑仅舵面基线 A；
 2. 再跑普通 CMG 分配基线 B；
 3. 从较小机动幅度逐步增大；
 4. 找出 A 有明显控制困难、而 B 仍能稳定完成或接近可行边界的任务；
 5. 不存在该区域就检查气动模型/任务高度/过渡时间（而不是改硬件参数）；
 6. B 本身也完不成的场景列为能力边界，不能拿来证明搜索优于普通 CMG。

用法：
    python experiments/cmg_incremental_search/scripts/scan_feasibility.py
    python experiments/cmg_incremental_search/scripts/scan_feasibility.py \
        --amplitudes 2 4 6 8 10 12 --durations 20

输出：results/metrics/feasibility_scan.csv 与 .json（逐配置一行，含全部判定量）
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np

EXP_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(EXP_ROOT))

from src.config import load_config                                       # noqa: E402
from src.simulation.logger import save_json                              # noqa: E402
from src.simulation.runner import run_case                               # noqa: E402

METRICS = EXP_ROOT / "results" / "metrics"
FIELDS = ["group", "delta_deg", "duration_s", "success", "att_final_deg", "att_peak_deg",
          "att_rmse_deg", "t_settle_s", "residual_max_nm", "infeasible_count",
          "fallback_count", "aero_sat_frac", "gamma_max_deg", "sigma_min_min",
          "H_norm_max", "gddot_peak_rad_s2", "ctrl_ms_max"]


def run_one(base: dict, group: str, delta_deg: float, duration_s: float) -> dict:
    cfg = {k: (dict(v) if isinstance(v, dict) else v) for k, v in base.items()}
    cfg["scenario"] = dict(base["scenario"])
    cfg["scenario"]["maneuver"] = {"axis": "pitch", "t0": 10.0, "duration": duration_s,
                                   "delta_deg": delta_deg}
    cfg["policy"] = group
    cfg["actuators"] = {"cmg_on": group != "A"}
    res = run_case(cfg, name=f"scan_{group}_{delta_deg:g}deg_{duration_s:g}s")
    m = res.metrics
    row = {"group": group, "delta_deg": delta_deg, "duration_s": duration_s,
           "success": bool(m["success"]), "att_final_deg": m["att_final_deg"],
           "att_peak_deg": m["att_peak_deg"], "att_rmse_deg": m["att_rmse_deg"],
           "t_settle_s": m["t_settle_s"], "residual_max_nm": m["residual_max_nm"],
           "infeasible_count": m["infeasible_count"], "fallback_count": m["fallback_count"],
           "aero_sat_frac": m["aero_sat_frac"], "gamma_max_deg": m["gamma_max_deg"],
           "sigma_min_min": m["sigma_min_min"], "H_norm_max": m["H_norm_max"],
           "gddot_peak_rad_s2": m["gddot_peak_rad_s2"], "ctrl_ms_max": m["ctrl_ms_max"]}
    return row


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--amplitudes", type=float, nargs="+",
                    default=[2.0, 4.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0])
    ap.add_argument("--durations", type=float, nargs="+", default=[20.0])
    ap.add_argument("--groups", nargs="+", default=["A", "B"])
    ap.add_argument("--tag", default="feasibility_scan")
    args = ap.parse_args()

    base = load_config("search.yaml", "scenario_high_altitude_maneuver.yaml")
    rows: list[dict] = []
    print(f"{'group':>5} {'delta':>6} {'T':>5} {'ok':>3} {'final':>8} {'peak':>8} "
          f"{'rmse':>8} {'resid_max':>9} {'gamma':>7} {'sigma':>7} {'Hmax':>7} {'fb':>4}")
    for duration in args.durations:
        for delta in args.amplitudes:
            for group in args.groups:
                row = run_one(base, group, delta, duration)
                rows.append(row)
                print(f"{row['group']:>5} {row['delta_deg']:>6.1f} {row['duration_s']:>5.1f} "
                      f"{str(row['success']):>5} {row['att_final_deg']:>8.3f} "
                      f"{row['att_peak_deg']:>8.3f} {row['att_rmse_deg']:>8.3f} "
                      f"{row['residual_max_nm']:>9.2f} {row['gamma_max_deg']:>7.1f} "
                      f"{row['sigma_min_min']:>7.2f} {row['H_norm_max']:>7.1f} "
                      f"{row['fallback_count']:>4}", flush=True)

    METRICS.mkdir(parents=True, exist_ok=True)
    csv_path = METRICS / f"{args.tag}.csv"
    with csv_path.open("w", encoding="utf-8", newline="") as fh:
        writer = csv.DictWriter(fh, fieldnames=FIELDS)
        writer.writeheader()
        writer.writerows(rows)
    save_json({"rows": rows, "config": {"amplitudes": args.amplitudes,
                                        "durations": args.durations,
                                        "groups": args.groups}},
              METRICS / f"{args.tag}.json")

    # 判定：A 困难 / B 可行 的区间
    ok_a = {r["delta_deg"]: r["success"] for r in rows if r["group"] == "A"}
    ok_b = {r["delta_deg"]: r["success"] for r in rows if r["group"] == "B"}
    both = sorted(set(ok_a) & set(ok_b))
    window = [d for d in both if (not ok_a[d]) and ok_b[d]]
    print(f"\nA 组失败的幅度: {[d for d in both if not ok_a[d]]}")
    print(f"B 组失败的幅度: {[d for d in both if not ok_b[d]]}")
    print(f"A 失败且 B 成功（搜索的目标区域）: {window}")
    print(f"-> {csv_path}")
    return 0 if window else 1


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    raise SystemExit(main())
