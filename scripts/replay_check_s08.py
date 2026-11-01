#!/usr/bin/env python
"""Reproduce the original S00/S03 results and check them against the previous run.

    python scripts/replay_check_s08.py

The S08 A4 arm is a byte-for-byte replay of the S00 (+30 deg) and S03 (+45 deg)
scenarios of `docs/CMG多场景姿态机动对照实验指导书_无CMG对照版.md`: same vehicle,
atmosphere, disturbance, reference, window, solver settings and acceptance
policy.  This script compares the S08 numbers with the archived run of that
experiment and records the difference; a mismatch would mean the S08 base chain
is not the frozen one the audit claims it is.

Output  results/torque_gap_s08/replay_check.md  and  .json
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
NEW = ROOT / "results" / "torque_gap_s08" / "A4_descent_replay"
OLD = ROOT / "results" / "torque_gap_no_cmg_comparison_rti2" / "P0"
PAIRS = [("A4_descent_replay_S00", "S00_pitch30"),
         ("A4_descent_replay_S03", "S03_pitch45")]
ARMS = ("no_cmg", "with_cmg")
KEYS = ("rmse", "max_err", "fallback_ratio", "cpu_p95")


def load(path):
    return json.loads(path.read_text(encoding="utf-8"))


def pick(m):
    return dict(
        rmse=(m.get("rmse_deg") or {}).get("all"),
        rmse_man0=(m.get("rmse_deg") or {}).get("man0"),
        rmse_post=(m.get("rmse_deg") or {}).get("post"),
        max_err=((m.get("max_err_deg") or {}).get("all")),
        fallback_ratio=m.get("fallback_ratio"),
        cpu_p95=(m.get("cpu_s") or {}).get("p95"),
        n_steps=m.get("n_steps"),
    )


def main():
    rows = []
    for new_case, old_case in PAIRS:
        for arm in ARMS:
            pn = NEW / ("%s_%s" % (new_case, arm)) / "metrics.json"
            po = OLD / ("%s_%s" % (old_case, arm)) / "metrics.json"
            rec = dict(case=new_case, arm=arm, new_file=str(pn.relative_to(ROOT)),
                       old_file=str(po.relative_to(ROOT)))
            if not pn.exists() or not po.exists():
                rec["status"] = "missing"
                rows.append(rec)
                continue
            n, o = pick(load(pn)), pick(load(po))
            rec.update(new=n, old=o, status="compared")
            rec["delta_rmse"] = (None if None in (n["rmse"], o["rmse"])
                                 else float(n["rmse"] - o["rmse"]))
            rows.append(rec)
    (ROOT / "results" / "torque_gap_s08" / "replay_check.json").write_text(
        json.dumps(rows, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    L = ["# S08-A4: reproduction of the original S00/S03 results", "",
         "| 工况 | 臂 | 本轮全程 RMSE | 原实验全程 RMSE | 差 | 本轮机动窗 RMSE | "
         "原实验机动窗 RMSE | 末段 RMSE（本轮/原） | 最大误差（本轮/原） | 回退（本轮/原） |",
         "|---|---|---|---|---|---|---|---|---|---|"]
    worst = 0.0
    for r in rows:
        if r["status"] != "compared":
            L.append("| %s | %s | 缺失 | | | | | | | |" % (r["case"], r["arm"]))
            continue
        n, o = r["new"], r["old"]
        worst = max(worst, abs(r["delta_rmse"] or 0.0))
        L.append("| %s | %s | %.4f | %.4f | %+.2e | %.4f | %.4f | %.4f / %.4f | "
                 "%.3f / %.3f | %.3f / %.3f |" % (
                     r["case"], r["arm"], n["rmse"], o["rmse"], r["delta_rmse"],
                     n["rmse_man0"], o["rmse_man0"], n["rmse_post"], o["rmse_post"],
                     n["max_err"], o["max_err"], n["fallback_ratio"],
                     o["fallback_ratio"]))
    L.append("")
    L.append("最大全程 RMSE 偏差：%.2e deg" % worst)
    L.append("")
    L.append("回退比例、步数与求解耗时同样一致时，说明本轮 A4 复现的是同一条数值轨迹，"
             "而不是「看起来很像」的另一条。")
    (ROOT / "results" / "torque_gap_s08" / "replay_check.md").write_text(
        "\n".join(L) + "\n", encoding="utf-8")
    print("[replay] worst |delta rmse| = %.2e deg" % worst)
    for r in rows:
        print("   %-26s %-9s %s" % (r["case"], r["arm"], r["status"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
