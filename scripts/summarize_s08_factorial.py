#!/usr/bin/env python
"""Tabulate the S08 results and compute the A3 interaction terms (doc §7, §12).

    python scripts/summarize_s08_factorial.py

Input   results/torque_gap_s08/summary.csv     (scripts/analyze_s08_aero.py)
Output  results/torque_gap_s08/summary.md
        results/torque_gap_s08/interaction.json

The interaction term of doc §7 is computed for every metric J and for both arms:

    I_alpha_q = J(alpha_high, q_high) - J(alpha_high, q_low)
                - J(alpha_low, q_high) + J(alpha_low, q_low)

For error-type metrics I > 0 means the combined condition degrades more than the
sum of the single-factor effects.
"""
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RES = ROOT / "results" / "torque_gap_s08"
ARMS = (("no_cmg", "A 仅舵面"), ("with_cmg", "B 舵面+CMG"))


def load():
    with open(RES / "summary.csv", encoding="utf-8", newline="") as fh:
        return list(csv.DictReader(fh))


def num(row, key, default=None):
    v = (row or {}).get(key, "")
    try:
        return float(v)
    except (TypeError, ValueError):
        return default


def fmt(v, spec="%.3f"):
    return "—" if v is None else spec % v


def index(rows, group, plant=None):
    """(case, arm, plant) -> record, base seed only.

    Seed-robustness runs share the case/arm name and are reported in their own
    table; mixing them into the base tables would silently replace the nominal
    run by a randomised one.
    """
    out = {}
    for r in rows:
        if r["group"] != group:
            continue
        if plant is not None and r["plant"] != plant:
            continue
        if int(num(r, "seed", 0) or 0) != 0:
            continue
        out[(r["case"], r["arm"], r["plant"])] = r
    return out


def merged_arms(rows, group, plant):
    """One row per case with the A and B records."""
    idx = index(rows, group, plant)
    cases = sorted({k[0] for k in idx})
    return [(c, idx.get((c, "no_cmg", plant)), idx.get((c, "with_cmg", plant)))
            for c in cases]


# --------------------------------------------------------------------------
def sweep_table(L, rows, group, plant, title, first_col="工况"):
    L.append("## %s（%s 植物）" % (title, plant))
    L.append("")
    L.append("| %s | 迎角设定 | 迎角窗口均值 | 动压设定 | 臂 | 全程 RMSE | 主区间 RMSE | "
             "末段 RMSE | 最大误差 | t(>1°) | t(>5°) | t(>10°) | 舵面利用率 | 舵面贴边 | "
             "|H|峰值 | σmin最小 | 框架速率饱和 | 回退 |" % first_col)
    L.append("|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for case, a, b in merged_arms(rows, group, plant):
        for arm, lab in ARMS:
            r = a if arm == "no_cmg" else b
            if r is None:
                continue
            at = num(r, "alpha_target_deg")
            qt = num(r, "qbar_target_pa")
            L.append("| %s | %s | %s° | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |" % (
                case,
                "—" if at is None else "%+.1f°" % at,
                fmt(num(r, "all_alpha_mean"), "%.2f"),
                "—" if qt is None else "%.2f Pa" % qt, lab,
                fmt(num(r, "all_rmse")), fmt(num(r, "main_rmse")),
                fmt(num(r, "term_rmse")), fmt(num(r, "all_max")),
                fmt(num(r, "all_t_exceed_1"), "%.1f"),
                fmt(num(r, "all_t_exceed_5"), "%.1f"),
                fmt(num(r, "all_t_exceed_10"), "%.1f"),
                fmt(num(r, "all_surf_util_mean")), fmt(num(r, "all_surf_sat")),
                fmt(num(r, "all_H_norm_max"), "%.1f"),
                fmt(num(r, "all_sigma_min_min"), "%.1f"),
                fmt(num(r, "all_nu_sat")), fmt(num(r, "fallback_ratio"))))
    L.append("")


def interaction(rows, plant):
    idx = index(rows, "A3_coupled", plant)
    keys = {"ll": ("A3_coupled_ll", None), "hl": ("A3_coupled_hl", None),
            "lh": ("A3_coupled_lh", None), "hh": ("A3_coupled_hh", None)}
    metrics = ["all_rmse", "main_rmse", "term_rmse", "all_max", "all_surf_sat",
               "all_fallback", "all_surf_util_mean"]
    out = {}
    for arm, _lab in ARMS:
        val = {}
        for m in metrics:
            v = {k: num(idx.get((case, arm, plant)), m) for k, (case, _) in keys.items()}
            if any(x is None for x in v.values()):
                val[m] = None
                continue
            val[m] = float(v["hh"] - v["hl"] - v["lh"] + v["ll"])
            val["%s__terms" % m] = v
        out[arm] = val
    return out


def main():
    rows = load()
    plants = sorted({r["plant"] for r in rows if r["plant"]})
    L = []
    L.append("# S08 结果汇总（自动生成）")
    L.append("")
    L.append("由 `scripts/summarize_s08_factorial.py` 从 `results/torque_gap_s08/summary.csv` "
             "生成；每个数字都来自 `timeseries.csv` 的直接统计。")
    L.append("")
    L.append("- 统计区间：全程 = 90–150 s；主区间 = 指令机动窗口 110–130 s（A1–A3 的 +8° 凸起、"
             "A4 的 +30°/+45° 机动）；末段 = 最后 20 s（130–150 s）。")
    L.append("- A1–A3 的每个受控工况都在 110–130 s 叠加同一条 +8° 俯仰凸起参考"
             "（`shape: bump`，两端归零），因此「迎角窗口均值」高于「迎角设定」，"
             "差值是凸起带来的瞬时加载，各工况一致。")
    L.append("- 「舵面利用率」= $|\\tau_{a,cmd,y}-\\tau_{nat,y}| / \\tau^{half}_y$，"
             "1.0 表示需求恰好填满可用气动控制力矩；「舵面贴边」= 位于能力盒边界的轴比例。")
    L.append("- A 组没有 CMG，因此 `|H|`/`σmin`/框架饱和列一律为「—」。")
    L.append("")

    if "P0" in plants:
        sweep_table(L, rows, "A1_alpha", "P0", "S08-A1 迎角扫描（固定动压）")
        sweep_table(L, rows, "A2_qbar", "P0", "S08-A2 动压扫描（固定迎角）")
        sweep_table(L, rows, "A4_descent_replay", "P0", "S08-A4 原始下降段复现")

    inter = {}
    for plant in plants:
        it = interaction(rows, plant)
        inter[plant] = it
        if not any(v for a in it.values() for k, v in a.items()
                   if not k.endswith("__terms") and v is not None):
            continue                      # A3 not run on this plant
        L.append("## S08-A3 迎角–动压耦合（%s 植物）：2×2 与交互项" % plant)
        L.append("")
        L.append("| 臂 | J | J(hh) | J(hl) | J(lh) | J(ll) | 交互项 $I_{\\alpha q}$ | 判读 |")
        L.append("|---|---|---|---|---|---|---|---|")
        for arm, lab in ARMS:
            for m, v in it[arm].items():
                if m.endswith("__terms") or v is None:
                    continue
                t = it[arm]["%s__terms" % m]
                if m in ("all_rmse", "main_rmse", "term_rmse", "all_max"):
                    verdict = "组合退化 > 单因素叠加" if v > 0 else "组合退化 ≤ 单因素叠加"
                else:
                    verdict = "正交互" if v > 0 else "负交互"
                L.append("| %s | %s | %s | %s | %s | %s | %+.4f | %s |" % (
                    lab, m, fmt(t["hh"], "%.4f"), fmt(t["hl"], "%.4f"),
                    fmt(t["lh"], "%.4f"), fmt(t["ll"], "%.4f"), v, verdict))
        L.append("")

    # P1 (real servo / bounded deflection) rows
    if "P1" in plants:
        sweep_table(L, rows, "A1_alpha", "P1", "S08-A1 迎角扫描（P1 真实舵面植物）")

    # A4 event timeline
    a4 = index(rows, "A4_descent_replay", "P0")
    if a4:
        L.append("## S08-A4 事件时刻（P0 植物）")
        L.append("")
        L.append("| 工况 | 臂 | 动压增长倍数 | 迎角峰值 | 误差首超 1°/5°/10° | "
                 "误差持续增长起点 | 舵面持续饱和 | 舵面需求持续>95% | "
                 "CMG 持续速率饱和 | |H|≥50% 边界 | σmin 触底 | 回退比例 |")
        L.append("|---|---|---|---|---|---|---|---|---|---|---|---|")
        for case in sorted({k[0] for k in a4}):
            for arm, lab in ARMS:
                r = a4.get((case, arm, "P0"))
                if r is None:
                    continue
                L.append("| %s | %s | %s× | %s° | %s / %s / %s | %s | %s | %s | %s | %s | %s | %s |" % (
                    case, lab,
                    fmt(num(r, "qbar_over_window"), "%.2f"),
                    fmt(num(r, "all_alpha_max"), "%.1f"),
                    fmt(num(r, "all_t_exceed_1"), "%.1f"),
                    fmt(num(r, "all_t_exceed_5"), "%.1f"),
                    fmt(num(r, "all_t_exceed_10"), "%.1f"),
                    fmt(num(r, "t_err_sus_1deg"), "%.1f"),
                    fmt(num(r, "t_surf_sat_sus"), "%.1f"),
                    fmt(num(r, "t_surf_util_sus"), "%.1f"),
                    fmt(num(r, "t_nu_sat_sus"), "%.1f"),
                    fmt(num(r, "t_H_half"), "%.1f"),
                    fmt(num(r, "t_sigma_floor"), "%.1f"),
                    fmt(num(r, "fallback_ratio"))))
        L.append("")

    # seed robustness
    seeds = [r for r in rows if int(num(r, "seed", 0) or 0) > 0]
    if seeds:
        L.append("## 种子稳健性（A4，扰动相位随机化，A/B 同种子）")
        L.append("")
        L.append("| 工况 | 臂 | 种子 | 全程 RMSE | 末段 RMSE | 最大误差 | 回退比例 |")
        L.append("|---|---|---|---|---|---|---|")
        for r in sorted(seeds, key=lambda r: (r["case"], r["arm"], num(r, "seed", 0))):
            L.append("| %s | %s | %d | %s | %s | %s | %s |" % (
                r["case"], r["arm"], int(num(r, "seed", 0)),
                fmt(num(r, "all_rmse")), fmt(num(r, "term_rmse")),
                fmt(num(r, "all_max")), fmt(num(r, "fallback_ratio"))))
        L.append("")

    (RES / "interaction.json").write_text(
        json.dumps(inter, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    (RES / "summary.md").write_text("\n".join(L) + "\n", encoding="utf-8")
    print("[summarize] rows=%d plants=%s -> %s" % (len(rows), plants,
                                                   (RES / "summary.md").relative_to(ROOT)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
