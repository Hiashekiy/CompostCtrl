"""A/B evaluation: metrics table, improvement rates, pass/fail gates.

    python scripts/evaluate_cmg_ab.py [--cases tracking disturbance ...]

Reads only the raw per-run artefacts written by run_cmg_ab.py
(``<case>/<A|B>/timeseries.csv`` + ``metrics.json``) and produces

    results/cmg_ab/metrics_all.csv      every metric for every (case, mode)
    results/cmg_ab/summary.csv          A/B comparison with improvement rates
    results/cmg_ab/summary.md           the same, as a markdown table

Improvement follows §10.2:  (J_A - J_B)/J_A * 100 %, reported as an absolute
difference when J_A is (near) zero, and "n/a" when A did not complete the task
so no meaningful denominator exists.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

OUTROOT = os.path.join(ROOT, "results", "cmg_ab")

# metric | label | unit | direction ("low" = smaller is better)
ROWS = [
    ("err_rmse_deg", "三轴姿态 RMSE", "°", "low"),
    ("err_peak_deg", "姿态误差峰值", "°", "low"),
    ("err_rmse_roll_deg", "滚转 RMSE", "°", "low"),
    ("err_rmse_pitch_deg", "俯仰 RMSE", "°", "low"),
    ("err_rmse_yaw_deg", "偏航 RMSE", "°", "low"),
    ("peak_err_dist_deg", "扰动窗峰值", "°", "low"),
    ("peak_err_dist_tail_deg", "扰动+5s 尾段峰值", "°", "low"),
    ("recovery_time_dist_s", "扰动后恢复时间（绝对带）", "s", "low"),
    ("recovery_time_dist_rel_s", "扰动后恢复时间（10%峰值带）", "s", "low"),
    ("peak_err_man_deg", "机动窗峰值", "°", "low"),
    ("maneuver_time_s", "机动完成时间", "s", "low"),
    ("hold_rmse_deg", "机动后保持 RMSE", "°", "low"),
    ("max_overshoot_pitch_deg", "俯仰最大超调", "°", "low"),
    ("delta_peak_all_deg", "舵面峰值偏角", "°", "low"),
    ("delta_sat_frac_all", "舵面饱和时间比例", "–", "low"),
    ("delta_travel_deg", "舵面平均动作量", "°/步", "low"),
    ("cmg_torque_peak_Nm", "CMG 峰值输出力矩", "N·m", None),
    ("cmg_momentum_peak_Nms", "CMG 峰值角动量", "N·m·s", None),
    ("cmg_gimbal_rate_peak_rad_s", "CMG 框架速率峰值", "rad/s", None),
    ("cmg_sigma_min", "最小奇异性指标 σ_min", "N·m·s", None),
    ("qp_fail_count", "QP 失败次数", "次", "low"),
    ("qp_fail_longest_run", "最长连续 QP 失败", "步", "low"),
    ("qp_not_converged_count", "QP 未正常收敛步数（仍可用）", "步", "low"),
    ("qp_iter_mean", "平均 QP 迭代次数", "次", "low"),
    ("fallback_count", "保底控制器触发次数", "次", "low"),
    ("max_constraint_violation", "最大约束违反量", "–", "low"),
    ("solve_time_mean_ms", "平均求解时间", "ms", "low"),
    ("solve_time_p95_ms", "P95 求解时间", "ms", "low"),
]


def load_cases():
    out = {}
    if not os.path.isdir(OUTROOT):
        return out
    for case in sorted(os.listdir(OUTROOT)):
        d = os.path.join(OUTROOT, case)
        if not os.path.isdir(d) or case in ("_cache", "baseline_legacy"):
            continue
        modes = {}
        for mode in ("A", "B"):
            p = os.path.join(d, mode, "metrics.json")
            if os.path.exists(p):
                with open(p, encoding="utf-8") as fh:
                    modes[mode] = json.load(fh)
        if modes:
            out[case] = modes
    return out


def _fmt(v, unit):
    if v is None:
        return "—"
    if isinstance(v, bool):
        return "是" if v else "否"
    if isinstance(v, (int,)):
        return "%d" % v
    if isinstance(v, float):
        if unit == "–" or abs(v) < 1e-3:
            return "%.3g" % v
        return "%.4g" % v
    return str(v)


def improvement(ja, jb, direction):
    if ja is None or jb is None or direction is None:
        return None
    if not isinstance(ja, (int, float)) or not isinstance(jb, (int, float)):
        return None
    if abs(ja) < 1e-9:
        return ("abs", jb - ja)
    r = (ja - jb) / abs(ja) * 100.0
    return ("pct", r if direction == "low" else -r)


def _r_of(case):
    """Weight actually in force for a sweep case, read from its run metadata."""
    p = os.path.join(OUTROOT, case, "B", "config_resolved.yaml")
    try:
        import yaml
        with open(p, encoding="utf-8") as fh:
            cfg = yaml.safe_load(fh)
        return cfg["mpc_cmg"]["r_cmg_transient"]
    except Exception:
        return "?"


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--cases", nargs="*", default=None)
    a = ap.parse_args(argv)
    data = load_cases()
    cases = a.cases or list(data)
    cases = [c for c in cases if c in data]
    if not cases:
        print("no results found under", OUTROOT)
        return 1

    # ---- full metrics table ------------------------------------------------
    os.makedirs(OUTROOT, exist_ok=True)
    keys = set()
    for c in cases:
        for m in data[c].values():
            keys.update(m)
    keys = sorted(keys)
    with open(os.path.join(OUTROOT, "metrics_all.csv"), "w", newline="",
              encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["case", "mode"] + keys)
        for c in cases:
            for mode in ("A", "B"):
                m = data[c].get(mode)
                if m is None:
                    continue
                w.writerow([c, mode] + [m.get(k) for k in keys])

    # ---- comparison --------------------------------------------------------
    sum_rows = []
    for c in cases:
        A, B = data[c].get("A"), data[c].get("B")
        if A is None or B is None:
            continue
        for key, label, unit, direction in ROWS:
            ja, jb = A.get(key), B.get(key)
            imp = improvement(ja, jb, direction)
            if imp is None:
                imptxt = "—"
            elif imp[0] == "abs":
                imptxt = "Δ=%+.4g" % imp[1]
            else:
                imptxt = "%+.1f%%" % imp[1]
            sum_rows.append(dict(case=c, metric=key, label=label, unit=unit,
                                 A=ja, B=jb, improvement=imptxt,
                                 direction=direction or ""))
    with open(os.path.join(OUTROOT, "summary.csv"), "w", newline="",
              encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=["case", "metric", "label", "unit",
                                           "A", "B", "improvement", "direction"])
        w.writeheader()
        w.writerows(sum_rows)

    # ---- markdown ----------------------------------------------------------
    lines = ["# A/B 指标汇总（由 metrics.json 自动生成，勿手工编辑）", ""]
    for c in cases:
        A, B = data[c].get("A"), data[c].get("B")
        if A is None or B is None:
            lines.append("## %s\n\n（缺少 A 或 B，未比较）\n" % c)
            continue
        lines.append("## %s\n" % c)
        lines.append("| 指标 | A：仅舵面 | B：舵面 + CMG | 变化 |")
        lines.append("|---|---|---|---|")
        for key, label, unit, direction in ROWS:
            ja, jb = A.get(key), B.get(key)
            imp = improvement(ja, jb, direction)
            if imp is None:
                imptxt = "—"
            elif imp[0] == "abs":
                imptxt = "Δ=%+.4g" % imp[1]
            else:
                imptxt = "%+.1f%%" % imp[1]
            lines.append("| %s [%s] | %s | %s | %s |"
                         % (label, unit, _fmt(ja, unit), _fmt(jb, unit), imptxt))
        lines.append("")
        # ---- §10.4 verdict ----
        lines.append("**判据核查（§10.4）**\n")
        lines.extend(_verdict(A, B))
        lines.append("")
    # ---- §7.2 weight sweep -------------------------------------------------
    sweeps = [c for c in cases if c.startswith("wscan_")]
    if sweeps:
        lines.append("## §7.2 CMG 代价权重扫描（B 方案，100 km 顶点脉冲工况）\n")
        lines.append("| 工况 | r_cmg_transient | RMSE [°] | 峰值 [°] | \\|H\\|max "
                     "[N·m·s] | \\|M_cmg\\|max [N·m] | σ_min | QP 失败 | "
                     "未收敛步 | 平均求解 [ms] |")
        lines.append("|---|---|---|---|---|---|---|---|---|---|")
        for c in sorted(sweeps):
            m = data[c].get("B")
            if m is None:
                continue
            lines.append("| %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |"
                         % (c, _r_of(c), _fmt(m["err_rmse_deg"], "°"),
                            _fmt(m["err_peak_deg"], "°"),
                            _fmt(m["cmg_momentum_peak_Nms"], ""),
                            _fmt(m["cmg_torque_peak_Nm"], ""),
                            _fmt(m["cmg_sigma_min"], ""),
                            _fmt(m["qp_fail_count"], "次"),
                            _fmt(m.get("qp_not_converged_count"), "步"),
                            _fmt(m["solve_time_mean_ms"], "ms")))
        lines.append("")

    with open(os.path.join(OUTROOT, "summary.md"), "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")

    print("\n".join(lines))
    print("\nwrote results/cmg_ab/{metrics_all.csv,summary.csv,summary.md}")
    return 0


def _verdict(A, B):
    def chk(cond, text):
        return "- [%s] %s" % ("x" if cond else " ", text)
    out = []
    out.append(chk(B["err_rmse_deg"] < A["err_rmse_deg"] * 0.99,
                   "B 的三轴 RMSE 有明显改善（>1%）"))
    out.append(chk(B["err_peak_deg"] < A["err_peak_deg"] * 0.99,
                   "B 的姿态误差峰值有明显改善（>1%）"))
    out.append(chk(B["delta_sat_frac_all"] <= A["delta_sat_frac_all"] + 1e-9,
                   "B 没有让舵面饱和时间比例变差"))
    out.append(chk(B["qp_fail_count"] <= max(A["qp_fail_count"], 1) * 3 + 5,
                   "B 的 QP 失败次数没有严重恶化"))
    out.append(chk(B["max_constraint_violation"] <= 1e-3,
                   "B 的最大约束违反量在容差内"))
    mk = B.get("maneuver_time_s")
    ak = A.get("maneuver_time_s")
    if mk is not None or ak is not None:
        out.append(chk(mk is not None and (ak is None or mk <= ak),
                       "机动任务：B 完成且不慢于 A（A=%s s, B=%s s）"
                       % ("未完成" if ak is None else "%.2f" % ak,
                          "未完成" if mk is None else "%.2f" % mk)))
    out.append(chk(B["cmg_momentum_peak_Nms"] <= 200.0,
                   "CMG 角动量峰值未超硬件集群容量"))
    return out


if __name__ == "__main__":
    raise SystemExit(main())
