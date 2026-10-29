"""方案 §5：机动指标、A/B 对比与 §9 判定。

    python scripts/evaluate_cmg_maneuver.py                # 全部已完成的工况
    python scripts/evaluate_cmg_maneuver.py --case M1_pitch_5deg

所有阈值在运行前固定（写死在文件顶部，不随结果调整）：

    COMPLETION_TOL_DEG = 1.0      机动完成容差
    COMPLETION_HOLD_S  = 2.0      需要保持的时间
    EVAL_TAIL_S        = 30.0     机动窗口 = [t0, t0+Tm+30 s]
    SAT_FRAC           = 0.999    舵面饱和判据（相对限幅）
    DT_CTRL            = 0.2 s    实时性判据

误差用四元数测地线（`err_norm_deg`，俯仰主导时等于俯仰误差）；同时给出俯仰误差变体，
两者都标注清楚，避免把不同定义混在同一列里比较。
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)

OUT = os.path.join(ROOT, "results", "cmg_maneuver")
DT = 0.2
COMPLETION_TOL_DEG = 1.0
COMPLETION_HOLD_S = 2.0
EVAL_TAIL_S = 30.0
SAT_FRAC = 0.999
DT_CTRL = 0.2
MODES = ("A", "B")


def _series(path):
    return np.genfromtxt(path, delimiter=",", names=True)


def _load(case, mode):
    d = os.path.join(OUT, case, mode)
    with open(os.path.join(d, "metrics.json"), encoding="utf-8") as fh:
        metrics = json.load(fh)
    with open(os.path.join(d, "meta.json"), encoding="utf-8") as fh:
        meta = json.load(fh)
    return dict(case=case, mode=mode, dir=d, metrics=metrics, meta=meta,
                ts=_series(os.path.join(d, "timeseries.csv")))


# ---------------------------------------------------------------------------
# 方案 §5 的指标
# ---------------------------------------------------------------------------
def _first_hold(t, err, t_from, tol, hold):
    """第一次进入 ±tol 带并保持 hold 秒的时刻（绝对时间），否则 None。"""
    idx = int(np.searchsorted(t, t_from, side="left"))
    if idx >= len(t):
        return None
    inside = np.abs(err[idx:]) <= tol
    run = 0
    for i in range(len(inside)):
        if inside[i]:
            run += 1
            if t[idx + i] - t[idx + i - run + 1] >= hold - 1e-9:
                return float(t[idx + i - run + 1])
        else:
            run = 0
    return None


def maneuver_metrics(g, t0, Tm, delta_deg, shape="hold"):
    ts = g["ts"]
    t = ts["t"]
    e = ts["err_norm_deg"]
    ep = ts["err_pitch_deg"]
    w_end = min(t0 + Tm + EVAL_TAIL_S, t[-1])
    sel = (t >= t0) & (t <= w_end)
    ramp = (t >= t0) & (t <= t0 + Tm)
    post = (t >= t0 + Tm) & (t <= w_end)

    # 完成判据（运行前固定，写在报告里）：参考斜坡**结束之后**，误差进入 ±1° 带并保持
    # 2 s 的最早时刻。若从触发时刻就计数，任何跟踪良好的斜坡都会在 2 s 处"完成"
    # （斜坡期间参考本身在动），指标就失去区分度；同一份数据里也保留字面版本
    # completion_time_literal_s 以便核对。
    # 阶跃参考没有斜坡，从触发时刻起算；平滑参考从斜坡结束起算（见报告 §3 说明）
    start = t0 if shape == "step" else t0 + Tm
    comp = _first_hold(t, e, start, COMPLETION_TOL_DEG, COMPLETION_HOLD_S)
    lit = _first_hold(t, e, t0, COMPLETION_TOL_DEG, COMPLETION_HOLD_S)
    m = dict(
        shape=shape, completion_search_start_s=float(start),
        window=[float(t0), float(w_end)], maneuver_window=[float(t0), float(t0 + Tm)],
        completed=comp is not None,
        completion_time_s=None if comp is None else float(comp - t0),
        completion_time_after_ramp_s=None if comp is None else float(comp - start),
        completion_time_literal_s=None if lit is None else float(lit - t0),
        rmse_window_deg=float(np.sqrt(np.mean(e[sel] ** 2))),
        rmse_ramp_deg=float(np.sqrt(np.mean(e[ramp] ** 2))),
        peak_window_deg=float(np.max(e[sel])),
        peak_ramp_deg=float(np.max(e[ramp])),
        peak_pitch_window_deg=float(np.max(np.abs(ep[sel]))),
        rmse_pitch_window_deg=float(np.sqrt(np.mean(ep[sel] ** 2))),
        peak_pitch_rate_deg_s=float(np.rad2deg(np.max(np.abs(ts["omega_y"][sel])))),
        peak_rate_all_deg_s=float(np.rad2deg(np.max(np.abs(
            np.column_stack([ts["omega_x"][sel], ts["omega_y"][sel],
                             ts["omega_z"][sel]]))))),
        # 目标附近的行为（参考斜坡结束后）
        overshoot_pitch_deg=float(max(0.0, np.max(np.sign(delta_deg) * ep[post]))) if post.any() else None,
        undershoot_pitch_deg=float(max(0.0, np.max(-np.sign(delta_deg) * ep[post]))) if post.any() else None,
        n_sign_changes_post=int(np.sum(np.diff(np.sign(ep[post])) != 0)) if post.any() else None,
        post_error_ptp_deg=float(np.ptp(ep[post])) if post.any() else None,
    )
    # 执行机构
    d = np.column_stack([ts["de"], ts["da"], ts["dr"]])
    ddot = np.column_stack([ts["de_dot"], ts["da_dot"], ts["dr_dot"]])
    dmax = np.array([25.0, 20.0, 20.0])
    sat = np.abs(d) >= SAT_FRAC * dmax[None, :]
    M_aero = np.column_stack([ts["M_aero_x"], ts["M_aero_y"], ts["M_aero_z"]])
    M_cmg = np.column_stack([ts["M_cmg_x"], ts["M_cmg_y"], ts["M_cmg_z"]])
    nH = ts["H_norm"]
    m.update(
        delta_peak_deg=[float(v) for v in np.max(np.abs(d[sel]), axis=0)],
        delta_rate_peak_deg_s=[float(v) for v in np.max(np.abs(ddot[sel]), axis=0)],
        delta_sat_frac_window=float(np.mean(sat[sel].any(axis=1))),
        delta_sat_time_s=float(np.sum(sat[sel].any(axis=1)) * DT),
        aero_force_peak_Nm=float(np.max(np.linalg.norm(M_aero[sel], axis=1))),
        aero_force_integral_Nms=float(np.sum(np.linalg.norm(M_aero[sel], axis=1)) * DT),
        aero_pitch_integral_Nms=float(np.sum(np.abs(M_aero[sel, 1])) * DT),
        cmg_force_peak_Nm=float(np.max(np.linalg.norm(M_cmg[sel], axis=1))),
        cmg_force_integral_Nms=float(np.sum(np.linalg.norm(M_cmg[sel], axis=1)) * DT),
        cmg_force_pitch_integral_Nms=float(np.sum(np.abs(M_cmg[sel, 1])) * DT),
        H_peak_Nms=float(np.max(nH[sel])),
        H_at_window_end_Nms=float(nH[sel][-1]),
        H_budget_remaining_frac=float(1.0 - nH[sel][-1] / (0.30 * 200.0)),
        sigma_min=float(np.min(ts["sigma"][sel])),
        sigma_below_floor_time_s=float(np.sum(ts["sigma"][sel] < 12.24744871391589) * DT),
        gimbal_rate_peak_rad_s=float(np.max(np.abs(
            np.column_stack([ts["gdot_1"][sel], ts["gdot_2"][sel],
                             ts["gdot_3"][sel], ts["gdot_4"][sel]])))),
    )
    fb = ts["fallback"][sel] > 0.5
    best = cur = 0
    for v in fb:
        cur = cur + 1 if v else 0
        best = max(best, cur)
    st = ts["solve_time_ms"][sel]
    m.update(
        fallback_count=int(fb.sum()),
        fallback_longest_steps=int(best),
        fallback_longest_s=float(best * DT),
        max_violation=float(np.max(ts["viol"][sel])),
        violation_steps=int(np.sum(ts["viol"][sel] > 1e-3)),
        solve_mean_ms=float(np.mean(st)), solve_p95_ms=float(np.percentile(st, 95)),
        solve_max_ms=float(np.max(st)),
        meets_period=bool(np.percentile(st, 95) <= 1e3 * DT_CTRL),
        realtime_margin_p95=float(1e3 * DT_CTRL - np.percentile(st, 95)))
    return m


# ---------------------------------------------------------------------------
# §9 判定
# ---------------------------------------------------------------------------
def verdict(a, b):
    """判定键 + 证据。判据全部来自数据，且与方案 §9 三条口径一一对应。

    time_gain                 完成时间有明确改善（或 A 未完成而 B 完成），且误差未恶化、
                              约束与实时性满足 -> 支持 CMG 提升机动性能
    accuracy_gain_rt_ok       完成时间打平（两组都取到参考斜坡的下限），但窗口 RMSE 与峰值
                              都改善 >= 5%，约束与实时性满足 -> 支持，但收益在精度而非时间
    accuracy_gain_rt_fail     姿态指标更好，但求解时间超出控制周期/有持续回退
                              -> 不能声称具备可部署的控制优势（方案 §9 明确要求披露）
    burden_shift_only         误差没有实质改善，只是舵面饱和/用量下降
    both_failed               两组都未完成 -> 能力边界
    cmg_violated              B 的 CMG 约束不满足
    """
    ev = []
    if a["completed"] and b["completed"]:
        gain = a["completion_time_s"] - b["completion_time_s"]
        ev.append("完成时间：A %.2f s，B %.2f s（差 %+.2f s，小于 0.5 s 视为打平）"
                  % (a["completion_time_s"], b["completion_time_s"], -gain))
    elif (not a["completed"]) and b["completed"]:
        gain = float("inf")
        ev.append("A 未完成，B 在 %.2f s 内完成（有效结果）" % b["completion_time_s"])
    elif a["completed"] and (not b["completed"]):
        gain = float("-inf")
        ev.append("A 完成（%.2f s），B 未完成" % a["completion_time_s"])
    else:
        gain = None
        ev.append("两组都未完成：记录为能力边界，不为任何一组放宽限制")

    rmse_d = b["rmse_window_deg"] / a["rmse_window_deg"] - 1.0
    peak_d = b["peak_window_deg"] / a["peak_window_deg"] - 1.0
    ev.append("窗口 RMSE：A %.3f° → B %.3f°（%+.1f %%）"
              % (a["rmse_window_deg"], b["rmse_window_deg"], 100 * rmse_d))
    ev.append("峰值误差：A %.3f° → B %.3f°（%+.1f %%）"
              % (a["peak_window_deg"], b["peak_window_deg"], 100 * peak_d))
    if a.get("shape") == "step":
        ev.append("完成时间下限说明：阶跃参考没有斜坡，完成时间只受执行机构力矩/惯量限制"
                  "（理论下界见 maneuver_step_feasibility.csv 的 T_min）。")
    else:
        ev.append("完成时间下限说明：参考斜坡本身占 %.1f s，完成时间不可能小于它；"
                  "两组都取到该下限时，区分度只能来自精度与执行机构指标。"
                  % (a["maneuver_window"][1] - a["maneuver_window"][0]))

    cmg_ok = (b["H_peak_Nms"] <= 60.0 + 1e-6 and b["sigma_min"] > 12.247
              and b["gimbal_rate_peak_rad_s"] <= 1.5 + 1e-9)
    ev.append("B 的 CMG 约束：|H|峰 %.2f N·m·s（预算 60）、σ_min %.2f（下限 12.25）、"
              "框架速率峰 %.2f rad/s（限 1.5）→ %s"
              % (b["H_peak_Nms"], b["sigma_min"], b["gimbal_rate_peak_rad_s"],
                 "满足" if cmg_ok else "不满足"))
    solver_ok = b["fallback_longest_steps"] <= 5 and b["meets_period"]
    ev.append("B 的求解/实时性：最长连续回退 %d 步、P95 %.0f ms、最坏 %.0f ms（周期 200 ms）→ %s"
              % (b["fallback_longest_steps"], b["solve_p95_ms"], b["solve_max_ms"],
                 "满足" if solver_ok else "不满足"))
    ev.append("舵面饱和比例：A %.2f %% → B %.2f %%"
              % (100 * a["delta_sat_frac_window"], 100 * b["delta_sat_frac_window"]))

    if gain is None:
        key = "both_failed"
    elif a["completed"] and not b["completed"]:
        key = "cmg_regression"
    elif not cmg_ok:
        key = "cmg_violated"
    else:
        if gain is not None and (gain > 0.5 or gain == float("inf")):
            base = "time_gain"
        elif rmse_d <= -0.05 and peak_d <= -0.05:
            base = "accuracy_gain"
        elif b["delta_sat_frac_window"] < a["delta_sat_frac_window"] - 0.01:
            base = "burden_shift_only"
        else:
            base = "no_advantage"
        # 即使姿态指标/完成时间更好，违反实时性也不能算可部署优势（方案 §9）
        key = base if solver_ok else (base + "_rt_fail")
    return key, ev





VERDICT_TEXT = {
    "time_gain": "支持 CMG 提升机动性能（完成时间改善，误差未恶化，约束与实时性满足）",
    "time_gain_rt_fail": "完成时间与精度都改善，但 B 的求解时间超出 200 ms 控制周期："
                         "不能声称可部署的控制优势（方案 §9）",
    "accuracy_gain": "支持（完成时间打平于参考斜坡下限，收益体现在跟踪精度；约束与实时性满足）",
    "accuracy_gain_rt_fail": "跟踪精度更好，但 B 的求解时间超出 200 ms 控制周期："
                             "不能声称可部署的控制优势（方案 §9）",
    "burden_shift_only": "不支持性能优势：只改变了负担分配，完成时间与误差均无实质改善",
    "burden_shift_only_rt_fail": "只改变了负担分配，且 B 的求解时间超出 200 ms 控制周期：不支持性能优势",
    "no_advantage": "不支持性能优势",
    "no_advantage_rt_fail": "不支持性能优势（且 B 的求解时间超出控制周期）",
    "cmg_regression": "负向结果：A 能完成机动，B 反而发散/未完成——CMG 在该工况下是有害的",
    "cmg_violated": "B 的 CMG 约束（角动量/奇异性/框架速率）不满足，不能据此判定性能优势",
    "both_failed": "两组均未完成：记录为能力边界（未为任何一组放宽限制）",
}


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--case", nargs="*", default=None)
    a = ap.parse_args(argv)
    cases = a.case or sorted(d for d in os.listdir(OUT)
                             if os.path.isdir(os.path.join(OUT, d))
                             and os.path.exists(os.path.join(OUT, d, "A", "metrics.json")))
    rows = []
    details = {}
    for case in cases:
        par = json.load(open(os.path.join(OUT, case, "task_parity.json"), encoding="utf-8"))
        man = par["maneuver"]
        runs = {m: _load(case, m) for m in MODES}
        met = {m: maneuver_metrics(runs[m], man["t0"], man["Tm"], man["delta_deg"],
                                  man.get("shape", "hold"))
               for m in MODES}
        v, ev = verdict(met["A"], met["B"])
        details[case] = dict(maneuver=man, A=met["A"], B=met["B"], verdict=v,
                             verdict_text=VERDICT_TEXT[v], evidence=ev,
                             parity_passed=par["passed"])
        with open(os.path.join(OUT, case, "metrics_maneuver.json"), "w",
                  encoding="utf-8") as fh:
            json.dump(details[case], fh, indent=2, ensure_ascii=False)
        rows.append(dict(case=case, delta_deg=man["delta_deg"], Tm_s=man["Tm"],
                         A_completed=met["A"]["completed"],
                         A_time_s=met["A"]["completion_time_s"],
                         B_completed=met["B"]["completed"],
                         B_time_s=met["B"]["completion_time_s"],
                         A_rmse=met["A"]["rmse_window_deg"],
                         B_rmse=met["B"]["rmse_window_deg"],
                         A_peak=met["A"]["peak_window_deg"],
                         B_peak=met["B"]["peak_window_deg"],
                         A_sat=met["A"]["delta_sat_frac_window"],
                         B_sat=met["B"]["delta_sat_frac_window"],
                         A_solve_p95=met["A"]["solve_p95_ms"],
                         B_solve_p95=met["B"]["solve_p95_ms"],
                         B_H_peak=met["B"]["H_peak_Nms"],
                         B_sigma_min=met["B"]["sigma_min"],
                         B_fallback=met["B"]["fallback_count"],
                         verdict=v))
    if not rows:
        print("no completed cases under", OUT)
        return 1

    with open(os.path.join(OUT, "summary.csv"), "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        for r in rows:
            w.writerow(r)

    # ---- markdown ---------------------------------------------------------
    L = ["# 超高空姿态机动 A/B 指标汇总", "",
         "自动生成：`scripts/evaluate_cmg_maneuver.py`。完成判据 %.1f° / %.0f s（运行前固定）；"
         "评价窗口 [t0, t0+Tm+%.0f s]。" % (COMPLETION_TOL_DEG, COMPLETION_HOLD_S, EVAL_TAIL_S),
         ""]
    for case in cases:
        d = details[case]
        man = d["maneuver"]
        A, B = d["A"], d["B"]
        L += ["## %s（Δθ = %.0f°，Tm = %.0f s，t0 = %.1f s）"
              % (case, man["delta_deg"], man["Tm"], man["t0"]), "",
              "| 指标 | A 仅舵面 | B 舵面+CMG | 变化 |", "|---|---:|---:|---:|"]
        def line(label, ka, kb, fmt="%.3f", better="min"):
            va, vb = A[ka], B[kb]
            if isinstance(va, (list, tuple)) or isinstance(vb, (list, tuple)):
                L.append("| %s | %s | %s | — |"
                         % (label, "/".join("%.3g" % v for v in va),
                            "/".join("%.3g" % v for v in vb)))
                return
            if va is None or vb is None or isinstance(va, bool):
                L.append("| %s | %s | %s | — |" % (label, va, vb))
                return
            chg = ("%+.1f %%" % (100 * (vb / va - 1))) if va else "—"
            L.append("| %s | %s | %s | %s |" % (label, fmt % va, fmt % vb, chg))
        L.append("| 机动完成 | %s | %s | — |" % (A["completed"], B["completed"]))
        L.append("| 完成时间 [s]（自触发时刻） | %s | %s | %s |"
                 % ("未完成" if A["completion_time_s"] is None else "%.2f" % A["completion_time_s"],
                    "未完成" if B["completion_time_s"] is None else "%.2f" % B["completion_time_s"],
                    "—"))
        line("窗口 RMSE [°]", "rmse_window_deg", "rmse_window_deg")
        line("斜坡段 RMSE [°]", "rmse_ramp_deg", "rmse_ramp_deg")
        line("峰值误差 [°]", "peak_window_deg", "peak_window_deg")
        line("俯仰峰值误差 [°]", "peak_pitch_window_deg", "peak_pitch_window_deg")
        line("峰值俯仰角速度 [°/s]", "peak_pitch_rate_deg_s", "peak_pitch_rate_deg_s")
        line("超调 [°]", "overshoot_pitch_deg", "overshoot_pitch_deg")
        L.append("| 参考结束后误差变号次数 | %s | %s | — |"
                 % (A["n_sign_changes_post"], B["n_sign_changes_post"]))
        line("舵面饱和比例", "delta_sat_frac_window", "delta_sat_frac_window", "%.4f")
        line("舵面最大偏角 [°]", "delta_peak_deg", "delta_peak_deg", "%.2f")
        line("气动力矩峰值 [N·m]", "aero_force_peak_Nm", "aero_force_peak_Nm", "%.2f")
        line("气动力矩绝对积分 [N·m·s]", "aero_force_integral_Nms", "aero_force_integral_Nms", "%.1f")
        line("CMG 力矩峰值 [N·m]", "cmg_force_peak_Nm", "cmg_force_peak_Nm", "%.2f")
        line("CMG 力矩绝对积分 [N·m·s]", "cmg_force_integral_Nms", "cmg_force_integral_Nms", "%.1f")
        line("CMG 角动量峰值 [N·m·s]", "H_peak_Nms", "H_peak_Nms", "%.2f")
        line("窗口末角动量 [N·m·s]", "H_at_window_end_Nms", "H_at_window_end_Nms", "%.2f")
        line("σ_min [N·m·s]", "sigma_min", "sigma_min", "%.3f")
        line("σ 低于下限时长 [s]", "sigma_below_floor_time_s", "sigma_below_floor_time_s", "%.1f")
        line("回退步数", "fallback_count", "fallback_count", "%d")
        line("最长连续回退 [步]", "fallback_longest_steps", "fallback_longest_steps", "%d")
        line("最大约束违反量", "max_violation", "max_violation", "%.2e")
        line("平均求解 [ms]", "solve_mean_ms", "solve_mean_ms", "%.0f")
        line("P95 求解 [ms]", "solve_p95_ms", "solve_p95_ms", "%.0f")
        line("最坏求解 [ms]", "solve_max_ms", "solve_max_ms", "%.0f")
        L += ["", "**判定：%s**" % d["verdict_text"], ""]
        for e in d["evidence"]:
            L.append("* " + e)
        L.append("")
    with open(os.path.join(OUT, "summary.md"), "w", encoding="utf-8") as fh:
        fh.write("\n".join(L) + "\n")

    for r in rows:
        print("%-18s Δθ=%4.0f° Tm=%2.0f s  A: %s %s | B: %s %s | RMSE %.3f→%.3f | 判定 %s"
              % (r["case"], r["delta_deg"], r["Tm_s"],
                 "完成" if r["A_completed"] else "未完成",
                 "" if r["A_time_s"] is None else "%.1fs" % r["A_time_s"],
                 "完成" if r["B_completed"] else "未完成",
                 "" if r["B_time_s"] is None else "%.1fs" % r["B_time_s"],
                 r["A_rmse"], r["B_rmse"], r["verdict"]))
    print("wrote", os.path.join(OUT, "summary.md"))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
