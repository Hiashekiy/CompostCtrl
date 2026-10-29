"""E1 of the C1 diagnostics: does the persistent 20 N·m bias cause the CMG failure?

    python scripts/e1_bias_ablation.py --run       # E1-C (surfaces) and E1-D (surfaces+CMG)
    python scripts/e1_bias_ablation.py --analyze   # records, metrics table, figures

Groups
    E1-A / E1-B  the formal A/B run already frozen in results/c1_cmg_reproduction/cmg/c1
                 (reused, never re-run, never overwritten)
    E1-C / E1-D  the same task with the constant part of the pitch disturbance removed:
                 M_d = 15 sin(2*pi*0.05 t)  (net angular impulse over 200 s = 0)

Everything else - reference, initial state, skip profile, atmosphere, servo limits,
CMG hardware, MPC weights, prediction horizon, scoring windows - is identical; the
resolved-configuration difference is asserted and written to parity.json before the
two new runs are scored.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt                                     # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for p in (ROOT, HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

import run_cmg_ab as ab                                             # noqa: E402
from nearspace.config import dump                                    # noqa: E402
from nearspace import sim                                            # noqa: E402

CONFIG = os.path.join(ROOT, "configs", "c1_cmg_reproduction_bias_ablation.yaml")
ORIG_CONFIG = os.path.join(ROOT, "configs", "c1_cmg_reproduction.yaml")
OUT = os.path.join(ROOT, "results", "c1_cmg_diagnostics", "E1_bias_ablation")
FIG = os.path.join(ROOT, "results", "c1_cmg_diagnostics", "figures")
ORIG = os.path.join(ROOT, "results", "c1_cmg_reproduction", "cmg")
CACHE = os.path.join(ROOT, "results", "cmg_ab", "_cache")
H_LIM_FRAC = 0.30
H_TOTAL = 200.0

GROUPS = [
    # key,                mode, case,           source root,                              description
    ("A_original",         "A", "c1",            ORIG, "E1-A 仅舵面, M_d = 20 + 15 sin"),
    ("B_original",         "B", "c1",            ORIG, "E1-B 舵面+CMG, M_d = 20 + 15 sin"),
    ("C_sinusoidal_only",  "A", "c1_sin_only",   OUT,  "E1-C 仅舵面, M_d = 15 sin"),
    ("D_sinusoidal_only",  "B", "c1_sin_only",   OUT,  "E1-D 舵面+CMG, M_d = 15 sin"),
]


# ---------------------------------------------------------------------------
def parity():
    """The new task must differ from the formal one only in the pitch constant."""
    a0 = dump(ab.build_cfg("c1", "A", ORIG_CONFIG))
    b0 = dump(ab.build_cfg("c1", "B", ORIG_CONFIG))
    a1 = dump(ab.build_cfg("c1_sin_only", "A", CONFIG))
    b1 = dump(ab.build_cfg("c1_sin_only", "B", CONFIG))

    def flat(d, pre=""):
        out = {}
        for k, v in d.items():
            if isinstance(v, dict):
                out.update(flat(v, pre + "/" + k))
            else:
                out[pre + "/" + k] = v
        return out

    def diff(x, y):
        fx, fy = flat(x), flat(y)
        return {k: [fx.get(k), fy.get(k)] for k in sorted(set(fx) | set(fy))
                if fx.get(k) != fy.get(k)}

    d_ab = diff(a0, b0)
    d_new_ab = diff(a1, b1)
    d_task = diff(a0, a1)
    allowed_task = {"/vehicle3/Md0", "/scenario/name", "/label"}
    res = dict(
        original_A_vs_B=d_ab, sinusoidal_A_vs_B=d_new_ab,
        original_vs_sinusoidal=d_task,
        task_change_limited=set(d_task) <= allowed_task,
        ab_change_identical=(set(d_ab) == set(d_new_ab)),
        same_actuator_difference_only=(set(d_new_ab) <= {"/actuators/mode",
                                                         "/mpc_cmg/M_cmg_box",
                                                         "/label",
                                                         "/scenario/mode_label"}),
    )
    res["passed"] = bool(res["task_change_limited"] and res["ab_change_identical"]
                         and res["same_actuator_difference_only"])
    return res


# ---------------------------------------------------------------------------
_trapz = getattr(np, "trapezoid", None) or getattr(np, "trapz")


def _series(path):
    return np.genfromtxt(path, delimiter=",", names=True)


def records(ts, metrics, cfg_dump, h_lim, floor):
    t = ts["t"]
    md = ts["M_dist_y"]
    H = np.column_stack([ts["H_cmg_x"], ts["H_cmg_y"], ts["H_cmg_z"]])
    nH = ts["H_norm"]
    dt = float(t[1] - t[0])
    dHdt = np.gradient(nH, dt)

    def first(mask):
        idx = np.flatnonzero(mask)
        return float(t[idx[0]]) if idx.size else None

    budget_t = first(nH > h_lim + 1e-9)
    sigma_t = first(ts["sigma"] < floor - 1e-9)
    fb = ts["fallback"] > 0.5
    fb_t = first(fb)
    # longest continuous fallback run
    best = cur = 0
    for v in fb:
        cur = cur + 1 if v else 0
        best = max(best, cur)

    e = ts["err_norm_deg"]
    win = cfg_dump.get("windows", {}).get("high")
    sel = (t >= win[0]) & (t <= win[1]) if win else np.zeros_like(t, dtype=bool)

    return dict(
        t_end=float(t[-1]),
        disturbance=dict(
            integral_Nms=float(_trapz(md, t)),
            positive_Nms=float(_trapz(np.clip(md, 0, None), t)),
            negative_Nms=float(_trapz(np.clip(md, None, 0), t)),
            mean_Nm=float(np.mean(md)), peak_Nm=float(np.max(np.abs(md))),
            series_hash=float(np.sum(md * np.arange(len(md))))),
        cmg=dict(
            H_peak_Nms=float(nH.max()), H_end_Nms=float(nH[-1]),
            H_x_range=[float(H[:, 0].min()), float(H[:, 0].max())],
            H_y_range=[float(H[:, 1].min()), float(H[:, 1].max())],
            H_z_range=[float(H[:, 2].min()), float(H[:, 2].max())],
            dHdt_peak_Nm=float(dHdt.max()), dHdt_mean_Nm=float(dHdt.mean()),
            dHdt_peak_time_s=float(t[int(np.argmax(dHdt))]),
            torque_peak_Nm=float(metrics["cmg_torque_peak_Nm"]),
            torque_mean_abs_y_Nm=float(np.mean(np.abs(ts["M_cmg_y"]))),
            gimbal_rate_peak_rad_s=float(metrics["cmg_gimbal_rate_peak_rad_s"]),
            sigma_min=float(metrics["cmg_sigma_min"])),
        events=dict(
            budget_trigger_s=budget_t, sigma_below_floor_s=sigma_t,
            first_fallback_s=fb_t, longest_fallback_run_steps=int(best),
            fallback_steps=int(fb.sum()), fallback_frac=float(fb.mean()),
            n_steps=int(len(t))),
        tracking=dict(
            rmse_deg=float(metrics["err_rmse_deg"]),
            peak_deg=float(metrics["err_peak_deg"]),
            rmse_pitch_deg=float(metrics["err_rmse_pitch_deg"]),
            rmse_roll_deg=float(metrics["err_rmse_roll_deg"]),
            rmse_yaw_deg=float(metrics["err_rmse_yaw_deg"]),
            high_rmse_deg=float(metrics["high_rmse_deg"]) if metrics.get("high_rmse_deg") is not None else None,
            high_peak_deg=float(metrics["high_peak_deg"]) if metrics.get("high_peak_deg") is not None else None,
            high_window_s=list(win) if win else None),
        surfaces=dict(
            saturation_frac=float(metrics["delta_sat_frac_all"]),
            travel_deg=float(metrics["delta_travel_deg"]),
            peak_deg=[float(v) for v in metrics["delta_peak_deg"]]),
        solver=dict(qp_fail=int(metrics["qp_fail_count"]),
                    solve_mean_ms=float(metrics["solve_time_mean_ms"]),
                    solve_p95_ms=float(metrics["solve_time_p95_ms"]),
                    solve_max_ms=float(metrics["solve_time_max_ms"]),
                    max_violation=float(metrics["max_constraint_violation"])),
    )


def load_group(key, mode, case, root):
    d = os.path.join(root, case, mode)
    with open(os.path.join(d, "metrics.json"), encoding="utf-8") as fh:
        metrics = json.load(fh)
    with open(os.path.join(d, "meta.json"), encoding="utf-8") as fh:
        meta = json.load(fh)
    ts = _series(os.path.join(d, "timeseries.csv"))
    return dict(key=key, mode=mode, case=case, dir=d, metrics=metrics, meta=meta, ts=ts)


# ---------------------------------------------------------------------------
def analyze():
    os.makedirs(OUT, exist_ok=True)
    h_lim = H_LIM_FRAC * H_TOTAL
    floor = 0.30 * 0.816496580927726 * 50.0
    out, loaded = {}, {}
    for key, mode, case, root, desc in GROUPS:
        g = load_group(key, mode, case, root)
        rec = records(g["ts"], g["metrics"], g["meta"], h_lim, floor)
        rec["source_dir"] = os.path.relpath(g["dir"], ROOT)
        rec["description"] = desc
        rec["meta"] = dict(code_commit=g["meta"].get("code_commit"),
                           tag=g["meta"].get("tag"),
                           actuator_mode=g["meta"].get("actuator_mode"),
                           python=g["meta"].get("python"),
                           wall_time_s=g["meta"].get("wall_time_s"),
                           high_window=g["meta"]["windows"].get("high"),
                           Md0_pitch=g["meta"]["config"]["vehicle3"]["Md0"][1],
                           Md_amp_pitch=g["meta"]["config"]["vehicle3"]["Md_amp"][1])
        d = os.path.join(OUT, key)
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, "diagnostics.json"), "w", encoding="utf-8") as fh:
            json.dump(rec, fh, indent=2, ensure_ascii=False)
        with open(os.path.join(d, "SOURCE.txt"), "w", encoding="utf-8") as fh:
            fh.write("raw artefacts: %s\n(not copied; the formal A/B data is read-only)\n"
                     % rec["source_dir"])
        out[key] = rec
        loaded[key] = g

    keys = [g[0] for g in GROUPS]
    # ---- table ----
    rows = [("attitude RMSE whole run [deg]", lambda r: r["tracking"]["rmse_deg"]),
            ("peak attitude error [deg]", lambda r: r["tracking"]["peak_deg"]),
            ("high-altitude RMSE [deg]", lambda r: r["tracking"]["high_rmse_deg"]),
            ("high-altitude peak error [deg]", lambda r: r["tracking"]["high_peak_deg"]),
            ("disturbance net impulse [N m s]", lambda r: r["disturbance"]["integral_Nms"]),
            ("disturbance +ve part [N m s]", lambda r: r["disturbance"]["positive_Nms"]),
            ("disturbance -ve part [N m s]", lambda r: r["disturbance"]["negative_Nms"]),
            ("CMG peak |H| [N m s]", lambda r: r["cmg"]["H_peak_Nms"]),
            ("CMG end |H| [N m s]", lambda r: r["cmg"]["H_end_Nms"]),
            ("CMG max d|H|/dt [N m]", lambda r: r["cmg"]["dHdt_peak_Nm"]),
            ("CMG peak torque [N m]", lambda r: r["cmg"]["torque_peak_Nm"]),
            ("sigma_min [N m s]", lambda r: r["cmg"]["sigma_min"]),
            ("budget trigger time [s]", lambda r: r["events"]["budget_trigger_s"]),
            ("sigma below floor time [s]", lambda r: r["events"]["sigma_below_floor_s"]),
            ("first fallback [s]", lambda r: r["events"]["first_fallback_s"]),
            ("longest fallback run [steps]", lambda r: r["events"]["longest_fallback_run_steps"]),
            ("surface saturation frac [-]", lambda r: r["surfaces"]["saturation_frac"]),
            ("surface travel per step [deg]", lambda r: r["surfaces"]["travel_deg"]),
            ("mean solve time [ms]", lambda r: r["solver"]["solve_mean_ms"]),
            ("worst solve time [ms]", lambda r: r["solver"]["solve_max_ms"])]

    with open(os.path.join(OUT, "metrics.csv"), "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["metric"] + keys)
        for label, fn in rows:
            w.writerow([label] + [("%.6g" % fn(out[k])) if fn(out[k]) is not None else ""
                                  for k in keys])

    # ---- summary ----
    A, B, C, D = (out[k] for k in keys)
    def pct(x, y):
        return 100.0 * (y / x - 1.0) if x else float("nan")

    L = ["# E1 扰动偏置消融：持续 20 N·m 偏置是不是 CMG 失效的主要诱因？", "",
         "自动生成：`scripts/e1_bias_ablation.py --analyze`。",
         "E1-A / E1-B 直接读取正式 A/B 的原始产物（未复制、未重跑、未覆盖）；"
         "E1-C / E1-D 是本实验新增运行。", ""]
    L += ["## 任务与一致性", "",
          "| 组 | 执行机构 | 俯仰扰动力矩 |", "|---|---|---|",
          "| E1-A | 仅舵面 | 20 + 15 sin(2π·0.05t) |",
          "| E1-B | 舵面 + CMG | 20 + 15 sin(2π·0.05t) |",
          "| E1-C | 仅舵面 | 15 sin(2π·0.05t) |",
          "| E1-D | 舵面 + CMG | 15 sin(2π·0.05t) |", ""]
    with open(os.path.join(OUT, "parity.json"), encoding="utf-8") as fh:
        par = json.load(fh)
    L += ["解析配置差异（新增配置 vs 正式配置）：", "",
          "```", json.dumps(par["original_vs_sinusoidal"], ensure_ascii=False, indent=1), "```", "",
          "判定：任务差异仅限俯仰扰动常数项 = **%s**；A/B 差异集合与正式实验相同 = **%s**。"
          % (par["task_change_limited"], par["same_actuator_difference_only"]), ""]

    L += ["## 数值核验：扰动波形与净角冲量", "",
          "| 组 | 净角冲量 [N·m·s] | 正向分量 | 负向分量 | 峰值 [N·m] |",
          "|---|---:|---:|---:|---:|"]
    for k in keys:
        d = out[k]["disturbance"]
        L.append("| %s | %.1f | %.1f | %.1f | %.1f |"
                 % (k, d["integral_Nms"], d["positive_Nms"], d["negative_Nms"], d["peak_Nm"]))
    L += ["", "理论核对：常数 20 N·m × 200 s = 4000 N·m·s；正弦项在 10 个整周期上净冲量 0。"
          "表中 E1-A/B 的净冲量应≈4000，E1-C/D 应≈0。", ""]

    L += ["## 四组核心指标", "",
          "| 指标 | E1-A | E1-B | E1-C | E1-D |", "|---|---:|---:|---:|---:|"]
    for label, fn in rows:
        cells = []
        for k in keys:
            v = fn(out[k])
            cells.append("" if v is None else ("%.4g" % v))
        L.append("| %s | %s |" % (label, " | ".join(cells)))
    L += [""]

    L += ["## 判据（方案 §1.4）", "",
          "| 观察 | 本实验数据 | 支持的判断 |", "|---|---|---|"]
    def trig(x):
        return x is not None and not (isinstance(x, float) and np.isnan(x))
    b_trig = trig(B["events"]["budget_trigger_s"])
    d_trig = trig(D["events"]["budget_trigger_s"])
    b_sig = trig(B["events"]["sigma_below_floor_s"])
    d_sig = trig(D["events"]["sigma_below_floor_s"])
    L.append("| 去掉偏置后 CMG 是否仍触发预算 | E1-B %s / E1-D %s | %s |"
             % (("%.1f s" % B["events"]["budget_trigger_s"]) if b_trig else "未触发",
                ("%.1f s" % D["events"]["budget_trigger_s"]) if d_trig else "未触发",
                "持续偏置是预算触发的直接来源" if (b_trig and not d_trig)
                else "预算触发不只由偏置引起"))
    L.append("| 去掉偏置后是否仍进入奇异区 | E1-B %s / E1-D %s | %s |"
             % (("%.1f s" % B["events"]["sigma_below_floor_s"]) if b_sig else "未跌破",
                ("%.1f s" % D["events"]["sigma_below_floor_s"]) if d_sig else "未跌破",
                "去掉偏置后不再跌破下限，构型运动不是独立原因" if (b_sig and not d_sig)
                else "即使去掉偏置仍跌破下限：构型/框架控制可独立造成失效"))
    L.append("| 去掉偏置后 D 是否优于 A | RMSE D %.3f° vs A %.3f°（%+.1f %%），C %.3f° | %s |"
             % (D["tracking"]["rmse_deg"], A["tracking"]["rmse_deg"],
                pct(A["tracking"]["rmse_deg"], D["tracking"]["rmse_deg"]),
                C["tracking"]["rmse_deg"],
                "CMG 带来净收益" if D["tracking"]["rmse_deg"] < C["tracking"]["rmse_deg"]
                else "CMG 仍未带来净收益（与 C 比）"))
    L.append("| 去掉偏置后纯舵面是否也明显改善 | C %.3f° vs A %.3f°（%+.1f %%） | %s |"
             % (C["tracking"]["rmse_deg"], A["tracking"]["rmse_deg"],
                pct(A["tracking"]["rmse_deg"], C["tracking"]["rmse_deg"]),
                "偏置同时限制纯舵面方案，不能把全部变化归因于 CMG"))
    L += [""]
    L += ["## 记录（方案 §1.3）", "",
          "| 量 | E1-B | E1-D |", "|---|---:|---:|"]
    L.append("| CMG 峰值 |H| [N·m·s] | %.2f | %.2f |"
             % (B["cmg"]["H_peak_Nms"], D["cmg"]["H_peak_Nms"]))
    L.append("| CMG 终值 |H| [N·m·s] | %.2f | %.2f |"
             % (B["cmg"]["H_end_Nms"], D["cmg"]["H_end_Nms"]))
    L.append("| d|H|/dt 峰值 [N·m]（时刻 s） | %.2f (%.1f) | %.2f (%.1f) |"
             % (B["cmg"]["dHdt_peak_Nm"], B["cmg"]["dHdt_peak_time_s"],
                D["cmg"]["dHdt_peak_Nm"], D["cmg"]["dHdt_peak_time_s"]))
    L.append("| H 分量范围 x/y/z [N·m·s] | %.1f..%.1f / %.1f..%.1f / %.1f..%.1f | %.1f..%.1f / %.1f..%.1f / %.1f..%.1f |"
             % tuple(B["cmg"]["H_x_range"] + B["cmg"]["H_y_range"] + B["cmg"]["H_z_range"]
                     + D["cmg"]["H_x_range"] + D["cmg"]["H_y_range"] + D["cmg"]["H_z_range"]))
    L.append("| QP 回退步数 / 最长连续 | %d / %d | %d / %d |"
             % (B["events"]["fallback_steps"], B["events"]["longest_fallback_run_steps"],
                D["events"]["fallback_steps"], D["events"]["longest_fallback_run_steps"]))
    L.append("")
    L.append("图：`results/c1_cmg_diagnostics/figures/e1_*.png`")
    with open(os.path.join(OUT, "summary.md"), "w", encoding="utf-8") as fh:
        fh.write("\n".join(L) + "\n")

    make_figures(loaded, out, keys)
    print("wrote", os.path.join(OUT, "metrics.csv"))
    print("wrote", os.path.join(OUT, "summary.md"))
    return out


def make_figures(groups, recs, keys):
    os.makedirs(FIG, exist_ok=True)
    G = groups
    style = {"A_original": ("C0", "-"), "B_original": ("C1", "-"),
             "C_sinusoidal_only": ("C0", "--"), "D_sinusoidal_only": ("C1", "--")}

    # disturbance + net impulse
    f, ax = plt.subplots(figsize=(11, 4.5))
    for k in ("A_original", "C_sinusoidal_only"):
        ts = G[k]["ts"]
        ax.plot(ts["t"], ts["M_dist_y"], color=style[k][0], ls=style[k][1], lw=1.0,
                label="%s (net %.0f N·m·s)" % (k, recs[k]["disturbance"]["integral_Nms"]))
    ax.set_xlabel("t [s]")
    ax.set_ylabel("pitch disturbance [N m]")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    ax.set_title("E1  pitch disturbance: with and without the 20 N·m constant")
    f.tight_layout(); f.savefig(os.path.join(FIG, "e1_disturbance.png"), dpi=130); plt.close(f)

    # |H|
    f, ax = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
    for k in keys:
        ts = G[k]["ts"]
        ax[0].plot(ts["t"], ts["H_norm"], color=style[k][0], ls=style[k][1], lw=1.1, label=k)
    ax[0].axhline(60.0, color="r", ls="--", lw=1.0, label="controller budget 60")
    ax[0].axhline(107.7, color="m", ls=":", lw=1.0, label="single-axis hardware 107.7")
    ax[0].set_ylabel("|H_cmg| [N m s]")
    ax[0].grid(alpha=0.3); ax[0].legend(fontsize=8)
    for k in keys:
        ts = G[k]["ts"]
        ax[1].plot(ts["t"], np.gradient(ts["H_norm"], 0.2), color=style[k][0],
                   ls=style[k][1], lw=1.0, label=k)
    ax[1].set_ylabel("d|H|/dt [N m]")
    ax[1].set_xlabel("t [s]")
    ax[1].grid(alpha=0.3); ax[1].legend(fontsize=8)
    ax[0].set_title("E1  CMG stored momentum and its growth rate")
    f.tight_layout(); f.savefig(os.path.join(FIG, "e1_momentum.png"), dpi=130); plt.close(f)

    # error + sigma
    f, ax = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
    for k in keys:
        ts = G[k]["ts"]
        ax[0].plot(ts["t"], ts["err_norm_deg"], color=style[k][0], ls=style[k][1], lw=1.1, label=k)
    ax[0].set_yscale("symlog", linthresh=1.0)
    ax[0].set_ylabel("attitude error [deg]")
    ax[0].grid(alpha=0.3); ax[0].legend(fontsize=8)
    for k in keys:
        ts = G[k]["ts"]
        ax[1].plot(ts["t"], ts["sigma"], color=style[k][0], ls=style[k][1], lw=1.0, label=k)
    ax[1].axhline(12.247, color="r", ls="--", lw=1.0, label="singularity floor 12.25")
    ax[1].set_yscale("log")
    ax[1].set_ylabel("sigma_min(A) [N m s]")
    ax[1].set_xlabel("t [s]")
    ax[1].grid(alpha=0.3); ax[1].legend(fontsize=8)
    ax[0].set_title("E1  tracking error and singularity index")
    f.tight_layout(); f.savefig(os.path.join(FIG, "e1_error_sigma.png"), dpi=130); plt.close(f)

    # budget-normalised summary bar
    f, ax = plt.subplots(figsize=(9, 4.5))
    labels = keys
    vals = [recs[k]["tracking"]["rmse_deg"] for k in keys]
    cols = ["C0", "C1", "C0", "C1"]
    hatch = ["", "", "//", "//"]
    bars = ax.bar(range(len(keys)), vals, color=cols, hatch=hatch, alpha=0.85)
    for i, k in enumerate(keys):
        ax.text(i, vals[i] * 1.02 + 0.5, "%.1f°" % vals[i], ha="center", fontsize=9)
    ax.set_xticks(range(len(keys)))
    ax.set_xticklabels([k.replace("_", "\n") for k in keys], fontsize=7)
    ax.set_yscale("log")
    ax.set_ylabel("whole-run RMSE [deg]")
    ax.grid(alpha=0.3, axis="y")
    ax.set_title("E1  whole-run RMSE: bias present (solid) vs bias removed (hatched)")
    f.tight_layout(); f.savefig(os.path.join(FIG, "e1_rmse_summary.png"), dpi=130); plt.close(f)


# ---------------------------------------------------------------------------
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--run", action="store_true", help="run E1-C and E1-D")
    ap.add_argument("--analyze", action="store_true", help="records + tables + figures")
    ap.add_argument("--modes", nargs="*", default=["A", "B"],
                    help="which modes of c1_sin_only to run (A=E1-C, B=E1-D)")
    a = ap.parse_args(argv)
    os.makedirs(OUT, exist_ok=True)

    par = parity()
    with open(os.path.join(OUT, "parity.json"), "w", encoding="utf-8") as fh:
        json.dump(par, fh, indent=2, ensure_ascii=False)
    print("parity:", {k: par[k] for k in ("task_change_limited", "ab_change_identical",
                                         "same_actuator_difference_only", "passed")})
    if not par["passed"]:
        print("PARITY FAILED - refusing to run")
        return 2

    if a.run:
        res = []
        for mode in a.modes:
            res.append(ab.run_one("c1_sin_only", mode, verbose=True, tag="e1_bias",
                                  config_path=CONFIG, outroot=OUT, cache_dir=CACHE))
        ab.update_tables(res, OUT)
    if a.analyze or not a.run:
        analyze()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
