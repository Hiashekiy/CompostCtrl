#!/usr/bin/env python
"""Collect the no-CMG (A) vs surface+CMG (B) runs into summary.csv / summary.md.

Everything reported here is read from the run directories written by
scripts/run_no_cmg_matrix.py (timeseries.csv / metrics.json / meta.json).  No
number is typed in by hand and no failed or fallback sample is dropped: the
solver-reliability columns report them as found.

python scripts/summarize_no_cmg_comparison.py
"""
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
RES = ROOT / "results" / "torque_gap_no_cmg_comparison_rti2"   # overridden by --root
PHASES = ("P0", "P1")
SCENARIO_ORDER = ["S00_pitch30", "S01_pitch_minus30", "S02_pitch10", "S03_pitch45",
                  "S04_roll30", "S05_yaw30", "S06_pitch30_roll20", "S07_sequential"]

CSV_COLUMNS = [
    "phase", "scenario", "arm", "seed", "plant", "cmg_present", "nx_model", "nu_model",
    "nlp_vars", "nlp_constraints", "t_start", "t_end", "n_steps",
    "rmse_all_deg", "rmse_pre_deg", "rmse_man0_deg", "rmse_man1_deg",
    "rmse_maneuver_deg", "max_err_maneuver_deg", "rate_rmse_maneuver",
    "err_x_deg", "err_y_deg", "err_z_deg",
    "settle_man0_s", "settled_man0", "settle_man1_s", "settled_man1",
    "tau_rms_x", "tau_rms_y", "tau_rms_z", "tau_peak_x", "tau_peak_y", "tau_peak_z",
    "tau_bound_used_maneuver", "fallback_ratio", "fallback_steps",
    "cpu_mean_s", "cpu_p95_s", "cpu_max_s", "realtime_met",
    "iters_mean", "max_violation", "gamma_clipped_steps", "n_status_kinds",
    "H_norm_max", "sigma_min_min", "nu_sat_ratio", "cmg_tau_rms_x", "cmg_tau_rms_y",
    "cmg_tau_rms_z", "cmg_tau_peak_x", "cmg_tau_peak_y", "cmg_tau_peak_z",
    "cmg_migration_lead_s", "cmg_H_start_x", "cmg_H_start_y", "cmg_H_start_z",
    "cmg_H_norm_start", "cmg_pre_gimbal_travel_rad", "cmg_gamma_dev_max_deg",
    "delta_rms_de", "delta_rms_da", "delta_rms_dr",
    "delta_peak_de", "delta_peak_da", "delta_peak_dr",
    "delta_rate_sat_ratio", "delta_pos_sat_ratio",
    "tau_cmd_vs_act_x", "tau_cmd_vs_act_y", "tau_cmd_vs_act_z",
    "p1_gimbal_mismatch_max", "run_id", "config",
]


def fnum(v, key, default=""):
    return v if v is not None else default


def load_runs():
    out = []
    for phase in PHASES:
        for d in sorted((RES / phase).iterdir()):
            mp = d / "metrics.json"
            if not d.is_dir() or not mp.exists():
                continue
            m = json.loads(mp.read_text(encoding="utf-8"))
            meta = json.loads((d / "meta.json").read_text(encoding="utf-8"))
            out.append(dict(dir=d, metrics=m, meta=meta))
    return out


def settle_from_csv(run_dir, metrics):
    """(settle_man0_s, settled_man0, settle_man1_s, settled_man1) from the raw log.

    Settle time = (last sample within [t0, t_end + 30 s] whose attitude error is
    above 1 deg) - t0.  A run that is still above 1 deg at the last sample of the
    simulation has NOT settled: the value is then the residual window length and
    `settled` is False, so a truncated number can never be read as convergence.
    """
    out = (None, None, None, None)
    csv_path = Path(run_dir) / "timeseries.csv"
    if not csv_path.exists():
        return out
    t, err = [], []
    with open(csv_path, encoding="utf-8") as fh:
        for row in csv.DictReader(fh):
            t.append(float(row["t_next"]))
            err.append(float(row["att_err_deg"]))
    t = np.array(t)
    err = np.array(err)
    dt = float(np.median(np.diff(t))) if t.size > 1 else 0.2
    wins = metrics.get("windows") or {}
    res = []
    for key in ("man0", "man1"):
        w = wins.get(key)
        if not w:
            res.extend([None, None])
            continue
        sel = (t >= w["t0"]) & (t <= min(w["t1"] + 30.0, t[-1]))
        bad = np.flatnonzero((err > 1.0) & sel)
        if not bad.size:
            res.extend([0.0, True])
        else:
            settle = float(t[bad[-1]] - w["t0"] + dt)
            settled = bool(t[bad[-1]] < t[-1] - 1.5 * dt)
            res.extend([settle, settled])
    return tuple(res)


def row_of(r):
    m, meta = r["metrics"], r["meta"]
    st = m.get("stats") or {}
    allw = st.get("all") or {}
    pre = st.get("pre") or {}
    man0 = st.get("man0") or {}
    man1 = st.get("man1") or {}
    comb = st.get("combined") or man0
    mig = m.get("cmg_migration") or {}
    exp = meta.get("experiment") or {}
    run_id = meta.get("run_id") or r["dir"].name
    settle0, settled0, settle1, settled1 = settle_from_csv(r["dir"], m)
    arm = meta.get("arm") or ("with_cmg" if run_id.endswith("with_cmg") else
                             "no_cmg" if "_no_cmg" in run_id else "?")
    row = dict(
        phase=meta.get("phase") or r["dir"].parent.name,
        scenario=meta.get("scenario") or run_id,
        arm=arm, seed=m.get("seed", meta.get("seed", 0)),
        plant=m.get("plant"), cmg_present=m.get("cmg_present"),
        nx_model=m.get("nx_model"), nu_model=m.get("nu_model"),
        nlp_vars=m.get("nlp_vars"), nlp_constraints=m.get("nlp_constraints"),
        t_start=m.get("t_start"), t_end=m.get("t_end"), n_steps=m.get("n_steps"),
        rmse_all_deg=m["rmse_deg"].get("all"),
        rmse_pre_deg=m["rmse_deg"].get("pre"),
        rmse_man0_deg=m["rmse_deg"].get("man0"),
        rmse_man1_deg=m["rmse_deg"].get("man1"),
        rmse_maneuver_deg=(comb or {}).get("rmse_deg"),
        max_err_maneuver_deg=(comb or {}).get("max_err_deg"),
        rate_rmse_maneuver=(comb or {}).get("rate_rmse"),
        err_x_deg=((comb or {}).get("err_axis_deg") or {}).get("x"),
        err_y_deg=((comb or {}).get("err_axis_deg") or {}).get("y"),
        err_z_deg=((comb or {}).get("err_axis_deg") or {}).get("z"),
        settle_man0_s=settle0, settled_man0=settled0,
        settle_man1_s=settle1, settled_man1=settled1,
        tau_rms_x=((comb or {}).get("tau_rms") or ["", "", ""])[0],
        tau_rms_y=((comb or {}).get("tau_rms") or ["", "", ""])[1],
        tau_rms_z=((comb or {}).get("tau_rms") or ["", "", ""])[2],
        tau_peak_x=((comb or {}).get("tau_peak") or ["", "", ""])[0],
        tau_peak_y=((comb or {}).get("tau_peak") or ["", "", ""])[1],
        tau_peak_z=((comb or {}).get("tau_peak") or ["", "", ""])[2],
        tau_bound_used_maneuver=(comb or {}).get("tau_bound_used"),
        fallback_ratio=m.get("fallback_ratio"), fallback_steps=m.get("fallback_steps"),
        cpu_mean_s=m["cpu_s"]["mean"], cpu_p95_s=m["cpu_s"]["p95"],
        cpu_max_s=m["cpu_s"]["max"], realtime_met=m.get("realtime_met"),
        iters_mean=(m.get("iterations") or {}).get("mean"),
        max_violation=m.get("max_violation"),
        gamma_clipped_steps=m.get("gamma_clipped_steps"),
        n_status_kinds=len(m.get("status_counts") or {}),
        H_norm_max=allw.get("H_norm_max"), sigma_min_min=allw.get("sigma_min_min"),
        nu_sat_ratio=allw.get("nu_sat_ratio"),
        cmg_tau_rms_x=((comb or {}).get("cmg_tau_rms") or ["", "", ""])[0],
        cmg_tau_rms_y=((comb or {}).get("cmg_tau_rms") or ["", "", ""])[1],
        cmg_tau_rms_z=((comb or {}).get("cmg_tau_rms") or ["", "", ""])[2],
        cmg_tau_peak_x=((comb or {}).get("cmg_tau_peak") or ["", "", ""])[0],
        cmg_tau_peak_y=((comb or {}).get("cmg_tau_peak") or ["", "", ""])[1],
        cmg_tau_peak_z=((comb or {}).get("cmg_tau_peak") or ["", "", ""])[2],
        cmg_migration_lead_s=mig.get("migration_lead_s"),
        cmg_H_start_x=(mig.get("H_at_start") or ["", "", ""])[0],
        cmg_H_start_y=(mig.get("H_at_start") or ["", "", ""])[1],
        cmg_H_start_z=(mig.get("H_at_start") or ["", "", ""])[2],
        cmg_H_norm_start=mig.get("H_norm_at_start"),
        cmg_pre_gimbal_travel_rad=mig.get("pre_gimbal_travel_rad"),
        cmg_gamma_dev_max_deg=mig.get("gamma_dev_max_deg"),
        delta_rms_de=((comb or {}).get("delta_rms_deg") or ["", "", ""])[0],
        delta_rms_da=((comb or {}).get("delta_rms_deg") or ["", "", ""])[1],
        delta_rms_dr=((comb or {}).get("delta_rms_deg") or ["", "", ""])[2],
        delta_peak_de=((comb or {}).get("delta_peak_deg") or ["", "", ""])[0],
        delta_peak_da=((comb or {}).get("delta_peak_deg") or ["", "", ""])[1],
        delta_peak_dr=((comb or {}).get("delta_peak_deg") or ["", "", ""])[2],
        delta_rate_sat_ratio=(comb or {}).get("delta_rate_sat_ratio"),
        delta_pos_sat_ratio=(comb or {}).get("delta_pos_sat_ratio"),
        tau_cmd_vs_act_x=((comb or {}).get("tau_cmd_vs_act_rms") or ["", "", ""])[0],
        tau_cmd_vs_act_y=((comb or {}).get("tau_cmd_vs_act_rms") or ["", "", ""])[1],
        tau_cmd_vs_act_z=((comb or {}).get("tau_cmd_vs_act_rms") or ["", "", ""])[2],
        p1_gimbal_mismatch_max=m.get("p1_gimbal_mismatch_max"),
        run_id=run_id, config=meta.get("config_path"),
    )
    return row


def fmt(v, nd=3):
    if v is None or v == "":
        return "—"
    if isinstance(v, bool):
        return "yes" if v else "no"
    if isinstance(v, (int, np.integer)):
        return str(int(v))
    try:
        x = float(v)
    except (TypeError, ValueError):
        return str(v)
    if not np.isfinite(x):
        return "—"
    return ("%%.%df" % nd) % x


def pct(a, b):
    """Relative change of a (B) vs b (A) in percent (negative = improvement)."""
    if a in (None, "") or b in (None, "") or float(b) == 0.0:
        return None
    return 100.0 * (float(a) - float(b)) / abs(float(b))


def settle_txt(row):
    s = row.get("settle_man0_s")
    ok = row.get("settled_man0")
    if s in (None, ""):
        return "—"
    return ("%s%s" % (fmt(s, 2), "" if ok else "（未收敛）"))


def pairs(rows, phase, seeds=(0,)):
    idx = {}
    for r in rows:
        if r["phase"] != phase or r["seed"] not in seeds:
            continue
        idx[(r["scenario"], r["arm"])] = r
    out = []
    for sc in SCENARIO_ORDER:
        a, b = idx.get((sc, "no_cmg")), idx.get((sc, "with_cmg"))
        if a and b:
            out.append((sc, a, b))
    return out


def main(argv=None):
    global RES
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=None)
    ap.add_argument("--out-suffix", default="")
    args = ap.parse_args(argv)
    if args.root:
        RES = (ROOT / args.root) if not Path(args.root).is_absolute() else Path(args.root)
        print("[summary] root =", RES)
    rows = [row_of(r) for r in load_runs()]
    rows.sort(key=lambda r: (r["phase"], SCENARIO_ORDER.index(r["scenario"])
                             if r["scenario"] in SCENARIO_ORDER else 99,
                             r["seed"], r["arm"]))
    if not rows:
        print("no runs found under", RES)
        return 1
    RES.mkdir(parents=True, exist_ok=True)
    with open(RES / "summary.csv", "w", encoding="utf-8", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=CSV_COLUMNS)
        w.writeheader()
        for r in rows:
            w.writerow({k: r.get(k, "") for k in CSV_COLUMNS})

    L = ["# 无 CMG（A 组）与 气动舵面 + CMG（B 组）多场景对照 —— 汇总", "",
         "由 `scripts/summarize_no_cmg_comparison.py` 从各次运行的 "
         "`metrics.json` / `meta.json` 自动生成；原始时序见 "
         "`results/torque_gap_no_cmg_comparison/<plant>/<run_id>/timeseries.csv`。", "",
         "评分窗口：`man0`/`man1` 为各次机动窗口本身（不含机动之间的保持阶段），"
         "`maneuver` 为所有机动窗口的并集，`pre` 为机动前 12 s，`all` 为全程。", "",
         "## 运行清单", "",
         "| phase | run_id | 场景 | 组 | seed | plant | 状态量/控制量 | NLP 变量/约束 | 步数 |",
         "|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        L.append("| %s | %s | %s | %s | %d | %s | %s/%s | %s/%s | %s |" % (
            r["phase"], r["run_id"], r["scenario"], r["arm"], r["seed"], r["plant"],
            fmt(r["nx_model"], 0), fmt(r["nu_model"], 0), fmt(r["nlp_vars"], 0),
            fmt(r["nlp_constraints"], 0), fmt(r["n_steps"], 0)))

    for phase, title in (("P0", "第一阶段 / 第二阶段：P0 虚拟力矩植物"),
                         ("P1", "第三阶段：P1 真实舵面执行器")):
        pr = pairs(rows, phase, seeds=(0,))
        if not pr:
            continue
        L += ["", "## %s" % title, "",
              "### 姿态跟踪（机动窗口）", "",
              "收敛时间 = 机动窗口内（含其后 30 s）最后一个姿态误差 > 1° 的时刻 − 机动起点；"
              "若仿真结束时仍 > 1°，记为“未收敛”并给出残余窗口长度。", "",
              "| 场景 | A RMSE [deg] | B RMSE [deg] | 改善 [deg] | 相对改善 | A 最大误差 | B 最大误差 |"
              " A 角速度 RMSE | B 角速度 RMSE | A 收敛时间 [s] | B 收敛时间 [s] | A 机动前 RMSE | B 机动前 RMSE |",
              "|---|---|---|---|---|---|---|---|---|---|---|---|---|"]
        for sc, a, b in pr:
            L.append("| %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |" % (
                sc, fmt(a["rmse_maneuver_deg"]), fmt(b["rmse_maneuver_deg"]),
                fmt((b["rmse_maneuver_deg"] or 0) - (a["rmse_maneuver_deg"] or 0))
                if a["rmse_maneuver_deg"] not in (None, "") else "—",
                ("%+.1f%%" % pct(b["rmse_maneuver_deg"], a["rmse_maneuver_deg"]))
                if pct(b["rmse_maneuver_deg"], a["rmse_maneuver_deg"]) is not None else "—",
                fmt(a["max_err_maneuver_deg"]), fmt(b["max_err_maneuver_deg"]),
                fmt(a["rate_rmse_maneuver"], 4), fmt(b["rate_rmse_maneuver"], 4),
                settle_txt(a), settle_txt(b),
                fmt(a["rmse_pre_deg"]), fmt(b["rmse_pre_deg"])))
        L += ["", "### 舵面负担与实际气动力矩", "",
              "| 场景 | A 舵面 RMS [N·m] (x/y/z) | B 舵面 RMS | A 舵面峰值 | B 舵面峰值 |"
              " A 贴边比例 | B 贴边比例 |",
              "|---|---|---|---|---|---|---|"]
        for sc, a, b in pr:
            L.append("| %s | %s | %s | %s | %s | %s | %s |" % (
                sc,
                "/".join(fmt(a["tau_rms_%s" % ax], 2) for ax in "xyz"),
                "/".join(fmt(b["tau_rms_%s" % ax], 2) for ax in "xyz"),
                "/".join(fmt(a["tau_peak_%s" % ax], 2) for ax in "xyz"),
                "/".join(fmt(b["tau_peak_%s" % ax], 2) for ax in "xyz"),
                fmt(a["tau_bound_used_maneuver"]), fmt(b["tau_bound_used_maneuver"])))
        if phase == "P1":
            L += ["", "### P1 舵偏角与执行器", "",
                  "| 场景 | A 舵偏 RMS [deg] (de/da/dr) | B 舵偏 RMS | A 速率饱和比例 |"
                  " B 速率饱和比例 | A 位置饱和比例 | B 位置饱和比例 |"
                  " A 期望-实际力矩偏差 [N·m] | B 期望-实际力矩偏差 |",
                  "|---|---|---|---|---|---|---|---|---|"]
            for sc, a, b in pr:
                L.append("| %s | %s | %s | %s | %s | %s | %s | %s | %s |" % (
                    sc,
                    "/".join(fmt(a["delta_rms_%s" % ax], 2) for ax in ("de", "da", "dr")),
                    "/".join(fmt(b["delta_rms_%s" % ax], 2) for ax in ("de", "da", "dr")),
                    fmt(a["delta_rate_sat_ratio"]), fmt(b["delta_rate_sat_ratio"]),
                    fmt(a["delta_pos_sat_ratio"]), fmt(b["delta_pos_sat_ratio"]),
                    "/".join(fmt(a["tau_cmd_vs_act_%s" % ax], 2) for ax in "xyz"),
                    "/".join(fmt(b["tau_cmd_vs_act_%s" % ax], 2) for ax in "xyz")))
        L += ["", "### 求解可靠性（不可删除项）", "",
              "| 场景 | A fallback | B fallback | A 求解均值 [s] | B 求解均值 [s] |"
              " A P95 [s] | B P95 [s] | A 满足 0.2 s 周期 | B 满足 0.2 s 周期 |"
              " A 最大约束违反 | B 最大约束违反 |",
              "|---|---|---|---|---|---|---|---|---|---|---|"]
        for sc, a, b in pr:
            L.append("| %s | %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |" % (
                sc, fmt(a["fallback_ratio"]), fmt(b["fallback_ratio"]),
                fmt(a["cpu_mean_s"]), fmt(b["cpu_mean_s"]),
                fmt(a["cpu_p95_s"]), fmt(b["cpu_p95_s"]),
                fmt(a["realtime_met"]), fmt(b["realtime_met"]),
                fmt(a["max_violation"], 2 if a["max_violation"] > 1e-3 else 8),
                fmt(b["max_violation"], 2 if b["max_violation"] > 1e-3 else 8)))
        if phase == "P0":
            L += ["", "### B 组 CMG 迁移与资源（A 组不适用）", "",
                  "| 场景 | 机动前 H [N·m·s] (x/y/z) | 机动前 |H| | 迁移提前量 [s] |"
                  " 机动前累计框架行程 [rad] | 框架角最大偏移 [deg] | |H| 峰值 |"
                  " sigma_min 最小 | 框架速率饱和比例 | CMG 力矩 RMS [N·m] (x/y/z) |",
                  "|---|---|---|---|---|---|---|---|---|---|"]
            for sc, a, b in pr:
                L.append("| %s | %s | %s | %s | %s | %s | %s | %s | %s | %s |" % (
                    sc,
                    "/".join(fmt(b["cmg_H_start_%s" % ax], 2) for ax in "xyz"),
                    fmt(b["cmg_H_norm_start"], 2), fmt(b["cmg_migration_lead_s"], 2),
                    fmt(b["cmg_pre_gimbal_travel_rad"], 3),
                    fmt(b["cmg_gamma_dev_max_deg"], 1), fmt(b["H_norm_max"], 1),
                    fmt(b["sigma_min_min"], 4), fmt(b["nu_sat_ratio"]),
                    "/".join(fmt(b["cmg_tau_rms_%s" % ax], 2) for ax in "xyz")))
            L += ["", "A 组没有 CMG 状态与 CMG 诊断量，故上述列一律记为“不适用”，"
                  "不以零值伪造成曲线或指标。"]

    # ---- multi-seed robustness
    seeds = sorted({r["seed"] for r in rows if r["seed"] != 0})
    if seeds:
        L += ["", "## 配对多随机种子稳健性（扰动的正弦分量相位由种子决定，A/B 同种子）", "",
              "| 场景 | seed | A RMSE | B RMSE | 相对改善 | A fallback | B fallback |",
              "|---|---|---|---|---|---|---|"]
        for sc in SCENARIO_ORDER:
            for seed in [0] + seeds:
                pr = pairs(rows, "P0", seeds=(seed,))
                for s2, a, b in pr:
                    if s2 != sc:
                        continue
                    rel = pct(b["rmse_maneuver_deg"], a["rmse_maneuver_deg"])
                    L.append("| %s | %d | %s | %s | %s | %s | %s |" % (
                        sc, seed, fmt(a["rmse_maneuver_deg"]),
                        fmt(b["rmse_maneuver_deg"]),
                        ("%+.1f%%" % rel) if rel is not None else "—",
                        fmt(a["fallback_ratio"]), fmt(b["fallback_ratio"])))

    # ---- mechanism evidence, fact only
    L += ["", "## 机制证据（B 组）", ""]
    for sc, a, b in pairs(rows, "P0", seeds=(0,)):
        if b["cmg_migration_lead_s"] in (None, ""):
            L.append("- **%s**：B 组未出现提前构型迁移。" % sc)
            continue
        Hv = [b["cmg_H_start_%s" % ax] for ax in "xyz"]
        L.append("- **%s**：B 组在机动开始前 %.1f s 起开始迁移构型，机动起始时刻 "
                 "H = (%s, %s, %s) N·m·s（|H| = %s），机动窗口 CMG 力矩 RMS 为 "
                 "(%s, %s, %s) N·m。" % (
                     sc, float(b["cmg_migration_lead_s"]), fmt(Hv[0], 2), fmt(Hv[1], 2),
                     fmt(Hv[2], 2), fmt(b["cmg_H_norm_start"], 1),
                     fmt(b["cmg_tau_rms_x"], 2), fmt(b["cmg_tau_rms_y"], 2),
                     fmt(b["cmg_tau_rms_z"], 2)))

    L += ["", "## 判读（按实验指导书 §13 的规则，逐条只陈述数据支持的部分）", ""]
    for phase in ("P0", "P1"):
        for sc, a, b in pairs(rows, phase, seeds=(0,)):
            ra, rb = a["rmse_maneuver_deg"], b["rmse_maneuver_deg"]
            tr_a = np.array([float(a["tau_rms_%s" % ax]) for ax in "xyz"])
            tr_b = np.array([float(b["tau_rms_%s" % ax]) for ax in "xyz"])
            rel = pct(rb, ra)
            parts = []
            if rel is not None:
                parts.append("机动窗口 RMSE %.3f → %.3f deg（%+.1f%%）" % (ra, rb, rel))
            parts.append("舵面 RMS 模长 %.2f → %.2f N·m" % (
                float(np.linalg.norm(tr_a)), float(np.linalg.norm(tr_b))))
            if rel is not None and rel < -5.0:
                verdict = "B 组跟踪改善"
            elif rel is not None and abs(rel) <= 5.0 and np.linalg.norm(tr_b) < 0.95 * np.linalg.norm(tr_a):
                verdict = "跟踪接近但 B 组舵面负担更低"
            elif rel is not None and abs(rel) <= 5.0:
                verdict = "跟踪与舵面负担均无明显差异"
            else:
                verdict = "B 组未改善（需查看失败/饱和情况）"
            L.append("- [%s] %s：%s；%s。" % (phase, sc, "；".join(parts), verdict))

    (RES / ("summary%s.md" % args.out_suffix)).write_text("\n".join(L) + "\n", encoding="utf-8")
    print("wrote", (RES / ("summary%s.csv" % args.out_suffix)),
          "(%d runs)" % len(rows))
    print("wrote", (RES / ("summary%s.md" % args.out_suffix)))
    return 0


if __name__ == "__main__":
    sys.exit(main())
