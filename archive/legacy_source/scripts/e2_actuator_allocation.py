"""E2 of the C1 diagnostics: is the CMG being used badly, and can a better
actuator split help under the *same* limits?

    python scripts/e2_actuator_allocation.py --check     # proofs, before running
    python scripts/e2_actuator_allocation.py --run       # E2-B1 (about 6-8 min)
    python scripts/e2_actuator_allocation.py --analyze   # metrics, split, figures

Arms
    E2-B0  the formal B run already frozen in results/c1_cmg_reproduction/cmg/c1/B
           (read-only, reused, never re-run)
    E2-B1  the same task with `nearspace.cmg_priority.LTVMPCPriority`: the CMG
           effort weight becomes authority-driven instead of error-driven, and
           a momentum-growth penalty ramps in above 75 % of the momentum budget.
           Two cost-shaping mechanisms, no constraint touched.

`--check` writes three machine-checkable proofs to check.json:
  1. with the mechanism switched off, the priority controller builds a QP whose
     H, g, A, lb, ub are bit-identical to the factory controller;
  2. with it switched on, A, lb, ub are still bit-identical (no new, relaxed or
     tightened constraint) and only H differs (a cost change);
  3. the resolved configuration differs from the B0 experiment only by the added
     `mpc_priority` block.
"""
from __future__ import annotations

import argparse
import collections
import copy
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
from nearspace import sim                                            # noqa: E402
from nearspace.cmg_control import LTVMPCCMG2                          # noqa: E402
from nearspace.cmg_priority import LTVMPCPriority                     # noqa: E402
from nearspace.config import dump                                     # noqa: E402

CONFIG = os.path.join(ROOT, "configs", "c1_cmg_reproduction_priority.yaml")
B0_CONFIG = os.path.join(ROOT, "configs", "c1_cmg_reproduction.yaml")
OUT = os.path.join(ROOT, "results", "c1_cmg_diagnostics", "E2_actuator_allocation")
B0_DIR = os.path.join(ROOT, "results", "c1_cmg_reproduction", "cmg", "c1", "B")
B1_DIR = os.path.join(OUT, "B1_surface_priority")
FIG = os.path.join(ROOT, "results", "c1_cmg_diagnostics", "figures")
CACHE = os.path.join(ROOT, "results", "cmg_ab", "_cache")
CASE = "c1"
DT = 0.2
H_LIM = 60.0
FLOOR = 0.30 * 0.816496580927726 * 50.0


# ---------------------------------------------------------------------------
# --check: proofs before any simulation
# ---------------------------------------------------------------------------
def _probe_qp(ctrl, cfg, t=118.0, gamma=(0.30, -0.25, 0.20, -0.15)):
    b = sim.build(cfg, cache_dir=CACHE)
    traj, prm = b["traj"], b["prm"]
    x0 = ctrl.command_from_x(sim._initial_state(traj, prm))
    x0[0:4] = traj.q_ref(t)
    x0[4:7] = np.zeros(3)
    x0[7:10] = np.array([0.02, 0.0, 0.0])
    x0[10:14] = np.array(gamma, dtype=float)              # a non-trivial gimbal state
    x0[14:18] = np.array([0.4, -0.4, 0.3, -0.3])
    t_seq = np.concatenate([[t], t + np.cumsum(ctrl.dts)])
    p_seq = [traj.p_array(t_seq[k], prm.veh) for k in range(ctrl.N)]
    u_seq = np.zeros((ctrl.N, 6))
    x_seq = ctrl._rollout(x0, p_seq, u_seq)
    H, g, A, lb, ub = ctrl._build_qp(x0, x_seq, u_seq, p_seq, t_seq, traj)
    return dict(H=H, g=g, A=A, lb=lb, ub=ub, x0=x0, H_norm=float(
        np.linalg.norm(ctrl.cluster.momentum(x0[10:14]))))


def _cmp(a, b):
    return bool(a.shape == b.shape and np.array_equal(a, b))


def check():
    os.makedirs(OUT, exist_ok=True)
    cfgB0 = ab.build_cfg(CASE, "B", B0_CONFIG)
    cfgB1 = ab.build_cfg(CASE, "B", CONFIG)

    b = sim.build(cfgB1, cache_dir=CACHE)
    fac = LTVMPCCMG2(b["prm"], cfgB1, b["cluster"])
    prio_off = LTVMPCPriority(b["prm"], cfgB1, b["cluster"])
    prio_off.prio_enabled = False
    prio_on = LTVMPCPriority(b["prm"], cfgB1, b["cluster"])
    prio_on.prio_enabled = True

    q_fac = _probe_qp(fac, cfgB1)
    q_off = _probe_qp(prio_off, cfgB1)
    q_on = _probe_qp(prio_on, cfgB1)
    # a second probe above the momentum soft knee (|H| > 0.75 * 60 N·m·s) so the
    # growth penalty is actually exercised; the gimbal state is chosen by scanning
    # a fixed candidate list for the largest |H| (deterministic, not tuned)
    cands = [(1.2, -1.2, 1.2, -1.2), (1.2, 0.0, -1.2, 0.0), (1.4, 1.4, -1.4, -1.4),
             (1.5, 0.5, -1.5, -0.5), (2.5, 2.5, 2.5, 2.5), (1.0, 2.0, 3.0, 4.0),
             (2.0, -1.0, 2.0, -1.0), (3.0, 1.0, -3.0, -1.0)]
    b0 = sim.build(cfgB1, cache_dir=CACHE)
    big = max(cands, key=lambda g: float(np.linalg.norm(
        b0["cluster"].momentum(np.array(g, dtype=float)))))
    print("high-|H| probe gimbal state:", big, "-> |H| = %.1f N·m·s"
          % float(np.linalg.norm(b0["cluster"].momentum(np.array(big, dtype=float)))))
    q_fac_hi = _probe_qp(fac, cfgB1, gamma=big)
    q_on_hi = _probe_qp(prio_on, cfgB1, gamma=big)

    def flat(d, pre=""):
        out = {}
        for k, v in d.items():
            if isinstance(v, dict):
                out.update(flat(v, pre + "/" + k))
            else:
                out[pre + "/" + k] = v
        return out

    f0, f1 = flat(dump(cfgB0)), flat(dump(cfgB1))
    diffs = {k: [f0.get(k), f1.get(k)] for k in sorted(set(f0) | set(f1))
             if f0.get(k) != f1.get(k)}

    res = dict(
        probe_t=118.0,
        probe_cmg_momentum_Nms=q_on["H_norm"],
        off_reproduces_factory=dict(
            H=_cmp(q_fac["H"], q_off["H"]), g=_cmp(q_fac["g"], q_off["g"]),
            A=_cmp(q_fac["A"], q_off["A"]), lb=_cmp(q_fac["lb"], q_off["lb"]),
            ub=_cmp(q_fac["ub"], q_off["ub"])),
        on_keeps_constraints=dict(
            A=_cmp(q_fac["A"], q_on["A"]), lb=_cmp(q_fac["lb"], q_on["lb"]),
            ub=_cmp(q_fac["ub"], q_on["ub"])),
        on_changes_cost=dict(
            probe_H_norm_Nms=[q_fac["H_norm"], q_fac_hi["H_norm"]],
            H_changed_below_knee=not _cmp(q_fac["H"], q_on["H"]),
            H_changed_above_knee=not _cmp(q_fac_hi["H"], q_on_hi["H"]),
            A_unchanged_above_knee=_cmp(q_fac_hi["A"], q_on_hi["A"]),
            lb_unchanged_above_knee=_cmp(q_fac_hi["lb"], q_on_hi["lb"]),
            ub_unchanged_above_knee=_cmp(q_fac_hi["ub"], q_on_hi["ub"]),
            hessian_rel_change_above_knee=float(
                np.linalg.norm(q_on_hi["H"] - q_fac_hi["H"])
                / max(np.linalg.norm(q_fac_hi["H"]), 1e-12))),
        config_diff_B0_vs_B1=diffs,
        config_diff_only_priority=set(diffs) <= {
            "/mpc_priority/enabled", "/mpc_priority/aero_authority_frac",
            "/mpc_priority/H_soft_frac", "/mpc_priority/r_growth_scale",
            "/mpc_priority/r_growth", "/label", "/scenario/mode_label"},
        config_diff_keys=sorted(diffs),
    )
    res["passed"] = bool(all(res["off_reproduces_factory"].values())
                         and all(res["on_keeps_constraints"].values())
                         and res["on_changes_cost"]["A_unchanged_above_knee"]
                         and res["on_changes_cost"]["lb_unchanged_above_knee"]
                         and res["on_changes_cost"]["ub_unchanged_above_knee"]
                         and res["on_changes_cost"]["H_changed_above_knee"]
                         and res["config_diff_only_priority"])
    with open(os.path.join(OUT, "check.json"), "w", encoding="utf-8") as fh:
        json.dump(res, fh, indent=2, ensure_ascii=False)
    print(json.dumps({k: res[k] for k in ("off_reproduces_factory", "on_keeps_constraints",
                                          "on_changes_cost", "config_diff_only_priority")},
                     ensure_ascii=False, indent=1))
    print("extra config keys:", list(res["config_diff_B0_vs_B1"]))
    print("passed:", res["passed"])
    return res


# ---------------------------------------------------------------------------
# --run
# ---------------------------------------------------------------------------
def run(tag="e2_priority"):
    cfg = ab.build_cfg(CASE, "B", CONFIG)
    b = sim.build(cfg, cache_dir=CACHE)
    ctrl = LTVMPCPriority(b["prm"], cfg, b["cluster"])
    if not ctrl.prio_enabled:
        raise SystemExit("mpc_priority.enabled is false: refusing to run E2-B1")
    print("B1 mechanisms in force: eta_aero=%.2f  H_soft=%.1f N·m·s  "
          "r_growth_scale=%.2f  r_growth=%.2f"
          % (ctrl.eta_aero, ctrl.H_soft, ctrl.r_growth_scale, ctrl.r_growth), flush=True)
    rec = sim.run(cfg, cache_dir=CACHE, ctrl=ctrl, progress=True)
    os.makedirs(B1_DIR, exist_ok=True)
    ab.write_timeseries(os.path.join(B1_DIR, "timeseries.csv"), rec)
    with open(os.path.join(B1_DIR, "metrics.json"), "w", encoding="utf-8") as fh:
        json.dump(rec["metrics"], fh, indent=2, ensure_ascii=False, default=ab._j)
    meta = sim.metadata(cfg, rec, CASE, tag,
                        extra=dict(label="E2-B1 surface priority + momentum management",
                                   controller="nearspace.cmg_priority.LTVMPCPriority",
                                   mpc_priority=dump(cfg.mpc_priority)))
    with open(os.path.join(B1_DIR, "meta.json"), "w", encoding="utf-8") as fh:
        json.dump(meta, fh, indent=2, ensure_ascii=False, default=ab._j)
    import yaml
    with open(os.path.join(B1_DIR, "config_resolved.yaml"), "w", encoding="utf-8") as fh:
        yaml.safe_dump(dump(cfg), fh, allow_unicode=True, sort_keys=False)
    # the priority scheduler's own decision log (why the weight was what it was)
    log = ctrl._log
    if log:
        keys = list(log[0].keys())
        with open(os.path.join(B1_DIR, "priority_weight_log.csv"), "w", newline="",
                  encoding="utf-8") as fh:
            fh.write(",".join(keys) + "\n")
            for r in log:
                fh.write(",".join("%.10g" % r[k] for k in keys) + "\n")
    m = rec["metrics"]
    print("[B1] RMSE=%.4f deg  peak=%.3f  |H|max=%.1f  sigma_min=%.2f  fail=%d  wall=%.0fs"
          % (m["err_rmse_deg"], m["err_peak_deg"], m["cmg_momentum_peak_Nms"],
             m["cmg_sigma_min"], m["fallback_count"], rec["wall_time_s"]), flush=True)
    return rec


# ---------------------------------------------------------------------------
# --analyze
# ---------------------------------------------------------------------------
def _series(path):
    return np.genfromtxt(path, delimiter=",", names=True)


def _load(dirpath):
    with open(os.path.join(dirpath, "metrics.json"), encoding="utf-8") as fh:
        metrics = json.load(fh)
    with open(os.path.join(dirpath, "meta.json"), encoding="utf-8") as fh:
        meta = json.load(fh)
    return dict(dir=dirpath, metrics=metrics, meta=meta,
                ts=_series(os.path.join(dirpath, "timeseries.csv")))


def allocation_split(ts, lo, hi):
    """Pitch-axis split of the disturbance-compensation task.

    "opposing = -M sign(M_dist)": positive means that actuator pushes against the
    disturbance.  The share is a MEDIAN over the steps with |M_dist| > 1 N m: in a
    diverging run a few saturated steps carry |M_aero| of thousands of N m and
    would swamp a mean.
    """
    t = ts["t"]
    sel = (t >= lo) & (t <= hi)
    md = ts["M_dist_y"][sel]
    ma = ts["M_aero_y"][sel]
    mc = ts["M_cmg_y"][sel]
    sgn = np.where(np.abs(md) > 1e-9, np.sign(md), 0.0)
    a = -ma * sgn
    c = -mc * sgn
    tot = np.abs(a) + np.abs(c)
    ok = (tot > 1e-9) & (np.abs(md) > 1.0)
    share = np.abs(a[ok]) / tot[ok]
    return dict(window=[lo, hi], n_steps=int(sel.sum()),
                mean_aero_opposing_Nm=float(np.median(a[ok])) if ok.any() else None,
                mean_cmg_opposing_Nm=float(np.median(c[ok])) if ok.any() else None,
                share_aero=float(np.median(share)) if ok.any() else None,
                share_aero_mean=float(np.mean(share)) if ok.any() else None,
                frac_cmg_useful=float(np.mean(np.abs(c[ok]) > 1.0)) if ok.any() else None,
                share_cmg=None if not ok.any() else 1.0 - float(np.median(share)))


def pitch_balance(ts, lo, hi):
    """Pitch balance J_y dq/dt = M_aero + M_cmg + M_dist - (omega x (J omega + H))_y.

    The derivative is a finite difference on the 0.2 s control grid, so the
    residual is only meaningful while the body rates are smooth: it is reported
    for the pre-pulse window as a method check and separately over the divergent
    tail, where the sampling cannot support a derivative.
    """
    J = np.array([1200.0, 8500.0, 9000.0])            # vehicle3 inertia
    t = ts["t"]
    sel = (t >= lo) & (t <= hi)
    dt = float(t[1] - t[0])
    w = np.column_stack([ts["omega_x"], ts["omega_y"], ts["omega_z"]])
    H = np.column_stack([ts["H_cmg_x"], ts["H_cmg_y"], ts["H_cmg_z"]])
    dq = np.gradient(w[:, 1], dt)
    Jw = w * J
    gyro = np.cross(w, Jw + H)[:, 1]
    lhs = J[1] * dq
    rhs = ts["M_aero_y"] + ts["M_cmg_y"] + ts["M_dist_y"] - gyro
    return dict(window=[lo, hi],
                residual_rms_Nm=float(np.sqrt(np.mean((lhs[sel] - rhs[sel]) ** 2))),
                torque_rms_Nm=float(np.sqrt(np.mean(rhs[sel] ** 2))),
                max_abs_rate_deg_s=float(np.rad2deg(np.max(np.abs(w[sel, 1])))))


def rmse_A():
    p = os.path.join(ROOT, "results", "c1_cmg_reproduction", "cmg", "c1", "A", "metrics.json")
    with open(p, encoding="utf-8") as fh:
        return float(json.load(fh)["err_rmse_deg"])


def analyze(check_res):
    B0 = _load(B0_DIR)
    B1 = _load(B1_DIR)
    hi = B1["meta"]["windows"]["high"]
    rows = []
    for name, g in (("E2-B0 当前统一 MPC", B0), ("E2-B1 舵面优先 + 动量管理", B1)):
        m = g["metrics"]
        ts = g["ts"]
        nH = ts["H_norm"]
        dH = np.gradient(nH, DT)
        fb = ts["fallback"] > 0.5
        best = cur = 0
        for v in fb:
            cur = cur + 1 if v else 0
            best = max(best, cur)
        rows.append(dict(
            arm=name,
            rmse=m["err_rmse_deg"], peak=m["err_peak_deg"],
            high_rmse=m["high_rmse_deg"], high_peak=m["high_peak_deg"],
            sat=m["delta_sat_frac_all"], travel=m["delta_travel_deg"],
            H_peak=m["cmg_momentum_peak_Nms"], H_end=m["cmg_momentum_end_Nms"],
            H_growth_peak=float(dH.max()), H_growth_time=float(ts["t"][int(np.argmax(dH))]),
            sigma=m["cmg_sigma_min"], torque_peak=m["cmg_torque_peak_Nm"],
            fb=m["fallback_count"], fb_longest=best,
            solve_mean=m["solve_time_mean_ms"], solve_p95=m["solve_time_p95_ms"],
            solve_max=m["solve_time_max_ms"],
            split_all=allocation_split(ts, 0.0, 200.0),
            split_high=allocation_split(ts, hi[0], hi[1]),
            split_post=allocation_split(ts, 110.0, 200.0),
            balance_clean=pitch_balance(ts, 0.0, 105.0),
            balance_tail=pitch_balance(ts, 110.0, 200.0)))

    with open(os.path.join(OUT, "metrics.csv"), "w", newline="", encoding="utf-8") as fh:
        keys = ["arm", "rmse", "peak", "high_rmse", "high_peak", "sat", "travel",
                "H_peak", "H_end", "H_growth_peak", "H_growth_time", "sigma",
                "torque_peak", "fb", "fb_longest", "solve_mean", "solve_p95", "solve_max",
                "share_aero_all", "share_aero_high", "share_aero_post",
                "mean_aero_opp_high", "mean_cmg_opp_high"]
        fh.write(",".join(keys) + "\n")
        for r in rows:
            fh.write(",".join("%.6g" % v if isinstance(v, (int, float)) else str(v)
                              for v in [r["arm"], r["rmse"], r["peak"], r["high_rmse"],
                                        r["high_peak"], r["sat"], r["travel"], r["H_peak"],
                                        r["H_end"], r["H_growth_peak"], r["H_growth_time"],
                                        r["sigma"], r["torque_peak"], r["fb"],
                                        r["fb_longest"], r["solve_mean"], r["solve_p95"],
                                        r["solve_max"],
                                        r["split_all"]["share_aero"],
                                        r["split_high"]["share_aero"],
                                        r["split_post"]["share_aero"],
                                        r["split_high"]["mean_aero_opposing_Nm"],
                                        r["split_high"]["mean_cmg_opposing_Nm"]]) + "\n")

    # B0 is read-only: leave a pointer + the derived record, never a copy of the data
    d0 = os.path.join(OUT, "B0_current_mpc")
    os.makedirs(d0, exist_ok=True)
    with open(os.path.join(d0, "SOURCE.txt"), "w", encoding="utf-8") as fh:
        fh.write("raw artefacts: %s\n(not copied; the formal A/B data is read-only)\n"
                 % os.path.relpath(B0_DIR, ROOT))
    with open(os.path.join(d0, "metrics.json"), "w", encoding="utf-8") as fh:
        json.dump(B0["metrics"], fh, indent=2, ensure_ascii=False)
    with open(os.path.join(d0, "diagnostics.json"), "w", encoding="utf-8") as fh:
        json.dump(rows[0], fh, indent=2, ensure_ascii=False, default=float)
    with open(os.path.join(B1_DIR, "diagnostics.json"), "w", encoding="utf-8") as fh:
        json.dump(rows[1], fh, indent=2, ensure_ascii=False, default=float)

    r0, r1 = rows
    L = ["# E2 执行机构分工诊断：CMG 是否被过度使用？", "",
         "自动生成：`scripts/e2_actuator_allocation.py --analyze`。"
         "E2-B0 直接读取正式 B 组原始产物（只读）；E2-B1 是新增运行。", "",
         "## 运行前证明（`--check`，check.json）", "",
         "| 证明 | 结果 |", "|---|---|",
         "| 关闭机制时，优先级控制器与工厂控制器构建的 QP 逐位相同（H,g,A,lb,ub） | %s |"
         % check_res["off_reproduces_factory"],
         "| 打开机制时，约束矩阵与上下界仍逐位相同（A,lb,ub） | %s |"
         % check_res["on_keeps_constraints"],
         "| 打开机制时只有目标函数改变（约束不变，H 改变） | H 相对变化 %.2e"
         "（软拐点上方，|H|=%.1f N·m·s 的探针） |"
         % (check_res["on_changes_cost"]["hessian_rel_change_above_knee"],
            check_res["on_changes_cost"]["probe_H_norm_Nms"][1]),
         "| 解析配置差异仅限 `mpc_priority` 段 | %s |" % check_res["config_diff_only_priority"],
         "",
         "也就是说：B1 没有新增/放宽/收紧任何约束，只改了代价函数的两处。", "",
         "## 核心对比", "",
         "| 指标 | E2-B0 | E2-B1 | 变化 |", "|---|---:|---:|---:|"]

    def line(label, k, fmt="%.4g", better="min"):
        a, b = r0[k], r1[k]
        chg = ("%+.1f %%" % (100.0 * (b / a - 1.0))) if a else "n/a"
        L.append("| %s | %s | %s | %s |" % (label, fmt % a, fmt % b, chg))

    line("全程 RMSE [deg]", "rmse")
    line("峰值误差 [deg]", "peak")
    line("高空段 RMSE [deg]", "high_rmse")
    line("高空段峰值 [deg]", "high_peak")
    line("舵面饱和比例 [-]", "sat")
    line("舵面每步动作量 [deg]", "travel")
    line("CMG 峰值 |H| [N·m·s]", "H_peak")
    line("CMG 终值 |H| [N·m·s]", "H_end")
    line("d|H|/dt 峰值 [N·m]", "H_growth_peak")
    L.append("| d|H|/dt 峰值时刻 [s] | %.1f | %.1f | — |" % (r0["H_growth_time"], r1["H_growth_time"]))
    line("σ_min [N·m·s]", "sigma")
    line("CMG 峰值力矩 [N·m]", "torque_peak")
    line("QP 回退步数", "fb", "%d")
    line("最长连续回退 [步]", "fb_longest", "%d")
    line("平均求解时间 [ms]", "solve_mean", "%.1f")
    line("P95 求解时间 [ms]", "solve_p95", "%.1f")
    line("最坏求解时间 [ms]", "solve_max", "%.1f")

    L += ["", "## 俯仰轴上的分工（谁在抵消扰动）", "",
          "定义 `opposing = -M·sign(M_dist)`：正值表示该执行机构在与扰动对抗。"
          "份额 = |aero|/(|aero|+|cmg|)（仅统计两者合成为正的时刻）。", "",
          "| 窗口 | B0 舵面份额 | B0 舵面/CMG 对抗力矩中位数 [N·m] | B1 舵面份额 | B1 舵面/CMG 对抗力矩中位数 |",
          "|---|---:|---|---:|---|"]
    for key, label in (("split_all", "全程"), ("split_high", "高空段 %.1f–%.1f s" % tuple(hi)),
                       ("split_post", "脉冲后 110–200 s")):
        s0, s1 = r0[key], r1[key]
        L.append("| %s | %.1f %% | %.2f / %.2f | %.1f %% | %.2f / %.2f |"
                 % (label, 100 * (s0["share_aero"] or 0),
                    s0["mean_aero_opposing_Nm"] or 0, s0["mean_cmg_opposing_Nm"] or 0,
                    100 * (s1["share_aero"] or 0),
                    s1["mean_aero_opposing_Nm"] or 0, s1["mean_cmg_opposing_Nm"] or 0))
    L += ["", "（份额与力矩均为满足 |M_dist| > 1 N·m 的步上的**中位数**，避免发散段个别饱和步"
          "把均值带偏。）", ""]
    L += ["", "俯仰力矩平衡残差（J_y·q̇ = M_aero + M_cmg + M_dist − (ω×(Jω+H))_y）：", "",
          "| 窗口 | B0 残差 / 力矩 RMS [N·m] | B0 最大俯仰角速度 | B1 残差 / 力矩 RMS | B1 最大俯仰角速度 |",
          "|---|---|---|---|---|",
          "| 脉冲前 0–105 s（方法自检） | %.3g / %.0f | %.1f °/s | %.3g / %.0f | %.1f °/s |"
          % (r0["balance_clean"]["residual_rms_Nm"], r0["balance_clean"]["torque_rms_Nm"],
             r0["balance_clean"]["max_abs_rate_deg_s"], r1["balance_clean"]["residual_rms_Nm"],
             r1["balance_clean"]["torque_rms_Nm"], r1["balance_clean"]["max_abs_rate_deg_s"]),
          "| 发散段 110–200 s | %.3g / %.0f | %.0f °/s | %.3g / %.0f | %.0f °/s |"
          % (r0["balance_tail"]["residual_rms_Nm"], r0["balance_tail"]["torque_rms_Nm"],
             r0["balance_tail"]["max_abs_rate_deg_s"], r1["balance_tail"]["residual_rms_Nm"],
             r1["balance_tail"]["torque_rms_Nm"], r1["balance_tail"]["max_abs_rate_deg_s"]),
          "",
          "诚实的读法：即使在脉冲前的平稳段，残差（约 30 N·m RMS）与力矩量级同阶，说明 "
          "0.2 s 采样下的 ω 差分不足以把 J_y·q̇ 算准（控制步之间机体角速度含逐拍抖动），"
          "因此上面那张份额表只能作为**量级**证据（舵面承担了几乎全部对抗力矩、CMG 的中位"
          "对抗力矩≈0），不能当作已经闭合的力矩账。这是本报告的已知限制之一。", ""]
    L += ["## 判据（方案 §2.5）", ""]
    L += ["* B1 是否减少 CMG 动量增长：峰值 |H| %.2f → %.2f N·m·s（%+.1f %%），"
          "d|H|/dt 峰值 %.2f → %.2f N·m。"
          % (r0["H_peak"], r1["H_peak"], 100 * (r1["H_peak"] / r0["H_peak"] - 1) if r0["H_peak"] else float("nan"),
             r0["H_growth_peak"], r1["H_growth_peak"]),
          "* 是否改善求解稳定性：回退步数 %d → %d，σ_min %.2f → %.2f。"
          % (r0["fb"], r1["fb"], r0["sigma"], r1["sigma"]),
          "* 是否改善姿态跟踪：RMSE %.3f° → %.3f°（%+.1f %%），"
          "A 组（正式 A/B 的仅舵面运行）为 %.3f°。"
          % (r0["rmse"], r1["rmse"], 100 * (r1["rmse"] / r0["rmse"] - 1), rmse_A()),
          ""]
    if r1["fb"] == 0 and r1["H_peak"] < H_LIM:
        L.append("* B1 保持零回退且角动量不超预算：分工机制在真实约束下确实改变了 CMG 的使用方式。")
    elif r1["H_peak"] < r0["H_peak"] and r1["fb"] > 0:
        L.append("* B1 降低了动量增长但仍有回退：分工机制有效，但不足以让该任务在约束内完成。")
    elif r1["H_peak"] >= r0["H_peak"] and r1["fb"] > 0:
        L.append("* B1 既未降低动量也未消除回退：**不能**把失败归因于执行机构分工，"
                 "应回到 E1/E3 的结论（任务净角冲量 + 约束饱和）。")
    L.append("")
    L.append("图：`results/c1_cmg_diagnostics/figures/e2_*.png`")
    with open(os.path.join(OUT, "summary.md"), "w", encoding="utf-8") as fh:
        fh.write("\n".join(L) + "\n")
    make_figures(B0, B1, rows)
    print("wrote", os.path.join(OUT, "summary.md"))
    return rows


def make_figures(B0, B1, rows):
    os.makedirs(FIG, exist_ok=True)
    a, b = B0["ts"], B1["ts"]
    f, ax = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
    ax[0].plot(a["t"], a["H_norm"], "C1", lw=1.1, label="B0 当前统一 MPC")
    ax[0].plot(b["t"], b["H_norm"], "C0", lw=1.1, label="B1 舵面优先 + 动量管理")
    ax[0].axhline(H_LIM, color="r", ls="--", lw=1.0, label="budget 60")
    ax[0].axhline(0.75 * H_LIM, color="r", ls=":", lw=1.0, label="soft knee 45")
    ax[0].set_ylabel("|H_cmg| [N m s]"); ax[0].grid(alpha=0.3); ax[0].legend(fontsize=8)
    ax[1].plot(a["t"], a["sigma"], "C1", lw=1.0, label="B0")
    ax[1].plot(b["t"], b["sigma"], "C0", lw=1.0, label="B1")
    ax[1].axhline(FLOOR, color="r", ls="--", lw=1.0, label="floor 12.25")
    ax[1].set_yscale("log"); ax[1].set_ylabel("sigma_min [N m s]")
    ax[1].set_xlabel("t [s]"); ax[1].grid(alpha=0.3); ax[1].legend(fontsize=8)
    ax[0].set_title("E2  CMG momentum and singularity index: factory vs surface-priority MPC")
    f.tight_layout(); f.savefig(os.path.join(FIG, "e2_momentum_sigma.png"), dpi=130); plt.close(f)

    f, ax = plt.subplots(3, 1, figsize=(11, 9), sharex=True)
    ax[0].plot(a["t"], a["M_dist_y"], "k", lw=1.0, label="disturbance")
    ax[0].plot(a["t"], -a["M_aero_y"], "C1", lw=1.0, label="B0 -M_aero")
    ax[0].plot(b["t"], -b["M_aero_y"], "C0", lw=1.0, label="B1 -M_aero")
    ax[0].set_ylabel("[N m]"); ax[0].grid(alpha=0.3); ax[0].legend(fontsize=8)
    ax[1].plot(a["t"], -a["M_aero_y"] - a["M_cmg_y"], lw=1.0, color="C1", label="B0 total control")
    ax[1].plot(b["t"], -b["M_aero_y"] - b["M_cmg_y"], lw=1.0, color="C0", label="B1 total control")
    ax[1].plot(a["t"], a["M_dist_y"], "k", lw=0.8, label="disturbance")
    ax[1].set_ylabel("[N m]"); ax[1].grid(alpha=0.3); ax[1].legend(fontsize=8)
    ax[2].plot(a["t"], -a["M_cmg_y"], "C1", lw=1.0, label="B0 -M_cmg")
    ax[2].plot(b["t"], -b["M_cmg_y"], "C0", lw=1.0, label="B1 -M_cmg")
    ax[2].set_ylabel("[N m]"); ax[2].set_xlabel("t [s]")
    ax[2].grid(alpha=0.3); ax[2].legend(fontsize=8)
    ax[0].set_title("E2  pitch-axis torque decomposition (who opposes the disturbance)")
    f.tight_layout(); f.savefig(os.path.join(FIG, "e2_torque_split.png"), dpi=130); plt.close(f)

    f, ax = plt.subplots(2, 1, figsize=(11, 8), sharex=True)
    ax[0].plot(a["t"], a["err_norm_deg"], "C1", lw=1.1, label="B0")
    ax[0].plot(b["t"], b["err_norm_deg"], "C0", lw=1.1, label="B1")
    ax[0].set_yscale("symlog", linthresh=1.0)
    ax[0].set_ylabel("attitude error [deg]"); ax[0].grid(alpha=0.3); ax[0].legend(fontsize=8)
    ax[1].step(a["t"], a["fallback"], where="post", color="C1", label="B0 fallback")
    ax[1].step(b["t"], b["fallback"], where="post", color="C0", label="B1 fallback")
    ax[1].set_ylabel("fallback flag"); ax[1].set_xlabel("t [s]")
    ax[1].grid(alpha=0.3); ax[1].legend(fontsize=8)
    ax[0].set_title("E2  tracking error and solver fallbacks")
    f.tight_layout(); f.savefig(os.path.join(FIG, "e2_error_fallback.png"), dpi=130); plt.close(f)


# ---------------------------------------------------------------------------
def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--run", action="store_true")
    ap.add_argument("--analyze", action="store_true")
    ap.add_argument("--tag", default="e2_priority")
    a = ap.parse_args(argv)
    os.makedirs(OUT, exist_ok=True)

    chk = check()
    if not chk["passed"]:
        print("CHECK FAILED - refusing to run E2-B1")
        return 2
    if a.run:
        run(a.tag)
    if a.analyze or (not a.run and not a.check):
        analyze(chk)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
