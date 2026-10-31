"""Analysis + figures for the pre-manoeuvre migration mechanism experiment.

Groups (configs/torque_gap_prediction_mechanism/cases):
  A  A_full_horizon          23.2 s horizon, 30 deg pitch manoeuvre at t = 110 s
  B  B_short_horizon          5.0 s horizon, same manoeuvre
  C  C_no_future_maneuver    23.2 s horizon, manoeuvre removed (delta_deg = 0)

Outputs
  results/torque_gap_prediction_mechanism/summary.csv / summary.md
  figures/torque_gap_prediction_mechanism/<group>_overview.png/.pdf
  figures/torque_gap_prediction_mechanism/mechanism_comparison.png/.pdf

python scripts/analyze_prediction_mechanism.py
"""
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from nearspace.cmg import CMGParams, CMGCluster                     # noqa: E402
from nearspace.config import load_config                            # noqa: E402
from nearspace.model6 import quat_to_euler                          # noqa: E402

RES = ROOT / "results" / "torque_gap_prediction_mechanism"
FIG = ROOT / "figures" / "torque_gap_prediction_mechanism"
GROUPS = {
    "A": dict(dir="A_full_horizon", title="A 完整时域（23.2 s，含未来机动）"),
    "B": dict(dir="B_short_horizon", title="B 短时域（5.0 s，含未来机动）"),
    "C": dict(dir="C_no_future_maneuver", title="C 完整时域，无未来机动"),
}
T_MAN = 110.0
T_MAN_C_IN = 105.0          # manoeuvre first visible in the 5.0 s window
DT = 0.2

plt = None


def _mpl():
    global plt
    if plt is not None:
        return plt
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as m
    m.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["Microsoft YaHei", "SimHei", "DejaVu Sans"],
        "axes.unicode_minus": False, "mathtext.fontset": "dejavusans",
        "font.size": 9, "axes.labelsize": 9, "axes.titlesize": 10,
        "xtick.labelsize": 8, "ytick.labelsize": 8, "legend.fontsize": 7.0,
        "axes.linewidth": 0.8, "grid.linewidth": 0.5, "grid.alpha": 0.35,
        "pdf.fonttype": 42, "ps.fonttype": 42, "savefig.bbox": "tight",
    })
    plt = m
    return plt


def load(tag):
    run = RES / GROUPS[tag]["dir"]
    with open(run / "timeseries.csv", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    d = {}
    for k in rows[0]:
        try:
            d[k] = np.array([float(r[k]) for r in rows])
        except ValueError:
            d[k] = np.array([r[k] for r in rows])
    cfg = load_config(str(run / "config_resolved.yaml"))
    prm = CMGParams.from_config(cfg)
    return dict(
        tag=tag, run=run, d=d, prm=prm, cluster=CMGCluster(prm),
        metrics=json.loads((run / "metrics.json").read_text(encoding="utf-8")),
        meta=json.loads((run / "meta.json").read_text(encoding="utf-8")),
        cfg=cfg,
    )


def pitch_deg(d):
    q = np.column_stack([d["q0"], d["q1"], d["q2"], d["q3"]])
    ref = np.column_stack([d["qref0"], d["qref1"], d["qref2"], d["qref3"]])
    f = lambda Q: np.array([np.rad2deg(quat_to_euler(Q[i])[1]) for i in range(len(Q))])
    return f(q), f(ref)


def hy_bound(g):
    """Exact attainable |H_y| over the gimbal torus (separable in the units)."""
    cl, prm = g["cluster"], g["prm"]
    amp = np.sqrt(cl.s0[1, :] ** 2 + cl.gxs0[1, :] ** 2)
    return float(prm.h0 * amp.sum())


def _f(v):
    """float(v) but None-safe: metric fields are None when a window is absent."""
    return None if v is None else float(v)


def stats(g):
    d, m, prm = g["d"], g["metrics"], g["prm"]
    t = d["t"]
    win = m.get("window") or [T_MAN, 130.0]
    man = (t >= win[0]) & (t <= win[1])
    pre = t < T_MAN
    gamma = np.column_stack([d["g%d" % (i + 1)] for i in range(4)])
    nu = np.column_stack([d["nu%d" % (i + 1)] for i in range(4)])
    H = np.column_stack([d["Hx"], d["Hy"], d["Hz"]])
    Hp = hy_bound(g)
    k110 = int(np.argmin(np.abs(t - T_MAN)))
    # first time the configuration leaves the home neighbourhood (10 deg)
    off = np.max(np.abs(gamma - gamma[0]), axis=1)
    ons = np.flatnonzero(off > np.deg2rad(10.0))
    onset = float(t[ons[0]]) if ons.size else None
    hi, lo = d["hi_y"], d["lo_y"]
    tol = 1e-6 * np.maximum(1.0, np.abs(hi - lo))
    on_bound = ((hi - d["tau_act_y"]) <= tol) | ((d["tau_act_y"] - lo) <= tol)
    opp = float(np.mean(np.sign(d["tau_act_y"][pre]) *
                        np.sign(d["cmg_y"][pre]) < 0)) if pre.any() else float("nan")
    out = dict(
        tag=g["tag"],
        horizon_s=float(g["meta"]["experiment"]["horizon_s"]),
        has_maneuver=bool(m.get("window")),
        migration_onset_t=onset,
        migration_onset_logged=_f(m.get("migration_onset_t")),
        migration_lead_s=(T_MAN - onset) if onset is not None else None,
        first_visible_t=max(float(m["t_start"]),
                            T_MAN - float(g["meta"]["experiment"]["horizon_s"])),
        gamma_at_110_deg=[float(np.rad2deg(x)) for x in gamma[k110]],
        max_gamma_dev_pre_deg=float(np.rad2deg(off[pre].max())),
        gimbal_travel_pre_rad=float(np.trapezoid(np.abs(nu[pre]).max(axis=1), t[pre])),
        Hy_bound=Hp,
        H_at_110=[float(x) for x in H[k110]],
        Hnorm_at_110=float(np.linalg.norm(H[k110])),
        Hy_at_110_norm=float((H[k110, 1] + Hp) / (2 * Hp)),
        Hy_max=float(H[:, 1].max()), Hy_min=float(H[:, 1].min()),
        Hnorm_max=_f(m["H_norm_max"]),
        rmse_all=_f(m["rmse_deg"]["all"]),
        rmse_man=_f(m["rmse_deg"]["maneuver"]) if m.get("window") else None,
        max_man=_f(m["max_err_deg"]["maneuver"]) if m.get("window") else None,
        max_post=_f(m["max_err_deg"]["post"]),
        rate_rmse_man=(float(np.sqrt(np.mean(d["rate_err"][man] ** 2)))
                       if man.any() and "rate_err" in d else None),
        pre_act_y=float(np.mean(np.abs(d["tau_act_y"][pre]))),
        pre_cmg_y=float(np.mean(d["cmg_y"][pre])),
        pre_sum_y=float(np.mean((d["tau_act_y"] + d["cmg_y"])[pre])),
        pre_opposite=opp,
        pre_nu_rms=float(np.sqrt(np.mean(nu[pre] ** 2))),
        pre_bound_frac=float(np.mean(on_bound[pre])),
        man_act_y=(float(np.mean(np.abs(d["tau_act_y"][man]))) if man.any() else None),
        man_cmg_y=(float(np.mean(d["cmg_y"][man])) if man.any() else None),
        man_nu_rms=(float(np.sqrt(np.mean(nu[man] ** 2))) if man.any() else None),
        man_nu_sat=(float(np.mean(np.abs(nu[man]) >= 0.999 * prm.gdot_max))
                    if man.any() else None),
        man_bound_frac=(float(np.mean(on_bound[man])) if man.any() else None),
        fallback=_f(m["fallback_ratio"]),
        cpu_mean=_f(m["cpu_s"]["mean"]), cpu_p95=_f(m["cpu_s"]["p95"]),
        cpu_max=_f(m["cpu_s"]["max"]),
        iters_mean=_f(m["iterations"]["mean"]),
        ok_frac=float(np.mean(d["status"] == "Solve_Succeeded")),
    )
    return out


def plot_group(g, st):
    p = _mpl()
    d, prm = g["d"], g["prm"]
    t = d["t"]
    ts = t + DT
    pitch, pref = pitch_deg(d)
    gamma = np.column_stack([d["g%d" % (i + 1)] for i in range(4)])
    nu = np.column_stack([d["nu%d" % (i + 1)] for i in range(4)])
    Hp = st["Hy_bound"]

    fig, axes = p.subplots(6, 1, figsize=(7.0, 12.4), sharex=True,
                           gridspec_kw=dict(hspace=0.13))
    c_g = ["tab:blue", "tab:orange", "tab:green", "tab:red"]

    ax = axes[0]
    ax.plot(t, pref, color="0.35", lw=2.6, alpha=0.55,
            label=r"参考 $\theta_{\mathrm{ref}}$")
    ax.plot(ts, pitch, color="tab:red", lw=1.3, label=r"实际 $\theta$")
    ax.set_ylabel("俯仰角 [deg]")
    ax.set_ylim(0, 34)
    ax.legend(loc="upper left", ncol=2, framealpha=0.9, handlelength=1.5)

    ax = axes[1]
    ax.plot(ts, d["att_err_deg"], color="tab:red", lw=1.3)
    ax.set_yscale("log")
    ax.set_ylim(1e-2, 30)
    ax.set_ylabel("姿态误差 [deg]")

    ax = axes[2]
    for i in range(4):
        ax.plot(ts, np.rad2deg(gamma[:, i]), color=c_g[i], lw=1.1,
                label=r"$\gamma_%d$" % (i + 1))
    ax.set_ylabel("框架角 [deg]")
    ax.legend(loc="upper left", ncol=4, framealpha=0.9, handlelength=1.2,
              columnspacing=0.8)

    ax = axes[3]
    ax.axhspan(-Hp, Hp, color="0.8", alpha=0.35, lw=0)
    for yv in (Hp, -Hp):
        ax.axhline(yv, color="0.45", ls=":", lw=0.9)
    ax.plot(ts, d["Hy"], color="tab:blue", lw=1.5, label=r"$H_y$")
    ax.axhline(0.0, color="k", lw=0.5, alpha=0.5)
    ax.set_ylabel("$H_y$ [N$\\cdot$m$\\cdot$s]")
    ax.set_ylim(-Hp * 1.3, Hp * 1.3)
    ax.legend(loc="lower left", ncol=2, framealpha=0.9, handlelength=1.1)

    ax = axes[4]
    ax.plot(t, d["tau_nom_y"], color="tab:blue", ls="--", lw=1.0,
            label=r"$\tau_{\mathrm{nom},y}$")
    ax.plot(t, d["tau_act_y"], color="tab:green", lw=1.2, label=r"$\tau_{a,y}$（舵面）")
    ax.plot(ts, d["cmg_y"], color="tab:red", lw=1.2, label=r"$\tau_{\mathrm{cmg},y}$")
    ax.plot(t, d["tau_act_y"] + d["cmg_y"], color="0.2", lw=1.4, alpha=0.8,
            label="合成")
    ax.axhline(0.0, color="k", lw=0.5, alpha=0.5)
    ax.set_ylabel("俯仰力矩 [N$\\cdot$m]")
    ax.set_ylim(-110, 130)
    ax.legend(loc="upper left", ncol=4, framealpha=0.9, handlelength=1.3,
              columnspacing=0.8)

    ax = axes[5]
    for i in range(4):
        ax.plot(t, nu[:, i], color=c_g[i], lw=1.0, label=r"$\nu_%d$" % (i + 1))
    for sgn in (1, -1):
        ax.axhline(sgn * prm.gdot_max, color="0.4", ls=":", lw=0.9)
    ax.set_ylabel("框架角速度 [rad/s]")
    ax.set_xlabel("时间 $t$ [s]")
    ax.legend(loc="upper left", ncol=4, framealpha=0.9, handlelength=1.2,
              columnspacing=0.8)
    ax.set_ylim(-1.15, 1.15)

    for ax in axes:
        ax.axvline(T_MAN, color="k", ls="--", lw=1.0)
        if g["tag"] == "B":
            ax.axvline(T_MAN_C_IN, color="tab:purple", ls="-.", lw=1.0)
        if st["migration_onset_t"] is not None:
            ax.axvline(st["migration_onset_t"], color="tab:orange", ls=":", lw=1.0)
        ax.grid(True)
        ax.set_xlim(90.0, 132.0)
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)
    note = ("黑虚线 = 机动开始 110 s；橙点线 = 迁移起点 %s；"
            % ("%.1f s" % st["migration_onset_t"] if st["migration_onset_t"] else "无"))
    if g["tag"] == "B":
        note += "紫点划线 = 机动首次进入 5 s 窗口 105 s"
    axes[0].set_title("%s\n%s" % (GROUPS[g["tag"]]["title"], note),
                      fontsize=9.5, pad=6)

    FIG.mkdir(parents=True, exist_ok=True)
    png = FIG / ("%s_overview.png" % g["tag"])
    pdf = FIG / ("%s_overview.pdf" % g["tag"])
    fig.savefig(png, dpi=300)
    fig.savefig(pdf)
    p.close(fig)
    return png, pdf


def plot_compare(gs, sts):
    p = _mpl()
    fig, axes = p.subplots(3, 1, figsize=(7.0, 8.2), sharex=True,
                           gridspec_kw=dict(hspace=0.14))
    cols = {"A": "tab:blue", "B": "tab:red", "C": "tab:green"}

    ax = axes[0]
    for tag in ("A", "B", "C"):
        d = gs[tag]["d"]
        ax.plot(d["t"] + DT, d["Hy"], color=cols[tag], lw=1.5 if tag != "C" else 1.2,
                label=GROUPS[tag]["title"])
        ax.axhline(ts := sts[tag]["Hy_bound"], color="0.5", ls=":", lw=0.8)
        ax.axhline(-ts, color="0.5", ls=":", lw=0.8)
    ax.axhline(0.0, color="k", lw=0.5, alpha=0.5)
    ax.set_ylabel("$H_y$ [N$\\cdot$m$\\cdot$s]")
    ax.legend(loc="lower left", ncol=3, framealpha=0.9, handlelength=1.3)
    ax.set_ylim(-215, 215)

    ax = axes[1]
    for tag in ("A", "B", "C"):
        d = gs[tag]["d"]
        gamma = np.column_stack([d["g%d" % (i + 1)] for i in range(4)])
        ax.plot(d["t"] + DT, np.rad2deg(np.max(np.abs(gamma - gamma[0]), axis=1)),
                color=cols[tag], lw=1.5 if tag != "C" else 1.2,
                label=GROUPS[tag]["title"])
    ax.set_ylabel("构型偏离起始角 $\\max_i|\\gamma_i-\\gamma_{i,0}|$ [deg]")
    ax.legend(loc="upper left", ncol=3, framealpha=0.9, handlelength=1.3)

    ax = axes[2]
    for tag in ("A", "B", "C"):
        d = gs[tag]["d"]
        ax.plot(d["t"] + DT, d["att_err_deg"], color=cols[tag],
                lw=1.5 if tag != "C" else 1.2, label=GROUPS[tag]["title"])
    ax.set_yscale("log")
    ax.set_ylim(1e-2, 30)
    ax.set_ylabel("姿态误差 [deg]")
    ax.set_xlabel("时间 $t$ [s]")

    for ax in axes:
        ax.axvline(T_MAN, color="k", ls="--", lw=1.0)
        ax.axvline(T_MAN_C_IN, color="tab:purple", ls="-.", lw=0.9, alpha=0.8)
        ax.grid(True)
        ax.set_xlim(88.0, 135.0)
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)
    axes[0].set_title("A/B/C 对照：黑虚线 = 机动开始 110 s，紫点划线 = 机动首次进入 B 的 5 s 窗口 105 s",
                      fontsize=9, pad=6)
    FIG.mkdir(parents=True, exist_ok=True)
    png = FIG / "mechanism_comparison.png"
    pdf = FIG / "mechanism_comparison.pdf"
    fig.savefig(png, dpi=300)
    fig.savefig(pdf)
    p.close(fig)
    return png, pdf


def write_tables(sts):
    keys = list(sts["A"].keys())
    with open(RES / "summary.csv", "w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["metric"] + ["%s" % t for t in ("A", "B", "C")])
        for k in keys:
            if k == "tag":
                continue
            row = [k]
            for t in ("A", "B", "C"):
                v = sts[t][k]
                if isinstance(v, list):
                    v = "[" + ", ".join("%.3f" % x for x in v) + "]"
                elif isinstance(v, float):
                    v = "%.4f" % v
                row.append("" if v is None else v)
            w.writerow(row)
    f = lambda v, fmt="%.3f": "—" if v is None else (fmt % v)
    md = ["# A/B/C 指标汇总（阶段 4）", "",
          "| 指标 | A 完整时域 | B 短时域 5 s | C 无未来机动 |", "|---|---|---|---|",
          "| 预测时域 [s] | %s | %s | %s |" % tuple(
              f(sts[t]["horizon_s"], "%.1f") for t in ("A", "B", "C")),
          "| 是否执行主机动 | %s | %s | %s |" % tuple(
              "是" if sts[t]["has_maneuver"] else "否" for t in ("A", "B", "C")),
          "", "## 5.1 提前迁移",
          "| 迁移起点（本次分析判据 $\\max_i|\\gamma_i-\\gamma_{i,0}|>10°$）[s] | %s | %s | %s |"
          % tuple(f(sts[t]["migration_onset_t"], "%.1f") for t in ("A", "B", "C")),
          "| 迁移起点（运行日志 `migration_onset_t`）[s] | %s | %s | %s |"
          % tuple(f(sts[t]["migration_onset_logged"], "%.1f") for t in ("A", "B", "C")),
          "| 相对机动的提前量 [s] | %s | %s | %s |" % tuple(
              f(sts[t]["migration_lead_s"]) for t in ("A", "B", "C")),
          "| 机动首次进入预测窗口 [s] | %s | %s | %s |" % tuple(
              f(sts[t]["first_visible_t"], "%.1f") for t in ("A", "B", "C")),
          "| 机动前构型最大偏离 [deg] | %s | %s | %s |" % tuple(
              f(sts[t]["max_gamma_dev_pre_deg"]) for t in ("A", "B", "C")),
          "| 机动前框架行程 [rad] | %s | %s | %s |" % tuple(
              f(sts[t]["gimbal_travel_pre_rad"]) for t in ("A", "B", "C")),
          "", "## 5.2 角动量预置（t = 110 s）",
          "| $H_x$ [N·m·s] | %s | %s | %s |" % tuple(
              f(sts[t]["H_at_110"][0]) for t in ("A", "B", "C")),
          "| $H_y$ [N·m·s] | %s | %s | %s |" % tuple(
              f(sts[t]["H_at_110"][1]) for t in ("A", "B", "C")),
          "| $H_z$ [N·m·s] | %s | %s | %s |" % tuple(
              f(sts[t]["H_at_110"][2]) for t in ("A", "B", "C")),
          "| $|H|$ [N·m·s] | %s | %s | %s |" % tuple(
              f(sts[t]["Hnorm_at_110"]) for t in ("A", "B", "C")),
          "| $H_y$ 归一化位置（0=下界,1=上界） | %s | %s | %s |" % tuple(
              f(sts[t]["Hy_at_110_norm"], "%.2f") for t in ("A", "B", "C")),
          "| 全程 $H_y$ 区间 [N·m·s] | %s | %s | %s |" % tuple(
              "[%s, %s]" % (f(sts[t]["Hy_min"], "%.1f"), f(sts[t]["Hy_max"], "%.1f"))
              for t in ("A", "B", "C")),
          "| $|H|$ 峰值 [N·m·s] | %s | %s | %s |" % tuple(
              f(sts[t]["Hnorm_max"], "%.1f") for t in ("A", "B", "C")),
          "", "## 5.3 姿态跟踪",
          "| 全程 RMSE [deg] | %s | %s | %s |" % tuple(
              f(sts[t]["rmse_all"]) for t in ("A", "B", "C")),
          "| 机动窗口 RMSE [deg] | %s | %s | %s |" % tuple(
              f(sts[t]["rmse_man"]) for t in ("A", "B", "C")),
          "| 机动窗口最大误差 [deg] | %s | %s | %s |" % tuple(
              f(sts[t]["max_man"]) for t in ("A", "B", "C")),
          "| 窗口后最大误差 [deg] | %s | %s | %s |" % tuple(
              f(sts[t]["max_post"]) for t in ("A", "B", "C")),
          "| 机动窗口角速率 RMSE [rad/s] | %s | %s | %s |" % tuple(
              f(sts[t]["rate_rmse_man"], "%.4f") for t in ("A", "B", "C")),
          "", "## 5.4 执行器分工（准备段 90–110 s / 机动段 110–130 s）",
          "| 准备段 $\\overline{|\\tau_{a,y}|}$ [N·m] | %s | %s | %s |" % tuple(
              f(sts[t]["pre_act_y"]) for t in ("A", "B", "C")),
          "| 准备段 $\\overline{\\tau_{cmg,y}}$ [N·m] | %s | %s | %s |" % tuple(
              f(sts[t]["pre_cmg_y"]) for t in ("A", "B", "C")),
          "| 准备段合成为力矩 [N·m] | %s | %s | %s |" % tuple(
              f(sts[t]["pre_sum_y"]) for t in ("A", "B", "C")),
          "| 准备段舵面/CMG 反号比例 | %s | %s | %s |" % tuple(
              f(sts[t]["pre_opposite"], "%.2f") for t in ("A", "B", "C")),
          "| 准备段 $\\nu$ RMS [rad/s] | %s | %s | %s |" % tuple(
              f(sts[t]["pre_nu_rms"], "%.4f") for t in ("A", "B", "C")),
          "| 准备段舵面贴边比例 | %s | %s | %s |" % tuple(
              f(sts[t]["pre_bound_frac"], "%.2f") for t in ("A", "B", "C")),
          "| 机动段 $\\overline{\\tau_{cmg,y}}$ [N·m] | %s | %s | %s |" % tuple(
              f(sts[t]["man_cmg_y"]) for t in ("A", "B", "C")),
          "| 机动段 $\\nu$ RMS [rad/s] | %s | %s | %s |" % tuple(
              f(sts[t]["man_nu_rms"], "%.4f") for t in ("A", "B", "C")),
          "| 机动段 $\\nu$ 饱和比例 | %s | %s | %s |" % tuple(
              f(sts[t]["man_nu_sat"], "%.3f") for t in ("A", "B", "C")),
          "", "## 5.5 求解器",
          "| 回退比例 | %s | %s | %s |" % tuple(
              f(sts[t]["fallback"], "%.3f") for t in ("A", "B", "C")),
          "| `Solve_Succeeded` 占比 | %s | %s | %s |" % tuple(
              f(sts[t]["ok_frac"], "%.3f") for t in ("A", "B", "C")),
          "| 单步耗时 均值/P95/最大 [s] | %s | %s | %s |" % tuple(
              "%s / %s / %s" % (f(sts[t]["cpu_mean"], "%.2f"),
                                f(sts[t]["cpu_p95"], "%.2f"),
                                f(sts[t]["cpu_max"], "%.2f"))
              for t in ("A", "B", "C")),
          "| 平均迭代次数 | %s | %s | %s |" % tuple(
              f(sts[t]["iters_mean"], "%.1f") for t in ("A", "B", "C")),
          ""]
    (RES / "summary.md").write_text("\n".join(md) + "\n", encoding="utf-8")
    return RES / "summary.md"


def main():
    try:
        sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    except Exception:                                       # noqa: BLE001
        pass
    gs, sts = {}, {}
    for tag in ("A", "B", "C"):
        gs[tag] = load(tag)
        sts[tag] = stats(gs[tag])
    for tag in ("A", "B", "C"):
        png, _ = plot_group(gs[tag], sts[tag])
        print("figure:", png)
    png, _ = plot_compare(gs, sts)
    print("figure:", png)
    print("table:", write_tables(sts))
    for tag in ("A", "B", "C"):
        s = sts[tag]
        print("%s: onset=%s lead=%s Hy(110)=%.1f |H|max=%.1f rmse_man=%s fb=%.3f"
              % (tag, ("%.1f" % s["migration_onset_t"]) if s["migration_onset_t"] else "—",
                 ("%.1f" % s["migration_lead_s"]) if s["migration_lead_s"] else "—",
                 s["H_at_110"][1], s["Hnorm_max"],
                 ("%.3f" % s["rmse_man"]) if s["rmse_man"] else "—", s["fallback"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
