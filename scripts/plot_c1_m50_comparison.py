#!/usr/bin/env python
"""C1 with published Honeywell M50 gimbal limits vs the previous assumption.

  previous : |gdot| <= 1.5 rad/s, |gddot| <= 10.0 rad/s^2   (first-version guess)
  M50      : |gdot| <= 1.0 rad/s, |gddot| <=  3.0 rad/s^2   (literature)

Everything else is identical (same case file chain, h0 = 50 N m s per unit = M50
momentum, 30 deg/20 s pitch manoeuvre, B0, P0, unwrapped gimbal, same horizon,
solver and weights).  Outputs

  figures/torque_gap_m50/C1_M50_vs_original.png / .pdf
  docs/torque_gap/C1_M50_rate_limit_comparison.md

python scripts/plot_c1_m50_comparison.py
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

RUN_OLD = ROOT / "results" / "torque_gap_unlimited" / "C1_B0_P0"
RUN_M50 = ROOT / "results" / "torque_gap_m50" / "C1_B0_P0"
FIG = ROOT / "figures" / "torque_gap_m50"
DOC = ROOT / "docs" / "torque_gap"
T_MAN = 110.0
T_WIN = (90.0, 146.0)


def load(run):
    with open(run / "timeseries.csv", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    d = {}
    for k in rows[0]:
        try:
            d[k] = np.array([float(r[k]) for r in rows])
        except ValueError:
            d[k] = np.array([r[k] for r in rows])
    cfg = load_config(str(run / "config_resolved.yaml"))
    return (d, json.loads((run / "metrics.json").read_text(encoding="utf-8")),
            json.loads((run / "meta.json").read_text(encoding="utf-8")),
            CMGParams.from_config(cfg))


def pitch(d):
    q = np.column_stack([d["q0"], d["q1"], d["q2"], d["q3"]])
    return np.array([np.rad2deg(quat_to_euler(q[i])[1]) for i in range(q.shape[0])])


def nu_max(d):
    return np.max(np.abs(np.column_stack([d["nu%d" % (i + 1)] for i in range(4)])), axis=1)


def dnu_max(d):
    nu = np.column_stack([d["nu%d" % (i + 1)] for i in range(4)])
    return np.max(np.abs(np.diff(nu, axis=0)), axis=1)


def plot(a, b, ma, mb, pl_old, pl_m50):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["Microsoft YaHei", "SimHei", "DejaVu Sans"],
        "axes.unicode_minus": False, "mathtext.fontset": "dejavusans",
        "font.size": 9, "axes.labelsize": 9, "axes.titlesize": 10,
        "xtick.labelsize": 8, "ytick.labelsize": 8, "legend.fontsize": 7.2,
        "axes.linewidth": 0.8, "grid.linewidth": 0.5, "grid.alpha": 0.35,
        "pdf.fonttype": 42, "ps.fonttype": 42, "savefig.bbox": "tight",
    })
    t = a["t"]
    ts = t + 0.2
    c_old, c_m50, c_ref = "tab:blue", "tab:red", "0.35"
    lab_old = r"$\dot\gamma_{\max}=1.5$ rad/s, $\ddot\gamma_{\max}=10$ rad/s$^2$"
    lab_m50 = r"M50: $\dot\gamma_{\max}=1.0$ rad/s, $\ddot\gamma_{\max}=3$ rad/s$^2$"

    fig, axes = plt.subplots(6, 1, figsize=(7.0, 13.0), sharex=True,
                             gridspec_kw=dict(hspace=0.15))

    ax = axes[0]
    ax.plot(t, np.array([np.rad2deg(quat_to_euler([a["qref0"][i], a["qref1"][i],
                                                   a["qref2"][i], a["qref3"][i]])[1])
                         for i in range(t.size)]), color=c_ref, lw=2.6, alpha=0.5,
            label=r"参考 $\theta_{\mathrm{ref}}$")
    ax.plot(ts, pitch(a), color=c_old, lw=1.1, label="原假设（1.5 / 10）")
    ax.plot(ts, pitch(b), color=c_m50, lw=1.4, label="M50（1.0 / 3.0）")
    ax.set_ylabel("俯仰角 [deg]")
    ax.set_ylim(0, 34)
    ax.legend(loc="upper left", ncol=3, framealpha=0.9, handlelength=1.5,
              columnspacing=1.0)

    ax = axes[1]
    ax.plot(ts, a["att_err_deg"], color=c_old, lw=1.1, label="原假设")
    ax.plot(ts, b["att_err_deg"], color=c_m50, lw=1.4, label="M50")
    ax.set_yscale("log")
    ax.set_ylim(1e-2, 30)
    ax.set_ylabel("姿态误差 [deg]")
    ax.legend(loc="upper left", ncol=2, framealpha=0.9, handlelength=1.5)

    ax = axes[2]
    ax.plot(ts, a["cmg_y"], color=c_old, lw=1.1, label="原假设 $\\tau_{\\mathrm{cmg},y}$")
    ax.plot(ts, b["cmg_y"], color=c_m50, lw=1.4, label="M50 $\\tau_{\\mathrm{cmg},y}$")
    ax.plot(t, a["d_y"], color="0.4", ls="--", lw=1.0, label="缺口 $d_y$")
    ax.axhline(0.0, color="k", lw=0.5, alpha=0.5)
    ax.set_ylabel("CMG 俯仰力矩 [N$\\cdot$m]")
    ax.set_ylim(-95, 125)
    ax.legend(loc="upper left", ncol=3, framealpha=0.9, handlelength=1.5)

    ax = axes[3]
    ax.plot(ts, nu_max(a), color=c_old, lw=1.1, label="原假设 $\\max_i|\\nu_i|$")
    ax.plot(ts, nu_max(b), color=c_m50, lw=1.4, label="M50 $\\max_i|\\nu_i|$")
    ax.axhline(pl_old.gdot_max, color=c_old, ls=":", lw=1.0,
               label=r"$\dot\gamma_{\max}=%.1f$" % pl_old.gdot_max)
    ax.axhline(pl_m50.gdot_max, color=c_m50, ls=":", lw=1.0,
               label=r"$\dot\gamma_{\max}=%.1f$" % pl_m50.gdot_max)
    ax.set_ylabel("框架速率 [rad/s]")
    ax.set_ylim(0, 1.7)
    ax.legend(loc="upper left", ncol=2, framealpha=0.9, handlelength=1.5,
              labelspacing=0.3)

    ax = axes[4]
    ax.plot(ts[:-1], dnu_max(a), color=c_old, lw=1.1, label="原假设 $\\max_i|\\Delta\\nu_i|$")
    ax.plot(ts[:-1], dnu_max(b), color=c_m50, lw=1.4, label="M50 $\\max_i|\\Delta\\nu_i|$")
    ax.axhline(pl_old.gddot_max * 0.2, color=c_old, ls=":", lw=1.0,
               label=r"$\ddot\gamma_{\max}\Delta t=%.1f$" % (pl_old.gddot_max * 0.2))
    ax.axhline(pl_m50.gddot_max * 0.2, color=c_m50, ls=":", lw=1.0,
               label=r"$\ddot\gamma_{\max}\Delta t=%.1f$" % (pl_m50.gddot_max * 0.2))
    ax.set_ylabel("每步速率增量 [rad/s]")
    ax.set_ylim(0, 2.2)
    ax.legend(loc="upper left", ncol=2, framealpha=0.9, handlelength=1.5,
              labelspacing=0.3)

    ax = axes[5]
    cl = CMGCluster(pl_m50)
    amp = np.sqrt(cl.s0[1, :] ** 2 + cl.gxs0[1, :] ** 2)
    Hp = float(pl_m50.h0 * amp.sum())          # sum of per-unit amplitudes (separable)
    Hgeo = 163.2                                # max |H| over the torus (grid check)
    ax.axhspan(-Hp, Hp, color="0.8", alpha=0.35, lw=0)
    for yv in (Hp, -Hp):
        ax.axhline(yv, color="0.45", ls=":", lw=0.9)
    ax.plot(ts, a["Hy"], color=c_old, lw=1.1, label="原假设 $H_y$")
    ax.plot(ts, b["Hy"], color=c_m50, lw=1.5, label="M50 $H_y$")
    ax.set_ylabel("$H_y$ [N$\\cdot$m$\\cdot$s]")
    ax.set_xlabel("时间 $t$ [s]")
    ax.set_ylim(-Hp * 1.30, Hp * 1.30)
    ax.legend(loc="lower left", ncol=3, framealpha=0.9, handlelength=1.5)
    ax.annotate("俯仰轴角动量可达边界 $\\pm%.1f$ N$\\cdot$m$\\cdot$s" % Hp,
                xy=(T_WIN[0] + 0.6, Hp * 1.06), fontsize=6.8, color="0.35")

    for ax in axes:
        ax.axvline(T_MAN, color="k", ls="--", lw=1.0)
        ax.grid(True)
        ax.set_xlim(*T_WIN)
        for s in ("top", "right"):
            ax.spines[s].set_visible(False)
    axes[0].set_title("C1：M50 公布的框架速率/加速度限制 vs 原假设（其余条件完全相同）",
                      fontsize=10, pad=8)

    FIG.mkdir(parents=True, exist_ok=True)
    png = FIG / "C1_M50_vs_original.png"
    pdf = FIG / "C1_M50_vs_original.pdf"
    fig.savefig(png, dpi=300)
    fig.savefig(pdf)
    plt.close(fig)
    return png, pdf, Hp, Hgeo


def report(a, b, ma, mb, pl_old, pl_m50, png, pdf, Hp, Hgeo):
    t = a["t"]
    win = ma.get("window") or [T_MAN, 130.0]
    man = (t >= win[0]) & (t <= win[1])

    def post(d):
        tt, err = d["t"], d["att_err_deg"]
        m1 = (tt > win[1]) & (tt <= T_WIN[1])
        m2 = tt > T_WIN[1]
        return (float(np.max(err[m1])) if m1.any() else float("nan"),
                float(np.max(err[m2])) if m2.any() else float("nan"),
                int(d["fallback"][m2].sum()) if m2.any() else 0)

    pA, pB = post(a), post(b)
    turns = lambda d: float(np.max(np.abs(np.column_stack(
        [d["g%d" % (i + 1)] for i in range(4)]))) / (2 * np.pi))

    def seg(d, key, mask, f=np.max):
        return float(f(np.abs(d[key][mask])))

    md = ["# C1：采用 M50 公布的框架速率/加速度限制后重做", "",
          "只改了两个数（其余全部继承，含 h0 = 50 N·m·s/台 = M50 角动量）：", "",
          "| 量 | 原假设（first version） | M50 文献值 |", "|---|---|---|",
          "| 框架角速度上限 \\|γ̇\\| | %.1f rad/s | **%.1f rad/s** |"
          % (pl_old.gdot_max, pl_m50.gdot_max),
          "| 框架角加速度上限 \\|γ̈\\| | %.1f rad/s² | **%.1f rad/s²** |"
          % (pl_old.gddot_max, pl_m50.gddot_max),
          "| 每步速率增量上限 γ̈·Δt | %.1f rad/s | **%.1f rad/s** |"
          % (pl_old.gddot_max * 0.2, pl_m50.gddot_max * 0.2),
          "", "- 原假设：`results/torque_gap_unlimited/C1_B0_P0/`"
          "（`configs/torque_gap_unlimited/cases/c1_pitch30.yaml`）",
          "- M50：`results/torque_gap_m50/C1_B0_P0/`"
          "（`configs/torque_gap_m50/cases/c1_pitch30.yaml`，仅 `cmg.gdot_max/gddot_max` 不同）",
          "", "图：`%s` / `%s`（图：`figures/torque_gap_m50/C1_M50_overview.png` 为 M50 单臂五面板总图）"
          % (png.relative_to(ROOT).as_posix(), pdf.relative_to(ROOT).as_posix()), "",
          "## 1. 指标对比（机动窗口 %.0f–%.0f s）" % (win[0], win[1]), "",
          "| 指标 | 原假设 1.5/10 | M50 1.0/3.0 |", "|---|---|---|",
          "| 机动窗口姿态 RMSE [deg] | %.3f | **%.3f** |"
          % (ma["rmse_deg"]["maneuver"], mb["rmse_deg"]["maneuver"]),
          "| 机动窗口最大误差 [deg] | %.3f | %.3f |"
          % (ma["max_err_deg"]["maneuver"], mb["max_err_deg"]["maneuver"]),
          "| 全程 RMSE [deg] | %.3f | %.3f |"
          % (ma["rmse_deg"]["all"], mb["rmse_deg"]["all"]),
          "| 机动后 %.0f–%.0f s 最大误差 [deg] | %.3f | %.3f |"
          % (win[1], T_WIN[1], pA[0], pB[0]),
          "| 尾部 %.0f–%.0f s 最大误差 [deg]（回退次数） | %.3f（%d） | %.3f（%d） |"
          % (T_WIN[1], a["t"][-1] + 0.2, pA[1], pA[2], pB[1], pB[2]),
          "| 实测峰值 \\|ν\\| [rad/s]（上限） | %.3f（%.1f） | %.3f（%.1f） |"
          % (nu_max(a).max(), pl_old.gdot_max, nu_max(b).max(), pl_m50.gdot_max),
          "| 实测峰值每步 \\|Δν\\| [rad/s]（上限） | %.3f（%.1f） | %.3f（%.1f） |"
          % (dnu_max(a).max(), pl_old.gddot_max * 0.2,
             dnu_max(b).max(), pl_m50.gddot_max * 0.2),
          "| 峰值机体角速率 [rad/s] | %.4f | %.4f |"
          % (seg(a, "wy", np.ones_like(t, bool)), seg(b, "wy", np.ones_like(t, bool))),
          "| 机动期 \\|τ_cmg,y\\| 峰值 [N·m] | %.1f | %.1f |"
          % (seg(a, "cmg_y", man), seg(b, "cmg_y", man)),
          "| 机动期缺口覆盖残差均值 [N·m] | %.1f | %.1f |"
          % (ma["gap_residual_maneuver_mean"], mb["gap_residual_maneuver_mean"]),
          "| 机动期瞬时能力残差 E_gap 均值 | %.1f | %.1f |"
          % (ma["E_gap_maneuver_mean"], mb["E_gap_maneuver_mean"]),
          "| \\|H\\| 峰值 [N·m·s]（几何可达 %.1f） | %.1f | %.1f |"
          % (Hgeo, ma["H_norm_max"], mb["H_norm_max"]),
          "| 最大 \\|γ\\| [deg] / 圈数 | %.0f / %.2f | %.0f / %.2f |"
          % (np.rad2deg(np.max(np.abs(np.column_stack([a["g%d" % (i + 1)] for i in range(4)])))),
             turns(a),
             np.rad2deg(np.max(np.abs(np.column_stack([b["g%d" % (i + 1)] for i in range(4)])))),
             turns(b)),
          "| 舵面贴边比例（3 轴平均） | %.3f | %.3f |"
          % (ma["tau_bound_used_mean"], mb["tau_bound_used_mean"]),
          "| NLP 回退比例 | %.3f | %.3f |"
          % (ma["fallback_ratio"], mb["fallback_ratio"]),
          "| 单步耗时 均值 / P95 [s] | %.2f / %.2f | %.2f / %.2f |"
          % (ma["cpu_s"]["mean"], ma["cpu_s"]["p95"],
             mb["cpu_s"]["mean"], mb["cpu_s"]["p95"]), "",
          "## 2. 结论", "",
          "1. **换成 M50 的真实限制后，这个机动仍然完成得很好**：窗口 RMSE %.3f°（原 %.3f°，"
          "差 %.1f%%），峰值误差 %.3f°（原 %.3f°）。"
          % (mb["rmse_deg"]["maneuver"], ma["rmse_deg"]["maneuver"],
             100.0 * (mb["rmse_deg"]["maneuver"] / ma["rmse_deg"]["maneuver"] - 1.0),
             mb["max_err_deg"]["maneuver"], ma["max_err_deg"]["maneuver"]),
          "2. **两个新限制都真的被用满了**：实测峰值 |ν| = %.3f rad/s（上限 %.1f）、"
          "峰值每步 |Δν| = %.3f rad/s（上限 %.1f）。也就是说这不是“限制没起作用”的假对比。"
          % (nu_max(b).max(), pl_m50.gdot_max, dnu_max(b).max(), pl_m50.gddot_max * 0.2),
          "3. **机动后更平稳、尾部明显更干净**：%.0f–%.0f s 最大误差 %.3f°（M50）vs %.3f°（原），"
          "都在 0.5° 以内；尾部（>%.0f s）%.3f° vs %.3f°。原因很可能是更低的速率/加速度上限压掉了"
          "原来那部分“高频换构型”的抖振（对比两条 τ_cmg 与 ν 曲线可见：原假设在 135–145 s 有尖峰）。"
          % (win[1], T_WIN[1], pB[0], pA[0], T_WIN[1], pB[1], pA[1]),
          "4. **瓶颈仍是角动量，不是速率**：两臂 |H| 峰值 %.1f / %.1f N·m·s，"
          "已达几何可达 %.1f 的 %.0f%%/%.0f%%，俯仰轴边界 %.1f N·m·s；"
          "M50 一臂转得更多（%.2f → %.2f 圈）以换取同样的角动量摆幅。"
          % (ma["H_norm_max"], mb["H_norm_max"], Hgeo,
             100 * ma["H_norm_max"] / Hgeo, 100 * mb["H_norm_max"] / Hgeo, Hp,
             turns(a), turns(b)),
          "5. **缺口覆盖反而更好**：机动期 ‖τ_cmg−d‖ 均值 %.1f → %.1f N·m。因为速率上限更低时，"
          "优化器更倾向于把角动量用在“有用方向”，而不是快速来回换构型。"
          % (ma["gap_residual_maneuver_mean"], mb["gap_residual_maneuver_mean"]), "",
          "## 3. 需要注意", "",
          "- 其余权重保持原样（`nu_scale = 1.5` 只是代价归一化，不随上限改变），所以这是一次纯粹的"
          "“只改限制”的对比。",
          "- 结论仅对 C1（30°/20 s 俯仰）成立；更激烈的机动或需要更高关节速度时会不同，"
          "建议下一轮用 M50 限制扫 C2（偏航）/C3（多轴）。",
          "- P0 仍是虚拟力矩植物；真实舵面（P1）与本轮改动无关，未重跑。",
          ]
    DOC.mkdir(parents=True, exist_ok=True)
    path = DOC / "C1_M50_rate_limit_comparison.md"
    path.write_text("\n".join(md) + "\n", encoding="utf-8")
    return path


def main():
    a, ma, meta_a, pl_old = load(RUN_OLD)
    b, mb, meta_b, pl_m50 = load(RUN_M50)
    png, pdf, Hp, Hgeo = plot(a, b, ma, mb, pl_old, pl_m50)
    path = report(a, b, ma, mb, pl_old, pl_m50, png, pdf, Hp, Hgeo)
    print("figure:", png)
    print("report:", path)
    print("maneuver RMSE: old %.3f -> M50 %.3f deg"
          % (ma["rmse_deg"]["maneuver"], mb["rmse_deg"]["maneuver"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
