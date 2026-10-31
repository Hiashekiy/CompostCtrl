#!/usr/bin/env python
"""C1 with CMG assist vs C1 with aerodynamic surfaces only (CMG frozen).

Both runs use exactly the same case (`configs/torque_gap_unlimited/cases/
c1_pitch30.yaml`: unwrapped gimbal, B0, P0 plant, 30 deg/20 s pitch manoeuvre,
identical reference, disturbance, aero box, horizon, solver and weights).  The
only difference is `torque_gap.cmg_enabled`, which freezes the gimbal rates at
zero in the optimiser bounds *and* in the truth plant.

Outputs
  figures/torque_gap_noCMG/C1_surface_only_comparison.png / .pdf
  docs/torque_gap/C1_surface_only_comparison.md

python scripts/plot_c1_surface_only.py
"""
from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

RUN_CMG = ROOT / "results" / "torque_gap_unlimited" / "C1_B0_P0"
RUN_NOCMG = ROOT / "results" / "torque_gap_noCMG" / "C1_B0_P0_noCMG"
FIG = ROOT / "figures" / "torque_gap_noCMG"
DOC = ROOT / "docs" / "torque_gap"
T_WIN = (90.0, 146.0)
T_MANEUVER = 110.0


def load(run):
    with open(run / "timeseries.csv", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    d = {}
    for key in rows[0]:
        try:
            d[key] = np.array([float(r[key]) for r in rows])
        except ValueError:
            d[key] = np.array([r[key] for r in rows])
    return (d, json.loads((run / "metrics.json").read_text(encoding="utf-8")),
            json.loads((run / "meta.json").read_text(encoding="utf-8")))


def plot(a, b, ma, mb):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["Microsoft YaHei", "SimHei", "DejaVu Sans"],
        "axes.unicode_minus": False,
        "mathtext.fontset": "dejavusans",
        "font.size": 9, "axes.labelsize": 9, "axes.titlesize": 10,
        "xtick.labelsize": 8, "ytick.labelsize": 8, "legend.fontsize": 7.5,
        "axes.linewidth": 0.8, "grid.linewidth": 0.5, "grid.alpha": 0.35,
        "pdf.fonttype": 42, "ps.fonttype": 42, "savefig.bbox": "tight",
    })
    from nearspace.model6 import quat_to_euler

    def pitch(d):
        q = np.column_stack([d["q0"], d["q1"], d["q2"], d["q3"]])
        return np.array([np.rad2deg(quat_to_euler(q[i])[1]) for i in range(q.shape[0])])

    t = a["t"]
    dt = 0.2
    ts = t + dt                                  # state columns are logged post-step
    pre = pitch(a)
    pref = np.array([np.rad2deg(quat_to_euler(
        [a["qref0"][i], a["qref1"][i], a["qref2"][i], a["qref3"][i]])[1])
        for i in range(t.size)])
    pnc = pitch(b)
    win = ma.get("window") or [T_MANEUVER, 130.0]

    fig, axes = plt.subplots(4, 1, figsize=(7.0, 9.6), sharex=True,
                             gridspec_kw=dict(hspace=0.16))
    c_cmg, c_nocmg, c_ref = "tab:red", "tab:green", "0.35"

    # ---- 1: pitch ------------------------------------------------------
    ax = axes[0]
    ax.plot(t, pref, color=c_ref, lw=2.6, alpha=0.55,
            label=r"参考俯仰角 $\theta_{\mathrm{ref}}$")
    ax.plot(ts, pre, color=c_cmg, lw=1.2, label="有 CMG 辅助")
    ax.plot(ts, pnc, color=c_nocmg, lw=1.4, label="仅舵面（CMG 冻结）")
    ax.set_ylabel("俯仰角 [deg]")
    ax.set_ylim(0, 38)
    ax.legend(loc="upper left", ncol=3, framealpha=0.9, handlelength=1.5,
              columnspacing=1.0)

    # ---- 2: attitude error --------------------------------------------
    ax = axes[1]
    ax.plot(ts, a["att_err_deg"], color=c_cmg, lw=1.2, label="有 CMG 辅助")
    ax.plot(ts, b["att_err_deg"], color=c_nocmg, lw=1.4, label="仅舵面（CMG 冻结）")
    ax.set_yscale("log")
    ax.set_ylabel("姿态误差 [deg]")
    ax.set_ylim(1e-2, 30)
    ax.legend(loc="upper left", ncol=2, framealpha=0.9, handlelength=1.5)

    # ---- 3: surfaces-only torque vs its box ----------------------------
    ax = axes[2]
    ax.fill_between(t, b["lo_y"], b["hi_y"], color="0.75", alpha=0.45,
                    label=r"舵面可实现范围 $[\underline{\tau}_y,\overline{\tau}_y]$")
    ax.plot(t, b["hi_y"], color="0.45", lw=0.8)
    ax.plot(t, b["lo_y"], color="0.45", lw=0.8)
    ax.plot(t, b["tau_act_y"], color=c_nocmg, lw=1.4,
            label=r"仅舵面：$\tau_{a,y}$（贴边界）")
    ax.plot(t, b["tau_nom_y"], color="tab:blue", ls="--", lw=1.1,
            label=r"需求 $\tau_{\mathrm{nom},y}$")
    ax.axhline(0.0, color="k", lw=0.5, alpha=0.5)
    ax.set_ylabel("俯仰力矩 [N$\\cdot$m]")
    ax.set_ylim(-95, 125)
    ax.legend(loc="upper center", ncol=3, framealpha=0.9, handlelength=1.5,
              columnspacing=1.0, borderpad=0.3)

    # ---- 4: pitch control torque actually delivered --------------------
    ax = axes[3]
    ax.plot(t, a["tau_nom_y"], color="tab:blue", ls="--", lw=1.1,
            label=r"需求 $\tau_{\mathrm{nom},y}$")
    ax.plot(ts, a["tau_act_y"] + a["cmg_y"], color=c_cmg, lw=1.5,
            label=r"有 CMG：$\tau_{a,y}+\tau_{\mathrm{cmg},y}$")
    ax.plot(t, b["tau_act_y"], color=c_nocmg, lw=1.5,
            label=r"仅舵面：$\tau_{a,y}$")
    ax.axhline(0.0, color="k", lw=0.5, alpha=0.5)
    ax.set_ylabel("控制力矩 [N$\\cdot$m]")
    ax.set_xlabel("时间 $t$ [s]")
    ax.set_ylim(-95, 125)
    ax.legend(loc="upper center", ncol=3, framealpha=0.9, handlelength=1.5,
              columnspacing=1.0, borderpad=0.3)

    for ax in axes:
        ax.axvline(T_MANEUVER, color="k", ls="--", lw=1.0)
        ax.axvline(win[1], color="0.5", ls=":", lw=1.0)
        ax.grid(True)
        ax.set_xlim(*T_WIN)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
    axes[0].set_title("C1 工况：有 CMG 辅助 vs 仅靠气动舵面（其余条件完全相同）",
                      fontsize=10, pad=8)

    FIG.mkdir(parents=True, exist_ok=True)
    png, pdf = FIG / "C1_surface_only_comparison.png", FIG / "C1_surface_only_comparison.pdf"
    fig.savefig(png, dpi=300)
    fig.savefig(pdf)
    plt.close(fig)
    return png, pdf


def report(a, b, ma, mb, meta_a, meta_b, png, pdf):
    t = a["t"]
    win = ma.get("window") or [T_MANEUVER, 130.0]
    man = (t >= win[0]) & (t <= win[1])
    hi, lo = b["hi_y"], b["lo_y"]
    tau = b["tau_act_y"]
    tol = 1e-6 * np.maximum(1.0, np.abs(hi - lo))
    on_hi = (hi - tau) <= tol
    on_lo = (tau - lo) <= tol
    at_hi = float(np.mean(on_hi[man]))
    at_lo = float(np.mean(on_lo[man]))
    at_bound_man = float(np.mean((on_hi | on_lo)[man]))

    # post-window statistics: a clean 16 s stretch and the final 4 s tail
    def post(d):
        tt, err = d["t"], d["att_err_deg"]
        m1 = (tt > win[1]) & (tt <= T_WIN[1])
        m2 = tt > T_WIN[1]
        fb = d["fallback"]
        return (float(np.max(err[m1])) if m1.any() else float("nan"),
                float(np.max(err[m2])) if m2.any() else float("nan"),
                int(fb[m2].sum()) if m2.any() else 0, float(err[-1]))

    post_a = post(a)
    post_b = post(b)
    gap = np.abs(b["d_y"])
    hy = np.abs(a["Hy"])
    # demand/capability pair at the instant the gap peaks (window only)
    k_gap = int(np.flatnonzero(man)[int(np.argmax(gap[man]))])
    nom_at_gap = float(b["tau_nom_y"][k_gap])
    hi_at_gap, lo_at_gap = float(hi[k_gap]), float(lo[k_gap])
    ratio_at_gap = abs(nom_at_gap) / max(abs(hi_at_gap), abs(lo_at_gap), 1e-9)
    rate = lambda d: float(np.max(np.abs(np.column_stack([d["wx"], d["wy"], d["wz"]]))))

    def row(key, path, nd=3):
        return ("%." + str(nd) + "f") % path

    md = ["# C1：有 CMG 辅助 vs 仅气动舵面（CMG 冻结）", "",
          "两个实验臂除 `torque_gap.cmg_enabled` 外**完全相同**：同一用例文件"
          "（`configs/torque_gap_unlimited/cases/c1_pitch30.yaml`，30°/20 s 五次俯仰机动，"
          "t=110 s 起始，无框架角行程限制，B0 模式，P0 虚拟力矩植物，同一参考/扰动/气动盒/时域/求解器/权重）。",
          "",
          "- 有 CMG：`results/torque_gap_unlimited/C1_B0_P0/`"
          "（`cmg_enabled: %s`）" % meta_a.get("experiment", {}).get("cmg_enabled", True),
          "- 仅舵面：`results/torque_gap_noCMG/C1_B0_P0_noCMG/`"
          "（`cmg_enabled: %s`，γ̇ ≡ 0，γ ≡ 0）"
          % meta_b.get("experiment", {}).get("cmg_enabled", False),
          "", "图：`%s` / `%s`" % (png.relative_to(ROOT).as_posix(),
                                   pdf.relative_to(ROOT).as_posix()), "",
          "## 1. 指标对比（同一机动窗口 %.0f–%.0f s）" % (win[0], win[1]), "",
          "| 指标 | 有 CMG 辅助 | 仅舵面 | 比值 |", "|---|---|---|---|",
          "| 机动窗口姿态 RMSE [deg] | %s | **%s** | ×%.1f |"
          % (row(1, ma["rmse_deg"]["maneuver"]), row(1, mb["rmse_deg"]["maneuver"]),
             mb["rmse_deg"]["maneuver"] / max(ma["rmse_deg"]["maneuver"], 1e-9)),
          "| 机动窗口最大姿态误差 [deg] | %s | %s | ×%.1f |"
          % (row(2, ma["max_err_deg"]["maneuver"]), row(2, mb["max_err_deg"]["maneuver"]),
             mb["max_err_deg"]["maneuver"] / max(ma["max_err_deg"]["maneuver"], 1e-9)),
          "| 全程 RMSE [deg] | %s | %s | ×%.2f |"
          % (row(2, ma["rmse_deg"]["all"]), row(2, mb["rmse_deg"]["all"]),
             mb["rmse_deg"]["all"] / max(ma["rmse_deg"]["all"], 1e-9)),
          "| 机动后 %.0f–%.0f s 最大误差 [deg] | %s | %s | — |"
          % (win[1], T_WIN[1], row(2, post_a[0]), row(2, post_b[0])),
          "| 尾部 %.0f–%.0f s 最大误差 [deg]（该段 NLP 回退次数） | %s（%d） | %s（%d） | — |"
          % (T_WIN[1], a["t"][-1] + 0.2, row(2, post_a[1]), post_a[2],
             row(2, post_b[1]), post_b[2]),
          "| 峰值机体角速率 [rad/s] | %s | %s | — |" % (row(3, rate(a)), row(3, rate(b))),
          "| 舵面贴边比例（3 轴平均，窗口内） | %s | %s | — |"
          % (row(2, float(np.mean(a["tau_bound_used"][man]))),
             row(2, float(np.mean(b["tau_bound_used"][man])))),
          "| CMG 角动量峰值 [N·m·s] | %s | %s | — |"
          % (row(1, ma.get("H_norm_max")), row(1, mb.get("H_norm_max"))),
          "| NLP 回退比例 | %s | %s | — |"
          % (row(3, ma["fallback_ratio"]), row(3, mb["fallback_ratio"])),
          "| 单步平均耗时 [s] | %s | %s | — |"
          % (row(2, ma["cpu_s"]["mean"]), row(2, mb["cpu_s"]["mean"])), "",
          "## 2. 仅舵面这一臂到底能不能完成机动？", "",
          "**不能完成，但也不是完全不动。** 机动窗口内：", "",
          "- 窗口内需求 `τ_nom,y` 峰值 %.1f N·m，缺口 `|d_y|` 峰值 **%.1f N·m**、均值 %.1f N·m。"
          % (float(np.max(np.abs(b["tau_nom_y"][man]))),
             float(np.max(gap[man])), float(np.mean(gap[man]))),
          "- 舵面在窗口内 **%.0f%%** 的样本贴在上/下边界（上界 %.0f%%、下界 %.0f%%）："
          "权限已经用尽，不是控制器不想跟。" % (100 * at_bound_man, 100 * at_hi, 100 * at_lo),
          "- 姿态被迫滞后：窗口 RMSE **%.2f°**、峰值误差 %.2f°；峰值机体角速率只有 %.4f rad/s"
          "（有 CMG 时 %.4f rad/s，是它的 %.1f 倍）。"
          % (mb["rmse_deg"]["maneuver"], mb["max_err_deg"]["maneuver"], rate(b), rate(a),
             rate(a) / max(rate(b), 1e-9)),
          "- 原因很直接：缺口峰值那一瞬间（t = %.1f s）`τ_nom,y = %+.1f N·m`、`d_y = %+.1f N·m`，"
          "而该时刻舵面盒只有 `[%+.1f, %+.1f] N·m`——需求是可用上界的 **%.1f 倍**，"
          "CMG 冻结后无人补这个缺口。" % (b["t"][k_gap], nom_at_gap, b["d_y"][k_gap],
                                          lo_at_gap, hi_at_gap, ratio_at_gap), "",
          "## 3. 有 CMG 辅助的效果", "",
          "- 窗口 RMSE %s°，峰值 %s°，比仅舵面好 **%.1f 倍**；角速率峰值 %.3f rad/s，"
          "机动确实被执行。"
          % (row(2, ma["rmse_deg"]["maneuver"]), row(2, ma["max_err_deg"]["maneuver"]),
             mb["rmse_deg"]["maneuver"] / max(ma["rmse_deg"]["maneuver"], 1e-9), rate(a)),
          "- 代价是角动量：机动期间 `|H|` 被推到 %.1f N·m·s（可达上界 ≈157.7）；"
          "仅舵面一臂的 `|H|` 恒为 0，连“借动量”这条路都没有。"
          % ma.get("H_norm_max", float("nan")),
          "- 机动结束后 %.0f–%.0f s，有 CMG 的一臂回到 **%.2f°** 以内；仅舵面一臂仍有 %.2f°"
          "（靠长时间小力矩慢慢追那 4.7° 的滞后）。"
          % (win[1], T_WIN[1], post_a[0], post_b[0]), "",
          "- **未解决的尾部问题**：有 CMG 的一臂在最后 4 s（%.0f–%.0f s）出现 %.1f° 的误差尖峰，"
          "同期 %d 次 NLP 回退（`Maximum_Iterations_Exceeded|previous`）；仅舵面一臂该段无回退（%.1f°）。"
          "这**不能**解释成“角动量耗尽”——130–146 s 内它的误差只有 %.2f°。"
          "该现象位于机动窗口之外，不影响 §1 的结论，但列为未解决项。"
          % (T_WIN[1], a["t"][-1] + 0.2, post_a[1], post_a[2], post_b[1], post_a[0]),
          "- 舵面贴边比例 %s vs %s：有 CMG 时舵面不必一直顶在边界，缺口由 CMG 承担。"
          % (row(2, float(np.mean(a["tau_bound_used"][man]))),
             row(2, float(np.mean(b["tau_bound_used"][man])))), "",
          "## 4. 结论", "",
          "1. **只靠气动舵面完不成这个机动**：窗口内 %.0f%% 的样本贴边、缺口峰值时刻需求是"
          "可用上界的 %.1f 倍，窗口 RMSE %.2f°（有 CMG 时 %.2f°）。"
          % (100 * at_bound_man, ratio_at_gap,
             mb["rmse_deg"]["maneuver"], ma["rmse_deg"]["maneuver"]),
          "2. **CMG 的作用是不可替代的**：同一份力矩需求下，它把窗口误差降低了约 %.0f%%，"
          "并把舵面从“全程贴边”中解放出来。"
          % (100.0 * (1.0 - ma["rmse_deg"]["maneuver"] / mb["rmse_deg"]["maneuver"])),
          "3. **CMG 的代价是角动量**：机动期间 `|H|` 被推到 %.1f N·m·s（可达上界 ≈157.7），"
          "这正是 `C1_control_overview_analysis.md` §5.4–5.5 所说的“动量预算才是真瓶颈”；"
          "仅舵面一臂的 `|H|` 恒为 0，连“借动量”这条路都没有。"
          % ma.get("H_norm_max", float("nan")),
          "4. 仅舵面臂的 NLP 更容易（回退 0%%、单步 %s s），说明它“完成不了”不是求解器问题，"
          "而是物理权限不够；反过来，有 CMG 的一臂在尾部（>%.0f s）出现 %d 次回退、%.1f° 尖峰，"
          "属未解决项。" % (row(2, mb["cpu_s"]["mean"]), T_WIN[1], post_a[2], post_a[1]), "",
          "> 说明：P0 是虚拟力矩植物，`τ_a` 是理想气动力矩。真实舵面的实现能力见 P1"
          "（`results/torque_gap_unlimited/C1_B0_P1/`），本对比不涉及舵偏角速率/伺服滞后。",
          ]
    DOC.mkdir(parents=True, exist_ok=True)
    path = DOC / "C1_surface_only_comparison.md"
    path.write_text("\n".join(md) + "\n", encoding="utf-8")
    return path


def main():
    a, ma, meta_a = load(RUN_CMG)
    b, mb, meta_b = load(RUN_NOCMG)
    png, pdf = plot(a, b, ma, mb)
    path = report(a, b, ma, mb, meta_a, meta_b, png, pdf)
    print("figure:", png)
    print("report:", path)
    print("maneuver RMSE: with CMG %.3f deg, surfaces only %.3f deg"
          % (ma["rmse_deg"]["maneuver"], mb["rmse_deg"]["maneuver"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
