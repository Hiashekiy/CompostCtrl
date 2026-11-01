#!/usr/bin/env python
"""S08 figures: per-case A/B overviews and the sweep summaries (doc §13).

    python scripts/plot_s08_overview.py --plant P0
    python scripts/plot_s08_overview.py --plant P0 --only A4_descent_replay

Panels of an overview figure (doc §13, 1-10):
    1 reference vs actual pitch attitude
    2 attitude error (with the 1/5/10 deg levels)
    3 actual angle of attack and dynamic pressure
    4 aerodynamic coefficients (Cm; CL/CD are not modelled by this attitude-only
      model and are therefore annotated, never drawn as zero curves)
    5 aerodynamic pitching moment vs the moment the surfaces must supply
    6 deflection, deflection rate and saturation (P1 only; annotated in P0)
    7 CMG three-axis torque (B only, the A panels say "no CMG")
    8 gimbal angle, rate and acceleration (B only)
    9 cluster momentum components, norm and the reachable boundary (B only)
   10 solver wall clock and fallback flag (both arms)

Summary figures
    figures/torque_gap_s08/summary/S08-1_alpha_sweep.png
    figures/torque_gap_s08/summary/S08-2_qbar_sweep.png
    figures/torque_gap_s08/summary/S08-3_alpha_qbar_map.png
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
RES = ROOT / "results" / "torque_gap_s08"
FIG = ROOT / "figures" / "torque_gap_s08"
AX = ("x", "y", "z")
C_REF, C_A, C_B = "#2b3a55", "tab:red", "tab:blue"
C_SURF, C_CMG, C_NOM = "tab:green", "tab:purple", "tab:orange"


def _style():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["Microsoft YaHei", "SimHei", "DejaVu Sans"],
        "axes.unicode_minus": False, "mathtext.fontset": "dejavusans",
        "font.size": 8.5, "axes.labelsize": 8.5, "axes.titlesize": 9,
        "xtick.labelsize": 7.5, "ytick.labelsize": 7.5, "legend.fontsize": 7,
        "axes.linewidth": 0.8, "lines.linewidth": 1.1, "grid.linewidth": 0.5,
        "grid.alpha": 0.35, "savefig.bbox": "tight", "savefig.dpi": 150,
    })
    return plt


def load_run(d):
    d = Path(d)
    with open(d / "timeseries.csv", encoding="utf-8", newline="") as fh:
        rows = list(csv.DictReader(fh))
    out = {}
    for key in rows[0]:
        try:
            out[key] = np.array([np.nan if r[key] in ("", None) else float(r[key])
                                 for r in rows], float)
        except ValueError:
            out[key] = np.array([r[key] for r in rows], dtype=object)
    out["_metrics"] = json.loads((d / "metrics.json").read_text(encoding="utf-8"))
    out["_meta"] = json.loads((d / "meta.json").read_text(encoding="utf-8"))
    return out


def euler_deg(q):
    q = np.asarray(q, float)
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.column_stack([
        np.degrees(np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))),
        np.degrees(np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))),
        np.degrees(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))),
    ])


def headroom(ax, frac=0.35):
    lo, hi = ax.get_ylim()
    span = hi - lo
    if not np.isfinite(span) or span <= 0:
        return
    ax.set_ylim(lo, hi + frac * span)


def mark(ax, t0, t1=None, label=None):
    ax.axvline(t0, color="k", ls="--", lw=0.9)
    if t1 is not None:
        ax.axvline(t1, color="k", ls="--", lw=0.9)
    if label:
        ax.text(t0, 1.01, label, transform=ax.get_xaxis_transform(),
                fontsize=7, color="0.25", ha="left", va="bottom")


# --------------------------------------------------------------------------
def fig_case(group, case, a, b, plant, out_png):
    plt = _style()
    n = min(len(a["t_next"]), len(b["t_next"]))
    t = b["t_next"][:n]
    mb = b["_metrics"]
    wins = mb.get("windows") or {}
    man = wins.get("man0")
    has_cmg_a = bool(a["_metrics"].get("cmg_present"))
    has_cmg_b = bool(b["_metrics"].get("cmg_present"))
    has_surf = "de_act" in b

    EA = euler_deg(np.column_stack([a["q%d" % i][:n] for i in range(4)]))
    EB = euler_deg(np.column_stack([b["q%d" % i][:n] for i in range(4)]))
    EREF = euler_deg(np.column_stack([b["qref%d" % i][:n] for i in range(4)]))

    fig, axes = plt.subplots(10, 1, figsize=(7.8, 17.0), sharex=True,
                             gridspec_kw=dict(hspace=0.16, top=0.968, bottom=0.028,
                                              left=0.085, right=0.905))
    fig.suptitle("%s / %s（%s 植物）：A 仅舵面 vs B 舵面+CMG" % (group, case, plant),
                 fontsize=10.5, y=0.993)

    def ev(ax):
        if man:
            mark(ax, man["t0"], man["t1"], "机动")
        ax.grid(True, which="major")

    # 1 attitude
    ax = axes[0]
    ax.plot(t, EREF[:, 1], color=C_REF, ls="--", lw=1.0, label="参考俯仰角 $\\theta_{ref}$")
    ax.plot(t, EA[:, 1], color=C_A, lw=1.1, label="A 实际俯仰角")
    ax.plot(t, EB[:, 1], color=C_B, lw=1.3, label="B 实际俯仰角")
    ax.set_ylabel("俯仰角 [°]")
    ax.legend(loc="upper left", ncol=3, framealpha=0.9)
    ev(ax)
    headroom(ax, 0.30)

    # 2 error
    ax = axes[1]
    for x, ls in ((1.0, ":"), (5.0, "-."), (10.0, "--")):
        ax.axhline(x, color="0.45", ls=ls, lw=0.7)
        ax.text(t[-1], x, " %g°" % x, fontsize=7, color="0.35", va="center")
    ax.plot(t, a["att_err_deg"][:n], color=C_A, lw=1.1, label="A 姿态误差")
    ax.plot(t, b["att_err_deg"][:n], color=C_B, lw=1.3, label="B 姿态误差")
    ax.set_ylabel("短弧姿态误差 [°]")
    ax.legend(loc="upper left", ncol=2, framealpha=0.9)
    ev(ax)
    headroom(ax, 0.35)

    # 3 alpha and qbar
    ax = axes[2]
    ax.plot(t, a["alpha_deg"][:n], color=C_A, lw=1.1, label="A 实际迎角 $\\alpha$")
    ax.plot(t, b["alpha_deg"][:n], color=C_B, lw=1.3, label="B 实际迎角 $\\alpha$")
    ax.plot(t, b["alpha_ref_deg"][:n], color=C_REF, ls="--", lw=0.9, label="参考迎角")
    ax.set_ylabel("迎角 [°]")
    ax.legend(loc="upper left", ncol=3, framealpha=0.9)
    ax2 = ax.twinx()
    ax2.plot(t, b["qbar_pa"][:n], color=C_NOM, lw=1.0, ls="-.", label="$\\bar q$（右轴）")
    ax2.set_ylabel("动压 [Pa]")
    ev(ax)

    # 4 coefficients
    ax = axes[3]
    ax.plot(t, a["Cm_nat"][:n], color=C_A, lw=1.1, label="A $C_m$（零舵偏）")
    ax.plot(t, b["Cm_nat"][:n], color=C_B, lw=1.3, label="B $C_m$（零舵偏）")
    ax.set_ylabel("$C_m$")
    ax.text(0.995, 0.05, "CL、CD、升力、阻力：本模型未建模（无力方程），故不绘制",
            transform=ax.transAxes, fontsize=7, color="0.3", ha="right")
    ax.legend(loc="upper left", ncol=2, framealpha=0.9)
    ev(ax)
    headroom(ax, 0.30)

    # 5 moments
    ax = axes[4]
    ax.plot(t, b["tau_nat_y"][:n], color=C_NOM, ls="--", lw=1.0,
            label="气动俯仰力矩（零舵偏）$\\tau_{nat,y}$")
    ax.plot(t, a["tau_req_y"][:n], color=C_A, lw=1.1, label="A 所需舵面控制力矩")
    ax.plot(t, b["tau_req_y"][:n], color=C_B, lw=1.3, label="B 所需舵面控制力矩")
    ax.plot(t, b["half_y"][:n], color=C_SURF, ls=":", lw=1.0,
            label="舵面可用俯仰力矩半宽")
    ax.plot(t, -b["half_y"][:n], color=C_SURF, ls=":", lw=1.0)
    ax.set_ylabel("俯仰力矩 [N·m]")
    ax.legend(loc="upper left", ncol=2, framealpha=0.9)
    ev(ax)
    headroom(ax, 0.45)

    # 6 deflection / utilisation
    ax = axes[5]
    if has_surf:
        ax.plot(t, np.degrees(b["de_act"][:n]), color=C_B, lw=1.2, label="B 实际舵偏 $\\delta_e$")
        if "de_cmd" in b:
            ax.plot(t, np.degrees(b["de_cmd"][:n]), color=C_REF, ls="--", lw=0.9,
                    label="B 舵偏指令")
        ax.axhline(25.0, color="0.5", ls=":", lw=0.8)
        ax.axhline(-25.0, color="0.5", ls=":", lw=0.8)
        ax.set_ylabel("舵偏角 [°]")
        axu = ax.twinx()
        axu.plot(t, b["surf_util_y"][:n], color=C_SURF, lw=1.0, ls="-.",
                 label="俯仰需求利用率（右轴）")
        axu.axhline(1.0, color="0.4", ls=":", lw=0.8)
        axu.set_ylabel("$|\\tau_{req,y}|/\\tau^{half}_y$")
        h1, l1 = ax.get_legend_handles_labels()
        h2, l2 = axu.get_legend_handles_labels()
        ax.legend(h1 + h2, l1 + l2, loc="upper left", ncol=3, framealpha=0.9)
    else:
        ax.plot(t, a["surf_util_y"][:n], color=C_A, lw=1.0, ls="--",
                label="A 俯仰需求利用率 $|\\tau_{req,y}|/\\tau^{half}_y$")
        ax.plot(t, b["surf_util_y"][:n], color=C_SURF, lw=1.2, ls="-.",
                label="B 俯仰需求利用率")
        ax.axhline(1.0, color="0.4", ls=":", lw=0.8)
        ax.set_ylabel("控制需求利用率")
        ax.text(0.995, 0.06, "P0 虚拟力矩植物：无舵偏/舵机状态（未建模），故只画需求利用率",
                transform=ax.transAxes, fontsize=7, color="0.3", ha="right")
        ax.legend(loc="upper left", ncol=2, framealpha=0.9)
    ev(ax)
    headroom(ax, 0.45)

    # 7 CMG torque
    ax = axes[6]
    if has_cmg_b:
        for i, ax_i in enumerate(AX):
            ax.plot(t, b["cmg_%s" % ax_i][:n], color="C%d" % (i + 1), lw=1.1,
                    label="$\\tau_{cmg,%s}$" % ax_i)
        ax.legend(loc="upper left", ncol=3, framealpha=0.9)
    ax.set_ylabel("CMG 力矩 [N·m]")
    if not has_cmg_a:
        ax.text(0.01, 0.05, "A 组：无 CMG（不适用）", transform=ax.transAxes,
                fontsize=8, color="0.35")
    ev(ax)
    headroom(ax, 0.50)

    # 8 gimbal angle / rate / acceleration
    ax = axes[7]
    if has_cmg_b:
        for i in range(4):
            ax.plot(t, np.degrees(b["g%d" % (i + 1)][:n]), color="C%d" % i, lw=1.0,
                    label="$\\gamma_%d$" % (i + 1))
        ax.set_ylabel("框架角 [°]")
        ax2 = ax.twinx()
        acc = np.zeros(len(t))
        for i in range(4):
            nu = b["nu%d" % (i + 1)][:n]
            ax2.plot(t, np.degrees(nu), color="C%d" % i, lw=0.7, ls=":", alpha=0.9)
            if len(nu) > 1:
                acc = np.maximum(acc, np.abs(np.diff(np.r_[nu[0], nu])) / 0.2)
        ax2.set_ylabel("框架速率 [°/s]（点线）")
        ax.text(0.01, 0.06, "峰值框架角加速度 %.2f rad/s²（限 3.0）" % np.nanmax(acc),
                transform=ax.transAxes, fontsize=7, color="0.3", ha="left")
        ax.legend(loc="upper left", ncol=4, framealpha=0.9)
    else:
        ax.text(0.5, 0.55, "A 组：无框架状态（不适用）", transform=ax.transAxes,
                fontsize=8.5, color="0.35", ha="center")
        ax.set_ylabel("框架角 [°]")
    ev(ax)
    headroom(ax, 0.55)

    # 9 momentum
    ax = axes[8]
    if has_cmg_b:
        for i, ax_i in enumerate(AX):
            ax.plot(t, b["H%s" % ax_i][:n], color="C%d" % (i + 1), lw=1.0,
                    label="$H_%s$" % ax_i)
        ax.plot(t, b["H_norm"][:n], color=C_REF, lw=1.3, label="$|H|$")
        htot = 4.0 * 50.0
        cm = (b["_meta"]["experiment"].get("c_mg") or {})
        if isinstance(cm, dict) and "h0" in cm:
            htot = 4.0 * float(cm["h0"])
        ax.axhline(htot, color="0.5", ls=":", lw=0.8)
        ax.text(t[0], htot, " $4h_0=%.0f$ N·m·s" % htot, fontsize=7, color="0.4")
        ax.legend(loc="upper left", ncol=4, framealpha=0.9)
        ax.set_ylabel("角动量 [N·m·s]")
        ax2 = ax.twinx()
        ax2.plot(t, b["sigma_min"][:n], color="0.4", lw=0.9, ls="-.",
                 label="$\\sigma_{min}(A)$（右轴）")
        ax2.set_ylabel("$\\sigma_{min}$ [N·m·s]")
    else:
        ax.text(0.5, 0.55, "A 组：无角动量状态（不适用）", transform=ax.transAxes,
                fontsize=8.5, color="0.35", ha="center")
        ax.set_ylabel("角动量 [N·m·s]")
    ev(ax)

    # 10 solver
    ax = axes[9]
    ax.plot(t, a["cpu_s"][:n], color=C_A, lw=0.9, label="A 单步求解耗时")
    ax.plot(t, b["cpu_s"][:n], color=C_B, lw=0.9, label="B 单步求解耗时")
    ax.axhline(0.2, color="0.4", ls=":", lw=0.8)
    ax.set_ylabel("求解耗时 [s]")
    ax.set_xlabel("时间 [s]")
    ax.legend(loc="upper left", ncol=2, framealpha=0.9)
    ax2 = ax.twinx()
    ax2.plot(t, a["fallback"][:n], color=C_A, lw=0.8, ls="--", alpha=0.6)
    ax2.plot(t, b["fallback"][:n], color=C_B, lw=0.8, ls="--", alpha=0.6)
    ax2.set_ylabel("回退标志（虚线）")
    ax2.set_ylim(-0.1, 1.6)
    ax.text(0.995, 0.92,
            "回退比例 A %.3f / B %.3f；求解耗时 P95 A %.3f s / B %.3f s"
            % (a["_metrics"].get("fallback_ratio", np.nan),
               b["_metrics"].get("fallback_ratio", np.nan),
               (a["_metrics"].get("cpu_s") or {}).get("p95", np.nan),
               (b["_metrics"].get("cpu_s") or {}).get("p95", np.nan)),
            transform=ax.transAxes, fontsize=7, color="0.3", ha="right")
    ev(ax)

    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png)
    plt.close(fig)
    return out_png


# --------------------------------------------------------------------------
def _f(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return np.nan


def read_summary():
    with open(RES / "summary.csv", encoding="utf-8", newline="") as fh:
        rows = list(csv.DictReader(fh))

    def f(r, k, default=np.nan):
        v = r.get(k, "")
        try:
            return float(v)
        except (TypeError, ValueError):
            return default
    return rows, f


def pick(rows, group, plant, arm, key="case"):
    out = {}
    for r in rows:
        if r["group"] != group or r["plant"] != plant or r["arm"] != arm:
            continue
        out.setdefault(r[key], r)
    return out


def fig_sweep(rows, group, plant, xkey, xlabel, title, out_png):
    plt = _style()
    A = pick(rows, group, plant, "no_cmg")
    B = pick(rows, group, plant, "with_cmg")
    names = [k for k in A if k in B]
    if not names:
        return None

    def num_or(v, default=np.nan):
        try:
            return float(v)
        except (TypeError, ValueError):
            return default

    def series(d, key, w="all"):
        """(x, y): x is the *nominal* scan level, i.e. the target the case was
        built for.  During a divergence the measured alpha drifts far from it, so
        the measured value cannot be used as a sweep coordinate."""
        xs, ys = [], []
        for name, r in d.items():
            if xkey == "alpha":
                x = num_or(r.get("alpha_target_deg", ""), num_or(r.get("all_alpha_mean")))
            else:
                x = num_or(r.get("qbar_target_pa", ""), num_or(r.get("all_qbar_mean")))
            ys.append(num_or(r.get(key, "")))
            xs.append(x)
        order = np.argsort(xs)
        return np.asarray(xs)[order], np.asarray(ys)[order]

    fig, axes = plt.subplots(2, 3, figsize=(11.0, 6.4))
    fig.suptitle(title, fontsize=11)

    ax = axes[0, 0]
    if xkey == "alpha":
        # measured mean Cm against the measured mean alpha: all points must fall
        # on the single straight line Cm = Cm0 + Cm_alpha alpha, which is the
        # audit's "the model has no non-linear moment break" made visible.
        for d, c, mk, lab in ((A, C_A, "s", "A 仅舵面"), (B, C_B, "o", "B 舵面+CMG")):
            xs = [_f(r.get("all_alpha_mean", "")) for r in d.values()]
            ys = [_f(r.get("Cm_nat_mean", "")) for r in d.values()]
            ax.plot(xs, ys, mk, color=c, ls="none", mfc="none",
                    label="%s 实测均值" % lab)
        aa = np.linspace(-15.0, 45.0, 50)
        ax.plot(aa, -0.35 * np.deg2rad(aa), "-", color="k", lw=0.8,
                label="$C_m=C_{m\\alpha}\\alpha$（$C_{m\\alpha}=-0.35$/rad）")
        ax.set_xlabel("实际迎角 [°]")
        ax.set_title("(a) 气动俯仰力矩系数（线性，无失速拐点）")
    else:
        xa, ya = series(A, "Cm_nat_mean")
        xb, yb = series(B, "Cm_nat_mean")
        ax.plot(xb, yb, "o-", color=C_B, label="B $C_m$（零舵偏）")
        ax.plot(xa, ya, "s--", color=C_A, label="A $C_m$（零舵偏）")
        ax.set_xlabel(xlabel)
        ax.set_title("(a) 固定迎角下 $C_m$ 不随动压变化")
    ax.set_ylabel("$C_m$")
    ax.legend(fontsize=6.5)
    ax.grid(alpha=0.35)

    ax = axes[0, 1]
    xa, ya = series(A, "all_tau_req_y_mean")
    xb, yb = series(B, "all_tau_req_y_mean")
    ax.plot(xb, yb, "o-", color=C_B, label="所需 $|\\tau_{req,y}|$ B")
    ax.plot(xa, ya, "s--", color=C_A, label="所需 $|\\tau_{req,y}|$ A")
    hx, hy = [], []
    for r in sorted((r for r in rows if r["group"] == group and r["plant"] == plant),
                    key=lambda r: float(r.get("all_qbar_mean", "nan"))):
        x = (r.get("alpha_target_deg") if xkey == "alpha" else r.get("qbar_target_pa"))
        hx.append(_f(x) if x not in (None, "") else _f(r["all_qbar_mean"]))
        hy.append(_f(r["half_y"]))
    ax.plot(hx, hy, "k:", label="舵面半宽 $\\tau^{half}_y$")
    ax.set_xlabel(xlabel)
    ax.set_ylabel("俯仰力矩 [N·m]")
    ax.legend()
    ax.grid(alpha=0.35)
    ax.set_title("(b) 需求 vs 可用舵面控制力矩")

    ax = axes[0, 2]
    xa, ya = series(A, "all_surf_util_mean")
    xb, yb = series(B, "all_surf_util_mean")
    ax.plot(xb, yb, "o-", color=C_B, label="B 利用率")
    ax.plot(xa, ya, "s--", color=C_A, label="A 利用率")
    ax.axhline(1.0, color="0.4", ls=":", label="饱和线")
    ax.set_xlabel(xlabel)
    ax.set_ylabel("$|\\tau_{req,y}|/\\tau^{half}_y$")
    ax.legend()
    ax.grid(alpha=0.35)
    ax.set_title("(c) 舵面控制需求利用率")

    ax = axes[1, 0]
    for d, c, lab in ((A, C_A, "A 仅舵面"), (B, C_B, "B 舵面+CMG")):
        x, y = series(d, "all_rmse")
        ax.plot(x, y, "o-", color=c, label="%s 全程 RMSE" % lab)
        x, y = series(d, "term_rmse")
        ax.plot(x, y, "s--", color=c, alpha=0.7, label="%s 末段 RMSE" % lab)
    ax.set_xlabel(xlabel)
    ax.set_ylabel("姿态 RMSE [°]")
    ax.legend()
    ax.grid(alpha=0.35)
    ax.set_title("(d) 姿态跟踪误差")

    ax = axes[1, 1]
    for d, c, lab in ((A, C_A, "A"), (B, C_B, "B")):
        x, y = series(d, "all_surf_sat")
        ax.plot(x, y, "o-", color=c, label="%s 贴边比例" % lab)
        x, y = series(d, "all_fallback")
        ax.plot(x, y, "s:", color=c, alpha=0.7, label="%s MPC 回退比例" % lab)
    ax.set_xlabel(xlabel)
    ax.set_ylabel("比例")
    ax.legend()
    ax.grid(alpha=0.35)
    ax.set_title("(e) 舵面饱和与求解回退")

    ax = axes[1, 2]
    x, y = series(B, "all_cmg_y_peak")
    ax.plot(x, y, "o-", color=C_CMG, label="B $|\\tau_{cmg,y}|$ 峰值")
    x, y = series(B, "all_H_norm_max")
    ax.plot(x, y, "s--", color=C_REF, label="B $|H|$ 峰值 [N·m·s]")
    x, y = series(B, "all_nu_sat")
    ax.plot(x, y, "^:", color="0.4", label="B 框架速率饱和比例")
    ax.set_xlabel(xlabel)
    ax.set_ylabel("CMG 负载 / 比例")
    ax.legend()
    ax.grid(alpha=0.35)
    ax.set_title("(f) CMG 负载（仅 B）")

    for ax in axes.ravel():
        ax.tick_params(labelsize=7.5)
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.94))
    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png)
    plt.close(fig)
    return out_png


def fig_factorial(rows, plant, out_png):
    """S08-3: the 2 x 2 map of attitude error, surface load and CMG boundary use."""
    plt = _style()
    fig, axes = plt.subplots(1, 4, figsize=(13.6, 3.7))
    fig.suptitle("S08-A3 迎角-动压二维组合（%s 植物；点旁标注为工况 ll/hl/lh/hh）" % plant,
                 fontsize=11)
    A = pick(rows, "A3_coupled", plant, "no_cmg")
    B = pick(rows, "A3_coupled", plant, "with_cmg")
    panels = [
        (axes[0], A, "all_rmse", "(a) A 仅舵面：全程姿态 RMSE [°]", "Reds", 1.0),
        (axes[1], B, "all_rmse", "(b) B 舵面+CMG：全程姿态 RMSE [°]", "Blues", 1.0),
        (axes[2], A, "all_surf_util_mean", "(c) A：舵面控制需求利用率", "viridis", None),
        (axes[3], B, "all_H_norm_max", "(d) B：角动量峰值占用 [N·m·s]", "magma", None),
    ]
    for ax, d, key, title, cmap, scale in panels:
        xs, ys, vs, tags = [], [], [], []
        for name, r in d.items():
            x = _f(r.get("alpha_target_deg", ""))
            y = _f(r.get("qbar_target_pa", ""))
            v = _f(r.get(key, ""))
            xs.append(x)
            ys.append(y)
            vs.append(v if scale is None else v * scale)
            tags.append(name.rsplit("_", 1)[-1])
        sc = ax.scatter(xs, ys, c=vs, cmap=cmap, s=190, edgecolor="k", zorder=3)
        for x, y, v, tag in zip(xs, ys, vs, tags):
            ax.annotate(tag, (x, y), xytext=(0, 13), textcoords="offset points",
                        fontsize=8, ha="center", color="0.25", zorder=4)
            ax.annotate("%.2f" % v, (x, y), xytext=(0, -17), textcoords="offset points",
                        fontsize=8, ha="center", color="0.1", zorder=4)
        ax.set_xticks([0.0, 30.0])
        ax.set_yticks([9.074, 36.295])
        ax.set_yticklabels(["9.07\n(低)", "36.30\n(高)"])
        ax.set_xticklabels(["0\n(低)", "30\n(高)"])
        ax.set_xlim(-12.0, 42.0)
        ax.set_ylim(2.0, 44.0)
        ax.set_xlabel("参考迎角 [°]")
        ax.set_ylabel("动压 [Pa]")
        ax.set_title(title, fontsize=9)
        ax.grid(alpha=0.35)
        fig.colorbar(sc, ax=ax, shrink=0.85)
    fig.tight_layout(rect=(0.0, 0.0, 1.0, 0.90))
    out_png.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png)
    plt.close(fig)
    return out_png


# --------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--plant", default="P0", choices=["P0", "P1"])
    ap.add_argument("--only", default=None, help="comma separated case ids")
    ap.add_argument("--summary-only", action="store_true")
    args = ap.parse_args()
    only = set(x.strip() for x in args.only.split(",")) if args.only else None
    made = []
    if not args.summary_only:
        for group_dir in sorted(p for p in RES.iterdir() if p.is_dir()):
            group = group_dir.name
            cases = set()
            for run_dir in sorted(p for p in group_dir.iterdir() if p.is_dir()):
                case = run_dir.name
                for arm in ("_no_cmg", "_with_cmg"):
                    if case.endswith(arm):
                        cases.add(case[: -len(arm)])
            for case in sorted(cases):
                if only and case not in only:
                    continue
                suffix = "_P1" if args.plant == "P1" else ""
                da = group_dir / ("%s_no_cmg%s" % (case, suffix))
                db = group_dir / ("%s_with_cmg%s" % (case, suffix))
                if not ((da / "metrics.json").exists() and (db / "metrics.json").exists()):
                    continue
                made.append(fig_case(group, case, load_run(da), load_run(db),
                                     args.plant,
                                     FIG / group / ("%s_%s.png" % (case, args.plant))))
        for p in made:
            print("[plot] %s" % p.relative_to(ROOT))
    rows, _f = read_summary()
    targets = [
        ("A1_alpha", "alpha", "迎角 [°]", "S08-1 迎角扫描（%s 植物）" % args.plant,
         FIG / "summary" / ("S08-1_alpha_sweep_%s.png" % args.plant)),
        ("A2_qbar", "qbar", "动压 [Pa]", "S08-2 动压扫描（%s 植物）" % args.plant,
         FIG / "summary" / ("S08-2_qbar_sweep_%s.png" % args.plant)),
    ]
    for group, xkey, xlab, title, out in targets:
        p = fig_sweep(rows, group, args.plant, xkey, xlab, title, out)
        if p:
            print("[plot] %s" % p.relative_to(ROOT))
    p = fig_factorial(rows, args.plant, FIG / "summary" / ("S08-3_map_%s.png" % args.plant))
    print("[plot] %s" % p.relative_to(ROOT))
    return 0


if __name__ == "__main__":
    sys.exit(main())
