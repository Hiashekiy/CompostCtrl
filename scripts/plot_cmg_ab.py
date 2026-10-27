"""Figures for the CMG A/B study (implementation plan §9).

    python scripts/plot_cmg_ab.py [--cases tracking maneuver10 ...]

For every case with both modes present it writes, into figures/cmg_ab/,

    <case>_tracking.png   8 panels: reference/actual, error, rates, surfaces,
                          aero moment, CMG torque, CMG momentum + budget,
                          solver status                     (§9.1)
    <case>_maneuver.png   9 panels: pitch reference/actual, pitch error,
                          pitch rate, surfaces + limits, aero moment,
                          CMG torque, CMG momentum + budget,
                          gimbal angle & sigma_min, solver status   (§9.2)

House style (§9.3): time in seconds, angles in degrees, torques in N·m,
momentum in N·m·s, A = blue solid, B = red dashed, every limit line labelled,
statistics windows shaded, no truncated peaks.
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
import matplotlib.pyplot as plt                        # noqa: E402
from matplotlib.lines import Line2D                    # noqa: E402

# CJK labels: pick the first font actually installed, otherwise fall back to
# English-only labels (matplotlib would silently draw empty boxes otherwise).
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "Noto Sans SC",
                                   "DengXian", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
RES = os.path.join(ROOT, "results", "cmg_ab")
FIG = os.path.join(ROOT, "figures", "cmg_ab")

STYLE = {"A": dict(color="#1f77b4", ls="-", lw=1.4, label="A 仅气动舵面"),
         "B": dict(color="#d62728", ls="--", lw=1.4, label="B 舵面 + CMG")}
MODES = ("A", "B")


# ---------------------------------------------------------------------------
def load(case, mode):
    d = os.path.join(RES, case, mode)
    tp = os.path.join(d, "timeseries.csv")
    if not os.path.exists(tp):
        return None, None
    with open(tp, newline="", encoding="utf-8") as fh:
        r = csv.reader(fh)
        cols = next(r)
        arr = np.array([[float(v) for v in row] for row in r], dtype=float)
    rec = {c: arr[:, i] for i, c in enumerate(cols)}
    meta = {}
    mp = os.path.join(d, "meta.json")
    if os.path.exists(mp):
        with open(mp, encoding="utf-8") as fh:
            meta = json.load(fh)
    return rec, meta


def _budget(meta):
    cfg = (meta or {}).get("config", {})
    m = cfg.get("mpc_cmg", {})
    cmg = cfg.get("cmg", {})
    htot = float(cmg.get("h0", 50.0)) * int(cmg.get("n", 4))
    return float(m.get("H_lim_frac", 0.3)) * htot, htot


def _sigma_floor(meta):
    cfg = (meta or {}).get("config", {})
    cmg = cfg.get("cmg", {})
    return float(cmg.get("sigma_floor_frac", 0.3)) * 0.816496580927726 * float(cmg.get("h0", 50.0))


def _startup_note(ax):
    ax.annotate("t < 0.5 s 为初始配平瞬态", xy=(0.005, 0.04),
                xycoords="axes fraction", fontsize=7, color="#666666")


def _shade(ax, windows, meta):
    for tag, (a, b) in (windows or {}).items():
        ax.axvspan(a, b, color="#cccccc", alpha=0.35, lw=0, zorder=0)
        ax.axvline(a, color="#888888", lw=0.6, ls=":", zorder=0)


def _legend(ax, **kw):
    ax.legend(fontsize=7, ncol=2, frameon=False, loc="best", **kw)


# ---------------------------------------------------------------------------
def tracking_figure(case):
    data = {}
    for m in MODES:
        rec, meta = load(case, m)
        if rec is not None:
            data[m] = (rec, meta)
    if len(data) < 2:
        print("skip %s (needs A and B)" % case)
        return None
    windows = data["B"][1].get("windows") or data["A"][1].get("windows") or {}
    Hbud, Htot = _budget(data["B"][1])

    fig, axs = plt.subplots(8, 1, figsize=(11, 19), sharex=True)
    ax = axs[0]
    r = data["A"][0]
    for i, nm in enumerate(("滚转 φ", "俯仰 θ", "偏航 ψ")):
        ax.plot(r["t"], r[["phi_ref", "theta_ref", "psi_ref"][i]], color="k",
                ls="-", lw=1.0, label=None if i else "参考姿态")
    for m in MODES:
        r = data[m][0]
        ax.plot(r["t"], r["phi"], **{**STYLE[m], "label": None})
    ax.set_ylabel("姿态角 [°]")
    ax.set_title("%s  （%s）第 1 行：参考（黑实线）与 A（蓝）/B（红）实际姿态；"
                 "φ/θ/ψ 三轴" % (case, "E1"), fontsize=10)
    _legend(ax)

    ax = axs[1]
    for m in MODES:
        r = data[m][0]
        ax.plot(r["t"], r["err_norm_deg"], **STYLE[m])
    ax.set_ylabel("误差 [°]")
    ax.set_title("第 2 行：四元数测地误差 ‖e‖（度）；灰色区为扰动/机动窗", fontsize=10)
    _shade(ax, windows, data["B"][1])
    _legend(ax)

    ax = axs[2]
    for i, nm in enumerate(("p", "q", "r")):
        for m in MODES:
            r = data[m][0]
            ax.plot(r["t"], np.rad2deg(r[["omega_x", "omega_y", "omega_z"][i]]),
                    **{**STYLE[m], "lw": 1.0,
                       "label": "%s %s" % (STYLE[m]["label"], nm)})
    ax.set_ylabel("角速度 [°/s]")
    ax.set_title("第 3 行：机体角速度", fontsize=10)
    ax.legend(fontsize=6, ncol=3, frameon=False)

    ax = axs[3]
    for i, nm in enumerate(("升降舵 δe", "副翼 δa", "方向舵 δr")):
        for m in MODES:
            r = data[m][0]
            ax.plot(r["t"], r[["de", "da", "dr"][i]], **{**STYLE[m], "lw": 1.0,
                    "label": "%s %s" % (STYLE[m]["label"], nm)})
    for lim in (25.0, 20.0):
        ax.axhline(lim, color="k", lw=0.6, ls=":")
        ax.axhline(-lim, color="k", lw=0.6, ls=":")
    ax.text(0.99, 0.92, "点线 = 偏角限幅 25°/20°/20°", transform=ax.transAxes,
            ha="right", fontsize=7)
    ax.set_ylabel("偏角 [°]")
    ax.set_title("第 4 行：实际舵面偏角", fontsize=10)
    ax.legend(fontsize=6, ncol=3, frameon=False)

    ax = axs[4]
    for m in MODES:
        r = data[m][0]
        ax.plot(r["t"], r["M_aero_y"], **{**STYLE[m], "label": STYLE[m]["label"] + " My"})
    ax.set_ylabel("气动力矩 [N·m]")
    ax.set_title("第 5 行：气动输出力矩（俯仰轴 My）", fontsize=10)
    _startup_note(ax)
    _legend(ax)

    ax = axs[5]
    for i, nm in enumerate(("Mx", "My", "Mz")):
        for m in MODES:
            r = data[m][0]
            ax.plot(r["t"], r[["M_cmg_x", "M_cmg_y", "M_cmg_z"][i]],
                    **{**STYLE[m], "lw": 1.0,
                       "label": "%s %s" % (STYLE[m]["label"], nm)})
    ax.set_ylabel("CMG 力矩 [N·m]")
    ax.set_title("第 6 行：CMG 三轴输出力矩（A 方案结构性为零，非缺数据）", fontsize=10)
    ax.legend(fontsize=6, ncol=3, frameon=False)

    ax = axs[6]
    for m in MODES:
        r = data[m][0]
        ax.plot(r["t"], r["H_norm"], **STYLE[m])
    ax.axhline(Hbud, color="#d62728", lw=0.8, ls=":",
               label="控制器动量预算 %.0f N·m·s" % Hbud)
    ax.axhline(Htot, color="k", lw=0.8, ls="-.",
               label="硬件集群角动量 %.0f N·m·s" % Htot)
    ax.set_ylabel("‖H_cmg‖ [N·m·s]")
    ax.set_title("第 7 行：CMG 角动量范数与预算/硬件线", fontsize=10)
    _legend(ax)

    ax = axs[7]
    for m in MODES:
        r = data[m][0]
        ax.plot(r["t"], r["qp_fail"], **{**STYLE[m], "lw": 1.0})
        ax.plot(r["t"], r["viol"], **{**STYLE[m], "lw": 1.0, "ls": ":",
                                      "alpha": 0.7})
    ax.set_yscale("symlog", linthresh=1e-6)
    ax.set_ylabel("失败标志 / 违反量")
    ax.set_xlabel("时间 [s]")
    ax.set_title("第 8 行：QP 失败/保底标志（实线）与约束违反量（点线，symlog 轴）",
                 fontsize=10)
    fig.tight_layout()
    path = os.path.join(FIG, "%s_tracking.png" % case)
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path


# ---------------------------------------------------------------------------
def maneuver_figure(case):
    data = {}
    for m in MODES:
        rec, meta = load(case, m)
        if rec is not None:
            data[m] = (rec, meta)
    if len(data) < 2:
        print("skip %s (needs A and B)" % case)
        return None
    windows = data["B"][1].get("windows") or data["A"][1].get("windows") or {}
    Hbud, Htot = _budget(data["B"][1])

    fig, axs = plt.subplots(9, 1, figsize=(11, 21), sharex=True)

    ax = axs[0]
    r = data["A"][0]
    ax.plot(r["t"], r["theta_ref"], color="k", lw=1.2, label="目标俯仰角 θ_ref")
    for m in MODES:
        ax.plot(data[m][0]["t"], data[m][0]["theta"], **STYLE[m])
    _shade(ax, windows, data["B"][1])
    ax.set_ylabel("θ [°]")
    ax.set_title("%s：第 1 行 目标俯仰角与 A/B 实际俯仰角（灰区 = 机动窗）" % case,
                 fontsize=10)
    _legend(ax)

    ax = axs[1]
    for m in MODES:
        ax.plot(data[m][0]["t"], data[m][0]["err_pitch_deg"], **STYLE[m])
    _shade(ax, windows, data["B"][1])
    ax.set_ylabel("俯仰误差 [°]")
    ax.set_title("第 2 行：俯仰误差（欧拉角差 θ-θ_ref）", fontsize=10)
    _legend(ax)

    ax = axs[2]
    for m in MODES:
        ax.plot(data[m][0]["t"], np.rad2deg(data[m][0]["omega_y"]), **STYLE[m])
    _shade(ax, windows, data["B"][1])
    ax.set_ylabel("q [°/s]")
    ax.set_title("第 3 行：俯仰角速度", fontsize=10)
    _startup_note(ax)
    _legend(ax)

    ax = axs[3]
    for m in MODES:
        ax.plot(data[m][0]["t"], data[m][0]["de"], **STYLE[m])
    ax.axhline(25.0, color="k", lw=0.7, ls=":")
    ax.axhline(-25.0, color="k", lw=0.7, ls=":")
    ax.text(0.99, 0.9, "点线 = 升降舵限幅 ±25°", transform=ax.transAxes,
            ha="right", fontsize=7)
    ax.set_ylabel("δe [°]")
    ax.set_title("第 4 行：升降舵偏角与限幅", fontsize=10)
    _legend(ax)

    ax = axs[4]
    for m in MODES:
        ax.plot(data[m][0]["t"], data[m][0]["M_aero_y"], **STYLE[m])
    ax.set_ylabel("气动力矩 [N·m]")
    ax.set_title("第 5 行：气动俯仰输出力矩", fontsize=10)
    _startup_note(ax)
    _legend(ax)

    ax = axs[5]
    for m in MODES:
        ax.plot(data[m][0]["t"], data[m][0]["M_cmg_y"], **STYLE[m])
    ax.set_ylabel("CMG 力矩 [N·m]")
    ax.set_title("第 6 行：CMG 俯仰输出力矩（A 方案结构性为零）", fontsize=10)
    _legend(ax)

    ax = axs[6]
    for m in MODES:
        ax.plot(data[m][0]["t"], data[m][0]["H_norm"], **STYLE[m])
    ax.axhline(Hbud, color="#d62728", lw=0.8, ls=":",
               label="控制器动量预算 %.0f N·m·s" % Hbud)
    ax.axhline(Htot, color="k", lw=0.8, ls="-.",
               label="硬件集群角动量 %.0f N·m·s" % Htot)
    ax.set_ylabel("‖H_cmg‖ [N·m·s]")
    ax.set_title("第 7 行：CMG 角动量范数与预算线", fontsize=10)
    _legend(ax)

    ax = axs[7]
    rB = data["B"][0]
    for i, col in enumerate(("#d62728", "#ff9896", "#8c564b", "#e377c2")):
        ax.plot(rB["t"], rB["gimbal_%d" % (i + 1)], color=col, lw=0.7,
                label="B γ%d" % (i + 1) if i < 2 else None)
    ax.set_ylabel("框架角 γ [°]")
    ax2 = ax.twinx()
    ax2.plot(data["A"][0]["t"], data["A"][0]["sigma"], **{**STYLE["A"], "lw": 1.0})
    ax2.plot(rB["t"], rB["sigma"], **{**STYLE["B"], "lw": 1.0})
    floor = _sigma_floor(data["B"][1])
    ax2.axhline(floor, color="k", lw=0.7, ls="-.",
                label="奇异下限 %.1f N·m·s" % floor)
    ax2.set_ylabel("σ_min(A) [N·m·s]（A=蓝/B=红）")
    ax.set_title("第 8 行：框架角（左轴，仅 B 有 CMG）与最小奇异度量 σ_min（右轴）",
                 fontsize=10)
    handles = [Line2D([], [], color="#d62728", lw=0.9, label="B γ1"),
               Line2D([], [], color="#ff9896", lw=0.9, label="B γ2"),
               Line2D([], [], color=STYLE["A"]["color"], lw=1.2, label="σ_min A"),
               Line2D([], [], color=STYLE["B"]["color"], lw=1.2, label="σ_min B"),
               Line2D([], [], color="k", lw=0.8, ls="-.", label="奇异下限 %.1f" % floor)]
    ax.legend(handles=handles, fontsize=5.5, ncol=2, frameon=False, loc="upper left")

    ax = axs[8]
    for m in MODES:
        ax.plot(data[m][0]["t"], data[m][0]["qp_fail"], **{**STYLE[m], "lw": 1.0})
        ax.plot(data[m][0]["t"], data[m][0]["viol"], **{**STYLE[m], "lw": 1.0,
                                                        "ls": ":", "alpha": 0.7})
    ax.set_yscale("symlog", linthresh=1e-6)
    ax.set_ylabel("失败标志 / 违反量")
    ax.set_xlabel("时间 [s]")
    ax.set_title("第 9 行：求解器成功/保底状态与约束违反量", fontsize=10)
    fig.tight_layout()
    path = os.path.join(FIG, "%s_maneuver.png" % case)
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path


# ---------------------------------------------------------------------------
def overview_figure(cases):
    """One compact comparison of the headline RMSE / peak / saturation numbers."""
    rows = []
    for c in cases:
        ents = {}
        for m in MODES:
            p = os.path.join(RES, c, m, "metrics.json")
            if os.path.exists(p):
                with open(p, encoding="utf-8") as fh:
                    ents[m] = json.load(fh)
        if len(ents) == 2:
            rows.append((c, ents))
    if not rows:
        return None
    fields = [("err_rmse_deg", "三轴 RMSE [°]"), ("err_peak_deg", "误差峰值 [°]"),
              ("delta_sat_frac_all", "舵面饱和比例"),
              ("cmg_momentum_peak_Nms", "CMG 峰值角动量 [N·m·s]")]
    fig, axs = plt.subplots(1, len(fields), figsize=(4 * len(fields), 4.2))
    x = np.arange(len(rows))
    for ax, (key, label) in zip(np.atleast_1d(axs), fields):
        va = [r[1]["A"].get(key) or 0.0 for r in rows]
        vb = [r[1]["B"].get(key) or 0.0 for r in rows]
        ax.bar(x - 0.2, va, 0.4, color=STYLE["A"]["color"], label=STYLE["A"]["label"])
        ax.bar(x + 0.2, vb, 0.4, color=STYLE["B"]["color"], label=STYLE["B"]["label"])
        ax.set_xticks(x)
        ax.set_xticklabels([r[0] for r in rows], rotation=45, ha="right", fontsize=7)
        ax.set_title(label, fontsize=9)
        ax.grid(axis="y", alpha=0.3)
    np.atleast_1d(axs)[0].legend(fontsize=7, frameon=False)
    fig.tight_layout()
    path = os.path.join(FIG, "overview_all_cases.png")
    fig.savefig(path, dpi=130)
    plt.close(fig)
    return path


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--cases", nargs="*", default=None)
    a = ap.parse_args(argv)
    os.makedirs(FIG, exist_ok=True)
    cases = a.cases
    if cases is None:
        cases = sorted(d for d in os.listdir(RES)
                       if os.path.isdir(os.path.join(RES, d))
                       and d not in ("_cache", "_logs", "baseline_legacy"))
    for c in cases:
        kind = "maneuver" if ("maneuver" in c) else "tracking"
        p = maneuver_figure(c) if kind == "maneuver" else tracking_figure(c)
        print("wrote", p)
    p = overview_figure(cases)
    print("wrote", p)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
