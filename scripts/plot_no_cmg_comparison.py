#!/usr/bin/env python
"""Figures of the no-CMG (A) vs surface+CMG (B) comparison.

Per scenario (implementation guide §12):
  1  three-axis reference and actual attitude            7  B: sigma_min and the
  2  attitude error                                         instantaneous torque
  3  achieved aerodynamic torque with its bounds            capability along the
  4  B only: achieved CMG torque                            task direction
  5  B only: four gimbal angles and gimbal rates          8  solve time, fallback
  6  B only: three-axis cluster momentum                     and constraint violation

Cross-scenario figures: manoeuvre-window RMSE, surface RMS / peak torque,
B-group pre-manoeuvre momentum vector, gimbal-rate saturation, solver cost and
fallback.

Every curve comes from the run's timeseries.csv; nothing is synthesised.  In the
A group the CMG panels carry an explicit "not applicable" note instead of a
fabricated zero curve.

python scripts/plot_no_cmg_comparison.py [--phase P0|P1]
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from nearspace.cmg import CMGCluster, CMGParams          # noqa: E402
from nearspace.config import load_config                # noqa: E402

RES = ROOT / "results" / "torque_gap_no_cmg_comparison"   # overridden by --root
FIG = ROOT / "figures" / "torque_gap_no_cmg_comparison"
TAG = ""
SCENARIO_ORDER = ["S00_pitch30", "S01_pitch_minus30", "S02_pitch10", "S03_pitch45",
                  "S04_roll30", "S05_yaw30", "S06_pitch30_roll20", "S07_sequential"]
AX = ("x", "y", "z")


# --------------------------------------------------------------------------
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
    })
    return plt


def load_run(d):
    d = Path(d)
    with open(d / "timeseries.csv", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    out = {}
    for key in rows[0]:
        try:
            vals = np.array([np.nan if r[key] in ("", None) else float(r[key])
                             for r in rows])
        except ValueError:               # text columns such as `status`
            vals = np.array([r[key] for r in rows], dtype=object)
        out[key] = vals
    out["_metrics"] = json.loads((d / "metrics.json").read_text(encoding="utf-8"))
    out["_meta"] = json.loads((d / "meta.json").read_text(encoding="utf-8"))
    return out


def run_dir(phase, scenario, arm):
    p = RES / phase / ("%s_%s" % (scenario, arm))
    return p if (p / "metrics.json").exists() else None


def capability_along(cluster, gdot_max, gamma, direction):
    """Max torque the cluster can deliver along `direction` at `gamma`.

    max over |nu_i| <= gdot of u^T(-A nu) = gdot * ||A^T u||_1  (dual norm).
    """
    A = cluster.jacobian(gamma)
    u = np.asarray(direction, float)
    n = np.linalg.norm(u)
    if n < 1e-12:
        return np.nan
    return float(gdot_max * np.abs(A.T @ (u / n)).sum())


def cmg_capability_series(run):
    cfg_path = run["_meta"].get("config_path")
    try:
        cfg = load_config(str(ROOT / cfg_path))
        prm = CMGParams.from_config(cfg)
    except Exception:                                        # noqa: BLE001
        e = run["_meta"]["experiment"]["c_mg"]
        prm = CMGParams(h0=float(e["h0"]), gdot_max=float(e["gdot_max"]),
                        gddot_max=float(e["gddot_max"]))
    cl = CMGCluster(prm)
    gdot = float(prm.gdot_max)
    n = len(run["t"])
    cap = np.full(n, np.nan)
    cap_min = np.full(n, np.nan)
    for i in range(n):
        g = np.array([run["g%d" % (k + 1)][i] for k in range(4)])
        if not np.all(np.isfinite(g)):
            continue
        d = np.array([run["d_%s" % ax][i] for ax in AX])
        A = cl.jacobian(g)
        cap_min[i] = cl.sigma_min(A) * gdot
        if np.linalg.norm(d) > 1e-9:
            cap[i] = capability_along(cl, gdot, g, d)
    return cap, cap_min


# --------------------------------------------------------------------------
def _euler_deg(q):
    """ZYX Euler angles [deg] from a stack of scalar-first quaternions."""
    q = np.asarray(q, float)
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    roll = np.degrees(np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y)))
    pitch = np.degrees(np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0)))
    yaw = np.degrees(np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z)))
    return np.column_stack([roll, pitch, yaw])


def _style():
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["Microsoft YaHei", "SimHei", "DejaVu Sans"],
        "axes.unicode_minus": False, "mathtext.fontset": "dejavusans",
        "font.size": 9, "axes.labelsize": 9, "axes.titlesize": 10,
        "xtick.labelsize": 8, "ytick.labelsize": 8, "legend.fontsize": 7.5,
        "axes.linewidth": 0.8, "lines.linewidth": 1.1,
        "grid.linewidth": 0.5, "grid.alpha": 0.35,
        "pdf.fonttype": 42, "ps.fonttype": 42, "savefig.bbox": "tight",
    })
    return plt


C_REF = "#2b3a55"
C_A = "tab:red"
C_B = "tab:blue"
C_SURF = "tab:green"
C_CMG = "tab:red"
C_NOM = "tab:blue"
C_BAND = "lightgray"
AXC = {"roll": 0, "pitch": 1, "yaw": 2}
AXNAME = {"roll": "滚转", "pitch": "俯仰", "yaw": "偏航"}
AXSYM = {"roll": "x", "pitch": "y", "yaw": "z"}


def _fit(ax, arrays, headroom=0.55, margin=0.15):
    """Fit the y range to the informative curves, leaving room for the legend.

    The aerodynamic torque envelope grows by an order of magnitude in the last
    seconds of the window (the vehicle descends, see the report section 15.6);
    plotting it to scale would flatten everything else, so the *envelope* is
    allowed to leave the axes and the annotation says so.
    """
    vals = []
    for a in arrays:
        a = np.asarray(a, float)
        a = a[np.isfinite(a)]
        if a.size:
            vals.append(a)
    if not vals:
        return False, 0.0, 0.0
    v = np.concatenate(vals)
    lo, hi = float(np.min(v)), float(np.max(v))
    span = max(hi - lo, 1e-6)
    ax.set_ylim(lo - margin * span, hi + (margin + headroom) * span)
    return True, lo - margin * span, hi + (margin + headroom) * span


def _headroom(ax, frac=0.42):
    """Expand the top of the y range so a framed legend cannot cover data."""
    lo_, hi_ = ax.get_ylim()
    span = max(hi_ - lo_, 1e-9)
    ax.set_ylim(lo_, hi_ + frac * span)


def fig_scenario(phase, sc, a, b, out_png):
    """A/B control-process overview, laid out like figures/torque_gap_noMod/C1_noMod_overview.png."""
    plt = _style()
    ma, mb = a["_metrics"], b["_metrics"]
    n = min(len(a["t_next"]), len(b["t_next"]))
    t = b["t_next"][:n]
    # main manoeuvre axis (first enabled manoeuvre of the scenario)
    wins = mb.get("windows") or {}
    axis = (wins.get("man0") or {}).get("axis") or "pitch"
    if axis not in AXC:
        axis = "pitch"
    k, sym = AXC[axis], AXSYM[axis]
    ax_name = AXNAME[axis]

    EA = _euler_deg(np.column_stack([a["q%d" % i][:n] for i in range(4)]))
    EB = _euler_deg(np.column_stack([b["q%d" % i][:n] for i in range(4)]))
    EREF = _euler_deg(np.column_stack([b["qref%d" % i][:n] for i in range(4)]))
    r2d = 180.0 / np.pi
    errA, errB = a["att_err_deg"][:n], b["att_err_deg"][:n]
    compA = np.column_stack([a["err_x"][:n], a["err_y"][:n], a["err_z"][:n]]) * r2d
    compB = np.column_stack([b["err_x"][:n], b["err_y"][:n], b["err_z"][:n]]) * r2d
    tauA = a["tau_act_%s" % sym][:n]
    tauB = b["tau_act_%s" % sym][:n]
    tau_nom = b["tau_nom_%s" % sym][:n]
    lo, hi = b["lo_%s" % sym][:n], b["hi_%s" % sym][:n]
    gap = b["d_%s" % sym][:n]
    cmg = {ax: b["cmg_%s" % ax][:n] for ax in AX}
    cmg_sum = np.vstack([cmg[ax] for ax in AX]).sum(axis=0)
    g_deg = {i: np.degrees(b["g%d" % (i + 1)][:n]) for i in range(4)}
    nu_dps = {i: np.degrees(b["nu%d" % (i + 1)][:n]) for i in range(4)}
    gdot_dps = None
    try:
        gdot_dps = np.degrees(float(b["_meta"]["experiment"]["c_mg"]["gdot_max"]))
    except Exception:                                            # noqa: BLE001
        pass
    H = {ax: b["H%s" % ax][:n] for ax in AX}
    Hn = b["H_norm"][:n]
    mig = mb.get("cmg_migration") or {}
    t_mig = mig.get("migration_onset_t")
    t_man0 = (wins.get("man0") or {}).get("t0")
    t_man1 = max([w.get("t1", 0.0) for key, w in wins.items()
                  if key.startswith("man")] or [0.0])
    idx = np.flatnonzero(np.abs(gap) > 1e-6)
    t_gap = float(t[idx[0]]) if idx.size else None

    fig, axes = plt.subplots(9, 1, figsize=(7.6, 15.6), sharex=True,
                             gridspec_kw=dict(hspace=0.16))
    backend = (b["_meta"].get("backend") or "ipopt")
    fig.suptitle("%s  %s（%s 后端）：A 无 CMG vs B 舵面+CMG — 姿态、误差、力矩与能力"
                 % (sc, phase, backend), fontsize=10.5, y=0.995)

    def events(ax):
        if t_mig:
            ax.axvline(t_mig, color="tab:red", ls=":", lw=1.0)
        for key, w in (mb.get("windows") or {}).items():
            if key.startswith("man"):
                ax.axvline(w.get("t0", 0.0), color="k", ls="--", lw=1.0)
                ax.axvline(w.get("t1", 0.0), color="k", ls="--", lw=1.0)
        if t_gap:
            ax.axvline(t_gap, color="tab:blue", ls="-.", lw=0.9)
        ax.grid(True, which="major")

    # 1 attitude, main manoeuvre axis, all in degrees
    ax = axes[0]
    ax.plot(t, EREF[:, k], color=C_REF, lw=1.0, ls="--", label="参考 %s角" % ax_name)
    ax.plot(t, EA[:, k], color=C_A, lw=1.1, label="实际 A（无 CMG）")
    ax.plot(t, EB[:, k], color=C_B, lw=1.3, label="实际 B（舵面+CMG）")
    ax.set_ylabel("%s角 [°]" % ax_name)
    ax.legend(loc="upper left", ncol=3, framealpha=0.9)
    events(ax)
    _headroom(ax, 0.30)
    ev = []
    if t_mig:
        ev.append((t_mig, "迁移开始 $t\\approx%.1f$ s" % t_mig))
    if t_man0:
        ev.append((t_man0, "机动开始 $t=%.0f$ s" % t_man0))
    if t_gap:
        ev.append((t_gap, "缺口出现 $t\\approx%.1f$ s" % t_gap))
    if t_man1:
        ev.append((t_man1, "机动结束 $t=%.0f$ s" % t_man1))
    parts = [txt for _tt, txt in sorted(ev)]
    if parts:
        ax.text(0.995, 1.015, "    ".join(parts), transform=ax.transAxes,
                fontsize=7.6, color="0.25", ha="right", va="bottom")

    # 2 attitude error (short arc), degrees
    ax = axes[1]
    ra = (ma.get("rmse_deg") or {}).get("combined")
    rb = (mb.get("rmse_deg") or {}).get("combined")
    ax.plot(t, errA, color=C_A, lw=1.1,
            label="A 无 CMG（机动窗 RMSE %.3f°，峰值 %.2f°）" % (ra or np.nan, np.nanmax(errA)))
    ax.plot(t, errB, color=C_B, lw=1.3,
            label="B 舵面+CMG（机动窗 RMSE %.3f°，峰值 %.2f°）" % (rb or np.nan, np.nanmax(errB)))
    ax.axhline(1.0, color="0.4", ls=":", lw=0.8)
    ax.set_ylabel("短弧姿态误差 [°]")
    ax.legend(loc="upper left", ncol=1, framealpha=0.9)
    events(ax)
    _headroom(ax, 0.35)

    # 3 three-axis error components, degrees
    ax = axes[2]
    comp_cols = ("#6a3d9a", "#b15928", "#1b7f5a")
    for i, nm in enumerate(("x", "y", "z")):
        ax.plot(t, compA[:, i], color=comp_cols[i], lw=0.9, ls="--", alpha=0.85,
                label="$e_%s$ A（无 CMG）" % nm)
        ax.plot(t, compB[:, i], color=comp_cols[i], lw=1.2,
                label="$e_%s$ B（舵面+CMG）" % nm)
    ax.set_ylabel("误差分量 [°]")
    ax.legend(loc="upper left", ncol=3, framealpha=0.9)
    events(ax)
    _headroom(ax, 0.60)

    # 4 torque decomposition, B group (reference style)
    ax = axes[3]
    ax.plot(t, tau_nom, color=C_NOM, ls="--", lw=1.0, label="$\\tau_{nom,%s}$" % sym)
    ax.plot(t, tauB, color=C_SURF, lw=1.1, label="$\\tau_{a,%s}$（舵面）" % sym)
    ax.plot(t, cmg[sym], color=C_CMG, lw=1.1, label="$\\tau_{cmg,%s}$（CMG）" % sym)
    ax.plot(t, tauB + cmg[sym], color=C_REF, lw=1.0, alpha=0.9,
            label="合成 $\\tau_{a}+\\tau_{cmg}$")
    ax.set_ylabel("%s力矩 [N·m]" % ax_name)
    _fit(ax, [tau_nom, tauB, cmg[sym]], headroom=0.55)
    ax.legend(loc="upper center", ncol=2, framealpha=0.9)
    events(ax)

    # 5 aero envelope with both arms
    ax = axes[4]
    ax.fill_between(t, lo, hi, color=C_BAND, alpha=0.55,
                    label="舵面可实现力矩范围 $[\\tau^-_{%s},\\ \\tau^+_{%s}]$" % (sym, sym))
    ax.plot(t, tau_nom, color=C_NOM, ls="--", lw=1.0, label="参考需求 $\\tau_{nom,%s}$" % sym)
    ax.plot(t, tauA, color=C_A, lw=1.0, alpha=0.9, label="A 实际 $\\tau_{a,%s}$" % sym)
    ax.plot(t, tauB, color=C_SURF, lw=1.2, label="B 实际 $\\tau_{a,%s}$" % sym)
    ax.set_ylabel("%s力矩 [N·m]" % ax_name)
    _fit(ax, [tau_nom, tauB, cmg[sym]], headroom=0.55)
    ax.legend(loc="upper center", ncol=2, framealpha=0.9)
    events(ax)

    # 6 CMG torque, three axes
    ax = axes[5]
    for i, ax_i in enumerate(AX):
        ax.plot(t, cmg[ax_i], color="C%d" % (i + 3), lw=1.1,
                label="$\\tau_{cmg,%s}$（%s）" % (ax_i, AXNAME[[kk for kk, vv in AXC.items() if vv == i][0]]))
    ax.set_ylabel("CMG 力矩 [N·m]")
    _headroom(ax, 0.50)
    ax.legend(loc="upper center", ncol=3, framealpha=0.9)
    if ma.get("cmg_present") is False:
        ax.text(0.01, 0.04, "A 组：不适用（无 CMG）", transform=ax.transAxes,
                fontsize=8, color="0.35")
    events(ax)

    # 7 gimbal angles [deg] and rates [deg/s]
    ax = axes[6]
    for i in range(4):
        ax.plot(t, g_deg[i], color="C%d" % i, lw=1.0, label="框架角 $\\gamma_%d$" % (i + 1))
    ax.set_ylabel("框架角 [°]")
    ax2 = ax.twinx()
    for i in range(4):
        ax2.plot(t, nu_dps[i], color="C%d" % i, lw=0.7, ls=":", alpha=0.9)
    ax2.set_ylabel("框架速率 [°/s]（点线）")
    if gdot_dps:
        ax2.axhline(gdot_dps, color="0.4", ls=":", lw=0.7)
        ax2.axhline(-gdot_dps, color="0.4", ls=":", lw=0.7)
    h1, l1 = ax.get_legend_handles_labels()
    _headroom(ax, 0.55)
    ax.legend(h1, l1, loc="upper left", ncol=4, framealpha=0.9)
    if ma.get("cmg_present") is False:
        ax.text(0.01, 0.04, "A 组：不适用（无 CMG）", transform=ax.transAxes,
                fontsize=8, color="0.35")
    events(ax)

    # 8 cluster momentum + gap, twin axis
    ax = axes[7]
    for i, ax_i in enumerate(AX):
        ax.plot(t, H[ax_i], color="C%d" % i, lw=1.0, label="$H_%s$" % ax_i)
    ax.set_ylabel("角动量 $H$ [N·m·s]")
    if ma.get("cmg_present") is False:
        ax.text(0.99, 0.04, "A 组：不适用（无 CMG）", transform=ax.transAxes,
                fontsize=8, color="0.35", ha="right")
    ax2 = ax.twinx()
    ax2.plot(t, Hn, color=C_REF, lw=1.2, alpha=0.85, label="$|H|$")
    ax2.plot(t, gap, color="0.45", lw=1.0, ls="--",
             label="缺口 $d_%s$（右轴，N·m）" % sym)
    hlim = None
    cm = b["_meta"]["experiment"].get("c_mg")
    if isinstance(cm, dict):
        hlim = 4.0 * float(cm["h0"])
        ax2.axhline(hlim, color="0.5", ls=":", lw=0.8)
        ax2.text(0.995, 0.93, "$4h_0=%.0f$ N·m·s" % hlim, transform=ax2.transAxes,
                 fontsize=7, color="0.35", va="top", ha="right")
    ax2.set_ylabel("$|H|$ / 缺口 [N·m·s] / [N·m]")
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    _headroom(ax, 0.45)
    ax.legend(h1 + h2, l1 + l2, loc="upper left", ncol=3, framealpha=0.9)
    events(ax)

    # 9 solver cost
    ax = axes[8]
    ax.plot(t, 1000.0 * a["cpu_s"][:n], color=C_A, lw=0.9, label="A 求解耗时")
    ax.plot(t, 1000.0 * b["cpu_s"][:n], color=C_B, lw=0.9, label="B 求解耗时")
    ax.axhline(200.0, color="k", ls="--", lw=0.9, label="控制周期 200 ms")
    ax.set_yscale("log")
    ax.set_ylabel("求解耗时 [ms]")
    ax.set_xlabel("时间 $t$ [s]")
    ax2 = ax.twinx()
    ax2.step(t, a["fallback"][:n], where="post", color=C_A, alpha=0.18, lw=0.7)
    ax2.step(t, b["fallback"][:n], where="post", color=C_B, alpha=0.18, lw=0.7)
    ax2.set_ylabel("fallback 标志")
    ax2.set_ylim(-0.05, 1.5)
    ax.legend(loc="lower left", ncol=3, framealpha=0.9)
    events(ax)

    for ax_ in axes[:-1]:
        ax_.tick_params(labelbottom=False)
    FIG.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=140)
    plt.close(fig)


def collect(phase):
    rows = []
    for sc in SCENARIO_ORDER:
        da, db = run_dir(phase, sc, "no_cmg"), run_dir(phase, sc, "with_cmg")
        if not (da and db):
            continue
        a, b = load_run(da), load_run(db)
        rows.append((sc, a, b))
    return rows


def fig_cross(rows, phase):
    plt = _style()
    scs = [r[0] for r in rows]
    xpos = np.arange(len(scs))

    def val(run, key, win="combined", idx=None, default=np.nan):
        st = (run["_metrics"].get("stats") or {}).get(win) or {}
        v = st.get(key, default)
        if isinstance(v, list):
            return v[idx] if idx is not None else v
        return default if v is None else v

    # RMSE
    fig, axs = plt.subplots(1, 2, figsize=(12.0, 3.6))
    ra = [val(a, "rmse_deg") for _sc, a, b in rows]
    rb = [val(b, "rmse_deg") for _sc, a, b in rows]
    w = 0.38
    axs[0].bar(xpos - w / 2, ra, w, label="A 无 CMG", color="C3")
    axs[0].bar(xpos + w / 2, rb, w, label="B 舵面+CMG", color="C0")
    axs[0].set_yscale("log")
    axs[0].set_ylabel("机动窗口 RMSE [deg]")
    axs[0].set_title("各场景机动窗口姿态 RMSE（对数）")
    axs[0].legend()
    axs[0].grid(alpha=0.3, axis="y")
    axs[1].bar(xpos - w / 2, [val(a, "max_err_deg") for _s, a, b in rows], w,
               label="A", color="C3")
    axs[1].bar(xpos + w / 2, [val(b, "max_err_deg") for _s, a, b in rows], w,
               label="B", color="C0")
    axs[1].set_ylabel("机动窗口最大姿态误差 [deg]")
    axs[1].set_title("各场景机动窗口最大姿态误差")
    axs[1].legend()
    axs[1].grid(alpha=0.3, axis="y")
    for a_ in axs:
        a_.set_xticks(xpos)
        a_.set_xticklabels([s.split("_")[0] for s in scs])
    fig.tight_layout()
    fig.savefig(FIG / ("cross_%s%s_rmse.png" % (phase, TAG)), dpi=150)
    plt.close(fig)

    # surface torque
    fig, axs = plt.subplots(1, 2, figsize=(12.0, 3.6))
    for j, (key, lab) in enumerate((("tau_rms", "RMS"), ("tau_peak", "峰值"))):
        for k, (arm, col) in enumerate((("A", "C3"), ("B", "C0"))):
            vals = []
            for _sc, a, b in rows:
                run = a if arm == "A" else b
                arr = [val(run, key, idx=i) for i in range(3)]
                vals.append(float(np.linalg.norm(arr)))
            axs[j].bar(xpos + (k - 0.5) * w, vals, w, label="%s %s" % (arm, "无 CMG" if arm == "A" else "舵面+CMG"),
                       color=col)
        axs[j].set_ylabel("三轴气动舵面力矩 %s 的模长 [N·m]" % lab)
        axs[j].set_title("各场景舵面力矩 %s（x/y/z 模长）" % lab)
        axs[j].legend()
        axs[j].grid(alpha=0.3, axis="y")
        axs[j].set_xticks(xpos)
        axs[j].set_xticklabels([s.split("_")[0] for s in scs])
    fig.tight_layout()
    fig.savefig(FIG / ("cross_%s%s_tau.png" % (phase, TAG)), dpi=150)
    plt.close(fig)

    # B pre-manoeuvre momentum + gimbal-rate saturation
    fig, axs = plt.subplots(1, 2, figsize=(12.0, 3.6))
    Hv = np.array([[b["_metrics"].get("cmg_migration", {}).get("H_at_start", [np.nan] * 3)[i]
                    for i in range(3)] for _s, a, b in rows], float)
    for i, name in enumerate(AX):
        axs[0].bar(xpos + (i - 1) * w, Hv[:, i], w, label="H" + name, color="C%d" % i)
    axs[0].set_ylabel("机动起始时刻角动量 [N·m·s]")
    axs[0].set_title("B 组机动前角动量向量")
    axs[0].legend()
    axs[0].grid(alpha=0.3, axis="y")
    sat = [val(b, "nu_sat_ratio", win="all") for _s, a, b in rows]
    axs[1].bar(xpos, sat, 0.5, color="C1")
    axs[1].set_ylabel("框架速率饱和比例")
    axs[1].set_title("B 组框架速率饱和比例（全程）")
    axs[1].grid(alpha=0.3, axis="y")
    for a_ in axs:
        a_.set_xticks(xpos)
        a_.set_xticklabels([s.split("_")[0] for s in scs])
    fig.tight_layout()
    fig.savefig(FIG / ("cross_%s%s_cmg.png" % (phase, TAG)), dpi=150)
    plt.close(fig)

    # solver cost and fallback
    fig, axs = plt.subplots(1, 2, figsize=(12.0, 3.6))
    ca = [a["_metrics"]["cpu_s"]["mean"] for _s, a, b in rows]
    cb = [b["_metrics"]["cpu_s"]["mean"] for _s, a, b in rows]
    axs[0].bar(xpos - w / 2, ca, w, label="A", color="C3")
    axs[0].bar(xpos + w / 2, cb, w, label="B", color="C0")
    axs[0].axhline(0.2, color="k", ls="--", lw=0.8, label="控制周期 0.2 s")
    axs[0].set_yscale("log")
    axs[0].set_ylabel("平均求解耗时 [s]")
    axs[0].set_title("平均求解耗时（虚线 = 控制周期）")
    axs[0].legend()
    axs[0].grid(alpha=0.3, axis="y")
    fa = [a["_metrics"]["fallback_ratio"] for _s, a, b in rows]
    fb = [b["_metrics"]["fallback_ratio"] for _s, a, b in rows]
    axs[1].bar(xpos - w / 2, fa, w, label="A", color="C3")
    axs[1].bar(xpos + w / 2, fb, w, label="B", color="C0")
    axs[1].set_ylabel("fallback 比例")
    axs[1].set_title("fallback 比例（失败样本保留）")
    axs[1].legend()
    axs[1].grid(alpha=0.3, axis="y")
    for a_ in axs:
        a_.set_xticks(xpos)
        a_.set_xticklabels([s.split("_")[0] for s in scs])
    fig.tight_layout()
    fig.savefig(FIG / ("cross_%s%s_solver.png" % (phase, TAG)), dpi=150)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--phase", default="all", choices=["P0", "P1", "all"])
    ap.add_argument("--root", default=None)
    ap.add_argument("--tag", default="")
    args = ap.parse_args()
    global RES, TAG
    TAG = args.tag
    if args.root:
        RES = (ROOT / args.root) if not Path(args.root).is_absolute() else Path(args.root)
        print("[plot] root =", RES)
    phases = ["P0", "P1"] if args.phase == "all" else [args.phase]
    for phase in phases:
        rows = collect(phase)
        print("[%s] scenarios with both arms: %d" % (phase, len(rows)))
        for sc, a, b in rows:
            out = FIG / (phase + args.tag) / ("%s_AB.png" % sc)
            out.parent.mkdir(parents=True, exist_ok=True)
            fig_scenario(phase, sc, a, b, out)
            print("   ", out.relative_to(ROOT))
        if rows:
            fig_cross(rows, phase)
    return 0


if __name__ == "__main__":
    sys.exit(main())
