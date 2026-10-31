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
def fig_scenario(phase, sc, a, b, out_png):
    plt = _style()
    fig, axes = plt.subplots(8, 1, figsize=(10.5, 21.0), sharex=True)
    ma, mb = a["_metrics"], b["_metrics"]
    t = b["t_next"]
    title = ("%s  %s   A = 无 CMG (nx=%d, nu=%d)   B = 舵面+CMG (nx=%d, nu=%d)"
             % (sc, phase, ma["nx_model"], ma["nu_model"],
                mb["nx_model"], mb["nu_model"]))
    fig.suptitle(title, fontsize=11, y=0.995)

    # 1 reference and actual attitude
    ax = axes[0]
    for i, lab in enumerate(("q0", "q1", "q2", "q3")):
        ax.plot(a["t_next"], a[lab], lw=0.8, ls="--", color="C%d" % i,
                label="A q%d" % i)
        ax.plot(b["t_next"], b[lab], lw=1.0, color="C%d" % i, label="B q%d" % i)
    ax.set_ylabel("quaternion [–]")
    ax.legend(ncol=4, loc="best")
    ax.grid(alpha=0.3)
    ax.set_title("1  参考姿态与两组实际姿态（四元数；参考见 figures/…/references/）")

    # 2 attitude error
    ax = axes[1]
    ax.plot(a["t_next"], a["att_err_deg"], lw=1.2, color="C3", label="A 无 CMG")
    ax.plot(b["t_next"], b["att_err_deg"], lw=1.2, color="C0", label="B 舵面+CMG")
    for key, st in (("man0", 0), ("man1", 1)):
        w = mb["windows"].get(key)
        if w:
            ax.axvspan(w["t0"], w["t1"], color="0.85", alpha=0.45, zorder=0)
    ax.set_yscale("symlog", linthresh=0.1)
    ax.set_ylabel("姿态误差 [deg]")
    ax.legend(loc="best")
    ax.grid(alpha=0.3)
    ax.set_title("2  短弧姿态误差（灰底 = 机动窗口）")

    # 3 aerodynamic torque vs bounds
    ax = axes[2]
    for i, name in enumerate(AX):
        ax.plot(b["t_next"], b["tau_act_%s" % name], lw=1.0, color="C%d" % i,
                label="B τ%s" % name)
        ax.plot(a["t_next"], a["tau_act_%s" % name], lw=1.0, ls="--",
                color="C%d" % i, alpha=0.8, label="A τ%s" % name)
        ax.plot(b["t_next"], b["hi_%s" % name], lw=0.6, ls=":", color="C%d" % i)
        ax.plot(b["t_next"], b["lo_%s" % name], lw=0.6, ls=":", color="C%d" % i)
    ax.set_ylabel("气动力矩 [N·m]")
    ax.legend(ncol=3, loc="best")
    ax.grid(alpha=0.3)
    ax.set_title("3  三轴实际气动力矩（虚线 = A，点线 = 实时上下限）")

    # 4 CMG torque
    ax = axes[3]
    for i, name in enumerate(AX):
        ax.plot(b["t_next"], b["cmg_%s" % name], lw=1.1, color="C%d" % i,
                label="CMG τ%s（区间平均）" % name)
        ax.plot(b["t_next"], b["cmg_end_%s" % name], lw=0.7, ls=":", color="C%d" % i)
    ax.legend(ncol=3, loc="best")
    ax.set_ylabel("CMG 力矩 [N·m]")
    if ma.get("cmg_present") is False:
        ax.text(0.01, 0.04, "A 组：不适用（无 CMG）", transform=ax.transAxes,
                fontsize=8.5, color="0.35")
    ax.grid(alpha=0.3)
    ax.set_title("4  B 组三轴 CMG 实际输出力矩（实线区间平均，点线周期末瞬时）")

    # 5 gimbal angles and rates
    ax = axes[4]
    for i in range(4):
        ax.plot(b["t_next"], np.rad2deg(b["g%d" % (i + 1)]), lw=1.0,
                color="C%d" % i, label="gimbal g%d [deg]" % (i + 1))
    ax.set_ylabel("框架角 [deg]")
    ax2 = ax.twinx()
    for i in range(4):
        ax2.plot(b["t_next"], b["nu%d" % (i + 1)], lw=0.7, ls=":",
                 color="C%d" % i)
    ax2.set_ylabel("框架速率 [rad/s]（点线）")
    ax.legend(ncol=4, loc="upper left")
    if ma.get("cmg_present") is False:
        ax.text(0.01, 0.04, "A 组：不适用（无 CMG）", transform=ax.transAxes,
                fontsize=8.5, color="0.35")
    ax.grid(alpha=0.3)
    ax.set_title("5  B 组四台框架角与框架速率")

    # 6 momentum
    ax = axes[5]
    for i, name in enumerate(AX):
        ax.plot(b["t_next"], b["H%s" % name], lw=1.0, color="C%d" % i,
                label="H%s" % name)
    ax.plot(b["t_next"], b["H_norm"], lw=1.2, color="k", label="|H|")
    ax.legend(ncol=4, loc="best")
    ax.set_ylabel("角动量 [N·m·s]")
    if ma.get("cmg_present") is False:
        ax.text(0.01, 0.04, "A 组：不适用（无 CMG）", transform=ax.transAxes,
                fontsize=8.5, color="0.35")
    ax.grid(alpha=0.3)
    ax.set_title("6  B 组三轴角动量与模长")

    # 7 sigma_min and task-direction capability
    ax = axes[6]
    cap, cap_min = cmg_capability_series(b)
    ax.plot(b["t_next"], b["sigma_min"], lw=1.0, color="C4", label="sigma_min(A)")
    ax.plot(b["t_next"], cap_min, lw=1.0, ls="--", color="C4",
            label="sigma_min x gdot_max（各向同性保证力矩）")
    ax.plot(b["t_next"], cap, lw=1.1, color="C1",
            label="沿任务方向的瞬时力矩能力")
    ax.plot(b["t_next"], np.abs(b["d_x"]) + np.abs(b["d_y"]) + np.abs(b["d_z"]),
            lw=0.8, ls=":", color="0.4", label="|d|_1（预测力矩缺口诊断）")
    ax.legend(ncol=2, loc="best")
    ax.set_ylabel("sigma_min [N·m·s] / 力矩 [N·m]")
    if ma.get("cmg_present") is False:
        ax.text(0.01, 0.04, "A 组：不适用（无 CMG）", transform=ax.transAxes,
                fontsize=8.5, color="0.35")
    ax.grid(alpha=0.3)
    ax.set_title("7  B 组 sigma_min 与任务方向的瞬时力矩能力（角动量储备 ≠ 力矩能力）")

    # 8 solver reliability
    ax = axes[7]
    ax.plot(a["t_next"], a["cpu_s"], lw=0.9, color="C3", label="A 求解耗时 [s]")
    ax.plot(b["t_next"], b["cpu_s"], lw=0.9, color="C0", label="B 求解耗时 [s]")
    ax.plot(a["t_next"], np.maximum(a["viol"], 1e-12), lw=0.7, ls=":",
            color="C3", label="A 最大约束违反")
    ax.plot(b["t_next"], np.maximum(b["viol"], 1e-12), lw=0.7, ls=":",
            color="C0", label="B 最大约束违反")
    ax.axhline(0.2, color="k", ls="--", lw=0.8, label="控制周期 0.2 s")
    ax.set_yscale("log")
    ax.set_ylabel("求解耗时 [s] / 约束违反")
    ax.set_xlabel("t [s]")
    ax2 = ax.twinx()
    ax2.step(a["t_next"], a["fallback"], where="post", color="C3", alpha=0.35)
    ax2.step(b["t_next"], b["fallback"], where="post", color="C0", alpha=0.35)
    ax2.set_ylabel("fallback 标志（阶梯）")
    ax2.set_ylim(-0.05, 1.5)
    ax.legend(loc="upper left", ncol=2)
    ax.grid(alpha=0.3)
    ax.set_title("8  两组求解耗时（对数）、最大约束违反、fallback")
    fig.tight_layout(rect=(0, 0, 1, 0.995))
    FIG.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_png, dpi=130)
    plt.close(fig)


# --------------------------------------------------------------------------
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
    fig.savefig(FIG / ("cross_%s_rmse.png" % phase), dpi=150)
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
    fig.savefig(FIG / ("cross_%s_tau.png" % phase), dpi=150)
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
    fig.savefig(FIG / ("cross_%s_cmg.png" % phase), dpi=150)
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
    fig.savefig(FIG / ("cross_%s_solver.png" % phase), dpi=150)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--phase", default="all", choices=["P0", "P1", "all"])
    ap.add_argument("--root", default=None)
    ap.add_argument("--tag", default="")
    args = ap.parse_args()
    global RES
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
