"""阶段 9：从原始时序与指标自动生成汇总表与全部图件（指导实施文件 §10）。

    python experiments/cmg_incremental_search/scripts/make_reports.py

图件全部由 results/raw/<run>/timeseries.npz 与 results/metrics/*.json 生成，
**不允许手工改数据**。输出：
    results/metrics/summary_by_case.csv / .md      分组汇总（均值/标准差/最坏）
    results/figures/fig01..fig10*.png              §10 要求的 10 类图
"""

from __future__ import annotations

import csv
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt          # noqa: E402
import numpy as np                       # noqa: E402

# 图里要写中文：显式挑一个本机可用的 CJK 字体，缺失时回退 DejaVu（届时中文会显示为方块）
for _f in ("Microsoft YaHei", "SimHei", "Noto Sans CJK SC", "Source Han Sans SC"):
    if any(_f in f.name for f in matplotlib.font_manager.fontManager.ttflist):
        matplotlib.rcParams["font.sans-serif"] = [_f, "DejaVu Sans"]
        break
matplotlib.rcParams["axes.unicode_minus"] = False

EXP_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(EXP_ROOT))

from src.config import load_config       # noqa: E402
from src.simulation.runner import build_spec  # noqa: E402

RAW = EXP_ROOT / "results" / "raw"
METRICS = EXP_ROOT / "results" / "metrics"
FIGS = EXP_ROOT / "results" / "figures"
CORE = ["A", "B", "C", "D"]
COLORS = {"A": "#7f7f7f", "B": "#1f77b4", "C": "#d62728", "D": "#2ca02c"}


def load_rows(tag: str) -> list[dict]:
    path = METRICS / f"{tag}.csv"
    if not path.exists():
        return []
    with path.open(encoding="utf-8", newline="") as fh:
        rows = list(csv.DictReader(fh))
    for r in rows:
        for k, v in list(r.items()):
            if k in ("case", "initial", "policy"):
                continue
            try:
                r[k] = float(v) if v not in ("True", "False") else (v == "True")
            except ValueError:
                pass
    return rows


def load_ts(name: str) -> dict:
    return dict(np.load(RAW / name / "timeseries.npz"))


def summary_table(rows: list[dict], keys=("att_rmse_deg", "att_peak_deg", "residual_max_nm",
                                          "fallback_count", "sigma_min_min", "ctrl_ms_max",
                                          "search_ms_mean")) -> list[dict]:
    out = []
    cases = sorted({r["case"] for r in rows}, key=lambda c: (len(c), c))
    for case in cases:
        sel = [r for r in rows if r["case"] == case]
        row = {"case": case, "n": len(sel)}
        for k in keys:
            vals = np.array([r[k] for r in sel if isinstance(r.get(k), (int, float))], float)
            row[f"{k}_mean"] = float(np.mean(vals)) if vals.size else float("nan")
            row[f"{k}_std"] = float(np.std(vals, ddof=1)) if vals.size > 1 else 0.0
            row[f"{k}_worst"] = float(np.max(vals)) if vals.size else float("nan")
        out.append(row)
    return out


def write_summary(rows: list[dict]) -> None:
    table = summary_table(rows)
    if not table:
        return
    keys = list(table[0])
    with (METRICS / "summary_by_case.csv").open("w", encoding="utf-8", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        w.writerows(table)
    lines = ["# 分组汇总（5 个初始构型，主场景 100 km 俯仰 10°/20 s）", "",
             "| 案例 | rmse 均值 | rmse 最坏 | 峰值误差 均值 | 残差 max 均值 | 残差 max 最坏 |"
             " 回退 均值 | σ_min 最小 | 控制耗时 最大 | 搜索耗时 均值 |",
             "|---|---|---|---|---|---|---|---|---|---|"]
    for r in table:
        lines.append(
            f"| {r['case']} | {r['att_rmse_deg_mean']:.3f} | {r['att_rmse_deg_worst']:.3f} | "
            f"{r['att_peak_deg_mean']:.3f} | {r['residual_max_nm_mean']:.1f} | "
            f"{r['residual_max_nm_worst']:.1f} | {r['fallback_count_mean']:.1f} | "
            f"{r['sigma_min_min_mean'] if 'sigma_min_min_mean' in r else 0:.2f} | "
            f"{r['ctrl_ms_max_worst']:.1f} ms | {r['search_ms_mean_mean']:.1f} ms |")
    (METRICS / "summary_by_case.md").write_text("\n".join(lines) + "\n", encoding="utf-8",
                                                newline="\n")


def fig_reference_and_tracking(name: str = "B__g1_small") -> None:
    d = load_ts(name)
    t = d["t"]
    fig, ax = plt.subplots(1, 2, figsize=(11, 3.6))
    for i, lab in enumerate(("roll", "pitch", "yaw")):
        ax[0].plot(t, np.asarray(d["q_ref"])[:, i], "--", lw=1.0, label=f"{lab} ref")
        ax[0].plot(t, np.asarray(d["q"])[:, i], lw=1.0, label=f"{lab}")
    ax[0].set_xlabel("t [s]"); ax[0].set_ylabel("quaternion component")
    ax[0].set_title(f"参考与实际姿态（{name}）"); ax[0].legend(fontsize=7, ncol=2)
    for k, lab in (("err_roll_deg", "roll"), ("err_pitch_deg", "pitch"), ("err_yaw_deg", "yaw"),
                   ("err_norm_deg", "geodesic")):
        ax[1].plot(t, d[k], lw=1.0, label=lab)
    ax[1].set_xlabel("t [s]"); ax[1].set_ylabel("error [deg]"); ax[1].set_yscale("symlog",
                                                                               linthresh=0.1)
    ax[1].set_title("三轴姿态误差与测地角"); ax[1].legend(fontsize=7)
    fig.tight_layout(); fig.savefig(FIGS / "fig01_reference_and_error.png", dpi=150)
    plt.close(fig)


def fig_torque_three_way(name: str = "D__g1_small") -> None:
    d = load_ts(name)
    t = d["t"]
    fig, ax = plt.subplots(1, 2, figsize=(11, 3.6))
    ax[0].plot(t, np.linalg.norm(np.atleast_2d(d["tau_req"]), axis=1), label="期望 tau_req")
    ax[0].plot(t, np.linalg.norm(np.atleast_2d(d["tau_pred"]), axis=1), label="预测 tau_pred")
    ax[0].plot(t, np.linalg.norm(np.atleast_2d(d["tau_actual"]), axis=1), label="真实 tau_actual")
    ax[0].set_xlabel("t [s]"); ax[0].set_ylabel("|tau| [N m]")
    ax[0].set_title(f"三路力矩（{name}）"); ax[0].legend(fontsize=8)
    ax[1].plot(t, d["residual_norm"], label="||tau_req - tau_pred||")
    ax[1].plot(t, np.linalg.norm(np.atleast_2d(d["tau_effective"]), axis=1), "--",
               label="等效力矩（反解）")
    ax[1].set_xlabel("t [s]"); ax[1].set_ylabel("N m"); ax[1].set_title("力矩缺口"); ax[1].legend(fontsize=8)
    fig.tight_layout(); fig.savefig(FIGS / "fig02_torque_three_way.png", dpi=150)
    plt.close(fig)


def fig_surfaces(name: str = "B__g1_small") -> None:
    d = load_ts(name)
    t, de = d["t"], np.atleast_2d(d["delta"])
    fig, ax = plt.subplots(1, 2, figsize=(11, 3.6))
    for i, lab in enumerate(("elevator", "aileron", "rudder")):
        ax[0].plot(t, de[:, i], label=lab)
    ax[0].axhline(25.0, color="k", ls=":", lw=0.8); ax[0].axhline(-25.0, color="k", ls=":", lw=0.8)
    ax[0].set_xlabel("t [s]"); ax[0].set_ylabel("deflection [deg]"); ax[0].set_title("舵面偏角与饱和线")
    ax[0].legend(fontsize=8)
    ax[1].plot(t, d["aero_sat_frac"], label="饱和比例")
    ax[1].plot(t, np.linalg.norm(np.atleast_2d(d["M_aero_actual"]), axis=1), label="|M_aero|")
    ax[1].set_xlabel("t [s]"); ax[1].set_title("气动饱和与力矩"); ax[1].legend(fontsize=8)
    fig.tight_layout(); fig.savefig(FIGS / "fig03_surfaces.png", dpi=150)
    plt.close(fig)


def fig_cmg(name: str = "D__g1_small") -> None:
    d = load_ts(name)
    t, g = d["t"], np.rad2deg(np.atleast_2d(d["gamma"]))
    gd = np.atleast_2d(d["gdot"])
    fig, ax = plt.subplots(1, 3, figsize=(13, 3.4))
    for i in range(g.shape[1]):
        ax[0].plot(t, g[:, i], label=f"γ{i + 1}")
        ax[1].plot(t, gd[:, i], label=f"γ̇{i + 1}")
    ax[0].axhline(90, color="k", ls=":", lw=0.8); ax[0].axhline(-90, color="k", ls=":", lw=0.8)
    ax[0].set_title("框架角与 ±90° 行程"); ax[0].set_xlabel("t [s]"); ax[0].set_ylabel("deg")
    ax[0].legend(fontsize=7)
    ax[1].set_title("框架角速度"); ax[1].set_xlabel("t [s]"); ax[1].legend(fontsize=7)
    ax[2].plot(t, np.linalg.norm(np.atleast_2d(d["M_cmg_actual"]), axis=1), label="|M_cmg|")
    ax[2].plot(t, d["H_norm"], label="|H_cmg|")
    ax[2].set_title("CMG 力矩与角动量"); ax[2].set_xlabel("t [s]"); ax[2].legend(fontsize=8)
    fig.tight_layout(); fig.savefig(FIGS / "fig04_cmg.png", dpi=150)
    plt.close(fig)


def fig_singularity(names=("B__g1_small", "D__g1_small")) -> None:
    fig, ax = plt.subplots(1, 2, figsize=(11, 3.6))
    cfg = load_config("search.yaml", "scenario_main.yaml")
    floor = build_spec(cfg).cmg.sigma_floor
    for name in names:
        d = load_ts(name)
        ax[0].plot(d["t"], d["sigma_min"], label=name)
        ax[1].plot(d["t"], d["gamma_travel_frac"], label=name)
    ax[0].axhline(floor, color="k", ls=":", lw=0.9, label="奇异地板")
    ax[0].set_title("σ_min(A) 与奇异地板"); ax[0].set_xlabel("t [s]"); ax[0].legend(fontsize=8)
    ax[1].set_title("行程占用 |γ|/90°"); ax[1].set_xlabel("t [s]"); ax[1].legend(fontsize=8)
    fig.tight_layout(); fig.savefig(FIGS / "fig05_singularity.png", dpi=150)
    plt.close(fig)


def fig_demand_gap(names=("B__g1_small", "D__g1_small")) -> None:
    fig, ax = plt.subplots(figsize=(7, 3.6))
    for name in names:
        d = load_ts(name)
        ax.plot(d["t"], d["residual_norm"], label=name)
    ax.set_xlabel("t [s]"); ax.set_ylabel("||tau_req - tau_pred|| [N m]")
    ax.set_title("未来需求与可实现力矩缺口"); ax.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(FIGS / "fig06_demand_gap.png", dpi=150)
    plt.close(fig)


def fig_search_cost(rows: list[dict]) -> None:
    sel = [r for r in rows if r["case"] in ("C", "D")]
    if not sel:
        return
    fig, ax = plt.subplots(1, 2, figsize=(11, 3.6))
    labels = sorted({r["initial"] for r in sel})
    x = np.arange(len(labels))
    for i, case in enumerate(("C", "D")):
        means = [np.mean([r["search_ms_mean"] for r in sel
                          if r["case"] == case and r["initial"] == lab] or [0]) for lab in labels]
        maxs = [np.max([r["search_ms_max"] for r in sel
                        if r["case"] == case and r["initial"] == lab] or [0]) for lab in labels]
        ax[0].bar(x + (i - 0.5) * 0.35, means, 0.35, label=f"{case} 均值", color=COLORS[case])
        ax[1].bar(x + (i - 0.5) * 0.35, maxs, 0.35, label=f"{case} 最坏", color=COLORS[case])
    for a in ax:
        a.set_xticks(x); a.set_xticklabels(labels, rotation=20, fontsize=8)
        a.set_ylabel("ms"); a.legend(fontsize=8)
    ax[0].set_title("每周期搜索耗时（均值）"); ax[1].set_title("每周期搜索耗时（最坏）")
    ax[1].axhline(200.0, color="k", ls="--", lw=0.9)
    fig.tight_layout(); fig.savefig(FIGS / "fig08_search_cost.png", dpi=150)
    plt.close(fig)


def fig_abcd_compare(rows: list[dict]) -> None:
    sel = [r for r in rows if r["case"] in CORE]
    if not sel:
        return
    fig, ax = plt.subplots(1, 3, figsize=(13, 3.4))
    metrics = (("att_rmse_deg", "姿态 RMSE [deg]"), ("residual_max_nm", "残差最大值 [N m]"),
               ("fallback_count", "安全回退次数"))
    for k, (key, title) in enumerate(metrics):
        data = [[r[key] for r in sel if r["case"] == c] for c in CORE]
        ax[k].boxplot(data, tick_labels=CORE)
        ax[k].set_title(title)
    fig.tight_layout(); fig.savefig(FIGS / "fig09_abcd_summary.png", dpi=150)
    plt.close(fig)


def fig_failure_mechanism() -> None:
    names = [("B__g4_edge", "B / 边缘初值"), ("D__g4_edge", "D / 边缘初值")]
    ok = all((RAW / n).exists() for n, _ in names)
    if not ok:
        return
    fig, ax = plt.subplots(1, 2, figsize=(11, 3.6))
    for name, lab in names:
        d = load_ts(name)
        ax[0].plot(d["t"], d["err_norm_deg"], label=lab)
        ax[1].plot(d["t"], d["gamma_travel_frac"], label=lab)
    ax[0].set_title("边缘初值的姿态误差"); ax[0].set_xlabel("t [s]"); ax[0].set_ylabel("deg")
    ax[1].set_title("行程占用（失败机理）"); ax[1].set_xlabel("t [s]")
    for a in ax:
        a.legend(fontsize=8)
    fig.tight_layout(); fig.savefig(FIGS / "fig10_failure_mechanism.png", dpi=150)
    plt.close(fig)


def fig_path_trajectory() -> None:
    """构型迁移路径：D 组某次运行的框架角轨迹（4 维两两投影）。"""
    name = "D__g1_small"
    if not (RAW / name).exists():
        return
    d = load_ts(name)
    g = np.rad2deg(np.atleast_2d(d["gamma"]))
    fig, ax = plt.subplots(1, 2, figsize=(10, 3.8))
    ax[0].plot(g[:, 0], g[:, 1], lw=0.9)
    ax[0].scatter(g[0, 0], g[0, 1], c="k", s=12, label="start")
    ax[0].scatter(g[-1, 0], g[-1, 1], c="r", s=12, label="end")
    ax[0].set_xlabel("γ1 [deg]"); ax[0].set_ylabel("γ2 [deg]"); ax[0].set_title("构型迁移（γ1-γ2）")
    ax[0].legend(fontsize=8)
    ax[1].plot(g[:, 2], g[:, 3], lw=0.9)
    ax[1].set_xlabel("γ3 [deg]"); ax[1].set_ylabel("γ4 [deg]"); ax[1].set_title("构型迁移（γ3-γ4）")
    fig.tight_layout(); fig.savefig(FIGS / "fig07_configuration_path.png", dpi=150)
    plt.close(fig)


def main() -> int:
    FIGS.mkdir(parents=True, exist_ok=True)
    rows = load_rows("matrix_core") + load_rows("matrix_ablation")
    if not rows:
        print("缺少 matrix_core / matrix_ablation 指标，先运行 run_matrix.py")
        return 1
    write_summary([r for r in rows if r["case"] in CORE])
    fig_reference_and_tracking()
    fig_torque_three_way()
    fig_surfaces()
    fig_cmg()
    fig_singularity()
    fig_demand_gap()
    fig_path_trajectory()
    fig_search_cost(rows)
    fig_abcd_compare(rows)
    fig_failure_mechanism()
    made = sorted(p.name for p in FIGS.glob("fig*.png"))
    print(f"{len(made)} figures -> {FIGS}")
    for m in made:
        print("  ", m)
    return 0


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    raise SystemExit(main())
