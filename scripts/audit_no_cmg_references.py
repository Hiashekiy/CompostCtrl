#!/usr/bin/env python
"""Sample, verify and plot every reference attitude of the A/B scenario set.

The manoeuvre definitions are only YAML parameters; before a formal run the
reference they produce must be sampled and checked (implementation guide §5,
§6, §11 stage 1.5):

  * axis, sign and amplitude of every commanded manoeuvre,
  * start / end time of the manoeuvre window,
  * the peak reference rate against the analytic quintic value,
  * superposition semantics of `Trajectory3D._man_parts()` for the sequential
    case S07 (the second manoeuvre must bring the attitude back to the baseline),
  * that the reference contains no NaN / Inf and is unit-norm.

Every check is made against a *manoeuvre-free* baseline reference built from the
same trajectory, so the slowly varying flight-path pitch and heading are removed
before the manoeuvre offset is measured.

Outputs
    figures/torque_gap_no_cmg_comparison/references/<ID>_reference.png
    figures/torque_gap_no_cmg_comparison/references/all_references.png
    results/torque_gap_no_cmg_comparison_rti2/reference_audit.md / .json

python scripts/audit_no_cmg_references.py
"""
from __future__ import annotations

import copy
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from nearspace.config import _to_ns, read_yaml                  # noqa: E402
from nearspace.model6 import quat_to_euler, wrap_pi             # noqa: E402
from nearspace.torque_gap_scenario import Scenario              # noqa: E402

CFG = ROOT / "configs" / "torque_gap_no_cmg_comparison"
OUT = ROOT / "results" / "torque_gap_no_cmg_comparison_rti2"   # overridden by --out-root
FIG = ROOT / "figures" / "torque_gap_no_cmg_comparison" / "references"
AXES = ("roll", "pitch", "yaw")


def scenario_files():
    out = []
    for sub in ("cases", "sequential"):
        out.extend(sorted((CFG / sub).glob("*_with_cmg.yaml")))
    return out


def check(path):
    cfg = _to_ns(read_yaml(str(path)))
    scn = Scenario(cfg)
    sid = path.name.replace("_with_cmg.yaml", "")
    mans = [m for m in scn.traj.maneuvers if m.enabled]
    base_traj = copy.deepcopy(scn.traj)
    base_traj.maneuvers = []
    t = np.arange(scn.t_start - 5.0, scn.t_end + 5.0 + 1e-9, 0.05)
    Q = np.array([scn.traj.q_ref(x) for x in t])
    Qb = np.array([base_traj.q_ref(x) for x in t])
    E = np.array([quat_to_euler(q) for q in Q])
    Eb = np.array([quat_to_euler(q) for q in Qb])
    # manoeuvre offset per axis, flight-path / heading removed
    off = np.rad2deg(np.array([[wrap_pi(a - b) for a, b in zip(row_e, row_b)]
                               for row_e, row_b in zip(E, Eb)]))
    W = np.array([scn.traj.omega_ref(x) for x in t])
    issues = []
    for name, arr in (("q_ref", Q), ("euler_ref", E), ("omega_ref", W)):
        if not np.all(np.isfinite(arr)):
            issues.append("non-finite %s sample" % name)
    if np.max(np.abs(np.linalg.norm(Q, axis=1) - 1.0)) > 1e-9:
        issues.append("reference quaternion not unit norm")

    rows = []
    # expected manoeuvre offset = sum of the commanded contributions per axis.
    # For S06 this is non-zero on two axes at once; for S07 the second manoeuvre
    # adds -30 deg on top of the held +30 deg, so the sum returns to zero.
    expected = np.zeros_like(off)
    for m in mans:
        k = AXES.index(m.axis)
        expected[:, k] += np.rad2deg(np.array([m.value(x) for x in t]))
    superposition_residual = float(np.max(np.abs(off - expected)))
    if superposition_residual > 1e-9:
        issues.append("manoeuvre superposition residual %.3e deg between the "
                      "reference and the sum of the commanded manoeuvres"
                      % superposition_residual)

    def at(tt):
        return int(np.argmin(np.abs(t - tt)))

    for i, m in enumerate(mans):
        k = AXES.index(m.axis)
        before, after = off[at(m.t0 - 1.0), k], off[at(m.t_end + 1.0), k]
        applied = float(after - before)
        want = float(np.rad2deg(m.delta))
        sel_win = (t >= m.t0 - 1.0) & (t <= m.t_end + 1.0)
        others = {AXES[j]: float(np.max(np.abs(off[sel_win, j] - expected[sel_win, j])))
                  for j in range(3) if j != k}
        worst_other = max(abs(v) for v in others.values()) if others else 0.0
        rate = np.gradient(off[:, k], t)
        peak = float(np.max(np.abs(rate[sel_win])))
        analytic = float(np.rad2deg(m.peak_rate))
        ok_axis = abs(applied - want) < 1e-6
        ok_other = worst_other < 1e-6
        ok_rate = abs(peak - analytic) / max(analytic, 1e-9) < 2e-2
        if not ok_axis:
            issues.append("man%d applied %.6f deg != commanded %.6f deg"
                          % (i, applied, want))
        if not ok_other:
            issues.append("man%d residual on the other axes: %s" % (i, others))
        if not ok_rate:
            issues.append("man%d peak reference rate %.4f deg/s != analytic %.4f"
                          % (i, peak, analytic))
        rows.append(dict(index=i, axis=m.axis, shape=m.shape, t0=m.t0, t_end=m.t_end,
                         commanded_deg=want, applied_deg=applied,
                         other_axes_delta_deg=others,
                         peak_offset_rate_deg_s=peak, analytic_peak_deg_s=analytic,
                         ok=bool(ok_axis and ok_other and ok_rate)))
    if len(mans) > 1:
        # superposition: after the last manoeuvre the offset must be back to the
        # commanded sum (zero for S07, +30 deg for S06 which holds both axes)
        tlast = float(max(m.t_end for m in mans)) + 2.0
        k = AXES.index(mans[0].axis)
        resid = float(off[at(tlast), k] - expected[at(tlast), k])
        mid = float(off[at(max(m.t_end for m in mans[:-1]) + 2.0), k])
        mid_exp = float(expected[at(max(m.t_end for m in mans[:-1]) + 2.0), k])
        if abs(resid) > 1e-6:
            issues.append("hold after the last manoeuvre does not match the "
                          "commanded sum (residual %.6f deg at %.1f s)"
                          % (resid, tlast))
        if abs(mid - mid_exp) > 1e-6:
            issues.append("hold between manoeuvres %.6f deg != commanded %.6f deg"
                          % (mid, mid_exp))
        rows.append(dict(index="return_to_commanded_sum", axis=mans[0].axis,
                         shape="—",
                         t0=max(m.t_end for m in mans[:-1]), t_end=tlast,
                         commanded_deg=expected[at(tlast), k], applied_deg=off[at(tlast), k],
                         other_axes_delta_deg={"residual_deg": resid},
                         peak_offset_rate_deg_s=None, analytic_peak_deg_s=None,
                         ok=bool(abs(resid) <= 1e-6)))
        rows.append(dict(index="hold_after_first", axis=mans[0].axis, shape="—",
                         t0=mans[0].t_end, t_end=mans[1].t0,
                         commanded_deg=mid_exp, applied_deg=mid,
                         other_axes_delta_deg={},
                         peak_offset_rate_deg_s=None, analytic_peak_deg_s=None,
                         ok=bool(abs(mid - mid_exp) <= 1e-6)))
    return dict(scenario=sid, config=str(path.relative_to(ROOT)), t=t, Q=Q, E=E,
                off=off, W=W, maneuvers=rows, issues=issues, ok=not issues,
                t_start=scn.t_start, t_end=scn.t_end,
                superposition_residual_deg=superposition_residual)


def _style(big=True):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    plt.rcParams.update({
        "font.family": "sans-serif",
        "font.sans-serif": ["Microsoft YaHei", "SimHei", "DejaVu Sans"],
        "axes.unicode_minus": False,
        "font.size": 9 if big else 8,
        "axes.labelsize": 9 if big else 8,
        "axes.titlesize": 10 if big else 8,
        "legend.fontsize": 8 if big else 6.5,
    })
    return plt


def plot_one(res):
    plt = _style(True)
    fig, ax = plt.subplots(2, 1, figsize=(9.0, 5.8), sharex=True,
                           gridspec_kw=dict(height_ratios=[2.0, 1.0]))
    for k, name in enumerate(AXES):
        ax[0].plot(res["t"], np.rad2deg(res["E"][:, k]), lw=1.2, label=name)
    for m in [m for m in res["maneuvers"] if isinstance(m["index"], int)]:
        ax[0].axvline(m["t0"], color="0.6", ls=":", lw=0.8)
        ax[0].axvline(m["t_end"], color="0.6", ls=":", lw=0.8)
    ax[0].set_ylabel("reference Euler angle [deg]")
    ax[0].set_title("%s  reference attitude (sim window %.0f-%.0f s)"
                    % (res["scenario"], res["t_start"], res["t_end"]))
    ax[0].legend(loc="best")
    ax[0].grid(alpha=0.3)
    for k, name in enumerate(AXES):
        ax[1].plot(res["t"], res["off"][:, k], lw=1.1, label="offset " + name)
    ax[1].set_ylabel("manoeuvre offset [deg]")
    ax[1].set_xlabel("t [s]")
    ax[1].legend(loc="best")
    ax[1].grid(alpha=0.3)
    fig.tight_layout()
    FIG.mkdir(parents=True, exist_ok=True)
    fig.savefig(FIG / ("%s_reference.png" % res["scenario"]), dpi=150)
    plt.close(fig)


def plot_all(results):
    plt = _style(False)
    n, ncol = len(results), 3
    nrow = int(np.ceil(n / ncol))
    fig, axes = plt.subplots(nrow, ncol, figsize=(3.4 * ncol, 2.2 * nrow))
    axes = np.atleast_2d(axes)
    for i, res in enumerate(results):
        a = axes[i // ncol][i % ncol]
        for k, name in enumerate(AXES):
            a.plot(res["t"], np.rad2deg(res["E"][:, k]), lw=1.0, label=name)
        for m in [m for m in res["maneuvers"] if isinstance(m["index"], int)]:
            a.axvline(m["t0"], color="0.6", ls=":", lw=0.7)
            a.axvline(m["t_end"], color="0.6", ls=":", lw=0.7)
        a.set_title(res["scenario"])
        a.grid(alpha=0.3)
        if i == 0:
            a.legend(loc="best", ncol=3)
        if i % ncol == 0:
            a.set_ylabel("ref Euler [deg]")
        if i // ncol == nrow - 1:
            a.set_xlabel("t [s]")
    for j in range(n, nrow * ncol):
        axes[j // ncol][j % ncol].axis("off")
    fig.tight_layout()
    FIG.mkdir(parents=True, exist_ok=True)
    fig.savefig(FIG / "all_references.png", dpi=150)
    plt.close(fig)


def main(argv=None):
    global OUT
    import argparse
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-root", default=None)
    args = ap.parse_args(argv)
    if args.out_root:
        OUT = ROOT / args.out_root if not Path(args.out_root).is_absolute() else Path(args.out_root)
        print("[audit] out-root =", OUT)
    res = [check(p) for p in scenario_files()]
    OUT.mkdir(parents=True, exist_ok=True)
    for r in res:
        plot_one(r)
    plot_all(res)
    lines = ["# Reference attitude audit", "",
             "Generated by `scripts/audit_no_cmg_references.py` from the same "
             "`Trajectory3D` the closed loop uses.  The manoeuvre offset is the "
             "reference Euler angle minus a manoeuvre-free baseline reference, so "
             "the slowly varying flight-path pitch and heading do not contaminate "
             "the comparison.  `applied` is measured 1 s after the manoeuvre ends "
             "relative to 1 s before it starts.", "",
             "| scenario | man | axis | t0 [s] | t_end [s] | commanded [deg] | applied [deg] | other axes [deg] | peak offset rate [deg/s] | analytic | verdict |",
             "|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in res:
        for m in r["maneuvers"]:
            oth = ", ".join("%s %+.2e" % (k, v)
                            for k, v in m["other_axes_delta_deg"].items()) or "—"
            peak = ("%.4f" % m["peak_offset_rate_deg_s"]
                    if m["peak_offset_rate_deg_s"] is not None else "—")
            ana = ("%.4f" % m["analytic_peak_deg_s"]
                   if m["analytic_peak_deg_s"] is not None else "—")
            lines.append("| %s | %s | %s | %.1f | %.1f | %+.3f | %+.3f | %s | %s | %s | %s |" % (
                r["scenario"], m["index"], m["axis"], m["t0"], m["t_end"],
                m["commanded_deg"], m["applied_deg"], oth, peak, ana,
                "ok" if m["ok"] else "FAIL"))
    lines += ["", "## Issues", ""]
    bad = [(r["scenario"], i) for r in res for i in r["issues"]]
    if bad:
        lines += ["- %s: %s" % (s, i) for s, i in bad]
    else:
        lines.append("None: every reference matches its hand-checked definition, "
                     "including the sequential superposition of S07 (the second "
                     "manoeuvre returns the pitch offset to 0).")
    lines += ["", "Figures: `figures/torque_gap_no_cmg_comparison/references/`."]
    (OUT / "reference_audit.md").write_text("\n".join(lines) + "\n", encoding="utf-8")
    (OUT / "reference_audit.json").write_text(json.dumps(
        [dict(scenario=r["scenario"], ok=r["ok"], issues=r["issues"],
              maneuvers=[dict(m) for m in r["maneuvers"]]) for r in res],
        indent=2, ensure_ascii=False, default=float), encoding="utf-8")
    print("references checked: %d, failing: %d"
          % (len(res), sum(1 for r in res if not r["ok"])))
    for r in res:
        print(" ", r["scenario"], "ok" if r["ok"] else "FAIL", r["issues"])
    return 0 if all(r["ok"] for r in res) else 1


if __name__ == "__main__":
    sys.exit(main())
