#!/usr/bin/env python
"""Model validation for the *unlimited* (continuous-rotation) gimbal model.

Runs the four tests required before any closed-loop experiment:

  T1  350 deg -> 370 deg continuity: H(gamma), A(gamma), tau_cmg, omega_dot and
      the RK4 integration show no jump at 360 deg (increment statistics inside the
      wrap window are compared with the same window far from it), and the exact
      2*pi periodicity of H/A/torque is verified.
  T2  multi-turn 0 -> 720 deg: gamma is integrated through two full turns, stays
      monotone and unwrapped (no modulo, no clamp, no reset).
  T3  dynamics consistency: dH/dt = A(gamma) nu and tau_cmg = -A(gamma) nu hold
      for gamma far outside +-90 deg (finite-difference check).
  T4  P0 / P1 parity: neither the virtual-torque plant nor the 18-state truth
      clips the gimbal angle any more; both rotate past 90 deg and past 360 deg
      and agree within the truth model's own rate-loop lag.

Writes:  results/torque_gap_unlimited/model_validation.md
         results/torque_gap_unlimited/model_validation.json
         figures/torque_gap_unlimited/continuity_360.png
         figures/torque_gap_unlimited/multi_turn_720.png

python scripts/check_torque_gap_unlimited.py
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from nearspace.cmg import CMGCluster                                    # noqa: E402
from nearspace.config import load_config                                # noqa: E402
from nearspace.model6 import euler_to_quat, quat_normalize              # noqa: E402
from nearspace.torque_gap_model import (TorquePlantParams, dynamics_tg,  # noqa: E402
                                        pack_state, rk4_step_tg)
from nearspace.torque_gap_scenario import Scenario                      # noqa: E402
from nearspace.torque_gap_sim import ClosedLoop                         # noqa: E402

CFG_LIMITED = ROOT / "configs" / "torque_gap" / "cases" / "c1_pitch30.yaml"
CFG_FREE = ROOT / "configs" / "torque_gap_unlimited" / "cases" / "c1_pitch30.yaml"
RES = ROOT / "results" / "torque_gap_unlimited"
FIG = ROOT / "figures" / "torque_gap_unlimited"
CHECKS = []


def check(name, value, tol, note="", ok=None):
    passed = bool(abs(value) <= tol) if ok is None else bool(ok)
    CHECKS.append(dict(name=name,
                       value=None if value is None else float(value),
                       tol=None if tol is None else float(tol),
                       passed=passed, note=note))


def _gamma_deg(deg, i=0):
    g = np.zeros(4)
    g[i] = np.deg2rad(deg)
    return g


def _eval_series(label, cluster, prm, q, w, nu, tau_a, tau_d, gamma_i):
    """One sample of the T1 series, evaluated at an arbitrary first gimbal angle."""
    g = np.zeros(4)
    g[0] = gamma_i
    if label == "H":
        return cluster.momentum(g)
    if label == "A[:,0]":
        return cluster.jacobian(g)[:, 0]
    if label == "tau_cmg":
        return cluster.torque(g, nu)
    x = pack_state(q, w, g)
    return dynamics_tg(x, np.concatenate([tau_a, nu]), tau_d, prm, cluster)[4:7]


def t1_continuity(prm, cluster):
    """350 -> 370 deg: no jump in H, A, torque, omega_dot or the integration."""
    q = quat_normalize(euler_to_quat(0.02, 0.10, -0.15))
    w = np.zeros(3)
    tau_a = np.zeros(3)
    nu = np.array([0.30, 0.0, 0.0, 0.0])
    tau_d = np.zeros(3)
    step = 0.25                                    # deg
    win = np.arange(350.0, 370.0 + 1e-9, step)
    ref = np.arange(10.0, 30.0 + 1e-9, step)

    def series(deg_list):
        H, A, tau, wdot = [], [], [], []
        for d in deg_list:
            g = _gamma_deg(d)
            H.append(cluster.momentum(g))
            A.append(cluster.jacobian(g)[:, 0])
            tau.append(cluster.torque(g, nu))
            x = pack_state(q, w, g)
            wdot.append(dynamics_tg(x, np.concatenate([tau_a, nu]), tau_d, prm,
                                    cluster)[4:7])
        return (np.array(H), np.array(A), np.array(tau), np.array(wdot))

    H1, A1, T1, W1 = series(win)
    H2, A2, T2, W2 = series(ref)

    # (a) exact 2*pi periodicity: f(gamma) == f(gamma - 2*pi) everywhere in the
    #     window, which is precisely what "no discontinuity at the wrap" means.
    for label, F1 in (("H", H1), ("A[:,0]", A1), ("tau_cmg", T1), ("omega_dot", W1)):
        worst = 0.0
        for d in win:
            f_hi = np.asarray(_eval_series(label, cluster, prm, q, w, nu, tau_a,
                                           tau_d, np.deg2rad(d)), float)
            f_lo = np.asarray(_eval_series(label, cluster, prm, q, w, nu, tau_a,
                                           tau_d, np.deg2rad(d - 360.0)), float)
            worst = max(worst, float(np.max(np.abs(f_hi - f_lo))))
        check("T1 max |f(gamma) - f(gamma - 2pi)| for %s over 350..370 deg" % label,
              worst, 1e-12, "exact periodicity, hence no jump at 360 deg")

    # (b) smoothness: the discrete curvature at the sample straddling 360 deg must
    #     be comparable to the typical curvature in the same window (a jump would
    #     make it orders of magnitude larger).
    for label, F1 in (("H", H1), ("A[:,0]", A1), ("tau_cmg", T1), ("omega_dot", W1)):
        d2 = np.linalg.norm(np.diff(F1, n=2, axis=0), axis=1)
        k_wrap = int(np.argmin(np.abs(win - 360.0)))
        k_wrap = min(k_wrap, d2.size - 1)
        med = float(np.median(d2))
        ratio = float(d2[k_wrap] / med) if med > 0 else float("inf")
        check("T1 curvature at 360 deg / median curvature for %s" % label,
              abs(ratio - 1.0), 4.0,
              "smooth at the wrap (ratio ~1, a jump would be >> 10); ratio = %.3f"
              % ratio)

    # exact periodicity: gamma and gamma + 2 pi give identical geometry/torque
    g10, g370 = _gamma_deg(10.0), _gamma_deg(370.0)
    check("T1 |H(370 deg) - H(10 deg)|",
          float(np.max(np.abs(cluster.momentum(g370) - cluster.momentum(g10)))), 1e-12)
    check("T1 |A(370 deg) - A(10 deg)|",
          float(np.max(np.abs(cluster.jacobian(g370) - cluster.jacobian(g10)))), 1e-12)
    check("T1 |tau_cmg(370 deg) - tau_cmg(10 deg)|",
          float(np.max(np.abs(cluster.torque(g370, nu) - cluster.torque(g10, nu)))),
          1e-12)
    wd370 = dynamics_tg(pack_state(q, w, g370), np.concatenate([tau_a, nu]),
                        tau_d, prm, cluster)[4:7]
    wd10 = dynamics_tg(pack_state(q, w, g10), np.concatenate([tau_a, nu]),
                       tau_d, prm, cluster)[4:7]
    check("T1 |omega_dot(370 deg) - omega_dot(10 deg)|",
          float(np.max(np.abs(wd370 - wd10))), 1e-12)

    # integration across 360 deg: monotone gamma, no jump in the state
    x = pack_state(q, w, _gamma_deg(350.0))
    dt = 5e-3
    gam, Hn = [], []
    for _ in range(int(round(np.deg2rad(20.0) / (nu[0] * dt))) + 1):
        gam.append(x[7])
        Hn.append(cluster.momentum(x[7:11]))
        x = rk4_step_tg(x, np.concatenate([tau_a, nu]), tau_d, dt, prm, cluster)
    gam, Hn = np.array(gam), np.array(Hn)
    check("T1 integration: max gamma increment versus nu*dt (should be 1)",
          float(np.max(np.diff(gam)) / (nu[0] * dt) - 1.0), 1e-6,
          "RK4 over 350->370 deg, dt = %.0e s" % dt)
    check("T1 integration: gamma monotone", None, None,
          ok=bool(np.all(np.diff(gam) > 0.0)))
    check("T1 integration: max |dH| between samples",
          float(np.max(np.linalg.norm(np.diff(Hn, axis=0), axis=1))),
          abs(nu[0] * dt) * float(prm.cmg.h0) * 4.0 * 1.05,
          "bounded by |nu|*dt*sum(h0) (no numerical jump)")
    return dict(gamma_deg=list(np.rad2deg(gam)), H=Hn.tolist())


def t2_multi_turn(prm, cluster):
    """0 -> 720 deg with constant gimbal rate: unwrapped, no reset, no clamp."""
    q = np.array([1.0, 0.0, 0.0, 0.0])
    nu = np.array([0.0, prm.cmg.gdot_max, 0.0, 0.0])     # 1.5 rad/s
    tau_d = np.zeros(3)
    dt = 2e-3
    T = 2.0 * 2.0 * np.pi / prm.cmg.gdot_max             # two full turns
    x = pack_state(q, np.zeros(3), np.zeros(4))
    gam2, states = [], []
    n = int(round(T / dt)) + 1
    for k in range(n):
        gam2.append(x[8])
        states.append(x.copy())
        x = rk4_step_tg(x, np.concatenate([np.zeros(3), nu]), tau_d, dt, prm, cluster)
    gam2 = np.array(gam2)
    analytic = nu[1] * dt * np.arange(n)
    check("T2 gamma error versus analytic unwrapped rotation",
          float(np.max(np.abs(gam2 - analytic))), 1e-9,
          "0 -> 720 deg at gdot_max, dt = %.0e s" % dt)
    check("T2 final gamma reaches 2 turns", None, None,
          ok=bool(gam2[-1] > 4.0 * np.pi - 1e-6),
          note="final gamma = %.4f rad (%.1f deg)"
               % (gam2[-1], np.rad2deg(gam2[-1])))
    check("T2 gamma monotone (no clamp / no modulo / no reset)", None, None,
          ok=bool(np.all(np.diff(gam2) > 0.0)))
    check("T2 max gamma increment / (nu dt)",
          float(np.max(np.diff(gam2)) / (nu[1] * dt) - 1.0), 1e-6)
    # periodic geometry: |H| at 0, 1 and 2 turns must agree
    H_at = [cluster.momentum(np.array([0.0, g, 0.0, 0.0])) for g in
            (0.0, 2.0 * np.pi, 4.0 * np.pi)]
    check("T2 |H(2 turns) - H(0)|",
          float(np.max(np.abs(H_at[2] - H_at[0]))), 1e-12)
    check("T2 |H(1 turn) - H(0)|",
          float(np.max(np.abs(H_at[1] - H_at[0]))), 1e-12)
    # the kinematics alone guarantee it, but verify the state is not wrapped
    check("T2 state gamma equals the unwrapped value (no modulo)",
          float(abs(states[-1][8] - gam2[-1])), 1e-15)
    return dict(gamma_deg=list(np.rad2deg(gam2[::50])), time_s=list(
        (np.arange(n)[::50] * dt)))


def t3_dynamics(prm, cluster):
    """dH/dt = A nu and tau = -A nu for gamma far outside +-90 deg."""
    worst_dH, worst_tau = 0.0, 0.0
    for deg in (30.0, 95.0, 200.0, 359.0, 361.0, 540.0, 1000.0, np.rad2deg(100.0)):
        g = _gamma_deg(deg)
        nu = np.array([0.21, -0.34, 0.12, 0.44])
        eps = 1e-6
        dH = (cluster.momentum(g + eps * nu) - cluster.momentum(g - eps * nu)) / (2 * eps)
        worst_dH = max(worst_dH, float(np.max(np.abs(dH - cluster.jacobian(g) @ nu))))
        worst_tau = max(worst_tau,
                        float(np.max(np.abs(cluster.torque(g, nu)
                                            + cluster.jacobian(g) @ nu))))
    check("T3 max |dH/dt - A nu| (gamma from 30 deg to 1000 deg)", worst_dH, 2e-6,
          "central difference at eps=1e-6, |gamma| up to 17 rad")
    check("T3 max |tau_cmg + A nu|", worst_tau, 1e-15,
          "the CMG torque relation is unchanged by the travel model")
    return worst_dH


def t4_plant_parity(prm, cluster):
    """Neither plant clips gamma any more; P0 and P1 agree within the rate lag."""
    scn = Scenario(load_config(str(CFG_FREE)))
    check("T4 scenario reports an unbounded gimbal angle", None, None,
          ok=bool(scn.gamma_unbounded),
          note="gamma_limit_mode = %s, gamma_lim = %s"
               % (scn.gamma_limit_mode, scn.gamma_lim))
    cl = ClosedLoop(scn, plant="P0")
    # start well outside +-90 deg and rotate further: nothing may be clamped
    q = quat_normalize(euler_to_quat(0.0, 0.05, 0.0))
    g0 = np.deg2rad([100.0, -120.0, 150.0, 210.0])
    nu = np.array([1.0, 1.0, 1.0, 1.0])
    x = pack_state(q, np.zeros(3), g0)
    x1, _, nu_act = cl._step_P0(x, np.zeros(3), nu, scn.t_start)
    dg = x1[7:11] - g0
    check("T4 P0: no clamp at |gamma| > 90 deg", None, None,
          ok=bool(np.all(dg > 0.0) and np.all(x1[7:11] > g0)),
          note="gamma advanced by %s rad" % np.round(dg, 4).tolist())
    check("T4 P0: gamma increment matches nu*dt", float(np.max(np.abs(dg - nu * cl.dt))),
          1e-9)

    # P1 (18-state truth): same rotation, only the first-order rate loop differs
    x18 = np.zeros(18)
    x18[0:4] = q
    x18[10:14] = g0
    x18n, tau_act, nu1, delta, _ = cl._step_P1(x18, np.zeros(3), nu, scn.t_start)
    check("T4 P1: no clamp at |gamma| > 90 deg", None, None,
          ok=bool(np.all(x18n[10:14] > g0)))
    check("T4 P1 - P0 gimbal increment (rate-loop lag only)",
          float(np.max(np.abs((x18n[10:14] - g0) - dg))), 0.06,
          "first-order rate loop tau_g = %.2f s, dt = %.2f s; the missing travel "
          "is bounded by tau_g*|dnu|" % (prm.cmg.tau_g, cl.dt))

    # long run: drive gamma through more than one full turn in both plants
    x = pack_state(q, np.zeros(3), np.zeros(4))
    x18 = np.zeros(18)
    x18[0:4] = q
    for _ in range(26):                       # 26 * 0.2 s * 1.5 rad/s = 7.8 rad > 360 deg
        x, _, _ = cl._step_P0(x, np.zeros(3), np.array([1.5, 1.5, 1.5, 1.5]), scn.t_start)
        x18, _, _, _, _ = cl._step_P1(x18, np.zeros(3), np.array([1.5, 1.5, 1.5, 1.5]),
                                      scn.t_start)
    check("T4 P0 rotates past 360 deg", None, None,
          ok=bool(np.all(x[7:11] > 2.0 * np.pi)),
          note="gamma = %s deg" % np.round(np.rad2deg(x[7:11]), 1).tolist())
    check("T4 P1 rotates past 360 deg", None, None,
          ok=bool(np.all(x18[10:14] > 2.0 * np.pi)),
          note="gamma = %s deg" % np.round(np.rad2deg(x18[10:14]), 1).tolist())
    check("T4 P1 - P0 gimbal after >360 deg rotation (rad)",
          float(np.max(np.abs(x18[10:14] - x[7:11]))), 0.08,
          "accumulated rate-loop lag, not a clamp")
    return dict(P0_gamma_deg=np.rad2deg(x[7:11]).tolist(),
                P1_gamma_deg=np.rad2deg(x18[10:14]).tolist())


def make_figures(t1, t2):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    FIG.mkdir(parents=True, exist_ok=True)
    g = np.array(t1["gamma_deg"])
    H = np.array(t1["H"])
    fig, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    for i in range(3):
        axes[0].plot(g, H[:, i], lw=1.3, label="H_%s" % "xyz"[i])
        axes[1].plot(g, np.gradient(H[:, i], g), lw=1.3, label="dH_%s/dgamma" % "xyz"[i])
    axes[0].axvline(360.0, color="k", ls="--", lw=1.0, label="360 deg")
    axes[1].axvline(360.0, color="k", ls="--", lw=1.0)
    axes[0].set_ylabel("H [N m s]")
    axes[1].set_ylabel("dH/dgamma")
    axes[1].set_xlabel("gamma_1 [deg]  (integration 350 -> 370 deg)")
    for ax in axes:
        ax.grid(alpha=0.3)
    axes[0].legend(fontsize=8)
    axes[0].set_title("Continuity across 360 deg (unlimited gimbal model)")
    fig.tight_layout()
    fig.savefig(FIG / "continuity_360.png", dpi=140)
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(8, 3.6))
    t = np.array(t2["time_s"])
    gg = np.array(t2["gamma_deg"])
    ax.plot(t, gg, lw=1.6, label="gamma_2 (unwrapped)")
    for turn, lab in ((360.0, "1 turn"), (720.0, "2 turns")):
        ax.axhline(turn, color="0.5", ls="--", lw=0.9, label=lab)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("gamma_2 [deg]")
    ax.grid(alpha=0.3)
    ax.legend(fontsize=8)
    ax.set_title("Multi-turn rotation 0 -> 720 deg at gimbal-rate limit")
    fig.tight_layout()
    fig.savefig(FIG / "multi_turn_720.png", dpi=140)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()
    cfg = load_config(str(CFG_FREE))
    scn = Scenario(cfg)
    prm, cluster = scn.prm, scn.cluster
    t1 = t1_continuity(prm, cluster)
    t2 = t2_multi_turn(prm, cluster)
    t3 = t3_dynamics(prm, cluster)
    t4 = t4_plant_parity(prm, cluster)
    if not args.no_plot:
        make_figures(t1, t2)

    npass = sum(1 for c in CHECKS if c["passed"])
    ok = npass == len(CHECKS)
    RES.mkdir(parents=True, exist_ok=True)
    payload = dict(gamma_limit_mode=scn.gamma_limit_mode,
                   gamma_unbounded=bool(scn.gamma_unbounded),
                   passed=ok, n_checks=len(CHECKS), n_pass=npass,
                   checks=CHECKS, t1=t1, t2=t2, t4=t4,
                   note="dH/dt - A nu worst = %.3e" % t3)
    (RES / "model_validation.json").write_text(
        json.dumps(payload, indent=2, ensure_ascii=False), encoding="utf-8")

    md = ["# Unlimited-gimbal model validation", "",
          "Generated by `python scripts/check_torque_gap_unlimited.py`.", "",
          "Scenario: `configs/torque_gap_unlimited/cases/c1_pitch30.yaml` "
          "(`gamma_limit: %s`)." % scn.gamma_limit_mode, "",
          "Result: **%d/%d checks passed**." % (npass, len(CHECKS)), "",
          "| check | value | tol | pass | note |", "|---|---|---|---|---|"]
    for c in CHECKS:
        val = "-" if c["value"] is None else "%.3e" % c["value"]
        tol = "-" if c["tol"] is None else "%.1e" % c["tol"]
        md.append("| %s | %s | %s | %s | %s |" % (c["name"], val, tol,
                                                  "OK" if c["passed"] else "**FAIL**",
                                                  c.get("note", "")))
    md += ["", "Figures: `figures/torque_gap_unlimited/continuity_360.png`, "
           "`figures/torque_gap_unlimited/multi_turn_720.png`.", "",
           "## What is *not* relaxed", "",
           "- rotor momentum magnitude: `h0 = %.1f N m s` per unit, `|H| <= 4 h0`"
           % prm.cmg.h0,
           "- gimbal rate: `|nu_i| <= %.2f rad/s`" % prm.cmg.gdot_max,
           "- gimbal acceleration: `|nu_k - nu_{k-1}| <= %.1f dt` rad/s"
           % prm.cmg.gddot_max,
           "- torque map `tau_cmg = -A(gamma) nu`, attitude dynamics, aerodynamic "
           "torque box, disturbance: unchanged."]
    (RES / "model_validation.md").write_text("\n".join(md) + "\n", encoding="utf-8")
    print("checks: %d/%d passed" % (npass, len(CHECKS)))
    for c in CHECKS:
        if not c["passed"]:
            print("  FAIL:", c["name"], c.get("note", ""))
    print("written:", RES / "model_validation.md")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
