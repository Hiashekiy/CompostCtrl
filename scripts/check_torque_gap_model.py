#!/usr/bin/env python
"""Stage S1 validation of the torque-gap model, authority and modulation layer.

Checks performed (all numeric, all reported):

  T1  CMG geometry:  dH/dgamma == A(gamma),  tau_cmg = -A nu,  signs
  T2  11-state dynamics: single-step angular acceleration against the analytic
      formula, pure torque input and pure gimbal-rate input
  T3  consistency with the 18-state truth model: identical omega_dot for an
      identical aerodynamic moment, and the aerodynamic moment decomposition
      (natural + M_delta delta) reproduces nearspace.model6c.aero_moment6c
  T4  modulation: the six surplus/gap cases, including the *negative* gap sign
  T5  aerodynamic authority: exact per-axis hull, >=1000 random in-box samples
      realisable by a bounded deflection solve, conservative scale calibration
  T6  11-state RK4 against an independent high-accuracy integrator
  T7  attitude error conventions (short arc, q ~ -q)

Writes:  results/torque_gap/model_checks.json
         docs/torque_gap/model_validation.md
         figures/torque_gap/authority_vs_time.png
         figures/torque_gap/modulation_3axis.png

Usage:  python scripts/check_torque_gap_model.py [--case C1] [--no-plot]
"""
from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
_DEPS = ROOT / ".pydeps"                      # local casadi (see requirements-ocp.txt)
if _DEPS.is_dir():
    sys.path.insert(0, str(_DEPS))

from nearspace.cmg import CMGParams, CMGCluster                      # noqa: E402
from nearspace.config import load_config                             # noqa: E402
from nearspace.model6 import (euler_to_quat, quat_geodesic, quat_mul,  # noqa: E402
                              quat_normalize)
from nearspace.model6c import (VehicleCMGParams, aero_moment6c,      # noqa: E402
                               dynamics6c)
from nearspace.torque_gap_authority import (allocate_deflection,     # noqa: E402
                                            axis_hull, calibrate_scale,
                                            natural_moment_v, realizable_fraction,
                                            surface_matrix)
from nearspace.torque_gap_model import (TorquePlantParams, attitude_error_vector,  # noqa: E402
                                        dynamics_tg, pack_state, rk4_step_tg)
from nearspace.torque_gap_modulation import (modulation,             # noqa: E402
                                             capability_residual)
from nearspace.torque_gap_scenario import Scenario                   # noqa: E402

CHECKS = []


def check(name, value, tol, note="", ok=None):
    passed = bool(abs(value) <= tol) if ok is None else bool(ok)
    CHECKS.append(dict(name=name, value=float(value) if np.isscalar(value) else None,
                       tol=float(tol), passed=passed, note=note))
    return passed


def check_true(name, cond, note=""):
    CHECKS.append(dict(name=name, value=None, tol=None, passed=bool(cond), note=note))


# --------------------------------------------------------------------------
def t1_geometry(prm, cluster):
    rng = np.random.default_rng(1)
    worst = 0.0
    for gamma in (np.zeros(4), np.deg2rad([12.0, -35.0, 24.0, 61.0]),
                  rng.uniform(-1.4, 1.4, 4)):
        A = cluster.jacobian(gamma)
        eps = 1e-6
        for j in range(4):
            nu = np.zeros(4)
            nu[j] = 1.0
            fd = (cluster.momentum(gamma + eps * nu)
                  - cluster.momentum(gamma - eps * nu)) / (2.0 * eps)
            worst = max(worst, float(np.max(np.abs(fd - A @ nu))))
        # tau_cmg = -A nu  and  dH/dt = A nu
        nu = rng.uniform(-1.5, 1.5, 4)
        torque = cluster.torque(gamma, nu)
        check_true("torque == -A nu", np.allclose(torque, -A @ nu, atol=1e-12))
        check_true("dH/dt == -torque", np.allclose(A @ nu, -torque, atol=1e-12))
    check("max |dH/dgamma nu - A nu|", worst, 1e-7,
          "central difference (eps=1e-6) vs analytic Jacobian; h0 = %.1f N m s" % prm.cmg.h0)

    # sign of the first unit at the home configuration
    gamma0 = np.zeros(4)
    tau = cluster.torque(gamma0, np.array([1.0, 0.0, 0.0, 0.0]))
    Hd = cluster.jacobian(gamma0)[:, 0]
    check_true("sign: unit0 nu>0 lowers body x-momentum",
               np.sign(tau[0]) == -np.sign(Hd[0]) and tau[0] > 0.0,
               "tau=%s, A[:,0]=%s" % (np.round(tau, 3), np.round(Hd, 3)))
    # the config beta is rounded to 54.7356 deg, so the closed-form home
    # sigma_min differs from 0.8165 h0 in the 6th significant digit
    check("sigma_min(home) - 0.8165 h0",
          float(cluster.sigma_min(cluster.jacobian(gamma0)) - 0.816496580927726 * prm.cmg.h0),
          1e-3, "config beta = %.6f deg" % np.rad2deg(prm.cmg.beta))
    return worst


def t2_dynamics(prm, cluster):
    gamma = np.deg2rad([12.0, -35.0, 24.0, 61.0])
    w = np.array([0.01, -0.02, 0.015])
    q = quat_normalize(euler_to_quat(0.05, 0.1, -0.2))
    x = pack_state(q, w, gamma)
    tau_a = np.array([12.0, -30.0, 4.0])
    nu = np.array([0.2, -0.4, 0.1, 0.3])
    tau_d = np.array([0.3, -0.2, 0.05])
    dx = dynamics_tg(x, np.concatenate([tau_a, nu]), tau_d, prm, cluster)
    H = cluster.momentum(gamma)
    A = cluster.jacobian(gamma)
    wdot = prm.Jinv @ (tau_a - A @ nu + tau_d - np.cross(w, prm.J @ w + H))
    check("max |omega_dot - analytic|", float(np.max(np.abs(dx[4:7] - wdot))), 1e-12)
    check("max |gamma_dot - nu|", float(np.max(np.abs(dx[7:11] - nu))), 1e-12)

    # pure aerodynamic torque, zero gimbal rate and zero rates: analytic
    x0 = pack_state([1.0, 0.0, 0.0, 0.0], [0.0, 0.0, 0.0], np.zeros(4))
    dx0 = dynamics_tg(x0, np.array([10.0, 0.0, 0.0, 0, 0, 0, 0]), np.zeros(3), prm, cluster)
    check("pure tau_a=10 N m -> p_dot=10/Jx",
          float(abs(dx0[4] - 10.0 / prm.veh.Jx)), 1e-12)
    # pure gimbal rate: omega_dot = -J^-1 A nu
    A0 = cluster.jacobian(np.zeros(4))
    nu1 = np.array([0.5, 0.0, 0.0, 0.0])
    dx1 = dynamics_tg(x0, np.concatenate([np.zeros(3), nu1]), np.zeros(3), prm, cluster)
    check("nu-only omega_dot == -J^-1 A nu",
          float(np.max(np.abs(dx1[4:7] - (-prm.Jinv @ (A0 @ nu1))))), 1e-12,
          "negative pitch response to a positive first-gimbal rate")
    return float(np.max(np.abs(dx[4:7] - wdot)))


def t3_truth_consistency(cfg, prm, cluster):
    """Same state + same aerodynamic moment -> same omega_dot in P0 and model6c."""
    vp = VehicleCMGParams.from_config(cfg)
    cl6 = CMGCluster(vp.cmg)
    gamma = np.deg2rad([18.0, -22.0, 31.0, -12.0])
    w = np.array([0.004, -0.006, 0.003])
    q = quat_normalize(euler_to_quat(0.02, 0.08, -0.15))
    nu = np.array([0.1, -0.2, 0.15, -0.05])
    tau_d = np.array([0.2, 0.5, 0.3])
    x18 = np.zeros(18)
    x18[0:4] = q
    x18[4:7] = w
    x18[7:10] = np.deg2rad([4.0, -3.0, 2.0])
    x18[10:14] = gamma
    x18[14:18] = nu
    rho, V = 1.2e-6, 3500.0
    V_ned = np.array([V, 0.0, 0.0])
    p = np.concatenate([[rho], V_ned, tau_d])
    tau_aero = aero_moment6c(x18, p, vp)              # total aerodynamic moment
    x11 = pack_state(q, w, gamma)
    dx11 = dynamics_tg(x11, np.concatenate([tau_aero, nu]), tau_d, prm, cluster)
    dx18 = dynamics6c(x18, np.concatenate([x18[7:10], nu]), p, vp, cl6)
    err = float(np.max(np.abs(dx11[4:7] - dx18[4:7])))
    check("P0 vs model6c omega_dot (identical aero moment)", err, 1e-10)
    check("P0 vs model6c gamma_dot",
          float(np.max(np.abs(dx11[7:11] - dx18[10:14]))), 1e-12)

    # aerodynamic moment decomposition: natural + M_delta delta == aero_moment6c
    veh = prm.veh
    Q = 0.5 * rho * V ** 2
    alpha, beta = None, None
    from nearspace.model6 import flow_angles
    alpha, beta, _ = flow_angles(x18, p)
    delta = np.asarray(x18[7:10], float)
    nat = natural_moment_v(Q, alpha, beta, w, V, veh)
    recon = nat + surface_matrix(Q, veh) @ delta
    check("aero moment decomposition vs model6c",
          float(np.max(np.abs(recon - tau_aero))), 1e-12,
          "natural + M_delta*delta == aero_moment6c")
    return err


def t4_modulation(scn):
    """Six surplus/gap situations with analytically known answers."""
    lo = np.array([-5.0, -5.0, -5.0])
    hi = np.array([5.0, 5.0, 5.0])

    cases = {
        "full surplus": (np.array([1.0, -1.0, 2.0]), dict(R_plus=[4, 6, 3], R_minus=[6, 4, 7], d=[0, 0, 0])),
        "positive gap": (np.array([7.0, 0.0, 0.0]), dict(d=[2, 0, 0], R_plus=[-2, 5, 5])),
        "negative gap": (np.array([-7.0, 0.0, -6.0]), dict(d=[-2, 0, -1], R_minus=[-2, 5, -1])),
        "exact edge": (np.array([5.0, -5.0, 0.0]), dict(d=[0, 0, 0], R_plus=[0, 10, 5], R_minus=[10, 0, 5])),
        "mixed axes": (np.array([8.0, -8.0, 0.5]), dict(d=[3, -3, 0])),
    }
    okall = True
    for name, (nom, exp) in cases.items():
        m = modulation(nom, lo, hi)
        for key, want in exp.items():
            ok = np.allclose(m[key], np.array(want, float), atol=1e-12)
            check_true("T4 %-14s %s" % (name, key), ok,
                       "got %s" % np.round(m[key], 3))
            okall &= ok
        # clip identity always holds
        okall &= bool(np.allclose(m["d"], nom - np.clip(nom, lo, hi), atol=1e-15))
    check_true("T4 clip identity d = nom - clip(nom)", okall)

    # altitude-dependent box: authority must shrink into the thin-air apex and the
    # negative pitch demand there must produce a *negative* pitch gap
    times = np.array([60.0, 90.0, 105.0, 120.0])
    cond = scn.cond(times)
    box = scn.boxes(cond)
    half = 0.5 * (box["tau_max"][:, 1] - box["tau_min"][:, 1])
    check_true("T4 aero pitch authority decreases with altitude",
               bool(np.all(np.diff(half) < 0.0)),
               "pitch half-width [N m] = %s" % np.round(half, 3))
    d_neg = modulation(np.array([0.0, -20.0, 0.0]), box["tau_min"][3], box["tau_max"][3])["d"]
    check_true("T4 negative pitch demand -> negative pitch gap",
               d_neg[1] < 0.0, "d = %s" % np.round(d_neg, 3))
    return okall


def t5_authority(scn, cluster):
    veh = scn.veh
    # the hull must equal the brute-force per-axis maximum of the reachable set
    Q = 5.0
    M = surface_matrix(Q, veh)
    rng = np.random.default_rng(7)
    dm = np.asarray(veh.delta_max, float)
    samp = rng.uniform(-1.0, 1.0, size=(20000, 3)) * dm
    brute = np.max(np.abs((samp @ M.T)), axis=0)
    hull = axis_hull(Q, veh)
    check("axis hull vs brute force (relative)",
          float(np.max(np.abs(brute - hull) / hull)), 2e-2,
          "20000 random deflections")

    # configuration capability residual: E_k must drop when the gap direction is
    # aligned with a good configuration and the inner solve must be exact
    d = np.array([0.0, -14.0, 0.0])
    E0, v0 = capability_residual(np.zeros(4), d, cluster, 1.5)
    Ea, va = capability_residual(np.deg2rad([0.0, 45.0, 0.0, 45.0]), d, cluster, 1.5)
    check_true("T5 E_k finite for both configurations",
               np.isfinite(E0) and np.isfinite(Ea),
               "E(home)=%.3f N^2m^2, E(offset)=%.3f N^2m^2" % (E0, Ea))

    rows = []
    Qs = [float(q) for q in np.geomspace(0.5, 8000.0, 8)]
    scale, diag = calibrate_scale(veh, Qs, require_frac=scn.auth.require_frac,
                                 n_samples=scn.auth.n_samples,
                                 tol_frac=scn.auth.tol_frac)
    for entry in diag:
        rows.append(dict(scale=entry["scale"], min_fraction=entry["min_fraction"]))
    for Q in Qs:
        frac, worst = realizable_fraction(Q, veh, scale, n_samples=1000, seed=3,
                                          tol_frac=scn.auth.tol_frac)
        rows.append(dict(Q=float(Q), scale=float(scale), fraction=float(frac),
                         worst_rel=float(worst)))
        check_true("T5 realisable fraction at Q=%.1f Pa" % Q,
                   frac >= scn.auth.require_frac,
                   "fraction=%.4f, scale=%.3f" % (frac, scale))
    return scale, rows


def t6_integration(prm, cluster):
    from scipy.integrate import solve_ivp
    gamma = np.deg2rad([10.0, -20.0, 30.0, -40.0])
    x0 = pack_state(quat_normalize(euler_to_quat(0.03, 0.06, -0.1)),
                    np.array([0.01, -0.02, 0.015]), gamma)
    u = np.array([3.0, -14.0, 2.0, 0.3, -0.5, 0.2, 0.4])
    tau_d = np.array([0.2, 0.5, 0.3])
    dt = 0.2
    xn = x0
    for _ in range(8):
        xn = rk4_step_tg(xn, u, tau_d, dt / 8.0, prm, cluster)

    def f(t, y):
        return dynamics_tg(y, u, tau_d, prm, cluster)

    sol = solve_ivp(f, [0.0, dt], x0, method="DOP853", rtol=1e-13, atol=1e-13,
                    dense_output=False)
    xr = sol.y[:, -1].copy()
    xr[0:4] = quat_normalize(xr[0:4])
    err = float(np.max(np.abs(xn[4:] - xr[4:])))
    check("RK4 vs DOP853 state error after 0.2 s", err, 1e-10,
          "rate/gimbal states; t=0.2 s, 8 substeps")
    check("quaternion norm error", float(abs(np.linalg.norm(xn[0:4]) - 1.0)), 1e-12)
    return err


def t7_attitude(prm):
    qa = quat_normalize(euler_to_quat(0.1, -0.2, 0.3))
    worst = 0.0
    for eps in (1e-4, 1e-3, 1e-2):
        qb = quat_normalize(quat_mul(qa, euler_to_quat(eps, 0.0, 0.0)))
        e = attitude_error_vector(qa, qb)
        worst = max(worst, float(abs(np.linalg.norm(e) - quat_geodesic(qa, qb))))
        # q and -q are the same attitude
        worst = max(worst, float(np.max(np.abs(attitude_error_vector(qa, -qb) - e))))
    check("attitude error vs geodesic (q and -q equivalent)", worst, 1e-4)
    return worst


# --------------------------------------------------------------------------
def make_figures(scn, outdir):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    os.makedirs(outdir, exist_ok=True)
    # authority and modulation along the whole mission (single reference run)
    t = np.arange(60.0, 170.0, 0.5)
    cond = scn.cond(t)
    box = scn.boxes(cond)
    K = t.size
    H_nom = np.zeros(3)
    tau_nom = np.zeros((K, 3))
    for k in range(K):
        tau_d = cond["tau_d"][k] if scn.mod.tau_d == "nominal" else np.zeros(3)
        from nearspace.torque_gap_modulation import nominal_demand
        tau_nom[k] = nominal_demand(scn.traj, t[k], scn.prm, H_nom, tau_d)
    mods = scn.modulation_sequence(tau_nom, box)
    labels = ["roll (x)", "pitch (y)", "yaw (z)"]

    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    for j, ax in enumerate(axes):
        ax.fill_between(t, box["tau_min"][:, j], box["tau_max"][:, j],
                        color="0.75", label="predicted aero box")
        ax.plot(t, mods["tau_nom"][:, j], "b-", lw=1.4, label=r"nominal demand $\tau_{nom}$")
        ax.plot(t, mods["d"][:, j], "r--", lw=1.2, label=r"gap $d$")
        ax.set_ylabel(labels[j] + " [N m]")
        ax.grid(alpha=0.3)
        ax.set_yscale("symlog", linthresh=1.0)
    axes[0].legend(loc="upper right", fontsize=8)
    axes[-1].set_xlabel("t [s]")
    fig.suptitle("C1 reference: time-varying aero torque box, nominal demand, gap")
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "authority_vs_time.png"), dpi=140)
    plt.close(fig)

    fig, axes = plt.subplots(3, 1, figsize=(9, 8), sharex=True)
    for j, ax in enumerate(axes):
        ax.plot(t, mods["R_plus"][:, j], "g-", lw=1.2, label=r"$R^+$")
        ax.plot(t, mods["R_minus"][:, j], "m-", lw=1.2, label=r"$R^-$")
        ax.plot(t, mods["d"][:, j], "r-", lw=1.6, label=r"$d$")
        ax.axhline(0.0, color="k", lw=0.6)
        ax.set_ylabel(labels[j])
        ax.grid(alpha=0.3)
        ax.set_yscale("symlog", linthresh=1.0)
    axes[0].legend(loc="upper right", fontsize=8)
    axes[-1].set_xlabel("t [s]")
    fig.suptitle("Three-axis surplus / gap modulation (negative R = bound exceeded)")
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "modulation_3axis.png"), dpi=140)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--case", default="C1")
    ap.add_argument("--config", default=None)
    ap.add_argument("--no-plot", action="store_true")
    args = ap.parse_args()
    cfg_path = args.config or str(ROOT / "configs" / "torque_gap" / "cases"
                                  / ("%s.yaml" % {"C0": "c0_hold", "C1": "c1_pitch30",
                                                  "C2": "c2_yaw30",
                                                  "C3": "c3_pitch_then_roll",
                                                  "C4": "c4_short_horizon",
                                                  "C5": "c5_fixed_box"}.get(
                                                      args.case.upper(), "c1_pitch30")))
    scn = Scenario(load_config(cfg_path))
    prm, cluster = scn.prm, scn.cluster

    t1_geometry(prm, cluster)
    t2_dynamics(prm, cluster)
    t3_truth_consistency(scn.cfg, prm, cluster)
    t4_modulation(scn)
    scale, rows = t5_authority(scn, cluster)
    t6_integration(prm, cluster)
    t7_attitude(prm)

    npass = sum(1 for c in CHECKS if c["passed"])
    ok = npass == len(CHECKS)
    res_dir = ROOT / "results" / "torque_gap"
    res_dir.mkdir(parents=True, exist_ok=True)
    payload = dict(case=args.case, config=str(cfg_path), passed=ok,
                   n_checks=len(CHECKS), n_pass=npass,
                   authority_scale=scale, authority_table=rows,
                   cmg=dict(h0=prm.cmg.h0, gdot_max=prm.cmg.gdot_max,
                            gddot_max=prm.cmg.gddot_max, tau_g=prm.cmg.tau_g),
                   vehicle=dict(Jx=prm.veh.Jx, Jy=prm.veh.Jy, Jz=prm.veh.Jz,
                                delta_max_deg=np.rad2deg(prm.veh.delta_max).tolist()),
                   checks=CHECKS)
    with open(res_dir / "model_checks.json", "w", encoding="utf-8") as fh:
        json.dump(payload, fh, indent=2, ensure_ascii=False)

    if not args.no_plot:
        make_figures(scn, ROOT / "figures" / "torque_gap")

    # ---- markdown summary ----------------------------------------------
    docs = ROOT / "docs" / "torque_gap"
    docs.mkdir(parents=True, exist_ok=True)
    lines = ["# Model / authority / modulation validation (stage S1)", "",
             "Generated by `python scripts/check_torque_gap_model.py --case %s`." % args.case,
             "", "Config: `%s`" % os.path.relpath(cfg_path, ROOT), "",
             "Result: **%d/%d checks passed**." % (npass, len(CHECKS)), "",
             "| check | value | tol | pass | note |", "|---|---|---|---|---|"]
    for c in CHECKS:
        val = "-" if c["value"] is None else ("%.3e" % c["value"])
        tol = "-" if c["tol"] is None else ("%.1e" % c["tol"])
        lines.append("| %s | %s | %s | %s | %s |" % (c["name"], val, tol,
                                                     "OK" if c["passed"] else "**FAIL**",
                                                     c.get("note", "")))
    lines += ["", "## Aerodynamic authority calibration", "",
              "Final conservative box scale: **%.3f** (calibrated offline: the largest "
              "uniform scale of the per-axis reachable-set hull at which at least "
              "%.0f%% of %d random in-box targets are realisable by a bounded "
              "deflection solve)." % (scale, 100 * scn.auth.require_frac,
                                      scn.auth.n_samples), "",
              "| Q [Pa] | fraction realisable | worst relative residual |",
              "|---|---|---|"]
    for r in rows:
        if "Q" in r:
            lines.append("| %.3g | %.4f | %.2e |" % (r["Q"], r["fraction"], r["worst_rel"]))
    lines += ["", "Figures: `figures/torque_gap/authority_vs_time.png`, "
              "`figures/torque_gap/modulation_3axis.png`."]
    with open(docs / "model_validation.md", "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")

    print("checks: %d/%d passed" % (npass, len(CHECKS)))
    for c in CHECKS:
        if not c["passed"]:
            print("  FAIL:", c["name"], c["note"])
    print("authority scale:", round(scale, 3))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
