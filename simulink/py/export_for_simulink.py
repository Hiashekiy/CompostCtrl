#!/usr/bin/env python
"""Export everything the Simulink port needs, from the Python reference model.

    python simulink/py/export_for_simulink.py --case A4_descent_replay_S00 --arm with_cmg
    python simulink/py/export_for_simulink.py --all

Two products per (case, arm, plant):

  1. ``<tag>_case.json``  -- a *self-describing* snapshot of one run:
        * scenario constants the plant needs (J, CMG pyramid geometry, rate
          limits, control period, RK4 sub-steps, state layout),
        * the reference flight condition at t_next (rho, V_ned, q_ref,
          omega_ref, qbar, alpha_ref) for the diagnostics,
        * the external torque tau_d sampled exactly at the sub-step midpoints
          that ``ClosedLoop._step_P0`` uses.  The MSIS2 atmosphere is *not*
          re-implemented in MATLAB; it is sampled here once, so both sides see
          the identical disturbance history,
        * the recorded control sequence (tau_a_cmd, nu_cmd) of the reference
          run -> used for the open-loop replay validation,
        * the recorded reference states/errors -> the comparison baseline.

  2. ``<tag>_plantcheck.json`` -- golden one-step plant tests: random reachable
        states/inputs and the exact one-step result of the Python plant.  This is
        the C1 acceptance data (<= 1e-12) for the MATLAB/S-Function port.

Nothing here changes the model: everything is read from the frozen scenario.
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from nearspace.config import _to_ns, deep_merge, read_yaml            # noqa: E402
from nearspace.torque_gap_scenario import Scenario                    # noqa: E402
from nearspace.torque_gap_sim import ClosedLoop                       # noqa: E402
from nearspace.torque_gap_model import (attitude_error_vector,        # noqa: E402
                                        rk4_step_tg, rk4_step_tg_nocmg,
                                        split_state, split_state_nocmg)
from nearspace.model6 import quat_normalize, quat_to_euler            # noqa: E402

CFG = ROOT / "configs" / "torque_gap_s08"
RESULTS = ROOT / "results" / "torque_gap_s08"
OUT = ROOT / "results" / "simulink_port" / "data"

CASES = [
    ("A4_descent_replay_S00", "A4_descent_replay", "no_cmg"),
    ("A4_descent_replay_S00", "A4_descent_replay", "with_cmg"),
    ("A4_descent_replay_S03", "A4_descent_replay", "no_cmg"),
    ("A4_descent_replay_S03", "A4_descent_replay", "with_cmg"),
    ("A1_alpha_p20", "A1_alpha", "no_cmg"),
    ("A1_alpha_p20", "A1_alpha", "with_cmg"),
    ("A2_qbar_p0p5", "A2_qbar", "no_cmg"),
]


class _OcpStub:
    """ClosedLoop only needs n_vars/n_cons for the run metadata."""

    n_vars = 0
    n_cons = 0


def build_closed_loop(case, arm, plant="P0", seed=0):
    path = CFG / "cases" / ("%s_%s.yaml" % (case, arm))
    raw = read_yaml(str(path))
    raw = deep_merge(raw, {"torque_gap": {"plant": plant, "seed": int(seed),
                                          "seed_randomize": False}})
    cfg = _to_ns(raw)
    scn = Scenario(cfg)
    scn.case = "%s_%s" % (case, arm)
    cl = ClosedLoop(scn, ocp=_OcpStub(), plant=plant)
    return scn, cl, raw


def cluster_geometry(scn):
    if not scn.cmg_present:
        return None
    cl = scn.cluster
    return dict(h0=float(scn.prm.cmg.h0),
                g_axis=cl.g_axis.tolist(), s0=cl.s0.tolist(),
                gxs0=cl.gxs0.tolist(),
                gdot_max=float(scn.prm.cmg.gdot_max),
                gddot_max=float(scn.prm.cmg.gddot_max),
                tau_g=float(scn.prm.cmg.tau_g))


def export_case(case, group, arm, plant="P0"):
    scn, cl, raw = build_closed_loop(case, arm, plant)
    tag = "%s_%s_%s" % (case, arm, plant)
    veh = scn.veh
    n_steps = cl.n_steps
    dt = float(cl.dt)
    sub = int(cl.substeps)
    h = dt / sub
    t0 = float(scn.t_start)

    # ---- external torque at every sub-step midpoint (exact plant input) ----
    tau_d = np.zeros((n_steps, sub, 3))
    times_mid = np.zeros((n_steps, sub))
    for k in range(n_steps):
        t = t0 + k * dt
        for i in range(sub):
            tm = t + (i + 0.5) * h
            times_mid[k, i] = tm
            tau_d[k, i] = cl._tau_d(tm)

    # ---- reference flight condition at t_next (diagnostics only) ----------
    t_next = np.array([t0 + (k + 1) * dt for k in range(n_steps)])
    rho = np.array([scn.traj.rho(t) for t in t_next])
    V = np.array([scn.traj.V(t) for t in t_next])
    V_ned = np.array([scn.traj.V_ned(t) for t in t_next])
    q_ref = np.array([scn.traj.q_ref(t) for t in t_next])
    w_ref = np.array([scn.traj.omega_ref(t) for t in t_next])
    alpha_ref = np.array([np.rad2deg(scn.traj.pitch_ref(t) - scn.traj.gamma(t))
                          for t in t_next])
    h_m = np.array([scn.traj.h(t) for t in t_next])
    mach = np.array([V[i] / float(scn.atm.a(h_m[i])) for i in range(n_steps)])
    kn = np.array([float(scn.atm.knudsen(h_m[i], veh.cbar)) for i in range(n_steps)])

    # ---- recorded reference run: commands + states + errors ---------------
    run_dir = RESULTS / group / ("%s_%s" % (case, arm))
    rec = read_csv(run_dir / "timeseries.csv")
    if len(rec["t_next"]) != n_steps:
        raise SystemExit("recorded run length %d != %d" % (len(rec["t_next"]), n_steps))

    payload = dict(
        tag=tag, case=case, arm=arm, plant=plant, group=group,
        case_file=str((CFG / "cases" / ("%s_%s.yaml" % (case, arm))).relative_to(ROOT)),
        nx=int(scn.nx), nu=int(scn.nu), cmg_present=bool(scn.cmg_present),
        cmg_enabled=bool(scn.cmg_enabled),
        cmg_geometry=cluster_geometry(scn),
        J=[float(veh.Jx), float(veh.Jy), float(veh.Jz)],
        S=float(veh.S), b=float(veh.b), cbar=float(veh.cbar),
        Cm0=float(veh.Cm0), Cm_alpha=float(veh.Cm_alpha), Cm_q=float(veh.Cm_q),
        Cm_de=float(veh.Cm_de),
        delta_max=[float(v) for v in veh.delta_max],
        x0=[float(v) for v in cl._init_plant()],
        t_start=t0, t_end=float(scn.t_end), dt=dt, substeps=sub, n_steps=n_steps,
        gamma_unbounded=bool(scn.gamma_unbounded),
        tau_d=tau_d.tolist(),                       # (n_steps, sub, 3)
        times_mid=times_mid.tolist(),
        diag=dict(t_next=t_next.tolist(), rho=rho.tolist(), V=V.tolist(),
                  V_ned=V_ned.tolist(), q_ref=q_ref.tolist(),
                  omega_ref=w_ref.tolist(), alpha_ref_deg=alpha_ref.tolist(),
                  h_m=h_m.tolist(), mach=mach.tolist(), knudsen=kn.tolist()),
        ref_run=dict(u=[[float(rec["tauc_cmd_%s" % ax][k]) for ax in "xyz"]
                        + ([float(rec["nuc%d" % (i + 1)][k]) for i in range(4)]
                           if scn.cmg_present else [])
                        for k in range(n_steps)],
                     x=[[float(rec["q%d" % i][k]) for i in range(4)]
                        + [float(rec["wx" if a == 0 else ("wy" if a == 1 else "wz")][k])
                           for a in range(3)]
                        + ([float(rec["g%d" % (i + 1)][k]) for i in range(4)]
                           if scn.cmg_present else [])
                        for k in range(n_steps)],
                     att_err_deg=rec["att_err_deg"].tolist(),
                     cmg=[[float(rec["cmg_%s" % ax][k]) if rec["cmg_%s" % ax][k] == rec["cmg_%s" % ax][k]
                           else None for ax in "xyz"] for k in range(n_steps)]
                     if scn.cmg_present else None),
    )
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / ("%s_case.json" % tag)).write_text(
        json.dumps(payload, ensure_ascii=False) + "\n", encoding="utf-8")

    # ---- golden one-step plant tests --------------------------------------
    rng = np.random.default_rng(20240808)
    tests = []
    for i in range(8):
        q = quat_normalize(rng.normal(size=4))
        w = rng.uniform(-0.02, 0.02, 3)
        if scn.cmg_present:
            x = np.concatenate([q, w, rng.uniform(-1.2, 1.2, 4)])
        else:
            x = np.concatenate([q, w])
        if scn.cmg_present:
            u = np.concatenate([rng.uniform(-60.0, 60.0, 3),
                                rng.uniform(-1.0, 1.0, 4)])
        else:
            u = rng.uniform(-60.0, 60.0, 3)
        td = rng.uniform(-5.0, 5.0, 3)
        x1 = one_plant_step(scn, cl, x, u, td)
        tests.append(dict(x0=x.tolist(), u=u.tolist(), tau_d=td.tolist(),
                          dt=dt, substeps=sub, x1=x1.tolist()))
    (OUT / ("%s_plantcheck.json" % tag)).write_text(
        json.dumps(dict(tag=tag, nx=int(scn.nx), tests=tests), ensure_ascii=False)
        + "\n", encoding="utf-8")
    return tag, n_steps


def one_plant_step(scn, cl, x, u, tau_d_scalar, substeps=None):
    """Exactly ClosedLoop._step_P0 for a single sub-step-constant tau_d.

    The sub-step disturbance of the reference run varies, but not measurably
    inside one control interval; the golden tests use a constant tau_d, and the
    replay validation uses the per-sub-step table from the case file.
    """
    sub = int(substeps or cl.substeps)
    h = cl.dt / sub
    x = np.asarray(x, float).copy()
    if scn.cmg_present:
        tau_a, nu = np.asarray(u[:3], float), np.asarray(u[3:], float)
        for _ in range(sub):
            x = rk4_step_tg(x, np.concatenate([tau_a, nu]),
                            np.asarray(tau_d_scalar, float), h, scn.prm, scn.cluster)
    else:
        for _ in range(sub):
            x = rk4_step_tg_nocmg(x, np.asarray(u[:3], float),
                                  np.asarray(tau_d_scalar, float), h, scn.prm)
    return x


def read_csv(path):
    with open(path, "r", encoding="utf-8", newline="") as fh:
        rows = list(csv.DictReader(fh))
    out = {}
    for k in rows[0]:
        try:
            out[k] = np.array([np.nan if r[k] in ("", None) else float(r[k])
                               for r in rows])
        except ValueError:
            out[k] = np.array([r[k] for r in rows], dtype=object)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--case", default=None)
    ap.add_argument("--arm", default="with_cmg")
    ap.add_argument("--plant", default="P0")
    ap.add_argument("--all", action="store_true")
    args = ap.parse_args()
    jobs = CASES if args.all else [(args.case or "A4_descent_replay_S00",
                                    "A4_descent_replay", args.arm)]
    for case, group, arm in jobs:
        tag, n = export_case(case, group, arm, args.plant)
        print("[export] %-42s steps=%d" % (tag, n))
    print("[export] -> %s" % OUT.relative_to(ROOT))
    return 0


if __name__ == "__main__":
    sys.exit(main())
