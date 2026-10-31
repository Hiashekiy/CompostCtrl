#!/usr/bin/env python
"""Throw-away microbenchmark: same state, different solver settings / grid.

Purpose: replace guesses about "how to make the MPC faster" with numbers.
`casadi.nlpsol` is wrapped locally, so no production code is touched.

Two questions:
  1. solver settings: which per-iteration cost and iteration count are reachable
     on the *same* S00 (B group) NLP, at a representative mid-manoeuvre state;
  2. problem formulation: how the cost scales with the shooting grid size.

python scripts/bench_no_cmg_solver.py
"""
from __future__ import annotations

import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

import nearspace.torque_gap_ocp as ocpmod                       # noqa: E402
from nearspace.config import _to_ns, deep_merge, read_yaml       # noqa: E402
from nearspace.model6 import euler_to_quat, quat_mul, quat_normalize  # noqa: E402
from nearspace.torque_gap_model import pack_state                # noqa: E402
from nearspace.torque_gap_scenario import Scenario               # noqa: E402

CASE = ROOT / "configs/torque_gap_no_cmg_comparison/cases/S00_pitch30_with_cmg.yaml"
T_PROBE = 106.0          # mid-manoeuvre onset: the torque gap is active here
ca = ocpmod._casadi()
_real_nlpsol = ca.nlpsol

IPOPT_VARIANTS = [
    ("ipopt exact+mumps tol1e-6 [current]", dict(ipopt={"tol": 1e-6})),
    ("ipopt exact+mumps tol1e-4", dict(ipopt={"tol": 1e-4})),
    ("ipopt exact+mumps tol1e-3", dict(ipopt={"tol": 1e-3})),
    ("ipopt exact+mumps tol1e-2", dict(ipopt={"tol": 1e-2})),
    ("ipopt lim-mem tol1e-3", dict(ipopt={"tol": 1e-3,
                                          "hessian_approximation": "limited-memory"})),
    ("ipopt ma27 tol1e-4", dict(ipopt={"tol": 1e-4, "linear_solver": "ma27"})),
    ("ipopt ma57 tol1e-4", dict(ipopt={"tol": 1e-4, "linear_solver": "ma57"})),
]

SQP_VARIANTS = [
    ("SQP GN qpoases 20 it", dict(qpsol="qpoases", hessian_approximation="gauss-newton",
                                  max_iter=20, tol_pr=1e-4, tol_du=1e-4)),
    ("SQP GN qpoases 3 it", dict(qpsol="qpoases", hessian_approximation="gauss-newton",
                                 max_iter=3, tol_pr=1e-6, tol_du=1e-6)),
    ("SQP GN qpoases 1 it (RTI)", dict(qpsol="qpoases",
                                      hessian_approximation="gauss-newton",
                                      max_iter=1, tol_pr=1e-4, tol_du=1e-4)),
    ("SQP GN hpipm 3 it", dict(qpsol="hpipm", hessian_approximation="gauss-newton",
                               max_iter=3, tol_pr=1e-4, tol_du=1e-4)),
]

HORIZONS = [
    ("22 nodes / 23.2 s [current]", None),
    ("14 nodes / 12.2 s", [dict(n=6, dt=0.2), dict(n=4, dt=0.5), dict(n=4, dt=3.0)]),
    ("10 nodes / 6.2 s", [dict(n=6, dt=0.2), dict(n=4, dt=1.25)]),
    ("6 nodes / 1.2 s", [dict(n=6, dt=0.2)]),
]


def probe_state(scn):
    """Mid-manoeuvre state with a 1 deg pitch offset (a real tracking problem)."""
    t = T_PROBE
    q = scn.traj.q_ref(t)
    dq = euler_to_quat(np.deg2rad(1.0), 0.0, 0.0)
    q = quat_normalize(quat_mul(dq, q))
    w = np.asarray(scn.traj.omega_ref(t), float)
    gamma0 = np.zeros(4)
    return pack_state(q, w, gamma0)


def make(horizon=None):
    raw = read_yaml(str(CASE))
    if horizon is not None:
        raw = deep_merge(raw, {"torque_gap": {"horizon": horizon}})
    scn = Scenario(_to_ns(raw))
    ocp = ocpmod.TorqueGapOCP(scn)
    x0 = probe_state(scn)
    return scn, ocp, scn.plan(T_PROBE, x0), x0


def solve_with(scn, x0, plan, opts_override, kind="ipopt", warm=None, reps=3):
    solver_name = "ipopt" if kind == "ipopt" else "sqpmethod"

    def patched(nm, sol, nlp, opts, *a, **k):
        opts = {kk: (dict(vv) if isinstance(vv, dict) else vv)
                for kk, vv in opts.items()}
        if kind == "ipopt":
            opts.setdefault("ipopt", {}).update(opts_override.get("ipopt", {}))
        else:
            opts = dict(opts_override)
        opts["print_time"] = False
        return _real_nlpsol(nm, solver_name, nlp, opts, *a, **k)

    ca.nlpsol = patched
    try:
        ocp = ocpmod.TorqueGapOCP(scn)
        times, res = [], None
        for i in range(reps):
            t0 = time.perf_counter()
            res = ocp.solve(plan, x0, np.zeros(4), np.zeros(3), warm)
            times.append(time.perf_counter() - t0)
    finally:
        ca.nlpsol = _real_nlpsol
    return dict(cpu=float(np.median(times)), iters=res.iter_count, ok=bool(res.ok),
                status=res.status, viol=float(res.max_violation),
                obj=float(res.objective), u0=np.asarray(res.u0, float))


def row(name, cold, warm):
    d = "" if warm is None else "%8.3f %6d %6s %8.1e %+.4f" % (
        warm["cpu"], warm["iters"], warm["ok"], warm["viol"], warm["obj"])
    print("%-34s %8.3f %6d %6s %8.1e %+.4f  %s"
          % (name, cold["cpu"], cold["iters"], cold["ok"], cold["viol"],
             cold["obj"], d))


def main():
    scn, ocp, plan, x0 = make()
    base = ocp.solve(plan, x0, np.zeros(4), np.zeros(3))
    warm = (base.X, base.U)
    print("state probe: t=%.1f, 1 deg pitch offset;  baseline cold solve %.3f s / "
          "%d iters\n" % (T_PROBE, base.cpu_s, base.iter_count))
    print("%-34s %8s %6s %6s %8s %8s  %8s %6s %6s %8s %8s"
          % ("variant", "cold[s]", "iters", "ok", "viol", "obj",
             "warm[s]", "iters", "ok", "viol", "obj"))

    for name, over in IPOPT_VARIANTS:
        try:
            cold = solve_with(scn, x0, plan, over, reps=1)
            w = solve_with(scn, x0, plan, over, warm=warm, reps=3)
            row(name, cold, w)
        except Exception as exc:                             # noqa: BLE001
            print("%-34s  FAILED %s: %s" % (name, type(exc).__name__,
                                            str(exc).split("\n")[0][:70]))
    for name, over in SQP_VARIANTS:
        try:
            cold = solve_with(scn, x0, plan, over, kind="sqp", reps=1)
            w = solve_with(scn, x0, plan, over, kind="sqp", warm=warm, reps=3)
            row(name, cold, w)
        except Exception as exc:                             # noqa: BLE001
            print("%-34s  FAILED %s: %s" % (name, type(exc).__name__,
                                            str(exc).split("\n")[0][:70]))

    print("\n-- grid size (ipopt exact+mumps tol1e-3, cold + warm) --")
    print("%-34s %8s %6s %8s %8s %8s" % ("grid", "cold[s]", "iters", "vars",
                                         "warm[s]", "obj(warm)"))
    for label, hz in HORIZONS:
        try:
            scn2, ocp2, plan2, x02 = make(horizon=hz)
            b2 = ocp2.solve(plan2, x02, np.zeros(4), np.zeros(3))
            w2 = solve_with(scn2, x02, plan2, dict(ipopt={"tol": 1e-3}),
                            warm=(b2.X, b2.U), reps=3)
            print("%-34s %8.3f %6d %8d %8.3f %+.4f"
                  % (label, b2.cpu_s, b2.iter_count, ocp2.n_vars,
                     w2["cpu"], w2["obj"]))
        except Exception as exc:                             # noqa: BLE001
            print("%-34s  FAILED %s" % (label, str(exc)[:60]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
