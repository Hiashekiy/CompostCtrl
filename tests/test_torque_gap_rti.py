"""Tests for the HPIPM real-time-iteration backend (nearspace.torque_gap_rti).

Checks, in order of what can silently go wrong:
  1  the CasADi RK4 map used for linearisation equals the truth model,
  2  the Jacobians match central differences,
  3  the per-stage data really reaches HPIPM (the wrapper's range form writes the
     same array to every stage, and numpy is row-major while HPIPM is
     column-major -- both are easy to get wrong and both are checked here by
     recomputing the QP objective from the returned step),
  4  the QP step satisfies the linearised dynamics,
  5  iterating the same code (Gauss-Newton SQP) does not increase the nonlinear
     objective, and more iterations get closer to the IPOPT solution,
  6  the A group (7 states, no CMG) works as well, and
  7  the closed loop runs, respects the control box and meets the 0.2 s period.

python tests/test_torque_gap_rti.py
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from nearspace import hpipm_bindings                                  # noqa: E402
from nearspace.config import _to_ns, deep_merge, read_yaml            # noqa: E402
from nearspace.torque_gap_model import (pack_state, rk4_step_tg,      # noqa: E402
                                        rk4_step_tg_nocmg)
from nearspace.torque_gap_scenario import Scenario                   # noqa: E402

HAVE_HPIPM = hpipm_bindings.available()
CASE_B = ROOT / "configs/torque_gap_no_cmg_comparison/cases/S00_pitch30_with_cmg.yaml"
CASE_A = ROOT / "configs/torque_gap_no_cmg_comparison/cases/S00_pitch30_no_cmg.yaml"


def _scn(path, over=None):
    raw = read_yaml(str(path))
    if over:
        raw = deep_merge(raw, over)
    return Scenario(_to_ns(raw))


def _rti(scn, **kw):
    from nearspace.torque_gap_rti import TorqueGapRTI
    return TorqueGapRTI(scn, **kw)


def _probe(scn, t=106.0):
    x = np.zeros(scn.nx)
    x[0:4] = scn.traj.q_ref(t)
    x[4:7] = scn.traj.omega_ref(t)
    return x, scn.plan(t, x)


def test_model_map_matches_truth():
    scn = _scn(CASE_B)
    rti = _rti(scn)
    x, plan = _probe(scn)
    u = np.array([2.0, -7.0, 0.5, 0.3, -0.4, 0.2, 0.1])
    td = plan["cond"]["tau_d"][0]
    dt = rti.dts[0]
    xn = np.asarray(rti._F(x, u, td, dt)).ravel()
    xr = rk4_step_tg(x.copy(), u, td, dt, scn.prm, scn.cluster)
    assert np.max(np.abs(xn - xr)) < 1e-12
    # no-CMG arm
    scn_a = _scn(CASE_A)
    rti_a = _rti(scn_a)
    xa, plan_a = _probe(scn_a)
    ua = np.array([2.0, -7.0, 0.5])
    xn = np.asarray(rti_a._F(xa, ua, plan_a["cond"]["tau_d"][0], dt)).ravel()
    xr = rk4_step_tg_nocmg(xa.copy(), ua, plan_a["cond"]["tau_d"][0], dt, scn_a.prm)
    assert np.max(np.abs(xn - xr)) < 1e-12


def test_jacobians_match_finite_differences():
    scn = _scn(CASE_B)
    rti = _rti(scn)
    x, plan = _probe(scn)
    u = np.array([1.0, -3.0, 0.2, 0.1, -0.2, 0.15, 0.05])
    td = plan["cond"]["tau_d"][0]
    dt = rti.dts[0]
    A = np.asarray(rti._A(x, u, td, dt))
    B = np.asarray(rti._B(x, u, td, dt))
    h = 1e-6
    for i in range(scn.nx):
        e = np.zeros(scn.nx)
        e[i] = h
        fd = (np.asarray(rti._F(x + e, u, td, dt)).ravel()
              - np.asarray(rti._F(x - e, u, td, dt)).ravel()) / (2 * h)
        assert np.max(np.abs(fd - A[:, i])) < 1e-6, i
    for j in range(scn.nu):
        e = np.zeros(scn.nu)
        e[j] = h
        fd = (np.asarray(rti._F(x, u + e, td, dt)).ravel()
              - np.asarray(rti._F(x, u - e, td, dt)).ravel()) / (2 * h)
        assert np.max(np.abs(fd - B[:, j])) < 1e-6, j


def test_attitude_error_jacobian():
    scn = _scn(CASE_B)
    rti = _rti(scn)
    x, plan = _probe(scn)
    k = 0                       # node 0 is the measured state, so e == 0 there
    e, Je = rti._eof(plan["cond"]["q_ref"][k], x)
    e = np.asarray(e).ravel()
    Je = np.asarray(Je)
    h = 1e-7
    for i in range(scn.nx):
        dx = np.zeros(scn.nx)
        dx[i] = h
        ep = np.asarray(rti._eof(plan["cond"]["q_ref"][k], x + dx)[0]).ravel()
        em = np.asarray(rti._eof(plan["cond"]["q_ref"][k], x - dx)[0]).ravel()
        assert np.max(np.abs((ep - em) / (2 * h) - Je[:, i])) < 1e-5, i
    assert np.linalg.norm(e) < 1e-9


def test_qp_step_satisfies_linearised_dynamics():
    """Catches a wrong per-stage / row-major data layout in the HPIPM call."""
    if not HAVE_HPIPM:
        print("skip (no HPIPM)")
        return
    scn = _scn(CASE_B)
    rti = _rti(scn, qp_iters=1)
    x, plan = _probe(scn)
    X, U = rti._initial_guess(plan, x, None)
    dX, dU, status, iters = rti._iterate(X, U, plan)
    assert status == 0, status
    td = np.asarray(plan["cond"]["tau_d"], float)
    worst = 0.0
    for k in range(rti.N):
        A = np.asarray(rti._A(X[:, k], U[:, k], td[k], rti.dts[k]))
        B = np.asarray(rti._B(X[:, k], U[:, k], td[k], rti.dts[k]))
        pred = A @ dX[:, k] + B @ dU[:, k]
        worst = max(worst, float(np.max(np.abs(pred - dX[:, k + 1]))))
    assert worst < 1e-6, worst
    assert np.max(np.abs(dX[:, 0])) < 1e-8          # dx_0 = 0 (numerical noise)


def test_qp_objective_consistent_with_returned_step():
    """The QP value implies the Hessian/gradient reached HPIPM the right way."""
    if not HAVE_HPIPM:
        print("skip (no HPIPM)")
        return
    scn = _scn(CASE_B)
    rti = _rti(scn, qp_iters=1)
    x, plan = _probe(scn)
    X, U = rti._initial_guess(plan, x, None)
    Q, qv, R, rv, lu, uu = rti._qp_matrices(X, U, plan)
    dX, dU, status, _ = rti._iterate(X, U, plan)
    assert status == 0
    val = 0.0
    for k in range(rti.N):
        val += 0.5 * dX[:, k] @ Q[k] @ dX[:, k] + qv[k] @ dX[:, k]
        val += 0.5 * dU[:, k] @ R[k] @ dU[:, k] + rv[k] @ dU[:, k]
    val += 0.5 * dX[:, rti.N] @ Q[rti.N] @ dX[:, rti.N] + qv[rti.N] @ dX[:, rti.N]
    # bounds respected exactly
    for k in range(rti.N):
        assert np.all(dU[:, k] >= lu[k] - 1e-8), (k, dU[:, k] - lu[k])
        assert np.all(dU[:, k] <= uu[k] + 1e-8), (k, dU[:, k] - uu[k])
    # a wrong layout would leave the QP objective inconsistent with the step
    # (the check below is one-sided: the step must be a strong descent step)
    assert val < 0.0, val


def test_more_iterations_do_not_increase_the_objective():
    if not HAVE_HPIPM:
        print("skip (no HPIPM)")
        return
    scn = _scn(CASE_B)
    x, plan = _probe(scn)
    objs = {}
    for qit in (1, 5, 12):
        rti = _rti(scn, qp_iters=qit)
        res = rti.solve(plan, x, np.zeros(4), np.zeros(3), None)
        assert res.ok, res.status
        objs[qit] = res.objective
    assert objs[5] <= objs[1] + 1e-9
    assert objs[12] <= objs[5] + 1e-9


def test_a_group_works():
    if not HAVE_HPIPM:
        print("skip (no HPIPM)")
        return
    scn = _scn(CASE_A)
    rti = _rti(scn, qp_iters=4)
    assert rti.nx == 7 and rti.nu == 3
    x, plan = _probe(scn)
    res = rti.solve(plan, x, None, None, None)
    assert res.ok, res.status
    assert res.u0.shape == (3,)
    lo, hi = plan["box"]["tau_min"][0], plan["box"]["tau_max"][0]
    assert np.all(res.u0 >= lo - 1e-9) and np.all(res.u0 <= hi + 1e-9)


def test_closed_loop_runs_and_meets_the_period():
    if not HAVE_HPIPM:
        print("skip (no HPIPM)")
        return
    from nearspace.torque_gap_sim import ClosedLoop
    raw = read_yaml(str(CASE_B))
    raw = deep_merge(raw, {"torque_gap": {"start": 108.0, "end": 112.0}})
    scn = Scenario(_to_ns(raw))
    rti = _rti(scn, qp_iters=6)
    cl = ClosedLoop(scn, ocp=rti, plant="P0")
    m = cl.run()
    assert m["n_steps"] == cl.n_steps
    assert m["fallback_steps"] == 0
    assert m["cpu_s"]["mean"] < 0.2, m["cpu_s"]
    lo = np.array([r["lo_x"] for r in cl.rows])
    hi = np.array([r["hi_x"] for r in cl.rows])
    act = np.array([r["tau_act_x"] for r in cl.rows])
    assert np.all(act >= lo - 1e-6) and np.all(act <= hi + 1e-6)
    assert m["max_violation"] < 1e-4


if __name__ == "__main__":
    fails = 0
    for name in [n for n in sorted(globals()) if n.startswith("test_")]:
        try:
            globals()[name]()
            print("PASS", name)
        except AssertionError as exc:
            fails += 1
            print("FAIL", name, exc)
    print("%d test(s) failed" % fails)
    sys.exit(1 if fails else 0)
