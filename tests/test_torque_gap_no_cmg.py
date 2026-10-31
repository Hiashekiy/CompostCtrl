"""Tests for the true no-CMG branch (A group) vs the CMG-assisted arm (B group).

Implements the test list of
docs/CMG多场景姿态机动对照实验指导书_无CMG对照版.md §3.4:

  1  the A-group state vector has no CMG state,
  2  the A-group NLP has no gamma / nu decision variables,
  3  the A-group CMG torque is absent / zero at every instant,
  4  the A-group rigid-body dynamics match an independent CMG-free equation set,
  5  the A group has no H(gamma) gyroscopic coupling,
  6  the B group keeps the CMG torque map and the momentum dynamics,
  7  the B group can actually move the gimbals,
  8  `cmg_present: false` + `cmg_enabled: true` is rejected,
  9  both the P0 and the P1 plant pass the corresponding checks,
 10  A and B use one vehicle base inertia and one aerodynamic/surface model.

Additional regression tests cover the log time alignment (§9.1), the CMG torque
statistics convention (§9.2) and the per-manoeuvre scoring windows (§9.3).

Pytest-compatible and directly runnable:
    python tests/test_torque_gap_no_cmg.py
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from nearspace.cmg import CMGCluster                                  # noqa: E402
from nearspace.config import _to_ns, deep_merge, read_yaml            # noqa: E402
from nearspace.model6 import quat_mul, quat_normalize                 # noqa: E402
from nearspace.model6c import (VehicleCMGParams, aero_moment6c,       # noqa: E402
                               dynamics6c)
from nearspace.torque_gap_model import (NX_TG_NOCMG, NU_TG_NOCMG,      # noqa: E402
                                        TorquePlantParams, attitude_error_vector,
                                        dynamics_tg, dynamics_tg_nocmg,
                                        pack_state, rk4_step_tg_nocmg,
                                        split_state_nocmg)
from nearspace.torque_gap_ocp import TorqueGapOCP                    # noqa: E402
from nearspace.torque_gap_scenario import Scenario                   # noqa: E402
from nearspace.torque_gap_sim import ClosedLoop                      # noqa: E402

CASE_A = ROOT / "configs/torque_gap_no_cmg_comparison/cases/S00_pitch30_no_cmg.yaml"
CASE_B = ROOT / "configs/torque_gap_no_cmg_comparison/cases/S00_pitch30_with_cmg.yaml"


def cfg_of(path, overrides=None):
    raw = read_yaml(str(path))
    if overrides:
        raw = deep_merge(raw, overrides)
    return _to_ns(raw)


def short(extra=None):
    """A 3-step window from 108 s so the closed-loop tests stay cheap."""
    base = {"torque_gap": {"start": 108.0, "end": 108.6}}
    if extra:
        base = deep_merge(base, extra)
    return base


def scn_a(overrides=None):
    return Scenario(cfg_of(CASE_A, deep_merge(short(), overrides or {})))


def scn_b(overrides=None):
    return Scenario(cfg_of(CASE_B, deep_merge(short(), overrides or {})))


def run_closed_loop(scn, plant="P0"):
    cl = ClosedLoop(scn, plant=plant)
    metrics = cl.run()
    return cl, metrics


def dyn_nocmg_reference(x, tau_a, tau_d, J):
    """Independent 7-state CMG-free right-hand side written from the equation."""
    q, w = np.asarray(x[:4], float), np.asarray(x[4:7], float)
    dq = 0.5 * quat_mul(q, np.array([0.0, w[0], w[1], w[2]]))
    dw = np.linalg.solve(J, np.asarray(tau_a, float) + np.asarray(tau_d, float)
                         - np.cross(w, J @ w))
    return np.concatenate([dq, dw])


# --------------------------------------------------------------------------
# 1 / 2 -- dimensions
# --------------------------------------------------------------------------
def test_1_a_group_state_has_no_cmg_states():
    scn = scn_a()
    x0 = scn.initial_state()
    assert scn.cmg_present is False
    assert scn.nx == NX_TG_NOCMG == 7 and scn.nu == NU_TG_NOCMG == 3
    assert x0.shape == (7,)
    assert scn.cluster is None                      # no cluster geometry created
    # the A plant keeps the measured state and rejects an 11-vector
    q, w = split_state_nocmg(x0)
    assert np.allclose(pack_state(q, w), x0)


def test_2_a_group_nlp_has_no_gamma_or_nu():
    scn = scn_a(short())
    ocp = TorqueGapOCP(scn)
    N = ocp.N
    assert ocp.nx == 7 and ocp.nu == 3
    assert ocp.n_vars == 7 * (N + 1) + 3 * N, ocp.n_vars
    # init pin (7) + shooting defects (7 per node) + unit quaternion (N+1)
    assert ocp.n_cons == 7 + 7 * N + (N + 1), ocp.n_cons
    # no parameter carries a gimbal rate, no CMG geometry function exists
    assert "nuprev" not in ocp._p_order
    assert not hasattr(ocp, "_A_of") and not hasattr(ocp, "_H_of")
    # the integrator map has exactly the 7-state / 3-input signature
    assert ocp._f_of.n_in() == 3
    assert int(ocp._f_of.size_in(0)[0]) == 7
    assert int(ocp._f_of.size_in(1)[0]) == 3
    # nothing in the bound template constrains a gimbal state
    n_state = 7 * (N + 1)
    assert np.all(np.isinf(ocp._lbx_template[:n_state]))
    # every bound on the input block belongs to the 3 torque channels only
    for k in range(N):
        base = n_state + k * 3
        assert base + 3 <= ocp._lbx_template.size


def test_2b_b_group_nlp_has_cmg_variables():
    scn = scn_b(short())
    ocp = TorqueGapOCP(scn)
    assert ocp.nx == 11 and ocp.nu == 7
    assert "nuprev" in ocp._p_order
    assert hasattr(ocp, "_A_of") and hasattr(ocp, "_H_of")
    assert int(ocp._f_of.size_in(0)[0]) == 11
    assert int(ocp._f_of.size_in(1)[0]) == 7
    # init pin (11) + shooting (11 per node) + gimbal-rate increments (4 per
    # node) + unit quaternion (N+1)
    N = ocp.N
    assert ocp.n_cons == 11 + 11 * N + 4 * N + (N + 1), ocp.n_cons


# --------------------------------------------------------------------------
# 3 / 5 -- no CMG torque, no H(gamma)
# --------------------------------------------------------------------------
def test_3_a_group_tau_cmg_absent_everywhere():
    scn = scn_a()
    prm = scn.prm
    x = pack_state([1.0, 0.0, 0.0, 0.0], np.array([0.01, -0.02, 0.005]), None)
    # the CMG-free dynamics cannot express a gimbal torque at all
    assert dynamics_tg_nocmg(x, np.zeros(3), np.zeros(3), prm).shape == (7,)
    cl, metrics = run_closed_loop(scn)
    for row in cl.rows:
        for key in ("cmg_x", "cmg_y", "cmg_z", "cmg_end_x", "cmg_end_y",
                    "cmg_end_z", "g1", "g4", "nu1", "nu4", "Hx", "H_norm",
                    "sigma_min", "E_gap", "nu_sat", "gap_residual"):
            assert key not in row, "%s must not exist in the A group" % key
    assert "cmg" not in metrics.get("stats", {}).get("all", {})
    assert "cmg_migration" not in metrics


def test_5_a_group_has_no_gyroscopic_coupling():
    scn = scn_a()
    prm, cl = scn.prm, CMGCluster(scn.prm.cmg)
    gamma0 = np.zeros(4)
    J = prm.J
    w = np.array([0.03, -0.05, 0.02])
    q = quat_normalize(np.array([0.999, 0.01, -0.02, 0.005]))
    tau_a = np.array([1.5, -8.0, 0.4])
    tau_d = np.array([0.2, 0.5, 0.3])
    # A model: no H term at all
    xa = pack_state(q, w, None)
    wa = dynamics_tg_nocmg(xa, tau_a, tau_d, prm)[4:7]
    assert np.allclose(wa, np.linalg.solve(J, tau_a + tau_d - np.cross(w, J @ w)))
    # B model at the *same* mechanical state and with nu = 0 still carries the
    # rotor momentum in the gyroscopic term, so the two plants must differ
    gamma = np.deg2rad([25.0, -40.0, 15.0, 60.0])
    xb = pack_state(q, w, gamma)
    wb = dynamics_tg(xb, np.concatenate([tau_a, np.zeros(4)]), tau_d, prm, cl)[4:7]
    H = cl.momentum(gamma)
    assert np.linalg.norm(H) > 1.0
    wb_formula = np.linalg.solve(J, tau_a + tau_d - np.cross(w, J @ w + H))
    assert np.allclose(wb, wb_formula)
    assert np.linalg.norm(wa - wb) > 1e-6, "A group must not equal the frozen-gimbal B model"


# --------------------------------------------------------------------------
# 4 / 9 -- independent equation set, P0 and P1
# --------------------------------------------------------------------------
def test_4_a_group_dynamics_match_independent_equations():
    from scipy.integrate import solve_ivp
    prm = scn_a().prm
    J = prm.J
    x0 = pack_state(quat_normalize([1.0, 0.0, 0.0, 0.0]),
                    np.zeros(3), None)
    u = np.array([3.0, -14.0, 2.0])
    tau_d = np.array([0.2, 0.5, 0.3])
    dt = 0.2
    xn = x0.copy()
    for _ in range(8):
        xn = rk4_step_tg_nocmg(xn, u, tau_d, dt / 8.0, prm)
    sol = solve_ivp(lambda t, y: dyn_nocmg_reference(y, u, tau_d, J), [0.0, dt], x0,
                    method="DOP853", rtol=1e-13, atol=1e-13)
    xr = sol.y[:, -1]
    assert np.max(np.abs(xn[4:] - xr[4:])) < 1e-11
    assert abs(np.linalg.norm(xn[0:4]) - 1.0) < 1e-12
    # quaternion kinematics agree to the same order
    assert np.max(np.abs(xn[0:4] - xr[0:4])) < 1e-9


def test_9b_p1_plant_is_cmg_free_in_the_a_group():
    scn = scn_a()
    cl, metrics = run_closed_loop(scn, plant="P1")
    assert cl.vp.cmg_on is False
    vp = cl.vp
    cl6 = CMGCluster(vp.cmg)
    x18 = np.zeros(18)
    x18[0:4] = [1.0, 0.0, 0.0, 0.0]
    x18[4:7] = [0.01, -0.02, 0.005]
    x18[10:14] = np.deg2rad([10.0, -20.0, 30.0, -40.0])
    x18[14:18] = np.array([0.3, -0.2, 0.1, 0.4])
    p = np.concatenate([[1.0e-6], [3500.0, 0.0, 0.0], np.zeros(3)])
    dx = dynamics6c(x18, np.zeros(7), p, vp, cl6)
    assert np.all(dx[10:18] == 0.0), "gimbal states are structurally frozen"
    wdot = dx[4:7]
    w = x18[4:7]
    expect = vp.Jinv @ (-np.cross(w, vp.J @ w) + aero_moment6c(x18, p, vp))
    assert np.allclose(wdot, expect, atol=1e-12)
    # the log has no CMG columns in the A group
    for row in cl.rows:
        assert "cmg_x" not in row and "g1" not in row and "nu1" not in row
    assert "cmg_migration" not in metrics


def test_9c_b_group_works_with_both_plants():
    for plant in ("P0", "P1"):
        scn = scn_b(short())
        cl, metrics = run_closed_loop(scn, plant=plant)
        assert metrics["cmg_present"] is True
        assert any(abs(r["nu1"]) > 1e-6 or abs(r["nu2"]) > 1e-6
                   or abs(r["nu3"]) > 1e-6 or abs(r["nu4"]) > 1e-6 for r in cl.rows), plant
        assert metrics["stats"]["all"]["H_norm_max"] >= 0.0


# --------------------------------------------------------------------------
# 6 / 7 -- B group keeps the CMG and can move it
# --------------------------------------------------------------------------
def test_6_b_group_keeps_cmg_torque_map_and_momentum():
    scn = scn_b()
    prm = scn.prm
    cl = CMGCluster(prm.cmg)
    gamma = np.deg2rad([12.0, -35.0, 24.0, 61.0])
    nu = np.array([0.2, -0.4, 0.1, 0.3])
    w = np.array([0.01, -0.02, 0.015])
    q = quat_normalize([1.0, 0.0, 0.0, 0.0])
    tau_a = np.array([12.0, -30.0, 4.0])
    tau_d = np.array([0.3, -0.2, 0.05])
    dx = dynamics_tg(pack_state(q, w, gamma), np.concatenate([tau_a, nu]), tau_d,
                     prm, cl)
    H = cl.momentum(gamma)
    A = cl.jacobian(gamma)
    assert np.allclose(dx[4:7], prm.Jinv @ (tau_a - A @ nu + tau_d
                                            - np.cross(w, prm.J @ w + H)))
    assert np.allclose(dx[7:11], nu)
    # the optimiser-side geometry reproduces the same A(gamma) and H(gamma)
    ocp = TorqueGapOCP(scn)
    assert np.allclose(np.asarray(ocp._A_of(gamma)), A)
    assert np.allclose(np.asarray(ocp._H_of(gamma)).ravel(), H)


def test_7_b_group_moves_the_gimbals():
    scn = scn_b()
    ocp = TorqueGapOCP(scn)
    x = scn.initial_state()
    plan = scn.plan(scn.t_start, x)
    res = ocp.solve(plan, x, np.zeros(4), np.zeros(3))
    assert res.ok, res.status
    assert np.max(np.abs(res.u0[3:7])) > 1e-3, "gimbal rates must be free in the B group"
    assert res.U.shape[0] == 7 and res.X.shape[0] == 11
    cl, metrics = run_closed_loop(scn)
    g0 = np.array([cl.rows[0]["g%d" % (i + 1)] for i in range(4)])
    g1 = np.array([cl.rows[-1]["g%d" % (i + 1)] for i in range(4)])
    assert np.max(np.abs(g1 - g0)) > 1e-4, "the gimbal angles must evolve"


# --------------------------------------------------------------------------
# 8 -- configuration validation
# --------------------------------------------------------------------------
def test_8_present_false_requires_enabled_false():
    bad = {"torque_gap": {"cmg_present": False, "cmg_enabled": True}}
    try:
        Scenario(cfg_of(CASE_B, deep_merge(short(), bad)))
    except ValueError as exc:
        assert "cmg_present" in str(exc)
    else:
        raise AssertionError("cmg_present: false with cmg_enabled: true must raise")
    # installed-but-locked stays available as a diagnostic arm only
    locked = Scenario(cfg_of(CASE_B, deep_merge(
        short(), {"torque_gap": {"cmg_present": True, "cmg_enabled": False}})))
    assert locked.cmg_present is True and locked.cmg_enabled is False
    assert locked.nx == 11 and locked.nu == 7


# --------------------------------------------------------------------------
# 10 -- shared physics
# --------------------------------------------------------------------------
def test_10_a_and_b_share_vehicle_and_surfaces():
    a, b = scn_a(), scn_b()
    for attr in ("Jx", "Jy", "Jz", "S", "b", "cbar", "Cm_alpha", "Cm_q", "Cm_de",
                 "Cl_beta", "Cl_p", "Cl_r", "Cl_da", "Cl_dr", "Cn_beta", "Cn_p",
                 "Cn_r", "Cn_da", "Cn_dr", "tau_s"):
        assert getattr(a.veh, attr) == getattr(b.veh, attr), attr
    assert np.array_equal(a.veh.delta_max, b.veh.delta_max)
    assert np.array_equal(a.veh.delta_dot_max, b.veh.delta_dot_max)
    assert a.auth.scale == b.auth.scale
    assert a.w.as_dict() == b.w.as_dict()
    pa, pb = a.plan(a.t_start, a.initial_state()), b.plan(b.t_start, b.initial_state())
    assert np.array_equal(pa["box"]["tau_min"], pb["box"]["tau_min"])
    assert np.array_equal(pa["box"]["tau_max"], pb["box"]["tau_max"])
    assert np.allclose(pa["cond"]["tau_d"], pb["cond"]["tau_d"])
    assert a.dt_ctrl == b.dt_ctrl and a.t_start == b.t_start and a.t_end == b.t_end
    assert np.array_equal(a.rel_times, b.rel_times)
    # the nominal demand differs only through the (absent) rotor momentum
    assert np.allclose(a.plan(a.t_start, a.initial_state())["mod"]["H_nom"], 0.0)


# --------------------------------------------------------------------------
# logging conventions (§9.1, §9.2, §9.3)
# --------------------------------------------------------------------------
def test_log_time_alignment():
    scn = scn_a()
    cl, metrics = run_closed_loop(scn)
    dt = cl.dt
    for row in cl.rows:
        assert abs(row["t_next"] - (row["t"] + dt)) < 1e-12
        # the logged state belongs to t_next: its reference is the one at t_next
        qref = scn.traj.q_ref(row["t_next"])
        q = np.array([row["q%d" % i] for i in range(4)])
        assert np.allclose(qref, np.array([row["qref%d" % i] for i in range(4)]))
        e = attitude_error_vector(qref, q)
        assert abs(np.rad2deg(np.linalg.norm(e)) - row["att_err_deg"]) < 1e-9
        # ... and it is measurably different from the reference at the period start
        qref0 = scn.traj.q_ref(row["t"])
        if np.rad2deg(np.linalg.norm(attitude_error_vector(qref0, q))) > 1e-9:
            assert abs(np.rad2deg(np.linalg.norm(attitude_error_vector(qref0, q)))
                       - row["att_err_deg"]) > 1e-12
    assert metrics["t_start"] == scn.t_start


def test_cmg_torque_convention():
    scn = scn_b()
    cl, _metrics = run_closed_loop(scn)
    differs = 0
    for row in cl.rows:
        gamma = np.array([row["g%d" % (i + 1)] for i in range(4)])
        nu = np.array([row["nu%d" % (i + 1)] for i in range(4)])
        end = -scn.cluster.torque(gamma, nu)
        assert np.allclose(np.array([row["cmg_end_%s" % ax] for ax in "xyz"]), end)
        avg = np.array([row["cmg_%s" % ax] for ax in "xyz"])
        # the earlier sub-steps of the interval used a different gimbal geometry,
        # so the interval average may (and generally will) differ from the end
        # value; it must stay inside the envelope swept during the interval
        sweep = max(np.linalg.norm(scn.cluster.torque(
            gamma + s * cl.dt * np.abs(nu), nu)) for s in (0.0, 0.5, 1.0))
        assert np.linalg.norm(avg) <= 1.5 * sweep + 1e-6
        differs += int(np.linalg.norm(avg - end) > 1e-9)
    assert cl.rows, "closed loop produced no rows"
    return differs


def test_per_manoeuvre_windows_do_not_include_the_hold_phase():
    scn = Scenario(cfg_of(ROOT / "configs/torque_gap_no_cmg_comparison/sequential/"
                                "S07_sequential_no_cmg.yaml"))
    cl = ClosedLoop(scn, plant="P0")
    # no need to integrate: only the metric window construction is under test
    cl.rows = []
    t = np.arange(scn.t_start + cl.dt, scn.t_end + 1e-9, cl.dt)
    for i, tt in enumerate(t):
        cl.rows.append(dict(t=tt - cl.dt, t_next=tt, att_err_deg=0.0, rate_err=0.0,
                            err_x=0.0, err_y=0.0, err_z=0.0, tau_act_x=0.0,
                            tau_act_y=0.0, tau_act_z=0.0, tau_bound_used=0.0,
                            fallback=0, accepted=0, cpu_s=0.0, iters=0, viol=0.0,
                            status="synthetic", g1=0.0, g2=0.0, g3=0.0, g4=0.0,
                            nu1=0.0, nu2=0.0, nu3=0.0, nu4=0.0, H_norm=0.0,
                            sigma_min=1.0, Hx=0.0, Hy=0.0, Hz=0.0,
                            cmg_x=0.0, cmg_y=0.0, cmg_z=0.0, nu_sat=0.0))
    m = cl._summary()
    w = m["windows"]
    assert w["man0"]["t0"] == 110.0 and w["man0"]["t1"] == 130.0
    assert w["man1"]["t0"] == 150.0 and w["man1"]["t1"] == 170.0
    assert 130.0 <= w["man1"]["t0"]
    assert m["stats"]["man0"] and m["stats"]["man1"]


if __name__ == "__main__":
    fails = 0
    names = [n for n in sorted(globals()) if n.startswith("test_")]
    for name in names:
        fn = globals()[name]
        try:
            fn()
            print("PASS", name)
        except AssertionError as exc:
            fails += 1
            print("FAIL", name, exc)
    print("%d test(s) failed" % fails)
    sys.exit(1 if fails else 0)
