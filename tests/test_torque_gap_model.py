"""Unit / consistency tests for the torque-gap model, authority and modulation.

Pytest-compatible (`pytest tests/test_torque_gap_model.py`) and directly runnable
(`python tests/test_torque_gap_model.py`) because pytest is not installed on the
experiment workstation.

The exhaustive numeric report lives in scripts/check_torque_gap_model.py; these
tests are the fast regression subset.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from nearspace.cmg import CMGCluster  # noqa: E402
from nearspace.config import load_config  # noqa: E402
from nearspace.model6 import euler_to_quat, quat_normalize  # noqa: E402
from nearspace.model6c import VehicleCMGParams, aero_moment6c, dynamics6c  # noqa: E402
from nearspace.torque_gap_authority import (allocate_deflection, axis_hull,  # noqa: E402
                                            natural_moment_v, surface_matrix)
from nearspace.torque_gap_model import (TorquePlantParams, attitude_error_vector,  # noqa: E402
                                        dynamics_tg, pack_state, rk4_step_tg)
from nearspace.torque_gap_modulation import (capability_residual,  # noqa: E402
                                             modulation, nominal_demand)

CFG_PATH = ROOT / "configs" / "torque_gap" / "cases" / "c1_pitch30.yaml"


def _prm():
    cfg = load_config(str(CFG_PATH))
    return cfg, TorquePlantParams.from_config(cfg)


def test_jacobian_is_momentum_derivative():
    _, prm = _prm()
    cl = CMGCluster(prm.cmg)
    for gamma in (np.zeros(4), np.deg2rad([12.0, -35.0, 24.0, 61.0])):
        A = cl.jacobian(gamma)
        for j in range(4):
            nu = np.zeros(4)
            nu[j] = 1.0
            eps = 1e-6
            fd = (cl.momentum(gamma + eps * nu) - cl.momentum(gamma - eps * nu)) / (2 * eps)
            assert np.max(np.abs(fd - A @ nu)) < 1e-7
        # body torque is minus the momentum rate
        nu = np.array([0.3, -0.2, 0.1, 0.4])
        assert np.allclose(cl.torque(gamma, nu), -A @ nu)


def test_dynamics_matches_analytic_formula():
    _, prm = _prm()
    cl = CMGCluster(prm.cmg)
    gamma = np.deg2rad([12.0, -35.0, 24.0, 61.0])
    w = np.array([0.01, -0.02, 0.015])
    q = quat_normalize(euler_to_quat(0.05, 0.1, -0.2))
    x = pack_state(q, w, gamma)
    tau_a = np.array([12.0, -30.0, 4.0])
    nu = np.array([0.2, -0.4, 0.1, 0.3])
    tau_d = np.array([0.3, -0.2, 0.05])
    dx = dynamics_tg(x, np.concatenate([tau_a, nu]), tau_d, prm, cl)
    H = cl.momentum(gamma)
    A = cl.jacobian(gamma)
    wdot = prm.Jinv @ (tau_a - A @ nu + tau_d - np.cross(w, prm.J @ w + H))
    assert np.max(np.abs(dx[4:7] - wdot)) < 1e-12
    assert np.max(np.abs(dx[7:11] - nu)) < 1e-12


def test_p0_matches_18state_truth_for_same_aero_moment():
    cfg, prm = _prm()
    cl = CMGCluster(prm.cmg)
    vp = VehicleCMGParams.from_config(cfg)
    cl6 = CMGCluster(vp.cmg)
    gamma = np.deg2rad([18.0, -22.0, 31.0, -12.0])
    w = np.array([0.004, -0.006, 0.003])
    q = quat_normalize(euler_to_quat(0.02, 0.08, -0.15))
    nu = np.array([0.1, -0.2, 0.15, -0.05])
    tau_d = np.array([0.2, 0.5, 0.3])
    x18 = np.zeros(18)
    x18[0:4], x18[4:7] = q, w
    x18[7:10] = np.deg2rad([4.0, -3.0, 2.0])
    x18[10:14], x18[14:18] = gamma, nu
    p = np.concatenate([[1.2e-6], [3500.0, 0.0, 0.0], tau_d])
    tau_aero = aero_moment6c(x18, p, vp)
    dx11 = dynamics_tg(pack_state(q, w, gamma), np.concatenate([tau_aero, nu]),
                       tau_d, prm, cl)
    dx18 = dynamics6c(x18, np.concatenate([x18[7:10], nu]), p, vp, cl6)
    assert np.max(np.abs(dx11[4:7] - dx18[4:7])) < 1e-10


def test_aero_moment_decomposition_matches_truth():
    from nearspace.model6 import flow_angles
    cfg, prm = _prm()
    vp = VehicleCMGParams.from_config(cfg)
    veh = prm.veh
    gamma = np.deg2rad([18.0, -22.0, 31.0, -12.0])
    w = np.array([0.004, -0.006, 0.003])
    q = quat_normalize(euler_to_quat(0.02, 0.08, -0.15))
    x18 = np.zeros(18)
    x18[0:4], x18[4:7] = q, w
    x18[7:10] = np.deg2rad([4.0, -3.0, 2.0])
    x18[10:14] = gamma
    rho, V = 1.2e-6, 3500.0
    p = np.concatenate([[rho], [V, 0.0, 0.0], np.zeros(3)])
    Q = 0.5 * rho * V ** 2
    alpha, beta, _ = flow_angles(x18, p)
    recon = natural_moment_v(Q, alpha, beta, w, V, veh) + \
        surface_matrix(Q, veh) @ x18[7:10]
    assert np.max(np.abs(recon - aero_moment6c(x18, p, vp))) < 1e-12


def test_modulation_signs():
    lo = np.array([-5.0, -5.0, -5.0])
    hi = np.array([5.0, 5.0, 5.0])
    m = modulation(np.array([-7.0, 0.0, -6.0]), lo, hi)
    assert m["d"][0] < 0.0 and m["d"][2] < 0.0
    assert m["R_minus"][0] < 0.0 and m["R_minus"][2] < 0.0
    m = modulation(np.array([7.0, 0.0, 0.0]), lo, hi)
    assert m["d"][0] > 0.0 and m["R_plus"][0] < 0.0
    m = modulation(np.array([5.0, -5.0, 0.0]), lo, hi)
    assert np.allclose(m["d"], 0.0)
    assert np.allclose(m["R_plus"], [0.0, 10.0, 5.0])


def test_axis_hull_and_allocation():
    _, prm = _prm()
    veh = prm.veh
    Q = 5.0
    M = surface_matrix(Q, veh)
    rng = np.random.default_rng(3)
    samp = rng.uniform(-1.0, 1.0, size=(4000, 3)) * np.asarray(veh.delta_max)
    brute = np.max(np.abs(samp @ M.T), axis=0)
    hull = axis_hull(Q, veh)
    assert np.max(np.abs(brute - hull) / hull) < 2e-2
    # a target inside the hull is reachable to high accuracy
    target = 0.5 * hull
    delta, res = allocate_deflection(target, M, veh)
    assert np.max(np.abs(res)) < 1e-6 * np.max(hull)


def test_capability_residual_zero_for_zero_gap():
    _, prm = _prm()
    cl = CMGCluster(prm.cmg)
    E, v = capability_residual(np.zeros(4), np.zeros(3), cl, 1.5)
    assert E == 0.0 and np.allclose(v, 0.0)


def test_rk4_against_dop853():
    from scipy.integrate import solve_ivp
    _, prm = _prm()
    cl = CMGCluster(prm.cmg)
    x0 = pack_state([1.0, 0.0, 0.0, 0.0], np.zeros(3), np.zeros(4))
    u = np.array([3.0, -14.0, 2.0, 0.3, -0.5, 0.2, 0.4])
    tau_d = np.array([0.2, 0.5, 0.3])
    dt = 0.1
    xn = x0.copy()
    for _ in range(4):
        xn = rk4_step_tg(xn, u, tau_d, dt / 4.0, prm, cl)
    sol = solve_ivp(lambda t, y: dynamics_tg(y, u, tau_d, prm, cl), [0.0, dt], x0,
                    method="DOP853", rtol=1e-13, atol=1e-13)
    xr = sol.y[:, -1]
    assert np.max(np.abs(xn[4:] - xr[4:])) < 1e-10
    assert abs(np.linalg.norm(xn[0:4]) - 1.0) < 1e-12


def test_attitude_error_short_arc():
    qa = quat_normalize(euler_to_quat(0.1, -0.2, 0.3))
    from nearspace.model6 import quat_mul
    qb = quat_normalize(quat_mul(qa, euler_to_quat(1e-3, 0.0, 0.0)))
    e = attitude_error_vector(qa, qb)
    assert abs(np.linalg.norm(e) - 1e-3) < 1e-9
    assert np.allclose(attitude_error_vector(qa, -qb), e)


def test_ocp_converges_on_c1():
    try:
        from nearspace.torque_gap_ocp import TorqueGapOCP
    except Exception as exc:                                # noqa: BLE001
        print("skip: casadi unavailable (%s)" % exc)
        return
    from nearspace.torque_gap_scenario import Scenario
    cfg = load_config(str(CFG_PATH))
    scn = Scenario(cfg)
    ocp = TorqueGapOCP(scn)
    x = scn.initial_state()
    plan = scn.plan(scn.t_start, x)
    res = ocp.solve(plan, x, np.zeros(4), np.zeros(3))
    assert res.ok, res.status
    assert res.max_violation < 1e-5
    assert np.all(np.abs(res.U[0:3, 0]) <= np.abs(plan["box"]["tau_max"][0]) + 1e-6)


if __name__ == "__main__":
    fails = 0
    for name, fn in sorted(globals().items()):
        if name.startswith("test_") and callable(fn):
            try:
                fn()
                print("PASS", name)
            except AssertionError as exc:
                fails += 1
                print("FAIL", name, exc)
    print("%d test(s) failed" % fails)
    sys.exit(1 if fails else 0)
