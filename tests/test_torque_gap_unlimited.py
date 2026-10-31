"""Regression tests for the continuous-rotation (unlimited) gimbal model.

Pytest-compatible and directly runnable (pytest is not installed on the
experiment workstation):

    python tests/test_torque_gap_unlimited.py

The exhaustive numeric report is scripts/check_torque_gap_unlimited.py.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from nearspace.cmg import CMGCluster  # noqa: E402
from nearspace.config import load_config  # noqa: E402
from nearspace.torque_gap_model import (pack_state, rk4_step_tg)  # noqa: E402
from nearspace.torque_gap_scenario import Scenario  # noqa: E402

FREE = ROOT / "configs" / "torque_gap_unlimited" / "cases" / "c1_pitch30.yaml"
BOUNDED = ROOT / "configs" / "torque_gap" / "cases" / "c1_pitch30.yaml"


def _scn(path=FREE):
    return Scenario(load_config(str(path)))


def test_gamma_limit_modes():
    free, lim = _scn(FREE), _scn(BOUNDED)
    assert free.gamma_unbounded and free.gamma_limit_mode == "none"
    assert np.all(np.isneginf(free.gamma_lim[0])) and np.all(np.isposinf(free.gamma_lim[1]))
    assert not lim.gamma_unbounded and lim.gamma_limit_mode == "bounded"
    assert abs(lim.gamma_lim[0] + np.pi / 2) < 1e-12
    assert abs(lim.gamma_lim[1] - np.pi / 2) < 1e-12


def test_geometry_is_periodic():
    cl = CMGCluster(_scn().prm.cmg)
    rng = np.random.default_rng(0)
    for _ in range(5):
        g = rng.uniform(-20.0, 20.0, 4)                # up to ~3 turns either way
        # H and A are periodic, but they are *computed* from unwrapped angles
        np.testing.assert_allclose(cl.momentum(g + 2 * np.pi), cl.momentum(g), atol=1e-12)
        np.testing.assert_allclose(cl.jacobian(g + 2 * np.pi), cl.jacobian(g), atol=1e-12)
        nu = rng.uniform(-1.5, 1.5, 4)
        np.testing.assert_allclose(cl.torque(g + 2 * np.pi, nu), cl.torque(g, nu),
                                   atol=1e-12)


def test_crossing_360_is_continuous():
    scn = _scn()
    cl = scn.cluster
    q = np.array([1.0, 0.0, 0.0, 0.0])
    nu = np.array([1.5, 0.0, 0.0, 0.0])
    x = pack_state(q, np.zeros(3), np.array([np.deg2rad(350.0), 0.0, 0.0, 0.0]))
    dt = 1e-3
    gam = []
    for _ in range(int(round(np.deg2rad(20.0) / (nu[0] * dt)))):
        gam.append(x[7])
        x = rk4_step_tg(x, np.concatenate([np.zeros(3), nu]), np.zeros(3), dt,
                        scn.prm, cl)
    gam = np.array(gam)
    assert np.all(np.diff(gam) > 0.0)                     # monotone through 360 deg
    np.testing.assert_allclose(np.diff(gam), nu[0] * dt, rtol=1e-9, atol=1e-12)
    assert gam[0] < np.deg2rad(360.0) < gam[-1]


def test_two_full_turns_unwrapped():
    scn = _scn()
    cl = scn.cluster
    gdot = float(scn.prm.cmg.gdot_max)
    nu = np.array([0.0, gdot, 0.0, 0.0])
    dt = 5e-3
    n = int(round(4.0 * np.pi / (gdot * dt))) + 1
    x = pack_state([1.0, 0, 0, 0], np.zeros(3), np.zeros(4))
    for _ in range(n):
        x = rk4_step_tg(x, np.concatenate([np.zeros(3), nu]), np.zeros(3), dt,
                        scn.prm, cl)
    assert x[8] >= 4.0 * np.pi - 1e-6                      # two turns, not modulo 2pi
    np.testing.assert_allclose(x[8], gdot * dt * n, rtol=1e-9, atol=1e-9)


def test_no_clamp_in_either_plant():
    from nearspace.torque_gap_sim import ClosedLoop
    scn = _scn()
    cl = ClosedLoop(scn, plant="P0")
    q = np.array([1.0, 0.0, 0.0, 0.0])
    g0 = np.deg2rad([100.0, -120.0, 150.0, 210.0])
    nu = np.ones(4)
    x1, _, _ = cl._step_P0(pack_state(q, np.zeros(3), g0), np.zeros(3), nu, scn.t_start)
    assert np.all(x1[7:11] > g0)
    x18 = np.zeros(18)
    x18[0:4] = q
    x18[10:14] = g0
    x18n, _, _, _, _ = cl._step_P1(x18, np.zeros(3), nu, scn.t_start)
    assert np.all(x18n[10:14] > g0)


def test_bounded_config_still_clamps():
    from nearspace.torque_gap_sim import ClosedLoop
    scn = _scn(BOUNDED)
    cl = ClosedLoop(scn, plant="P0")
    q = np.array([1.0, 0.0, 0.0, 0.0])
    g0 = np.deg2rad([85.0, 0.0, 0.0, 0.0])
    x1, _, _ = cl._step_P0(pack_state(q, np.zeros(3), g0), np.zeros(3), np.ones(4),
                           scn.t_start)
    assert x1[7] <= np.pi / 2 + 1e-12                      # the old arm is unchanged


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
