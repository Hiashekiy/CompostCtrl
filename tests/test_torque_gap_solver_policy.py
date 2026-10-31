"""Tests for the closed-loop solver acceptance policy (Tier-1 speed work).

  1  `accept_feasible: true` uses a non-converged but feasible solution instead of
     throwing the control step away, and marks the log row as `accepted`;
  2  `accept_feasible: false` restores the historical behaviour (fallback);
  3  the shifting warm start survives a fallback (`warm_shift_on_fallback`);
  4  SolverConfig reads the new keys and keeps the old defaults.

python tests/test_torque_gap_solver_policy.py
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from nearspace.config import _to_ns, deep_merge, read_yaml       # noqa: E402
from nearspace.torque_gap_ocp import SolverConfig                # noqa: E402
from nearspace.torque_gap_scenario import Scenario               # noqa: E402
from nearspace.torque_gap_sim import ClosedLoop                  # noqa: E402

CASE_B = ROOT / "configs/torque_gap_no_cmg_comparison/cases/S00_pitch30_with_cmg.yaml"
WINDOW = {"torque_gap": {"start": 108.0, "end": 109.0}}


def _cl(solver_over):
    raw = deep_merge(read_yaml(str(CASE_B)), WINDOW)
    raw = deep_merge(raw, {"torque_gap": {"solver": solver_over}})
    scn = Scenario(_to_ns(raw))
    cl = ClosedLoop(scn, plant="P0")
    metrics = cl.run()
    return cl, metrics, scn


# a cap low enough to stop IPOPT at the iteration limit on some steps, but high
# enough that the returned point is still feasible
CAP = {"max_iter": 40, "tol": 1e-8}


def test_accept_feasible_uses_non_converged_solutions():
    cl, m, scn = _cl(dict(CAP, accept_feasible=True))
    assert m["accepted_steps"] > 0, m["status_counts"]
    assert any(r["status"].endswith("|accepted") for r in cl.rows)
    for r in cl.rows:
        assert r["accepted"] in (0, 1)
        assert np.isfinite(r["cpu_s"])
        # the contract: only feasible solutions may be accepted
        if r["accepted"] == 1:
            assert r["viol"] <= 1e-4, r["viol"]
            assert not r["status"].split("|")[0] in (
                "Solve_Succeeded", "Solved_To_Acceptable_Level")
    # the applied control is still inside the aerodynamic box
    for r in cl.rows:
        assert r["lo_x"] - 1e-6 <= r["tauc_cmd_x"] <= r["hi_x"] + 1e-6


def test_reject_non_converged_restores_the_fallback():
    cl, m, scn = _cl(dict(CAP, accept_feasible=False))
    assert m["accepted_steps"] == 0
    assert m["fallback_steps"] > 0, m["status_counts"]
    assert any("previous" in s or "stabiliser" in s for s in m["status_counts"])


def test_warm_start_survives_a_fallback():
    on = SolverConfig.from_config(_to_ns(deep_merge(
        deep_merge(read_yaml(str(CASE_B)), WINDOW),
        {"torque_gap": {"solver": {"max_iter": 2, "accept_feasible": False,
                                   "warm_shift_on_fallback": True}}})))
    off = SolverConfig.from_config(_to_ns(deep_merge(
        deep_merge(read_yaml(str(CASE_B)), WINDOW),
        {"torque_gap": {"solver": {"max_iter": 2, "accept_feasible": False,
                                   "warm_shift_on_fallback": False}}})))
    assert on.warm_shift_on_fallback is True
    assert off.warm_shift_on_fallback is False
    # the shift helper advances the stored solution by one node
    X = np.arange(11 * 4, dtype=float).reshape(11, 4)
    U = np.arange(7 * 3, dtype=float).reshape(7, 3)
    Xn, Un = ClosedLoop._shift_warm((X, U))
    assert np.allclose(Xn[:, :-1], X[:, 1:]) and np.allclose(Xn[:, -1], X[:, -1])
    assert np.allclose(Un[:, :-1], U[:, 1:]) and np.allclose(Un[:, -1], U[:, -1])


def test_solver_config_defaults_and_override():
    base = SolverConfig.from_config(_to_ns(read_yaml(str(CASE_B))))
    assert base.accept_feasible is True
    assert base.warm_shift_on_fallback is True
    assert base.quat_norm in ("all", "n0", "none")
    assert base.accept_feasible_viol == 1e-4
    cfg = _to_ns(deep_merge(read_yaml(str(CASE_B)), {"torque_gap": {"solver": {
        "tol": 1e-3, "max_iter": 77, "accept_feasible": False,
        "accept_feasible_viol": 5e-5, "quat_norm": "n0"}}}))
    s = SolverConfig.from_config(cfg)
    assert s.tol == 1e-3 and s.max_iter == 77
    assert s.accept_feasible is False and s.accept_feasible_viol == 5e-5
    assert s.quat_norm == "n0"


def test_quat_norm_changes_the_constraint_count():
    from nearspace.torque_gap_ocp import TorqueGapOCP
    counts = {}
    for mode in ("all", "n0", "none"):
        scn = Scenario(_to_ns(deep_merge(read_yaml(str(CASE_B)),
                                        WINDOW | {"torque_gap": {"solver": {
                                            "quat_norm": mode}}})))
        counts[mode] = TorqueGapOCP(scn).n_cons
    assert counts["all"] > counts["n0"] > counts["none"]
    N = 22
    assert counts["all"] - counts["n0"] == N          # N redundant equalities


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
