"""Verification of the CMG A/B experiment machinery.

Run from the workspace root (no pytest required)::

    python tests/test_cmg_ab.py

Checks
------
1. Quintic manoeuvre reference: endpoints, peak rate / peak acceleration.
2. CMG cluster capability figures (guaranteed torque 86.8 N·m, single-axis
   usable momentum 107.7 N·m·s) reproduce the values quoted in the docs.
3. Mode A really has no CMG: gimbals frozen, zero CMG torque, zero momentum.
4. A/B fairness: the two resolved configurations differ ONLY in the actuator
   block, and both start from the same state.
5. Metric definitions: settling time, saturation fraction, longest failure run
   on synthetic data with a known answer.
6. Config overlay chain: `base:` + `extends:` resolution.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
for p in (ROOT, ROOT / "scripts"):
    if str(p) not in sys.path:
        sys.path.insert(0, str(p))

from nearspace.cmg import CMGCluster                                    # noqa: E402
from nearspace.config import deep_merge, dump, read_yaml                # noqa: E402
from nearspace.metrics import compute                                   # noqa: E402
from nearspace.model6c import VehicleCMGParams, make_cluster, plant_step6c   # noqa: E402
from nearspace.scenarios import (_axis_momentum, _guaranteed_torque,    # noqa: E402
                                 maneuver_demand)
from nearspace.trajectory3 import Maneuver                              # noqa: E402
from run_cmg_ab import build_cfg, resolve_case, load_matrix             # noqa: E402

_FAILURES = []


def check(cond, msg):
    if cond:
        print("      ok   %s" % msg)
    else:
        print("      FAIL %s" % msg)
        _FAILURES.append(msg)


def banner(text):
    print("\n" + "=" * 78)
    print(text)
    print("=" * 78)


# ---------------------------------------------------------------------------
def test_quintic():
    banner("1. quintic manoeuvre reference")
    mv = Maneuver("pitch", t0=10.0, duration=4.0, delta_deg=10.0)
    d = np.deg2rad(10.0)
    check(abs(mv.value(9.0)) < 1e-15, "value is 0 before t0")
    check(abs(mv.value(10.0)) < 1e-15, "value is 0 at t0 (C1/C2 start)")
    check(abs(mv.value(14.0) - d) < 1e-12, "value equals delta at t0+Tm")
    check(abs(mv.value(30.0) - d) < 1e-12, "value held after the ramp")
    v = mv.value(12.0)
    check(abs(v - 0.5 * d) < 1e-12, "value is delta/2 at mid ramp")
    # numeric peak rate / acceleration match the closed-form constants
    ts = np.linspace(10.0, 14.0, 400001)
    ys = np.array([mv.value(x) for x in ts])
    rate = np.max(np.abs(np.diff(ys) / np.diff(ts)))
    check(abs(rate / mv.peak_rate - 1.0) < 2e-3,
          "peak rate %.4f rad/s matches 1.875*delta/Tm" % rate)
    dem = maneuver_demand(VehicleCMGParams.from_config(build_cfg("maneuver10", "B")),
                          10.0, 4.0)
    check(abs(dem["peak_rate_rad_s"] / mv.peak_rate - 1.0) < 1e-9,
          "maneuver_demand uses the same quintic constants")


def test_cluster_spec():
    banner("2. CMG cluster capability (docs quote 86.7 N·m and 107.7 N·m·s)")
    cfg = build_cfg("tracking", "B")
    prm = VehicleCMGParams.from_config(cfg)
    cl = make_cluster(prm)
    g = _guaranteed_torque(cl)
    lo, hi = _axis_momentum(cl, 1)
    print("      guaranteed torque = %.3f N·m, axis momentum = [%.2f, %.2f]"
          % (g, lo, hi))
    check(abs(g - 86.77) < 0.15, "guaranteed torque 86.77 N·m")
    check(abs(min(hi, -lo) - 107.735) < 0.05, "single-axis usable momentum 107.7 N·m·s")
    check(abs(cl.sigma_min(cl.jacobian(np.zeros(4))) - 40.8248) < 1e-3,
          "home sigma_min = 40.82 N·m·s")
    check(cl.prm.h_total == 200.0, "cluster momentum = 200 N·m·s")
    # pyramid: every pair of gimbal axes has |g_i . g_j| = 1/3
    G = cl.g_axis
    dots = G.T @ G
    m = ~np.eye(4, dtype=bool)
    check(np.allclose(np.abs(dots[m]), 1.0 / 3.0),
          "gimbal axes satisfy |g_i.g_j| = 1/3 for every pair")


def test_mode_a_has_no_cmg():
    banner("3. mode A really removes the CMG from the truth model")
    prmA = VehicleCMGParams.from_config(build_cfg("tracking", "A"), cmg_on=False)
    prmB = VehicleCMGParams.from_config(build_cfg("tracking", "B"), cmg_on=True)
    check(prmA.cmg_on is False and prmB.cmg_on is True, "cmg_on flag follows the mode")
    check(abs(float(build_cfg("tracking", "A").mpc_cmg.M_cmg_box)) < 1e-12,
          "mode A config carries a zero CMG torque box")
    check(float(build_cfg("tracking", "B").mpc_cmg.M_cmg_box) > 0.0,
          "mode B config carries a non-zero CMG torque box")
    cl = make_cluster(prmA)
    x = np.zeros(18)
    x[0] = 1.0
    p = np.array([5e-7, 3500.0, 0.0, -300.0, 0.5, 2.0, 0.3])
    u = np.concatenate([np.zeros(3), np.full(4, 1.5)])   # full gimbal rate demand
    for _ in range(200):
        x, uu = plant_step6c(x, u, p, 0.005, prmA, cl, substeps=1)
    check(np.allclose(x[10:14], 0.0) and np.allclose(x[14:18], 0.0),
          "gimbals stay frozen at home in mode A")
    check(np.allclose(uu[3:7], 0.0), "gimbal rate command is zeroed in mode A")
    check(float(np.linalg.norm(cl.momentum(x[10:14]))) < 1e-9,
          "CMG momentum is zero in mode A")
    # ... and mode B does move them
    clB = make_cluster(prmB)
    xb = x.copy()
    for _ in range(200):
        xb, _ = plant_step6c(xb, u, p, 0.005, prmB, clB, substeps=1)
    check(np.max(np.abs(xb[10:14])) > 0.1, "gimbals do move in mode B (sanity)")


def test_fairness():
    banner("4. A/B fairness: the two configs differ only in the actuator block")
    A = dump(build_cfg("disturbance", "A"))
    B = dump(build_cfg("disturbance", "B"))
    diffs = []

    def walk(a, b, path=""):
        keys = set(a) | set(b)
        for k in sorted(keys):
            pa = a.get(k, "<missing>")
            pb = b.get(k, "<missing>")
            if isinstance(pa, dict) and isinstance(pb, dict):
                walk(pa, pb, path + "/" + k)
            elif pa != pb:
                diffs.append("%s/%s: %r -> %r" % (path, k, pa, pb))

    walk(A, B)
    print("      differing keys: %s" % diffs)
    # display-only labels are not part of the physics; everything else must match
    allowed = {"/actuators/mode", "/mpc_cmg/M_cmg_box", "/label",
               "/scenario/mode_label"}
    extra = [d for d in diffs if d.split(":")[0] not in allowed]
    check(not extra, "no unexpected A/B configuration difference")
    check(A["trajectory3"] == B["trajectory3"], "identical reference trajectory block")
    check(A["vehicle3"] == B["vehicle3"], "identical vehicle / aero parameters")
    check(A["environment"] == B["environment"], "identical atmosphere backend")
    # identical initial state (the trim uses the aero model only)
    from nearspace import sim
    xa = sim._initial_state(sim.build(build_cfg("disturbance", "A"))["traj"],
                            VehicleCMGParams.from_config(build_cfg("disturbance", "A")))
    xb = sim._initial_state(sim.build(build_cfg("disturbance", "B"))["traj"],
                            VehicleCMGParams.from_config(build_cfg("disturbance", "B")))
    check(np.allclose(xa, xb), "identical initial state in A and B")


def test_metrics():
    banner("5. metric definitions on synthetic data")
    dt = 0.2
    t = np.arange(0.0, 40.0 + 1e-9, dt)
    en = np.where(t < 10.0, 5.0 * np.exp(-(t - 10.0) ** 2 / 1.0) / 1.0,
                  5.0 * np.exp(-(t - 10.0) / 2.0))
    en = np.abs(en)
    rec = dict(
        t=t, err_norm_deg=en,
        err_roll_deg=np.zeros_like(t), err_pitch_deg=en, err_yaw_deg=np.zeros_like(t),
        phi=np.zeros_like(t), theta=np.zeros_like(t), psi=np.zeros_like(t),
        phi_ref=np.zeros_like(t), theta_ref=np.zeros_like(t), psi_ref=np.zeros_like(t),
        delta_deg=np.outer(np.ones_like(t), [30.0, 0.0, 0.0]),
        M_aero=np.zeros((len(t), 3)), M_cmg=np.zeros((len(t), 3)),
        H_norm=np.zeros_like(t), gdot=np.zeros((len(t), 4)),
        sigma=np.ones_like(t) * 40.0, qp_fail=np.zeros_like(t),
        fallback=np.zeros_like(t), viol=np.zeros_like(t),
        solve_time_ms=np.ones_like(t) * 5.0, maneuver_delta_deg=0.0)
    m = compute(rec, {"dist": (10.0, 11.0)}, delta_max_deg=np.array([25.0, 20.0, 20.0]))
    check(abs(m["err_rmse_deg"] - float(np.sqrt(np.mean(en ** 2)))) < 1e-12,
          "RMSE is the plain RMS of the error record")
    check(m["peak_err_dist_deg"] > 4.0, "disturbance window peak picked up")
    check(m["delta_sat_frac_all"] == 1.0, "deflection 30 deg > 25 deg counts as saturated")
    check(m["recovery_time_dist_s"] is not None and m["recovery_time_dist_s"] > 0,
          "recovery time measured after the window")
    rec["qp_fail"] = np.array([0, 1, 1, 1, 0, 0, 1, 0] + [0] * (len(t) - 8), dtype=float)
    m2 = compute(rec, {"dist": (10.0, 11.0)}, delta_max_deg=np.array([25.0, 20.0, 20.0]))
    check(m2["qp_fail_count"] == 4, "QP failure count")
    check(m2["qp_fail_longest_run"] == 3, "longest consecutive QP failure run")
    # manoeuvre: never enters the band -> must be None, not a number
    rec["err_norm_deg"] = np.full_like(t, 8.0)
    m3 = compute(rec, {"man": (12.0, 32.0)}, delta_max_deg=np.array([25.0, 20.0, 20.0]))
    check(m3["maneuver_time_s"] is None and m3["maneuver_completed"] is False,
          "incomplete manoeuvre reported as NOT COMPLETED")


def test_config_chain():
    banner("6. configuration overlay chain")
    base, blocks = load_matrix()
    check("trajectory3" in base and "cmg" in base, "matrix inherits config.yaml via base:")
    m10 = resolve_case(blocks, "maneuver10")
    m20 = resolve_case(blocks, "maneuver20")
    check(m10["scenario"]["name"] == "maneuver10" and m20["scenario"]["name"] == "maneuver20",
          "case blocks resolve to their own scenario")
    ext = resolve_case(blocks, "wscan_0p01")
    check(ext["trajectory3"]["dist_pulse_amp"] == [0.0, 80.0, 0.0],
          "extends: inherits the parent case fields (disturbance pulse)")
    check(ext["mpc_cmg"]["r_cmg_transient"] == 0.01,
          "extends: child overrides only what it declares")
    ext = resolve_case(blocks, "h115_maneuver10")
    check(ext["scenario"]["vertex_km"] == 115.0, "extends: child overrides the vertex")
    cfg = build_cfg("h90_maneuver10", "A")
    check(abs(cfg.trajectory.amp - 60000.0) < 1e-9,
          "vertex altitude 90 km -> skip amplitude 60 km")
    check(abs(cfg.simulation.T - 170.0) < 1e-9, "case T overrides the base T")
    merged = deep_merge({"a": {"b": 1, "c": 2}}, {"a": {"c": 3}})
    check(merged == {"a": {"b": 1, "c": 3}}, "deep_merge merges nested dicts")
    raw = read_yaml(str(ROOT / "configs" / "cmg_ab" / "matrix.yaml"))
    check("base" not in raw, "base: key is consumed by the loader")


def main():
    banner("CMG A/B experiment machinery verification")
    test_quintic()
    test_cluster_spec()
    test_mode_a_has_no_cmg()
    test_fairness()
    test_metrics()
    test_config_chain()
    banner("SUMMARY")
    if _FAILURES:
        print("      %d check(s) FAILED:" % len(_FAILURES))
        for msg in _FAILURES:
            print("        - %s" % msg)
        print("\nRESULT: FAILED")
        return 1
    print("      all checks passed")
    print("\nRESULT: OK")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
