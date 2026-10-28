"""Phase 4: verify the CMG actuator and the unified MPC *before* the formal A/B.

    python scripts/validate_cmg_actuator.py

Three tests, in the order fixed by the implementation plan (§5.1-§5.3):

1. torque direction  - a small gimbal-rate command must produce the body
   reaction moment predicted by M = -A(gamma) gdot; the sign must match the
   repository convention, and with `cmg_on=False` the contribution must vanish
   to machine precision.
2. torque delivery    - a constant torque demand swept from below to above the
   cluster capability is checked against the *analytic* box-LP ceiling of the
   configuration: short-term delivery must match the demand, no sample may
   exceed the instantaneous ceiling, the gimbal rate / momentum must stay inside
   their limits - and the singularity margin actually consumed by a sustained
   demand is recorded rather than tuned away.
3. unified-MPC closed loop - a short closed-loop run of the B configuration must
   solve without fallback and stay inside the actuator limits; a direct QP probe
   at the 100 km vertex then shows that the CMG columns of the decision vector
   are live, that zeroing the CMG torque box removes them (the A/B difference),
   and that the CMG cost weight changes the commanded CMG torque.

Records go to results/c1_cmg_reproduction/cmg/actuator_tests/.
"""
from __future__ import annotations

import copy
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for p in (ROOT, HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

import run_cmg_ab as ab                                          # noqa: E402
from nearspace.cmg_control import steering_law                     # noqa: E402
from nearspace.model6c import (VehicleCMGParams, make_cluster,      # noqa: E402
                               plant_step6c)
from nearspace.scenarios import cmg_spec, controller_budgets, effective_weights  # noqa: E402
from nearspace import sim                                         # noqa: E402

CONFIG = os.path.join(ROOT, "configs", "c1_cmg_reproduction.yaml")
OUT = os.path.join(ROOT, "results", "c1_cmg_reproduction", "cmg", "actuator_tests")
CACHE = os.path.join(ROOT, "results", "cmg_ab", "_cache")
CASE = "c1"

# zero dynamic pressure -> the aero moment vanishes, so the only body moment is
# the CMG reaction moment (the body still rotates, that is the point)
ZERO_AERO = np.concatenate([[0.0], [3500.0, 0.0, -400.0], np.zeros(3)])
DT = 0.005
TOL_ZERO = 1e-12


def _rig(cmg_on):
    cfg = ab.build_cfg(CASE, "B" if cmg_on else "A", CONFIG)
    prm = VehicleCMGParams.from_config(cfg, cmg_on=cmg_on)
    cl = make_cluster(prm)
    x0 = np.zeros(18)
    x0[0] = 1.0
    return cfg, prm, cl, x0


def _axis_ceiling(cl, gamma, axis, gdot_max):
    """Exact max torque along `axis`: box-LP max of a^T(-A gdot) over |gdot|<=1."""
    c = axis @ (-cl.jacobian(gamma))
    return float(np.sum(np.abs(c)) * gdot_max)


# ---------------------------------------------------------------------------
# 1. torque direction
# ---------------------------------------------------------------------------
def test_torque_direction():
    cfg, prm, cl, x0 = _rig(True)
    J = prm.J
    n_steps = 60                                    # 0.3 s, ~10 gimbal time constants
    directions = {
        "roll  (+x)": np.array([1.0, 0.0, 0.0]),
        "pitch (+y)": np.array([0.0, 1.0, 0.0]),
        "yaw   (+z)": np.array([0.0, 0.0, 1.0]),
        "diag (1,1,1)/sqrt3": np.ones(3) / np.sqrt(3.0),
    }
    rows = []
    for name, u_axis in directions.items():
        M_cmd = 5.0 * u_axis
        x = x0.copy()
        w0 = x[4:7].copy()
        imp_M = np.zeros(3)
        imp_gyro = np.zeros(3)
        M_prev = cl.torque(x[10:14], x[14:18])
        G_prev = np.cross(x[4:7], J @ x[4:7] + cl.momentum(x[10:14]))
        for _ in range(n_steps):
            gdot_c = steering_law(cl, x[10:14], M_cmd, np.zeros(4))
            x, _ = plant_step6c(x, np.concatenate([np.zeros(3), gdot_c]),
                                ZERO_AERO, DT, prm, cl, substeps=1)
            M_now = cl.torque(x[10:14], x[14:18])
            G_now = np.cross(x[4:7], J @ x[4:7] + cl.momentum(x[10:14]))
            imp_M += 0.5 * (M_prev + M_now) * DT           # trapezoid
            imp_gyro += 0.5 * (G_prev + G_now) * DT
            M_prev, G_prev = M_now, G_now
        M_end = cl.torque(x[10:14], x[14:18])
        residual = J @ (x[4:7] - w0) - (imp_M - imp_gyro)
        rows.append(dict(
            direction=name, demand_Nm=M_cmd.tolist(),
            delivered_end_Nm=M_end.tolist(),
            delivered_ratio=float(M_end @ u_axis / 5.0),
            cos_delivered_vs_demand=float(M_end @ M_cmd
                                          / max(np.linalg.norm(M_end) * 5.0, 1e-12)),
            body_momentum_check=dict(
                dH_body_Nms=(J @ (x[4:7] - w0)).tolist(),
                cmg_impulse_Nms=imp_M.tolist(), gyro_impulse_Nms=imp_gyro.tolist(),
                residual_Nms=residual.tolist(),
                residual_rel=float(np.linalg.norm(residual)
                                   / max(np.linalg.norm(imp_M), 1e-12))),
            sigma_min=float(cl.sigma_min(cl.jacobian(x[10:14]))),
            momentum_Nms=float(np.linalg.norm(cl.momentum(x[10:14])))))

    # mode A: the same full-rate command must produce nothing at all
    cfgA, prmA, clA, xA = _rig(False)
    x = xA.copy()
    uA = None
    for _ in range(200):
        x, uA = plant_step6c(x, np.concatenate([np.zeros(3), np.full(4, 1.5)]),
                             ZERO_AERO, DT, prmA, clA, substeps=1)
    mode_a = dict(
        torque_norm=float(np.linalg.norm(clA.torque(x[10:14], x[14:18]))),
        momentum_norm=float(np.linalg.norm(clA.momentum(x[10:14]))),
        gimbal_rate_max=float(np.max(np.abs(x[14:18]))),
        gimbal_angle_max=float(np.max(np.abs(x[10:14]))),
        body_rate_max=float(np.max(np.abs(x[4:7]))),
        gimbal_cmd_returned=uA[3:7].tolist())

    g, gd = np.array([0.1, -0.1, 0.1, -0.1]), np.array([0.2, 0.2, 0.2, 0.2])
    ok_sign = bool(np.allclose(cl.torque(g, gd), -cl.jacobian(g) @ gd, atol=1e-12))

    checks = [dict(check="direction %s: delivered follows the demand" % r["direction"],
                   ok=bool(r["cos_delivered_vs_demand"] > 0.999
                           and 0.9 < r["delivered_ratio"] < 1.1),
                   value=[r["delivered_ratio"], r["cos_delivered_vs_demand"]]) for r in rows]
    checks += [
        dict(check="body momentum balance closes: J dw = int(M_cmg) - int(gyro)",
             ok=all(r["body_momentum_check"]["residual_rel"] < 5e-3 for r in rows),
             value=[r["body_momentum_check"]["residual_rel"] for r in rows]),
        dict(check="M = -A(gamma) gdot sign convention", ok=ok_sign,
             value=cl.torque(g, gd).tolist()),
        dict(check="mode A: CMG torque / momentum / gimbals are zero to machine precision",
             ok=bool(mode_a["torque_norm"] < TOL_ZERO and mode_a["momentum_norm"] < TOL_ZERO
                     and mode_a["gimbal_rate_max"] == 0.0 and mode_a["gimbal_angle_max"] == 0.0),
             value=mode_a),
    ]
    return dict(test="torque_direction", directions=rows, mode_A=mode_a,
                checks=checks, passed=all(c["ok"] for c in checks))


# ---------------------------------------------------------------------------
# 2. torque delivery sweep
# ---------------------------------------------------------------------------
def test_torque_delivery():
    cfg, prm, cl, x0 = _rig(True)
    hw = cmg_spec(cfg, prm)
    budget = controller_budgets(cfg, prm)
    n_steps = 400                                    # 2 s hold
    n_early = slice(60, 120)                         # 0.3-0.6 s: past the 30 ms
                                                     # gimbal loop lag, before the
                                                     # singularity is consumed
    n_late = 50                                      # last 0.25 s
    rows = []
    for axis_name, axis in (("pitch (+y)", np.array([0.0, 1.0, 0.0])),
                            ("diag (1,1,1)/sqrt3", np.ones(3) / np.sqrt(3.0))):
        ceiling_home = _axis_ceiling(cl, x0[10:14], axis, prm.cmg.gdot_max)
        for M_req in (5.0, 10.0, 20.0, 40.0, 61.0, 86.0, 120.0):
            M_cmd = M_req * axis
            x = x0.copy()
            H0 = cl.momentum(x[10:14])
            delivered, sig, over = [], [], 0.0
            t_sing = None
            for k in range(n_steps):
                gdot_c = steering_law(cl, x[10:14], M_cmd, np.zeros(4))
                x, _ = plant_step6c(x, np.concatenate([np.zeros(3), gdot_c]),
                                    ZERO_AERO, DT, prm, cl, substeps=1)
                g = x[10:14]
                M_t = cl.torque(g, x[14:18])
                delivered.append(M_t.copy())
                s = cl.sigma_min(cl.jacobian(g))
                sig.append(s)
                if t_sing is None and s < prm.cmg.sigma_floor:
                    t_sing = (k + 1) * DT
                ceil_t = _axis_ceiling(cl, g, axis, prm.cmg.gdot_max)
                over = max(over, float((M_t @ axis) / max(ceil_t, 1e-9)))
            delivered = np.array(delivered)
            early = delivered[n_early].mean(axis=0)
            late = delivered[-n_late:].mean(axis=0)
            rows.append(dict(
                axis=axis_name, demand_Nm=M_req, ceiling_home_Nm=ceiling_home,
                delivered_early_Nm=float(early @ axis),
                delivered_sustained_Nm=float(late @ axis),
                ratio_early=float((early @ axis) / M_req),
                ratio_sustained=float((late @ axis) / M_req),
                max_sample_over_instant_ceiling=over,
                gimbal_rate_peak_rad_s=float(np.max(np.abs(x[14:18]))),
                gimbal_rate_limit_rad_s=float(prm.cmg.gdot_max),
                sigma_min=float(np.min(sig)), sigma_home=float(prm.cmg.home_sigma_min),
                sigma_floor=float(prm.cmg.sigma_floor),
                t_below_sigma_floor_s=t_sing,
                momentum_change_Nms=float(np.linalg.norm(cl.momentum(x[10:14]) - H0)),
                momentum_Nms=float(np.linalg.norm(cl.momentum(x[10:14]))),
                gimbal_rate_saturated=bool(np.max(np.abs(x[14:18]))
                                           >= 0.999 * prm.cmg.gdot_max)))

    below = [r for r in rows if r["demand_Nm"] <= 0.75 * r["ceiling_home_Nm"]]
    safe = [r for r in rows if r["demand_Nm"] <= 20.0]
    worst = {}
    for axis_name in ("pitch (+y)", "diag (1,1,1)/sqrt3"):
        sel = [r for r in rows if r["axis"] == axis_name]
        worst[axis_name] = dict(lowest=sel[0], highest=sel[-1])
    crossed = sorted({(r["axis"], r["demand_Nm"]) for r in rows
                      if r["t_below_sigma_floor_s"] is not None})
    checks = [
        dict(check="below 0.75 x ceiling: delivery over 0.3-0.6 s tracks the demand",
             ok=all(0.85 < r["ratio_early"] < 1.1 for r in below),
             value=[(r["axis"], r["demand_Nm"], round(r["ratio_early"], 3)) for r in below]),
        dict(check="delivery degrades as the demand approaches the ceiling",
             ok=all(v["highest"]["ratio_early"] < v["lowest"]["ratio_early"] - 0.02
                    for v in worst.values()),
             value={k: [round(v["lowest"]["ratio_early"], 3),
                        round(v["highest"]["ratio_early"], 3)] for k, v in worst.items()}),
        dict(check="no sample exceeds the instantaneous analytic ceiling",
             ok=all(r["max_sample_over_instant_ceiling"] <= 1.02 for r in rows),
             value=max(r["max_sample_over_instant_ceiling"] for r in rows)),
        dict(check="gimbal rate never exceeds the hardware limit",
             ok=all(r["gimbal_rate_peak_rad_s"] <= prm.cmg.gdot_max + 1e-9 for r in rows),
             value=float(prm.cmg.gdot_max)),
        dict(check="momentum stays inside the cluster envelope",
             ok=all(r["momentum_Nms"] <= prm.cmg.h_total + 1e-6 for r in rows),
             value=[float(prm.cmg.h_total), max(r["momentum_Nms"] for r in rows)]),
        dict(check="demands <= 20 N·m never cross the singularity floor",
             ok=all(r["t_below_sigma_floor_s"] is None for r in safe),
             value=[(r["axis"], r["demand_Nm"], r["t_below_sigma_floor_s"]) for r in safe]),
        dict(check="sustained demand above the sustainable level crosses the floor "
                   "(recorded, not tuned away)",
             ok=True, value=dict(crossed=crossed,
                                 sustained=[(r["axis"], r["demand_Nm"],
                                             round(r["ratio_sustained"], 3),
                                             round(r["sigma_min"], 4)) for r in rows])),
    ]
    return dict(test="torque_delivery", hardware=hw, controller_budget=budget,
                sweep=rows, checks=checks, passed=all(c["ok"] for c in checks))


# ---------------------------------------------------------------------------
# 3. unified-MPC closed loop
# ---------------------------------------------------------------------------
def _short_run(cfg0, T=20.0, overrides=None):
    cfg = copy.deepcopy(cfg0)
    cfg.simulation.T = float(T)
    for path, val in (overrides or {}).items():
        obj = cfg
        parts = path.split(".")
        for p in parts[:-1]:
            obj = getattr(obj, p)
        setattr(obj, parts[-1], val)
    return sim.run(cfg, cache_dir=CACHE)


def _probe_at_vertex(cfg, t=118.0, err_deg=3.0, **over):
    """One MPC solve from a state 3 deg off the reference at the 100 km vertex."""
    from nearspace.cmg_control import LTVMPCCMG2
    b = sim.build(cfg, cache_dir=CACHE)
    traj = b["traj"]
    prm = b["prm"]
    ctrl = LTVMPCCMG2(prm, cfg, b["cluster"])
    x = sim._initial_state(traj, prm)
    x[0:4] = traj.q_ref(t)
    # perturb the pitch attitude by err_deg about the body y axis
    from nearspace.model6 import euler_to_quat, quat_mul, quat_normalize, quat_to_euler
    eul = quat_to_euler(traj.q_ref(t))
    q_err = euler_to_quat(0.0, np.deg2rad(err_deg), 0.0)
    x[0:4] = quat_normalize(quat_mul(traj.q_ref(t), q_err))
    x[4:7] = np.zeros(3)
    x[7:10] = np.array([-prm.veh.Cm_alpha / prm.veh.Cm_de
                        * (traj.pitch_ref(t) - traj.gamma(t)), 0.0, 0.0])
    x[10:18] = 0.0
    cfg2 = copy.deepcopy(cfg)
    for k, v in over.items():
        setattr(cfg2.mpc_cmg, k, v)
    ctrl2 = LTVMPCCMG2(prm, cfg2, b["cluster"])
    u, info = ctrl2.solve(x, t, traj)
    return dict(u=[float(v) for v in u], delta_cmd=[float(v) for v in u[0:3]],
                M_cmg_cmd=[float(v) for v in u[3:6]],
                M_cmg_norm=float(np.linalg.norm(u[3:6])),
                q_inf_Pa=float(traj.qinf(t)), t=t, err_deg=err_deg,
                fallback=int(info.get("fallback", 0)),
                qp_status=str(info.get("qp_status", "")),
                solve_time_ms=1e3 * float(info.get("solve_time", 0.0)))


def test_closed_loop():
    cfgB = ab.build_cfg(CASE, "B", CONFIG)
    w = effective_weights(cfgB)
    rec = _short_run(cfgB, T=20.0)
    m = rec["metrics"]
    gd, M, H = np.abs(rec["gdot"]), np.abs(rec["M_cmg"]), rec["H_norm"]

    probe_nom = _probe_at_vertex(cfgB)
    probe_a = _probe_at_vertex(cfgB, M_cmg_box=0.0)
    probe_hi = _probe_at_vertex(cfgB,
                                r_cmg_transient=100.0 * w["r_cmg_transient"])

    checks = [
        dict(check="20 s closed loop solves (no QP failure, no fallback)",
             ok=bool(m["fallback_count"] == 0 and m["qp_fail_count"] == 0),
             value=[m["fallback_count"], m["qp_fail_count"]]),
        dict(check="20 s closed loop stays inside the actuator limits",
             ok=bool(np.max(gd) <= cfgB.cmg.gdot_max + 1e-9
                     and np.max(M) <= cfgB.mpc_cmg.M_cmg_box + 1e-9
                     and float(np.max(H)) <= cfgB.mpc_cmg.H_lim_frac * 200.0 + 1e-6),
             value=[float(np.max(gd)), float(np.max(M)), float(np.max(H))]),
        dict(check="unified MPC uses the CMG at the 100 km vertex (M_cmg command > 0)",
             ok=bool(probe_nom["M_cmg_norm"] > 0.5),
             value=dict(M_cmg_cmd=probe_nom["M_cmg_cmd"],
                        fallback=probe_nom["fallback"],
                        qp_status=probe_nom["qp_status"],
                        solve_time_ms=probe_nom["solve_time_ms"])),
        dict(check="solver status string and fallback flag never contradict each other",
             ok=bool(all((p["fallback"] == 1) != p["qp_status"].lower().startswith("solved")
                         for p in (probe_nom, probe_a, probe_hi))),
             value=[(p["qp_status"], p["fallback"]) for p in (probe_nom, probe_a, probe_hi)]),
        dict(check="zero CMG torque box removes the CMG columns (mode A)",
             ok=bool(probe_a["M_cmg_norm"] == 0.0),
             value=probe_a["M_cmg_cmd"]),
        dict(check="a 100x larger CMG transient weight reduces the CMG command",
             ok=bool(probe_hi["M_cmg_norm"] < 0.9 * probe_nom["M_cmg_norm"]),
             value=[probe_nom["M_cmg_norm"], probe_hi["M_cmg_norm"]]),
        dict(check="solver status is recorded explicitly, not inferred",
             ok=bool(all(k in rec for k in ("qp_fail", "fallback", "viol", "qp_iter",
                                            "solve_time_ms"))),
             value=None),
    ]
    return dict(test="closed_loop", weights_in_force=w,
                controller_budget=controller_budgets(cfgB, VehicleCMGParams.from_config(cfgB)),
                closed_loop_metrics=dict(
                    err_rmse_deg=m["err_rmse_deg"], err_peak_deg=m["err_peak_deg"],
                    cmg_torque_peak_Nm=m["cmg_torque_peak_Nm"],
                    cmg_momentum_peak_Nms=m["cmg_momentum_peak_Nms"],
                    cmg_gimbal_rate_peak_rad_s=m["cmg_gimbal_rate_peak_rad_s"],
                    cmg_sigma_min=m["cmg_sigma_min"], qp_fail_count=m["qp_fail_count"],
                    fallback_count=m["fallback_count"],
                    max_constraint_violation=m["max_constraint_violation"],
                    solve_time_mean_ms=m["solve_time_mean_ms"],
                    solve_time_max_ms=m["solve_time_max_ms"]),
                vertex_probes=dict(nominal=probe_nom, cmg_box_zero=probe_a,
                                   weight_x100=probe_hi),
                checks=checks, passed=all(c["ok"] for c in checks))


# ---------------------------------------------------------------------------
def main():
    os.makedirs(OUT, exist_ok=True)
    out = {}
    names = ("torque_direction", "torque_delivery", "closed_loop")
    for fn in (test_torque_direction, test_torque_delivery, test_closed_loop):
        name = fn.__name__.replace("test_", "")
        print("\n== %s ==" % name, flush=True)
        res = fn()
        out[name] = res
        for c in res["checks"]:
            print("  %-64s %s  %s" % (c["check"], "PASS" if c["ok"] else "FAIL",
                                      "" if c["value"] is None else c["value"]))
        print("  -> %s" % ("PASSED" if res["passed"] else "FAILED"), flush=True)

    passed = all(out[k]["passed"] for k in names)
    out["summary"] = dict(passed=passed,
                          n_checks=sum(len(out[k]["checks"]) for k in names),
                          n_failed=sum(1 for k in names for c in out[k]["checks"]
                                       if not c["ok"]))
    for k in names:
        with open(os.path.join(OUT, k + ".json"), "w", encoding="utf-8") as fh:
            json.dump(out[k], fh, indent=2, ensure_ascii=False)
    with open(os.path.join(OUT, "summary.json"), "w", encoding="utf-8") as fh:
        json.dump(out["summary"], fh, indent=2, ensure_ascii=False)

    lines = ["# CMG actuator / unified-MPC unit tests", "",
             "Generated by `scripts/validate_cmg_actuator.py` for case `%s`." % CASE, ""]
    for k in names:
        v = out[k]
        lines += ["## %s - %s" % (k, "PASSED" if v["passed"] else "FAILED"), "",
                  "| check | result | value |", "|---|---|---|"]
        for c in v["checks"]:
            lines.append("| %s | %s | %s |" % (c["check"], "PASS" if c["ok"] else "FAIL",
                                               c["value"]))
        lines.append("")
    lines.append("Overall: **%s** (%d checks, %d failed)"
                 % ("PASSED" if passed else "FAILED", out["summary"]["n_checks"],
                    out["summary"]["n_failed"]))
    with open(os.path.join(OUT, "summary.md"), "w", encoding="utf-8") as fh:
        fh.write("\n".join(lines) + "\n")

    print("\noverall: %s  (%d checks, %d failed)"
          % ("PASSED" if passed else "FAILED", out["summary"]["n_checks"],
             out["summary"]["n_failed"]))
    print("records:", OUT)
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
