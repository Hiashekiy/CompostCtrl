"""§7.1 / §7.2 verification: CMG weights and the CMG torque chain.

    python scripts/verify_cmg_weights.py

Three independent checks, all printed, all reproducible:

1.  **Where the weights are defined / overridden.**  Scans config.yaml, the
    experiment matrix and the controller defaults, then prints the value that
    is *actually in force* after the overlay chain has been resolved (not just
    what one YAML file says).

2.  **Does the weight reach the optimiser?**  One MPC solve from a state with a
    known attitude error, repeated for a log-spaced set of `r_cmg`.  Reports the
    CMG torque block of the objective, which per-node weight is active
    (transient vs trim), and the resulting first CMG torque command.  If the
    command does not move, the weight is not wired to the objective.

3.  **Is the torque chain real?**  Holds a constant CMG torque command on the
    18-state truth model and reports commanded vs delivered torque, gimbal
    rate, gimbal angle and sigma_min over time.
"""
from __future__ import annotations

import json
import os
import re
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)

from nearspace.cmg_control import LTVMPCCMG2, steering_law        # noqa: E402
from nearspace.model6c import (VehicleCMGParams, make_cluster,    # noqa: E402
                               plant_step6c)
from nearspace.scenarios import cmg_spec, effective_weights      # noqa: E402
from nearspace.trajectory3 import Trajectory3D                    # noqa: E402
from run_cmg_ab import build_cfg                                  # noqa: E402

PATTERNS = ("r_cmg_transient", "r_cmg_trim", "e_thresh_deg", "w_H", "w_gamma",
            "M_cmg_box", "H_lim_frac", "H_planner_max", "q_unload_min")
SCAN_FILES = ["config.yaml",
              os.path.join("configs", "cmg_ab", "matrix.yaml"),
              os.path.join("nearspace", "cmg_control.py")]


def scan_definitions():
    print("=" * 78)
    print("1. 关 键 权 重 / 限 值 的 定 义 与 覆 盖 位 置")
    print("=" * 78)
    for rel in SCAN_FILES:
        path = os.path.join(ROOT, rel)
        if not os.path.exists(path):
            continue
        print("\n-- %s" % rel)
        with open(path, encoding="utf-8") as fh:
            for i, line in enumerate(fh, 1):
                for p in PATTERNS:
                    if re.search(r"\b%s\b" % p, line):
                        print("   %4d: %s" % (i, line.rstrip()[:110]))
                        break


def resolved_weights():
    print("\n" + "=" * 78)
    print("2. 叠 加 链 解 析 后 真 正 生 效 的 值（日志以这里为准）")
    print("=" * 78)
    hist_cfg = build_cfg("disturbance", "B")     # original 0.5 / 0.5 case
    frozen = build_cfg("tracking", "B")          # frozen weights in matrix common
    print("  matrix common (frozen):", json.dumps(effective_weights(frozen), ensure_ascii=False))
    ov = dict(hist_cfg.mpc_cmg.__dict__)
    ov.update({"r_cmg_transient": 0.5, "r_cmg_trim": 0.5})
    print("  historical (config.yaml 0.5/0.5, reproduced as case wscan_0p5):")
    print("      r_cmg_transient=%s r_cmg_trim=%s  adaptive=%s"
          % (ov["r_cmg_transient"], ov["r_cmg_trim"],
             ov["r_cmg_transient"] != ov["r_cmg_trim"]))
    prm = VehicleCMGParams.from_config(frozen)
    print("  CMG hardware:", json.dumps(cmg_spec(frozen, prm), ensure_ascii=False))
    return frozen


# ---------------------------------------------------------------------------
# single-step sensitivity test
# ---------------------------------------------------------------------------
def _state_with_error(cfg, pitch_err_deg=1.0):
    b = __import__("nearspace.sim", fromlist=["sim"]).build(cfg)
    traj, prm, cl = b["traj"], b["prm"], b["cluster"]
    from nearspace.model6 import euler_to_quat
    t = 112.0
    e = traj.euler_ref(t).copy()
    e[1] += np.deg2rad(pitch_err_deg)
    x = np.concatenate([euler_to_quat(*e), np.zeros(3), np.zeros(3),
                        np.zeros(prm.cmg.n), np.zeros(prm.cmg.n)])
    return traj, prm, cl, x, t


def single_step_sensitivity(cfg):
    print("\n" + "=" * 78)
    print("3. 单 步 灵 敏 度：降低 CMG 代价是否改变最优 CMG 指令")
    print("=" * 78)
    print("   初值：t=112 s（100 km 顶点附近），俯仰姿态误差 1.0 deg，角速度 0")
    print("   %-10s %-9s %-13s %-12s %-9s %s"
          % ("r_cmg_tr", "R_eff", "active_nodes", "|M_cmg(0)| Nm", "solve", "gdot_cmd peak"))
    rows = []
    for r in (0.5, 0.05, 0.01, 5e-3, 5e-4):
        traj, prm, cl, x, t = _state_with_error(cfg)
        cfg2 = build_cfg("disturbance", "B")
        cfg2.mpc_cmg.r_cmg_transient = float(r)
        cfg2.mpc_cmg.r_cmg_trim = float(max(r, 0.5))
        cfg2.mpc_cmg.n_sqp = 1
        mpc = LTVMPCCMG2(prm, cfg2, cl)
        u, info = mpc.solve(x, t, traj)
        # objective block that the CMG input sees at node 0
        Rcmg = 2.0 * float(cfg2.mpc_cmg.r_cmg_transient) * mpc.dts[0]
        # which per-node weight is active along the horizon
        act = []
        for k in range(mpc.N):
            ek = np.linalg.norm(mpc._att_error(traj.q_ref(t + sum(mpc.dts[:k])), x[0:4]))
            act.append("tr" if ek > mpc.e_thresh else "trim")
        fb = int(info.get("fallback", 0))
        if mpc.u_seq is not None:
            M0 = np.asarray(mpc.u_seq[0][3:6], dtype=float)
        else:                                     # solver fell back: no plan
            M0 = np.array([np.nan, np.nan, np.nan])
        g0 = np.asarray(u[3:7], dtype=float)
        rows.append(dict(r=r, R_eff_Rcmg=float(Rcmg), M0=float(np.linalg.norm(M0)),
                         gdot=float(np.max(np.abs(g0))), fallback=fb,
                         status=info.get("qp_status"), viol=info.get("viol")))
        print("   %-10.4g %-14.4g %-13s %-13.4f %-10s %.4f  (%s)"
              % (r, Rcmg, "%d tr/%d trim" % (act.count("tr"), act.count("trim")),
                 np.linalg.norm(M0), ("FALLBACK" if fb else "ok"),
                 np.max(np.abs(g0)), info.get("qp_status")))
    good = [x["M0"] for x in rows if np.isfinite(x["M0"])]
    spread = (max(good) - min(good)) if len(good) > 1 else 0.0
    print("\n   |M_cmg(0)| 在扫描范围内的变化幅度 = %.4f N·m" % spread)
    print("   判定：%s" % ("权重确实进入目标函数并改变最优指令"
                          if spread > 1e-3 else
                          "!! 权重没有改变最优指令 —— 链路未接通或该项被其他项淹没"))
    return rows


# ---------------------------------------------------------------------------
# torque chain test
# ---------------------------------------------------------------------------
def torque_chain(cfg, M_cmd=20.0, T=5.0):
    print("\n" + "=" * 78)
    print("4. 力 矩 链 路：常 值 CMG 力 矩 指 令 下 的 实 际 交 付")
    print("=" * 78)
    b = __import__("nearspace.sim", fromlist=["sim"]).build(cfg)
    traj, prm, cl = b["traj"], b["prm"], b["cluster"]
    from nearspace.model6 import quat_normalize
    x = np.concatenate([traj.q_ref(112.0), np.zeros(3), np.zeros(3),
                        np.zeros(prm.cmg.n), np.zeros(prm.cmg.n)])
    p = traj.p_array(112.0, prm.veh)
    dt = 0.005
    print("   %-6s %-12s %-12s %-12s %-10s %-10s"
          % ("t [s]", "M_cmd [Nm]", "delivered", "ratio", "max|gdot|", "sigma_min"))
    out = []
    t = 0.0
    nxt = 0.5
    while t < T:
        Mcmd = np.array([0.0, M_cmd, 0.0])
        gdot = steering_law(cl, x[10:14], Mcmd, np.zeros(prm.cmg.n))
        u = np.concatenate([np.zeros(3), gdot])
        x, _ = plant_step6c(x, u, p, dt, prm, cl, substeps=1)
        t += dt
        if t >= nxt:
            M = -cl.jacobian(x[10:14]) @ x[14:18]
            sig = cl.sigma_min(cl.jacobian(x[10:14]))
            out.append(dict(t=t, delivered=float(M[1]), sigma=float(sig),
                            gdot=float(np.max(np.abs(x[14:18]))),
                            gimbal=float(np.max(np.abs(np.rad2deg(x[10:14]))))))
            print("   %-6.2f %-12.1f %-12.3f %-12.3f %-10.3f %-10.2f"
                  % (t, M_cmd, M[1], M[1] / M_cmd, np.max(np.abs(x[14:18])), sig))
            nxt += 0.5
    return out


def solve_time_benchmark(cfg, n=20):
    """Single-process, no-contention MPC solve time (real-time feasibility)."""
    print("\n" + "=" * 78)
    print("5. 单 步 求 解 耗 时（单 进 程、无 并 发）")
    print("=" * 78)
    b = __import__("nearspace.sim", fromlist=["sim"]).build(cfg)
    traj, prm, cl = b["traj"], b["prm"], b["cluster"]
    from nearspace.cmg_control import LTVMPCCMG2
    from nearspace.model6 import euler_to_quat
    t = 112.0
    e = traj.euler_ref(t).copy()
    e[1] += np.deg2rad(1.0)
    x = np.concatenate([euler_to_quat(*e), np.zeros(3), np.zeros(3),
                        np.zeros(prm.cmg.n), np.zeros(prm.cmg.n)])
    mpc = LTVMPCCMG2(prm, cfg, cl)
    ts = []
    for _ in range(n):
        _, info = mpc.solve(x, t, traj)
        ts.append(1e3 * info["solve_time"])
    ts = np.array(ts)
    print("   n=%d  mean=%.1f ms  median=%.1f ms  p95=%.1f ms  max=%.1f ms"
          % (n, ts.mean(), np.median(ts), np.percentile(ts, 95), ts.max()))
    print("   控制周期 dt_ctrl = %.1f ms -> %s"
          % (1e3 * float(cfg.simulation.dt_ctrl),
             "满足实时" if ts.mean() * 1e3 < 1e3 * float(cfg.simulation.dt_ctrl)
             else "!! 超过一个控制周期，不可实时"))
    return dict(mean_ms=float(ts.mean()), median_ms=float(np.median(ts)),
                p95_ms=float(np.percentile(ts, 95)), max_ms=float(ts.max()), n=n)


def main():
    scan_definitions()
    cfg = resolved_weights()
    rows = single_step_sensitivity(cfg)
    chain = torque_chain(cfg)
    bench = solve_time_benchmark(cfg)
    with open(os.path.join(ROOT, "results", "cmg_ab", "weight_verification.json"),
              "w", encoding="utf-8") as fh:
        json.dump(dict(sensitivity=rows, chain=chain, solve_time=bench,
                       frozen_weights=effective_weights(cfg)),
                  fh, indent=2, ensure_ascii=False, default=float)
    print("\nwrote results/cmg_ab/weight_verification.json")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
