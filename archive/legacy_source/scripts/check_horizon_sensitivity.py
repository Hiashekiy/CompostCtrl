"""Prediction-horizon sensitivity of the C-group / T-group arms.

    python scripts/check_horizon_sensitivity.py

Why this exists: the CMG MPC must use SHORT nodes (a gimbal at 1.5 rad/s sweeps
1.5 rad per 1 s node, so a coarse node invalidates the linearisation of
A(gamma)).  Its horizon is therefore 14 nodes / 7.2 s, while the historical
wheel-era controller used 20 nodes / 10.6 s.  The two truth models are provably
identical for the device-free case (states 0..9 agree to 0.0 over 750 steps), so
any difference between a "surfaces only" number measured in the two pipelines is
the CONTROLLER horizon, not the physics.  This script measures that effect so it
is reported instead of hidden.

Matrix: {cseries, tseries} x {surfaces only, CMG} x {7.2 s, 10.6 s}, n_sqp = 2.
Prints a markdown table plus, for the device-free case, the trajectory-agreement
check between the 16-state wheel model and the 18-state CMG model.
"""
from __future__ import annotations

import copy
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)

import run_device_sweep as S                                     # noqa: E402
from nearspace.atmosphere_factory import make_atmosphere         # noqa: E402
from nearspace.cmg_control import LTVMPCCMG2                     # noqa: E402
from nearspace.model6 import (VehicleParams, plant_step3,        # noqa: E402
                              quat_geodesic)
from nearspace.model6c import (VehicleCMGParams, make_cluster,   # noqa: E402
                               plant_step6c)
from nearspace.mpc3 import LTVMPC3                               # noqa: E402
from nearspace.sim import _initial_state                         # noqa: E402
from nearspace.trajectory3 import Trajectory3D                   # noqa: E402

HORIZONS = {"short": None, "long": "mpc3"}   # None = the CMG MPC's own 7.2 s


def _setup(case):
    cfg = S.build_cfg(case)
    atm = make_atmosphere(cfg, cache_dir=os.path.join(ROOT, "results", "_cache"),
                          backend=getattr(cfg.environment, "backend", "msis2"))
    return cfg, Trajectory3D(cfg, atm)


def plant_identity(cfg, traj, steps=750):
    """Prove that the device-free plant does not depend on which module is used."""
    prmW = VehicleParams.from_config(cfg, wheel_on=False)
    prmC = VehicleCMGParams.from_config(cfg, cmg_on=False)
    cl = make_cluster(prmC)
    de = -prmC.veh.Cm_alpha / prmC.veh.Cm_de * (traj.pitch_ref(0.0) - traj.gamma(0.0))
    xW = np.concatenate([traj.q_ref(0.0), np.zeros(3), [de, 0, 0], np.zeros(3), np.zeros(3)])
    xC = np.concatenate([traj.q_ref(0.0), np.zeros(3), [de, 0, 0], np.zeros(4), np.zeros(4)])
    rng = np.random.default_rng(0)
    t, worst, n = 0.0, 0.0, 0
    dtp = float(cfg.simulation.dt_plant)
    nsub = max(1, int(round(float(cfg.simulation.dt_ctrl) / dtp)))
    while n < steps:
        u6 = np.concatenate([np.deg2rad(rng.uniform(-8, 8, 3)), np.zeros(3)])
        u7 = np.concatenate([u6[0:3], np.zeros(4)])
        for _ in range(nsub):
            p = traj.p_array(t, prmW)
            xW = plant_step3(xW, u6, p, dtp, prmW)[0]
            xC = plant_step6c(xC, u7, p, dtp, prmC, cl)[0]
            t += dtp
        worst = max(worst, float(np.max(np.abs(xW[0:10] - xC[0:10]))))
        n += 1
    return worst, n


def run(case, device, horizon, n_sqp=2):
    cfg, traj = _setup(case)
    T = float(cfg.simulation.T)
    dtc, dtp = float(cfg.simulation.dt_ctrl), float(cfg.simulation.dt_plant)
    nsub = max(1, int(round(dtc / dtp)))
    c = copy.deepcopy(cfg)
    if HORIZONS[horizon]:
        c.mpc_cmg.horizon = copy.deepcopy(cfg.mpc3.horizon)
    c.mpc_cmg.n_sqp = n_sqp

    if device == "cmg":
        prm = VehicleCMGParams.from_config(cfg, cmg_on=True)
        cl = make_cluster(prm)
        ctrl = LTVMPCCMG2(prm, c, cl)
        step = lambda xx, uu, p, dt: plant_step6c(xx, uu, p, dt, prm, cl)[0]
        mom = lambda xx: float(np.linalg.norm(cl.momentum(xx[10:14])))
        sig = lambda xx: cl.sigma_min(cl.jacobian(xx[10:14]))
        mtor = lambda xx, uu: float(np.linalg.norm(cl.torque(xx[10:14], xx[14:18])))
        nu = 7
    else:
        prm = VehicleCMGParams.from_config(cfg, cmg_on=False)
        cl = make_cluster(prm)
        ctrl = LTVMPCCMG2(prm, c, cl)      # same controller, CMG switched off
        step = lambda xx, uu, p, dt: plant_step6c(xx, uu, p, dt, prm, cl)[0]
        mom = lambda xx: 0.0
        sig = lambda xx: cl.prm.home_sigma_min
        mtor = lambda xx, uu: 0.0
        nu = 7
    x = _initial_state(traj, prm)
    t, en, d, H, S_, M_, fails = 0.0, [], [], [], [], [], 0
    while t < T - 1e-9:
        u, info = ctrl.solve(x, t, traj)
        for _ in range(nsub):
            x = step(x, u, traj.p_array(t, prm.veh), dtp)
            t += dtp
        fails += int(info.get("fallback", 0))
        en.append(np.rad2deg(quat_geodesic(traj.q_ref(t), x[0:4])))
        d.append(np.rad2deg(x[7:10]))
        H.append(mom(x))
        S_.append(sig(x))
        M_.append(mtor(x, u))
    en, d, H, S_ = map(np.array, (en, d, H, S_))
    sat = float(np.mean(np.any(np.abs(d) >= 0.999 * np.rad2deg(prm.veh.delta_max)[None, :],
                               axis=1)))
    return dict(case=case, device=device, horizon=horizon,
                rmse=float(np.sqrt(np.mean(en ** 2))), peak=float(en.max()),
                Hmax=float(H.max()), Mmax=float(max(M_)), sigmin=float(S_.min()),
                sat=sat, fails=fails)


def main():
    print("# 预测时域敏感性（同一真值、同一权重、同一初值，只换时域；n_sqp=2）\n")
    worst, n = plant_identity(*_setup("tseries"))
    print("真值模型一致性：16 状态(wheel_on=False) 与 18 状态(cmg_on=False) 的前 10 个状态"
          "在 %d 个控制步内最大偏差 = %.3e\n" % (n, worst))
    print("| 场景 | 装置 | 时域 | 三轴 RMSE [°] | 峰值 [°] | \\|H\\|max [N·m·s] | "
          "\\|M\\|max [N·m] | σ_min | 舵面饱和 | 保底触发 |")
    print("|---|---|---|---|---|---|---|---|---|---|")
    rows = []
    for case in ("cseries", "tseries"):
        for dev in ("surface", "cmg"):
            for hz in ("short", "long"):
                r = run(case, dev, hz)
                rows.append(r)
                print("| %s | %s | %s（%s） | %.4f | %.3f | %.2f | %.2f | %.2f | %.1f%% | %d |"
                      % (r["case"], r["device"], hz,
                         "7.2 s/14 节点" if hz == "short" else "10.6 s/20 节点",
                         r["rmse"], r["peak"], r["Hmax"], r["Mmax"], r["sigmin"],
                         100 * r["sat"], r["fails"]))
    print("\n读法：")
    print("* 无装置臂在两个时域下真值完全一致，任何差异都是控制器造成的；")
    print("* 时域应 ≳ 机动持续时间：C 组机动 10 s → 7.2 s 更好；T 组机动 20 s → 10.6 s 更好；")
    print("* 所以同一个数在不同时域下可以差几倍甚至从收敛变发散，报告时必须注明时域。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
