"""基础模型测试（指导实施文件 §3.5 的六项，外加坐标约定复核）。

不使用 pytest：在仓库根目录直接运行
    python experiments/cmg_incremental_search/tests/test_base_models.py
任何一项失败都会以非零退出码结束。
"""

from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

EXP_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(EXP_ROOT))

from src.config import load_config                                       # noqa: E402
from src.control.attitude_controller import AttitudeController, PDGains   # noqa: E402
from src.control.control_allocator import AllocatorWeights, ControlAllocator  # noqa: E402
from src.dynamics.aerodynamic_surface import aero_jacobian_delta, aero_moment  # noqa: E402
from src.dynamics.cmg_pyramid import CMGSpec, PyramidCMG                # noqa: E402
from src.dynamics.rigid_body import (                                   # noqa: E402
    cross_matrix,
    euler_to_quat,
    quat_error,
    quat_mul,
    quat_normalize,
    quat_to_R,
)
from src.dynamics.vehicle_plant import FlightEnv                        # noqa: E402
from src.simulation.runner import build_spec, run_case                  # noqa: E402

RESULTS: list[tuple[str, bool, str]] = []


def check(name: str, ok: bool, detail: str) -> None:
    RESULTS.append((name, bool(ok), detail))
    print(f"[{'PASS' if ok else 'FAIL'}] {name}: {detail}")


def load_cfg() -> dict:
    return load_config("search.yaml", "scenario_high_altitude_maneuver.yaml")


def test_cmg_zero_rate(cfg: dict) -> None:
    cmg = PyramidCMG(CMGSpec.from_dict(cfg["cmg"]))
    gamma = np.array([0.4, -0.9, 1.2, -0.3])
    M = cmg.torque(gamma, np.zeros(4))
    check("T1 框架角速度为零时 CMG 力矩为零", np.allclose(M, 0.0),
          f"|M| = {np.linalg.norm(M):.3e} N m")


def test_cmg_mapping(cfg: dict) -> None:
    spec = CMGSpec.from_dict(cfg["cmg"])
    cmg = PyramidCMG(spec)
    rng = np.random.default_rng(3)
    fd_err, indep_err = 0.0, 0.0
    for _ in range(25):
        gamma = rng.uniform(-np.pi / 2, np.pi / 2, 4)
        gdot = rng.uniform(-spec.gdot_max, spec.gdot_max, 4)
        A = cmg.jacobian(gamma)
        # (a) A == dH/dgamma
        fd = np.zeros((3, 4))
        for i in range(4):
            h = 1e-7
            gp, gm = gamma.copy(), gamma.copy()
            gp[i] += h
            gm[i] -= h
            fd[:, i] = (cmg.momentum(gp) - cmg.momentum(gm)) / (2 * h)
        fd_err = max(fd_err, float(np.max(np.abs(A - fd))))
        # (b) 独立公式：M = sum_i -h0 (g_i x s_i) gdot_i
        s = cmg.spin_axes(gamma)
        M_indep = np.zeros(3)
        for i in range(spec.n):
            M_indep -= spec.h0 * np.cross(cmg.g_axis[:, i], s[:, i]) * gdot[i]
        indep_err = max(indep_err, float(np.max(np.abs(cmg.torque(gamma, gdot) - M_indep))))
    check("T2 力矩映射一致性（A=dH/dgamma，M=-A*gdot 独立公式）",
          fd_err < 1e-5 and indep_err < 1e-12,
          f"max|A-fd| = {fd_err:.2e}, max|M-M_indep| = {indep_err:.2e}")

    # 角动量守恒：零外矩下 R(q)(J w + H) 应保持（定住 -A gdot 的符号）
    plant = build_spec(cfg)
    from src.dynamics.vehicle_plant import initial_state, step as plant_step

    x = initial_state({"omega_deg_s": [3.0, -2.0, 1.5],
                       "gamma_deg": [5.0, -12.0, 17.0, -8.0],
                       "gdot_deg_s": [20.0, -12.0, 9.0, -15.0]})
    env = FlightEnv(rho=0.0, V_ned=np.array([4000.0, 0.0, 0.0]), M_dist=np.zeros(3))
    H0 = quat_to_R(x[0:4]) @ (plant.inertia.J @ x[4:7] + cmg.momentum(x[10:14]))
    u = np.concatenate([np.zeros(3), x[14:18]])
    peak = 0.0
    for _ in range(500):
        x, _ = plant_step(x, u, env, plant, 0.005, 1)
        H = quat_to_R(x[0:4]) @ (plant.inertia.J @ x[4:7] + cmg.momentum(x[10:14]))
        peak = max(peak, float(np.linalg.norm(H - H0)))
    rel = peak / float(np.linalg.norm(H0))
    check("T2b 惯性系角动量守恒（力矩符号）", rel < 1e-6,
          f"|H(t)-H(0)|/|H(0)| = {rel:.3e} over 2.5 s, zero external torque")


def test_actuator_limits(cfg: dict) -> None:
    plant = build_spec(cfg)
    cmg = PyramidCMG(plant.cmg)
    from src.dynamics.vehicle_plant import initial_state, step as plant_step

    x = initial_state({})
    env = FlightEnv(rho=plant_env_rho(cfg), V_ned=np.array([4000.0, 0.0, 0.0]),
                    M_dist=np.zeros(3))
    peak_gdot, peak_gddot, peak_delta, peak_rate = 0.0, 0.0, 0.0, 0.0
    for _ in range(300):
        u = np.concatenate([10.0 * np.ones(3), 10.0 * np.ones(4)])   # 全部饱和
        x_new, u_app = plant_step(x, u, env, plant, 0.2, 40)
        peak_gdot = max(peak_gdot, float(np.max(np.abs(x_new[14:18]))))
        peak_gddot = max(peak_gddot, float(np.max(np.abs(x_new[14:18] - x[14:18]) / 0.2)))
        peak_delta = max(peak_delta, float(np.max(np.abs(x_new[7:10]) / plant.aero.delta_max)))
        peak_rate = max(peak_rate, float(np.max(np.abs(x_new[7:10] - x[7:10]) / 0.2
                                               / plant.aero.delta_dot_max)))
        x = x_new
    gamma_max = float(np.max(np.abs(x[10:14])))
    ok = (peak_gdot <= plant.cmg.gdot_max + 1e-9
          and peak_gddot <= plant.cmg.gddot_max + 1e-3
          and peak_delta <= 1.0 + 1e-9
          and peak_rate <= 1.0 + 1e-9
          and gamma_max <= plant.cmg.travel + 1e-9)
    check("T3 执行器限制（舵面幅值/速率、框架角速度/加速度、机械行程）", ok,
          f"gdot={peak_gdot:.4f}/{plant.cmg.gdot_max}, gddot={peak_gddot:.3f}/"
          f"{plant.cmg.gddot_max}, |delta|/max={peak_delta:.4f}, "
          f"|delta_dot|/max={peak_rate:.4f}, |gamma|max={np.rad2deg(gamma_max):.2f}deg/"
          f"{plant.cmg.travel_deg:g}deg")


def plant_env_rho(cfg: dict) -> float:
    from src.dynamics.atmosphere import TableAtmosphere

    return TableAtmosphere().rho(1000.0 * float(cfg["scenario"]["altitude_km"]))


def test_allocator_matches_plant(cfg: dict) -> None:
    spec = build_spec(cfg)
    cmg = PyramidCMG(spec.cmg)
    alloc = ControlAllocator(spec, AllocatorWeights.from_dict(cfg["allocation"]))
    from src.dynamics.vehicle_plant import initial_state, step as plant_step, torque_terms

    x = initial_state({"euler_deg": [1.0, 2.0, -1.0], "omega_deg_s": [1.0, -1.5, 0.5],
                       "delta_deg": [2.0, -1.0, 1.5], "gamma_deg": [8.0, -6.0, 11.0, -4.0],
                       "gdot_deg_s": [10.0, -5.0, 7.0, -9.0]})
    env = FlightEnv(rho=plant_env_rho(cfg), V_ned=np.array([4000.0, 0.0, 0.0]),
                    M_dist=np.zeros(3))
    tau_req = np.array([3.0, -8.0, 1.5])
    res = alloc.allocate(x, tau_req, env, np.zeros(4), 0.2)

    # (a) 同一状态、同一 delta/gamma/gdot：分配器模型 vs 共用 predicted_torque
    same = alloc.plant_consistent_torque(x, res.delta_cmd, x[10:14], res.gdot_cmd, env)
    err_same = float(np.max(np.abs(same - res.predicted_torque)))
    # (b) 一步之后，plant 实际力矩 == 在"实际被执行的"状态上重新计算的模型力矩
    x_next, u_app = plant_step(x, np.concatenate([res.delta_cmd, res.gdot_cmd]), env, spec, 0.2, 40)
    pred_achieved = alloc.plant_consistent_torque(x_next, x_next[7:10], x_next[10:14],
                                                  x_next[14:18], env)
    actual = torque_terms(x_next, env, spec)
    err_ach_aero = float(np.max(np.abs(pred_achieved - actual["M_total"])))
    # 伺服/框架环在本周期内的爬升造成的瞬时差（不是模型误差，量级须可控）
    lag_gap = float(np.max(np.abs(res.predicted_torque - pred_achieved)))
    # (c) 气动雅可比 vs 有限差分
    B = aero_jacobian_delta(x[0:4], x[4:7], env.V_ned, env.rho, spec.aero)
    fd = np.zeros((3, 3))
    for j in range(3):
        h = 1e-8
        dp, dm = x[7:10].copy(), x[7:10].copy()
        dp[j] += h
        dm[j] -= h
        fd[:, j] = (aero_moment(x[0:4], x[4:7], dp, env.V_ned, env.rho, spec.aero)
                    - aero_moment(x[0:4], x[4:7], dm, env.V_ned, env.rho, spec.aero)) / (2 * h)
    err_B = float(np.max(np.abs(B - fd)))
    ok = err_same < 1e-9 and err_ach_aero < 1e-6 and err_B < 1e-3
    check("T4 分配器预测力矩与 plant 力矩一致", ok,
          f"same-state {err_same:.2e} N m, achieved-state {err_ach_aero:.2e} N m, "
          f"dB_delta vs fd {err_B:.2e}, 伺服爬升引起的瞬时差 {lag_gap:.2f} N m; "
          f"residual {res.residual_norm:.3f} N m, feasible={res.feasible}")


def test_closed_loop(cfg: dict) -> None:
    cfg = dict(cfg)
    cfg["simulation"] = dict(cfg["simulation"])
    cfg["scenario"] = dict(cfg["scenario"])
    cfg["scenario"]["maneuver"] = {"axis": "pitch", "t0": 5.0, "duration": 20.0,
                                   "delta_deg": 3.0}
    res = run_case(cfg, name="base_closed_loop",
                   x0={"euler_deg": [0.5, -0.8, 0.4], "omega_deg_s": [0.3, -0.2, 0.1],
                       "gamma_deg": [5.0, -4.0, 7.0, -3.0]})
    m = res.metrics
    ok = (m["att_final_deg"] <= 0.5 and m["att_rmse_deg"] < 1.0 and m["att_peak_deg"] < 3.0
          and m["t_settle_s"] is not None and m["infeasible_count"] <= 5)
    check("T5 无搜索的基础闭环稳定", ok,
          f"final={m['att_final_deg']:.3f} deg, rmse={m['att_rmse_deg']:.3f} deg, "
          f"peak={m['att_peak_deg']:.3f} deg, settle={m['t_settle_s']} s, "
          f"residual_max={m['residual_max_nm']:.3f} N m, fallback={m['fallback_count']}, "
          f"request_infeasible={m['infeasible_count']}, sigma_min={m['sigma_min_min']:.2f} N m s, "
          f"alloc_ms(mean/max)={m['alloc_ms_mean']:.2f}/{m['alloc_ms_max']:.2f}")


def test_logging(cfg: dict) -> None:
    res = run_case(cfg, name="log_check")
    arr = res.log.arrays()
    keys = set(arr)
    need = {"tau_req", "tau_pred", "tau_actual", "tau_effective", "M_aero_actual",
            "M_cmg_actual", "solve_time_ms", "ctrl_time_ms", "search_time_ms",
            "feasible", "request_feasible", "fallback", "residual_norm", "sigma_min"}
    missing = need - keys
    n_expected = int(round(cfg["simulation"]["T"] / cfg["simulation"]["dt_ctrl"]))
    all_finite = all(np.all(np.isfinite(np.asarray(arr[k], dtype=float)))
                     for k in ("tau_req", "tau_pred", "tau_actual", "tau_effective"))
    req_pred = np.asarray(arr["tau_req"], dtype=float) - np.asarray(arr["tau_pred"], dtype=float)
    gap_ins = float(np.mean(np.linalg.norm(np.asarray(arr["tau_pred"], dtype=float)
                                           - np.asarray(arr["tau_actual"], dtype=float), axis=1)))
    ok = (not missing) and all_finite and len(arr["t"]) == n_expected
    check("T6 日志区分期望/预测/真实/等效力矩与耗时", ok,
          f"missing={sorted(missing)}; 样本 {len(arr['t'])}/{n_expected}; "
          f"max|tau_req-tau_pred|={np.max(np.linalg.norm(req_pred, axis=1)):.3f} N m "
          f"(不可实现部分), mean|tau_pred-tau_actual|={gap_ins:.3f} N m (瞬时口径), "
          f"mean|tau_pred-tau_effective|={res.metrics.get('torque_eff_gap_mean_nm', float('nan')):.3f} N m")


def test_conventions(cfg: dict) -> None:
    # 四元数运动学：固定 omega 下 R 与指数映射一致
    q = quat_normalize(np.array([0.8, -0.3, 0.2, 0.1]))
    omega = np.array([0.12, -0.07, 0.05])
    R0 = quat_to_R(q)
    dt = 1e-4
    for _ in range(10000):
        q = quat_normalize(q + dt * 0.5 * quat_mul(q, np.array([0.0, *omega])))
    th = float(np.linalg.norm(omega))
    K = cross_matrix(omega / th)
    R_exp = R0 @ (np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * (K @ K))
    err = float(np.max(np.abs(quat_to_R(q) - R_exp)))
    # 误差映射：小角度下 2*dq_vec 与欧拉角差只差二阶项（2.2° 合成旋转约差 3e-4 rad）
    e0 = np.deg2rad([1.0, -2.0, 0.5])
    q2 = euler_to_quat(*e0)
    err2 = float(np.max(np.abs(quat_error(np.array([1.0, 0, 0, 0]), q2) - e0)))
    check("T7 坐标/姿态约定（R_dot=R[omega]x，误差映射）", err < 1e-10 and err2 < 1e-3,
          f"max|R-R_exp| = {err:.2e}, 小角度误差映射二阶偏差 = {np.rad2deg(err2):.2e} deg "
          f"（应为二阶量级，不是 0）")


def main() -> int:
    cfg = load_cfg()
    print(f"config: {EXP_ROOT / 'configs'}/scenario_high_altitude_maneuver.yaml (+base+search)")
    print(f"T={cfg['simulation']['T']} s, dt_ctrl={cfg['simulation']['dt_ctrl']} s, "
          f"altitude={cfg['scenario']['altitude_km']} km, "
          f"maneuver={cfg['scenario']['maneuver']}\n")
    test_cmg_zero_rate(cfg)
    test_cmg_mapping(cfg)
    test_actuator_limits(cfg)
    test_allocator_matches_plant(cfg)
    test_closed_loop(cfg)
    test_logging(cfg)
    test_conventions(cfg)
    failed = [r for r in RESULTS if not r[1]]
    print(f"\n{len(RESULTS) - len(failed)}/{len(RESULTS)} tests pass")
    return 0 if not failed else 1


if __name__ == "__main__":
    sys.stdout.reconfigure(encoding="utf-8")
    raise SystemExit(main())
