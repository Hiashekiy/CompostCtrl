"""闭环 runner（指导实施文件 §3.4/§8/§11）。

每个控制周期：参考 -> 姿态控制器（期望总力矩）-> 分配器（+可选搜索给出的框架角速度参考）
-> 可行性检查/安全回退 -> plant 积分 -> 三路力矩与耗时落盘。

`search` 参数为 None 时就是 B 组（无构型迁移参考）；C/D 组在后续阶段注入同一接口的搜索器：
    search.step(x, t, ref, env, dt) -> (gdot_ref, info)
只执行搜索给出的第一步，且**必须**再通过分配器的可行性检查才会下发。
"""

from __future__ import annotations

import time
from dataclasses import dataclass, field

import numpy as np

from ..control.attitude_controller import AttitudeController, PDGains
from ..control.control_allocator import AllocatorWeights, ControlAllocator
from ..control.safety_fallback import SafetyFallback
from ..dynamics.atmosphere import TableAtmosphere
from ..dynamics.aerodynamic_surface import AeroSpec
from ..dynamics.cmg_pyramid import CMGSpec, PyramidCMG
from ..dynamics.rigid_body import quat_error, quat_geodesic, quat_to_euler
from ..dynamics.vehicle_plant import (
    FlightEnv,
    Inertia,
    PlantSpec,
    initial_state,
    step as plant_step,
    torque_terms,
)
from .logger import TimeSeriesLogger
from .metrics import compute_metrics
from .reference import Reference


@dataclass
class SimSpec:
    T: float = 60.0
    dt_ctrl: float = 0.2
    dt_plant: float = 0.005

    @property
    def substeps(self) -> int:
        return max(1, int(round(self.dt_ctrl / self.dt_plant)))


def build_spec(cfg: dict) -> PlantSpec:
    return PlantSpec(
        inertia=Inertia(**{k: v for k, v in cfg.get("vehicle", {}).items()
                           if k in Inertia.__dataclass_fields__}),
        aero=AeroSpec.from_dict(cfg.get("aero", {})),
        cmg=CMGSpec.from_dict(cfg.get("cmg", {})),
        cmg_on=bool((cfg.get("actuators", {}) or {}).get("cmg_on", True)),
    )


@dataclass
class RunResult:
    name: str
    metrics: dict
    log: TimeSeriesLogger
    meta: dict = field(default_factory=dict)


def run_case(cfg: dict, name: str = "B", search=None, x0: dict | None = None) -> RunResult:
    sim = SimSpec(**{k: v for k, v in cfg.get("simulation", {}).items()
                     if k in SimSpec.__dataclass_fields__})
    spec = build_spec(cfg)
    atm = TableAtmosphere()
    ref = Reference(cfg.get("scenario", {}), atm)
    ctrl = AttitudeController(spec, PDGains.from_dict(cfg.get("control", {})))
    alloc = ControlAllocator(spec, AllocatorWeights.from_dict(cfg.get("allocation", {})))
    fallback = SafetyFallback(spec, alloc)
    cmg = PyramidCMG(spec.cmg)

    x = initial_state(x0 or cfg.get("initial_state", {}))
    log = TimeSeriesLogger()
    n = int(round(sim.T / sim.dt_ctrl))
    for k in range(n):
        t = k * sim.dt_ctrl
        ctrl_t0 = time.perf_counter()
        env = ref.env(t)
        q_ref, w_ref = ref.q_ref(t), ref.omega_ref(t)
        tau_req = ctrl.torque_request(x, q_ref, w_ref)

        gdot_ref = np.zeros(spec.cmg.n)
        search_time_ms, search_nodes, search_status = 0.0, 0, "off"
        if search is not None:
            gdot_ref, info = search.step(x, t, ref, env, sim.dt_ctrl)
            search_time_ms = float(info.get("time_ms", 0.0))
            search_nodes = int(info.get("nodes", 0))
            search_status = str(info.get("status", "ok"))
            if search_status not in ("ok", "warm", "repaired"):
                gdot_ref = np.zeros(spec.cmg.n)     # 搜索失败：停止迁移，走安全回退

        res = alloc.allocate(x, tau_req, env, gdot_ref, sim.dt_ctrl)
        request_feasible = bool(res.feasible)      # 原始期望力矩本身是否可实现
        if search_status not in ("ok", "warm", "repaired", "off"):
            res = fallback.actuate(x, tau_req, env, sim.dt_ctrl, f"search:{search_status}")
        elif not res.feasible:
            res = fallback.actuate(x, tau_req, env, sim.dt_ctrl, "allocator-infeasible")

        u_cmd = np.concatenate([res.delta_cmd, res.gdot_cmd])
        ctrl_time_ms = 1e3 * (time.perf_counter() - ctrl_t0)
        x_next, u_applied = plant_step(x, u_cmd, env, spec, sim.dt_ctrl, sim.substeps)

        terms_before = torque_terms(x, env, spec)
        terms_after = torque_terms(x_next, env, spec)
        # 本周期**等效平均**总力矩：由角动量方程反解，J w_dot + w x (J w + H)。
        # 它与瞬时力矩的区别就是框架/舵面在本周期内的爬升，报告里两者都列。
        J = spec.inertia.J
        w1 = x_next[4:7]
        H1 = terms_after["M_cmg_momentum"]
        tau_eff = J @ ((w1 - x[4:7]) / sim.dt_ctrl) + np.cross(w1, J @ w1 + H1) \
            - np.asarray(env.M_dist, dtype=float)
        eul = quat_to_euler(x_next[0:4])
        eul_ref = quat_to_euler(q_ref)
        delta_cmd_deg = np.rad2deg(u_applied[0:3])
        delta_max_deg = np.rad2deg(spec.aero.delta_max)
        log.log(
            t,
            rho=env.rho, V=float(np.linalg.norm(env.V_ned)), qinf=ref.qinf(t),
            h_km=ref.altitude_km,
            q=x_next[0:4], q_ref=q_ref, omega=x_next[4:7],
            err_norm_deg=np.rad2deg(quat_geodesic(q_ref, x_next[0:4])),
            err_roll_deg=np.rad2deg(eul[0] - eul_ref[0]),
            err_pitch_deg=np.rad2deg(eul[1] - eul_ref[1]),
            err_yaw_deg=np.rad2deg(eul[2] - eul_ref[2]),
            tau_req=tau_req, tau_pred=res.predicted_torque, tau_actual=terms_after["M_total"],
            tau_effective=tau_eff,
            M_aero_actual=terms_before["M_aero"], M_cmg_actual=terms_after["M_cmg"],
            H_cmg=terms_after["M_cmg_momentum"],
            H_norm=float(np.linalg.norm(terms_after["M_cmg_momentum"])),
            H_cmg_norm=float(np.linalg.norm(terms_after["M_cmg_momentum"])),
            tau_req_norm=float(np.linalg.norm(tau_req)),
            tau_pred_norm=float(np.linalg.norm(res.predicted_torque)),
            tau_actual_norm=float(np.linalg.norm(terms_after["M_total"])),
            tau_effective_norm=float(np.linalg.norm(tau_eff)),
            residual=res.residual, residual_norm=res.residual_norm,
            delta_cmd=delta_cmd_deg, delta=np.rad2deg(x_next[7:10]),
            delta_dot=np.rad2deg(u_applied[0:3] - x[7:10]) / sim.dt_ctrl,
            gamma=x_next[10:14], gdot=x_next[14:18], gdot_ref_norm=float(np.linalg.norm(gdot_ref)),
            sigma_min=cmg.sigma_min(cmg.jacobian(x_next[10:14])),
            gamma_travel_frac=float(np.max(np.abs(x_next[10:14])) / spec.cmg.travel),
            aero_sat_frac=float(np.mean(np.abs(delta_cmd_deg) >= 0.98 * delta_max_deg)),
            feasible=bool(res.feasible), request_feasible=request_feasible,
            fallback=bool(res.fallback),
            solve_time_ms=res.solve_time_ms, ctrl_time_ms=ctrl_time_ms,
            search_time_ms=search_time_ms, search_nodes=search_nodes,
            search_status=search_status,
        )
        x = x_next
        if not np.all(np.isfinite(x)):
            break

    metrics = compute_metrics(log.arrays(), tol_deg=float(cfg.get("metrics", {}).get("tol_deg", 0.5)))
    meta = {
        "name": name,
        "case": cfg.get("case", name),
        "policy": cfg.get("policy", "B"),
        "scenario": cfg.get("scenario", {}),
        "simulation": cfg.get("simulation", {}),
        "cmg": cfg.get("cmg", {}),
        "control": cfg.get("control", {}),
        "allocation": cfg.get("allocation", {}),
        "maneuver": (cfg.get("scenario", {}) or {}).get("maneuver", {}),
        "search": cfg.get("search", {}),
        "tol_deg": float(cfg.get("metrics", {}).get("tol_deg", 0.5)),
    }
    return RunResult(name=name, metrics=metrics, log=log, meta=meta)
