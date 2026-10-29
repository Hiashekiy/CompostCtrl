"""方案 §4：机动可行性扫描（正式 A/B 之前必须完成）。

    python scripts/validate_maneuver_feasibility.py                     # 默认扫描
    python scripts/validate_maneuver_feasibility.py --amps 5 10 15 --tms 10 20 30 40
    python scripts/validate_maneuver_feasibility.py --heights 90 100 110

对每个 (高度, 机动幅度 Δθ, 机动时间 Tm) 候选，分别检查方案 §4 的六项，并且
**明确区分**三种"不可行"：

    reference   参考轨迹本身：峰值角速度 / 角加速度 / C² 连续性
    actuator    执行机构物理能力：所需峰值力矩 vs 气动力矩上限、舵偏角与速率、
                CMG 力矩上限 / 半程角冲量 vs 角动量预算
    controller  MPC 在真实约束下的一次求解（机动最吃力时刻的参考状态）

判定口径（运行前固定）：需求 > 115 % 能力 → infeasible；> 85 % → marginal；
否则 feasible。控制器探针单独报告，不并入执行机构判定。

输出 results/cmg_maneuver/maneuver_feasibility.csv 与同名 .md，并在最后给出
**预先写死的选型规则**：每个机动幅度取"B 组判定为 feasible 的最小 Tm"，
A 组在同一 Tm 下的判定只记录、不参与选择（避免为让 A 失败或让 B 好看而挑 Tm）。
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
for p in (ROOT, HERE):
    if p not in sys.path:
        sys.path.insert(0, p)

import run_cmg_ab as ab                                             # noqa: E402
from nearspace.cmg_control import LTVMPCCMG2                          # noqa: E402
from nearspace.cmg import CMGCluster                                  # noqa: E402
from nearspace.model6c import VehicleCMGParams                        # noqa: E402
from nearspace.scenarios import (maneuver_demand, maneuver_feasibility,
                                 vertex_time)                          # noqa: E402
from nearspace.trajectory3 import Maneuver, Trajectory3D              # noqa: E402
from nearspace import sim                                             # noqa: E402

CONFIG = os.path.join(ROOT, "configs", "cmg_maneuver", "common.yaml")
BASE_CASE = "M2_pitch_10deg"
OUT = os.path.join(ROOT, "results", "cmg_maneuver")
DT = 0.2
T0_OFFSET = 2.0            # 机动在顶点前 2 s 触发（t = 118 s）
PEAK_RATE_LIMIT_DEG_S = 10.0     # 参考合理性阈值（运行前固定）
PEAK_ACCEL_LIMIT_DEG_S2 = 1.0
MARGINAL, INFEASIBLE = 1.15, 0.85   # 需求/能力 判定口径


def _cfg_for(amp, Tm, h_km):
    cfg = ab.build_cfg(BASE_CASE, "B", CONFIG)
    cfg.trajectory.amp = h_km * 1000.0 - float(cfg.trajectory.h0)
    cfg.trajectory3.maneuver = []
    return cfg


def _reference_metrics(cfg, t0, amp, Tm):
    """Numeric peak rate / accel of the quintic reference, and C² continuity."""
    mv = Maneuver("pitch", t0=t0, duration=Tm, delta_deg=amp, shape="hold")
    t = np.arange(t0 - 1.0, t0 + Tm + 1.0, DT)
    y = np.array([mv.value(x) for x in t])
    rate = np.gradient(y, DT)
    accel = np.gradient(rate, DT)
    jerk = np.gradient(accel, DT)
    return dict(
        peak_rate_deg_s=float(np.rad2deg(np.max(np.abs(rate)))),
        peak_accel_deg_s2=float(np.rad2deg(np.max(np.abs(accel)))),
        peak_jerk_deg_s3=float(np.rad2deg(np.max(np.abs(jerk)))),
        value_start_deg=float(np.rad2deg(mv.value(t0))),
        value_end_deg=float(np.rad2deg(mv.value(t0 + Tm))),
        value_after_deg=float(np.rad2deg(mv.value(t0 + Tm + 5.0))),
        analytic_peak_rate_deg_s=float(np.rad2deg(mv.peak_rate)),
        analytic_peak_accel_deg_s2=float(np.rad2deg(mv.peak_accel)))


def _actuator_checks(cfg, prm, traj, t0, amp, Tm):
    v = prm.veh
    dem = maneuver_demand(prm, amp, Tm)
    Q = traj.qinf(t0)
    M_aero_max = Q * v.S * v.cbar * abs(v.Cm_de)          # 满偏气动力矩（俯仰）
    M_req = dem["peak_pitch_torque_Nm"]
    delta_req = float(np.rad2deg(M_req / max(M_aero_max, 1e-12)))
    cl = CMGCluster(prm.cmg)
    A_home = cl.jacobian(np.zeros(prm.cmg.n))
    # 俯仰轴的 CMG 力矩上限（box-LP：沿 +y 的最大可交付力矩）与所需框架角速度
    cap_per_unit = float(np.sum(np.abs(A_home[1, :])))
    M_cmg_max = cap_per_unit * prm.cmg.gdot_max
    gdot_req = M_req / max(cap_per_unit, 1e-12)
    H_budget = float(cfg.mpc_cmg.H_lim_frac) * prm.cmg.h_total
    box = float(cfg.mpc_cmg.M_cmg_box)
    return dict(
        q_inf_Pa=float(Q), aero_Mmax_Nm=float(M_aero_max),
        demand_peak_torque_Nm=float(M_req),
        demand_half_impulse_Nms=float(dem["impulse_half_Nms"]),
        delta_required_deg=delta_req,
        delta_limit_deg=float(np.rad2deg(v.delta_max[1])),
        delta_rate_needed_deg_s=float(delta_req / max(Tm / 2.0, 1e-9)),
        delta_rate_limit_deg_s=float(np.rad2deg(v.delta_dot_max[1])),
        cmg_Mmax_pitch_Nm=M_cmg_max, cmg_box_Nm=box,
        cmg_gdot_needed_rad_s=float(gdot_req),
        cmg_gdot_limit_rad_s=float(prm.cmg.gdot_max),
        H_budget_Nms=H_budget,
        H_demand_over_budget=float(dem["impulse_half_Nms"] / max(H_budget, 1e-9)))


def _controller_probe(cfg, mode, t_probe, amp, Tm):
    """One MPC solve at the most demanding instant of the reference."""
    cfg2 = ab.build_cfg(BASE_CASE, mode, CONFIG)
    cfg2.trajectory.amp = cfg.trajectory.amp
    cfg2.simulation.T = float(cfg.simulation.T)
    cfg2 = _with_maneuver(cfg2, amp, Tm)
    b = sim.build(cfg2, cache_dir=os.path.join(OUT, "_cache"))
    traj, prm = b["traj"], b["prm"]
    ctrl = LTVMPCCMG2(prm, cfg2, b["cluster"])
    x = sim._initial_state(traj, prm)
    x[0:4] = traj.q_ref(t_probe)
    x[4:7] = traj.omega_ref(t_probe)
    x[10:18] = 0.0
    u, info = ctrl.solve(x, t_probe, traj)
    return dict(mode=mode, t=t_probe,
                err_at_probe_deg=float(np.rad2deg(
                    np.linalg.norm(ctrl._att_error(traj.q_ref(t_probe), x[0:4])))),
                fallback=int(info.get("fallback", 0)),
                qp_status=str(info.get("qp_status", "")),
                qp_iter=int(info.get("qp_iter", 0)),
                violation=float(info.get("viol", 0.0)),
                solve_ms=1e3 * float(info.get("solve_time", 0.0)),
                delta_cmd_deg=[float(np.rad2deg(v)) for v in u[0:3]],
                M_cmg_cmd_Nm=[float(v) for v in u[3:6]])


def _with_maneuver(cfg, amp, Tm):
    cfg.trajectory3.maneuver = [dict(axis="pitch", start=118.0, duration=float(Tm),
                                     delta_deg=float(amp), shape="hold")]
    from nearspace.config import _to_ns
    cfg.trajectory3 = _to_ns(dict(
        **{k: v for k, v in vars(cfg.trajectory3).items() if k != "maneuver"},
        maneuver=cfg.trajectory3.maneuver))
    return cfg


def step_feasibility(amps, heights, cache_dir):
    """阶跃参考：由执行机构力矩/惯量给出的最快到位时间（乐观下界）。

    阶跃参考在 t0 处不可导，方案 §4 中"参考角速度/角加速度连续"这一项不再适用；
    换成更有意义的物理问题：**以满偏气动权限（以及 aero+CMG 力矩盒）做 bang-bang
    加速-减速，把一个 Δθ 的姿态阶跃搬完需要多久**。

        α = M_max / J_y,   T = 2 sqrt(Δθ/α),   ω_peak = α T / 2

    这个下界忽略了 Cm_q 阻尼、配平攻角与气动权限随高度/动压的变化，因此只是**乐观下界**：
    闭环实际完成时间只会更长。
    """
    rows = []
    for h in heights:
        atm_cfg = ab.build_cfg(BASE_CASE, "B", CONFIG)
        atm_cfg.trajectory.amp = h * 1000.0 - float(atm_cfg.trajectory.h0)
        sb = sim.build(atm_cfg, cache_dir=cache_dir)
        traj, prm = sb["traj"], sb["prm"]
        t0 = vertex_time(atm_cfg) - T0_OFFSET
        Q = float(traj.qinf(t0))
        v = prm.veh
        M_aero = Q * v.S * v.cbar * abs(v.Cm_de)
        Jy = float(v.Jy)
        box = float(atm_cfg.mpc_cmg.M_cmg_box) * float(atm_cfg.mpc_cmg.torque_margin)
        H_budget = float(atm_cfg.mpc_cmg.H_lim_frac) * prm.cmg.h_total
        for amp in amps:
            th = np.deg2rad(abs(float(amp)))
            aA = M_aero / Jy
            aB = (M_aero + box) / Jy
            TA = 2.0 * np.sqrt(th / aA)
            TB = 2.0 * np.sqrt(th / aB)
            wA = aA * TA / 2.0
            wB = aB * TB / 2.0
            rows.append(dict(
                h_km=float(h), delta_deg=float(amp), t0_s=float(t0), q_inf_Pa=Q,
                aero_Mmax_Nm=float(M_aero), cmg_box_Nm=float(box),
                alpha_A_deg_s2=float(np.rad2deg(aA)), T_min_A_s=float(TA),
                alpha_B_deg_s2=float(np.rad2deg(aB)), T_min_B_s=float(TB),
                omega_peak_A_deg_s=float(np.rad2deg(wA)),
                omega_peak_B_deg_s=float(np.rad2deg(wB)),
                J_omega_peak_B_Nms=float(Jy * wB), H_budget_Nms=float(H_budget),
                cmg_storage_frac=float(Jy * wB / H_budget)))
    return rows


def write_step_section(rows, out_dir):
    path = os.path.join(out_dir, "maneuver_step_feasibility.csv")
    if rows:
        keys = list(rows[0].keys())
        with open(path, "w", newline="", encoding="utf-8") as fh:
            w = csv.DictWriter(fh, fieldnames=keys)
            w.writeheader()
            for r in rows:
                w.writerow({k: ("%.6g" % v if isinstance(v, float) else v)
                            for k, v in r.items()})
    L = ["# 阶跃参考的可行性（乐观下界）", "",
         "α = M_max/J_y，T = 2√(Δθ/α)，ω_peak = αT/2。忽略 Cm_q 阻尼与配平攻角，"
         "因此闭环实际完成时间只会更长。", "",
         "| h [km] | Δθ [°] | 气动上限 [N·m] | α_A [°/s²] | T_min_A [s] | α_B [°/s²] | "
         "T_min_B [s] | ω_peak_A [°/s] | ω_peak_B [°/s] | J·ω_B / 预算 |",
         "|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|"]
    for r in rows:
        L.append("| %.0f | %.0f | %.2f | %.3f | %.2f | %.3f | %.2f | %.2f | %.2f | %.2f |"
                 % (r["h_km"], r["delta_deg"], r["aero_Mmax_Nm"], r["alpha_A_deg_s2"],
                    r["T_min_A_s"], r["alpha_B_deg_s2"], r["T_min_B_s"],
                    r["omega_peak_A_deg_s"], r["omega_peak_B_deg_s"],
                    r["cmg_storage_frac"]))
    with open(os.path.join(out_dir, "maneuver_step_feasibility.md"), "w", encoding="utf-8") as fh:
        fh.write("\n".join(L) + "\n")
    return path


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--amps", nargs="*", type=float, default=[5.0, 10.0, 15.0])
    ap.add_argument("--tms", nargs="*", type=float, default=[10.0, 20.0, 30.0, 40.0])
    ap.add_argument("--heights", nargs="*", type=float, default=[90.0, 100.0, 110.0])
    ap.add_argument("--probe", action="store_true", default=True)
    ap.add_argument("--no-probe", dest="probe", action="store_false")
    ap.add_argument("--step-only", action="store_true",
                    help="只算阶跃参考的最快到位下界（不跑五次多项式扫描）")
    a = ap.parse_args(argv)
    os.makedirs(OUT, exist_ok=True)

    if a.step_only:
        cache = os.path.join(OUT, "_cache")
        rows = step_feasibility(a.amps, a.heights, cache)
        p = write_step_section(rows, OUT)
        for r in rows:
            print("  h=%3.0f km Δ=%4.1f°  T_min_A=%.2f s  T_min_B=%.2f s  "
                  "ω_peak_B=%.2f °/s  J·ω_B/预算=%.2f"
                  % (r["h_km"], r["delta_deg"], r["T_min_A_s"], r["T_min_B_s"],
                     r["omega_peak_B_deg_s"], r["cmg_storage_frac"]))
        print("wrote", p)
        return 0

    rows = []
    cache_dir = os.path.join(OUT, "_cache")
    for h in a.heights:
        for amp in a.amps:
            for Tm in a.tms:
                # 110 km 只扫 5°/10°，控制组合数
                if h != 100.0 and amp > 10.0:
                    continue
                cfg = _cfg_for(amp, Tm, h)
                atm_cfg = ab.build_cfg(BASE_CASE, "B", CONFIG)
                atm_cfg.trajectory.amp = h * 1000.0 - float(atm_cfg.trajectory.h0)
                simb = sim.build(atm_cfg, cache_dir=cache_dir)
                traj, prm = simb["traj"], simb["prm"]
                t0 = vertex_time(atm_cfg) - T0_OFFSET
                ref = _reference_metrics(cfg, t0, amp, Tm)
                act = _actuator_checks(cfg, prm, traj, t0, amp, Tm)
                fz = maneuver_feasibility(cfg, prm, traj, t0, amp, Tm)
                row = dict(h_km=h, delta_deg=amp, Tm_s=Tm, t0_s=t0, **ref, **act,
                           A_verdict=fz["mode_A"]["verdict"],
                           B_verdict=fz["mode_B"]["verdict"],
                           A_cap_Nm=fz["mode_A"]["peak_torque_Nm"],
                           B_cap_Nm=fz["mode_B"]["peak_torque_Nm"])
                row["reference_ok"] = bool(
                    ref["peak_rate_deg_s"] <= PEAK_RATE_LIMIT_DEG_S
                    and ref["peak_accel_deg_s2"] <= PEAK_ACCEL_LIMIT_DEG_S2
                    and abs(ref["value_start_deg"]) < 1e-9
                    and abs(ref["value_end_deg"] - amp) < 1e-6
                    and abs(ref["value_after_deg"] - amp) < 1e-6)
                row["delta_ok"] = bool(act["delta_required_deg"] <= act["delta_limit_deg"])
                row["delta_rate_ok"] = bool(
                    act["delta_rate_needed_deg_s"] <= act["delta_rate_limit_deg_s"])
                row["cmg_torque_ok"] = bool(act["demand_peak_torque_Nm"]
                                            <= act["cmg_Mmax_pitch_Nm"])
                row["cmg_budget_ok"] = bool(act["H_demand_over_budget"] <= 1.0)
                rows.append(row)
                print("  h=%3.0f km Δ=%4.1f° Tm=%4.1f s  ref=%s  A=%-10s B=%-10s"
                      % (h, amp, Tm, "ok" if row["reference_ok"] else "BAD",
                         row["A_verdict"], row["B_verdict"]), flush=True)

    # ---- 控制器探针：只看 100 km 主工况 ---------------------------------
    probes = []
    if a.probe:
        for amp in a.amps:
            for Tm in a.tms:
                cfg = _cfg_for(amp, Tm, 100.0)
                atm_cfg = ab.build_cfg(BASE_CASE, "B", CONFIG)
                atm_cfg.trajectory.amp = 70000.0
                sb = sim.build(atm_cfg, cache_dir=cache_dir)
                t0 = vertex_time(atm_cfg) - T0_OFFSET
                # 最吃力时刻：五次多项式的峰值角加速度出现在 τ≈0.211/0.789
                t_probe = t0 + 0.211 * Tm
                for mode in ("A", "B"):
                    pr = _controller_probe(cfg, mode, t_probe, amp, Tm)
                    probes.append(dict(h_km=100.0, delta_deg=amp, Tm_s=Tm, **pr))
                    print("  probe Δ=%4.1f° Tm=%4.1f s %s: %s fb=%d viol=%.2e %.0f ms"
                          % (amp, Tm, mode, pr["qp_status"], pr["fallback"],
                             pr["violation"], pr["solve_ms"]), flush=True)

    # ---- 选型规则（运行前固定） ------------------------------------------
    selection = {}
    for amp in a.amps:
        cand = [r for r in rows if r["h_km"] == 100.0 and r["delta_deg"] == amp
                and r["reference_ok"] and r["B_verdict"] == "feasible"]
        cand.sort(key=lambda r: r["Tm_s"])
        selection["%gdeg" % amp] = dict(
            Tm_s=cand[0]["Tm_s"] if cand else None,
            A_verdict=cand[0]["A_verdict"] if cand else None,
            B_verdict=cand[0]["B_verdict"] if cand else None,
            rule="smallest Tm with an admissible reference and B verdict = feasible")

    path = os.path.join(OUT, "maneuver_feasibility.csv")
    keys = list(rows[0].keys())
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=keys)
        w.writeheader()
        for r in rows:
            w.writerow({k: ("%.6g" % v if isinstance(v, float) else v) for k, v in r.items()})
    with open(os.path.join(OUT, "maneuver_probes.csv"), "w", newline="", encoding="utf-8") as fh:
        pk = list(probes[0].keys()) if probes else ["mode"]
        w = csv.DictWriter(fh, fieldnames=pk)
        w.writeheader()
        for p in probes:
            w.writerow({k: ("%.6g" % v if isinstance(v, float)
                            else (json.dumps(v) if isinstance(v, list) else v))
                        for k, v in p.items()})
    with open(os.path.join(OUT, "maneuver_selection.json"), "w", encoding="utf-8") as fh:
        json.dump(dict(selection=selection, thresholds=dict(
            peak_rate_limit_deg_s=PEAK_RATE_LIMIT_DEG_S,
            peak_accel_limit_deg_s2=PEAK_ACCEL_LIMIT_DEG_S2,
            marginal_ratio=MARGINAL, infeasible_ratio=INFEASIBLE)), fh,
            indent=2, ensure_ascii=False)

    # ---- markdown --------------------------------------------------------
    L = ["# 机动可行性扫描（方案 §4）", "",
         "判定口径（运行前固定）：需求 > 115 %% 能力 → infeasible；> 85 %% → marginal；"
         "否则 feasible。参考合理性阈值：峰值角速度 ≤ %.0f °/s、峰值角加速度 ≤ %.1f °/s²。"
         % (PEAK_RATE_LIMIT_DEG_S, PEAK_ACCEL_LIMIT_DEG_S2), "",
         "| h [km] | Δθ [°] | Tm [s] | 峰值角速度 [°/s] | 峰值角加速度 [°/s²] | "
         "需求峰值力矩 [N·m] | 气动上限 [N·m] | 需用舵偏 [°] | 需用舵速 [°/s] | "
         "CMG 力矩上限 [N·m] | 半程冲量/预算 | A | B |",
         "|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|"]
    for r in rows:
        L.append("| %.0f | %.0f | %.0f | %.3f | %.4f | %.2f | %.2f | %.2f | %.1f | %.1f | "
                 "%.2f | %s | %s |"
                 % (r["h_km"], r["delta_deg"], r["Tm_s"], r["peak_rate_deg_s"],
                    r["peak_accel_deg_s2"], r["demand_peak_torque_Nm"],
                    r["aero_Mmax_Nm"], r["delta_required_deg"],
                    r["delta_rate_needed_deg_s"], r["cmg_Mmax_pitch_Nm"],
                    r["H_demand_over_budget"], r["A_verdict"], r["B_verdict"]))
    L += ["", "## MPC 控制器探针（100 km，机动峰值角加速度时刻，单次冷启动求解）", "",
          "| Δθ | Tm | 组 | 状态 | 回退 | 违反量 | 迭代 | 求解 [ms] | M_cmg 指令 [N·m] |",
          "|---:|---:|---|---|---|---:|---:|---:|---|"]
    for p in probes:
        L.append("| %.0f | %.0f | %s | %s | %d | %.2e | %d | %.0f | %s |"
                 % (p["delta_deg"], p["Tm_s"], p["mode"], p["qp_status"], p["fallback"],
                    p["violation"], p["qp_iter"], p["solve_ms"],
                    json.dumps([round(v, 2) for v in p["M_cmg_cmd_Nm"]])))
    L += ["", "## 选型（规则在运行前写死，不按结果调整）", "",
          "规则：`smallest Tm with an admissible reference and B verdict = feasible`；"
          "A 组在同一 Tm 下的判定只记录、不参与选择。", "",
          "| 幅度 | 选定 Tm [s] | A 判定 | B 判定 |", "|---|---:|---|---|"]
    for k, v in selection.items():
        L.append("| %s | %s | %s | %s |"
                 % (k, v["Tm_s"], v["A_verdict"], v["B_verdict"]))
    with open(os.path.join(OUT, "maneuver_feasibility.md"), "w", encoding="utf-8") as fh:
        fh.write("\n".join(L) + "\n")
    print("\nselection:", json.dumps(selection, ensure_ascii=False))
    print("wrote", path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
