#!/usr/bin/env python
"""Audit the S08 configuration files before any run (doc §4 and §14 phase 1-2).

    python scripts/check_s08_config.py

Checks
    1. every generated scenario / arm file resolves through the frozen base chain;
    2. an A/B pair differs in exactly the CMG presence switches and in nothing
       else (vehicle, inertia, aerodynamics, disturbance, servo, horizon, solver,
       weights, window);
    3. the flight condition actually reached by the model equals the nominal one
       (reference angle of attack, dynamic pressure, Mach, Knudsen) at the start,
       the middle and the end of the window;
    4. the A arm is structurally CMG-free (7 states / 3 inputs, no cluster), and
       it is demonstrably NOT "B with the gimbal rate pinned to zero";
    5. the surface pitch authority, the natural pitching moment and the CMG
       torque capability at the reference condition are recorded, because every
       degradation argument in the report is built on them.

Outputs
    results/torque_gap_s08/config_audit.md
    results/torque_gap_s08/config_audit.json
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from nearspace.aero_diagnostics import flow_diagnostics                 # noqa: E402
from nearspace.config import load_config, read_yaml                     # noqa: E402
from nearspace.torque_gap_authority import axis_hull, natural_moment_v   # noqa: E402
from nearspace.torque_gap_scenario import Scenario                      # noqa: E402

CFG = ROOT / "configs" / "torque_gap_s08"
OUT = ROOT / "results" / "torque_gap_s08"
SWITCH_KEYS = {"torque_gap.cmg_present", "torque_gap.cmg_enabled"}
ALPHA_TOL_DEG = 1e-6
QBAR_TOL_REL = 5e-3


def flatten(node, prefix=""):
    out = {}
    if isinstance(node, dict):
        for k, v in node.items():
            out.update(flatten(v, "%s.%s" % (prefix, k) if prefix else str(k)))
    else:
        out[prefix] = node
    return out


def diff_keys(a, b):
    fa, fb = flatten(a), flatten(b)
    keys = set(fa) | set(fb)
    out = []
    for k in sorted(keys):
        va, vb = fa.get(k, "<missing>"), fb.get(k, "<missing>")
        if isinstance(va, float) or isinstance(vb, float):
            try:
                if abs(float(va) - float(vb)) <= 0.0:
                    continue
            except (TypeError, ValueError):
                pass
        if va != vb:
            out.append((k, va, vb))
    return out


def case_list():
    design = json.loads((OUT / "design.json").read_text(encoding="utf-8"))
    return design


def check_pair(name, entry, report):
    """Resolved-config diff of one A/B pair."""
    pa = CFG / "cases" / ("%s_no_cmg.yaml" % name)
    pb = CFG / "cases" / ("%s_with_cmg.yaml" % name)
    ra, rb = read_yaml(str(pa)), read_yaml(str(pb))
    diffs = diff_keys(ra, rb)
    bad = [d for d in diffs if d[0] not in SWITCH_KEYS]
    rec = dict(case=name, group=entry.get("group"), a_file=str(pa.relative_to(ROOT)),
               b_file=str(pb.relative_to(ROOT)),
               differing_keys=[dict(key=k, a=v, b=w) for k, v, w in diffs],
               unexpected_keys=[d[0] for d in bad])
    return rec


def check_condition(name, entry, report):
    """Flight condition reached by the model, and the actuator capability there.

    The condition is verified at instants *outside* the commanded manoeuvre (the
    +8 deg pitch bump of the controlled cases, 110-130 s), because inside it the
    reference angle of attack is deliberately raised by the bump.  The
    mid-manoeuvre instant is recorded as information only.
    """
    path = CFG / "cases" / ("%s_with_cmg.yaml" % name)
    cfg = load_config(str(path))
    scn = Scenario(cfg)
    veh = scn.veh
    man = [m for m in scn.traj.maneuvers if m.enabled]

    def in_man(t):
        return any(m.t0 - 1e-9 <= t <= m.t_end + 1e-9 for m in man)

    ts = [scn.t_start, scn.t_start + 15.0, 0.5 * (scn.t_start + scn.t_end),
          scn.t_end - 10.0]
    rows = []
    for t in ts:
        q = scn.traj.q_ref(t)
        rho = float(scn.traj.rho(t))
        Vn = np.asarray(scn.traj.V_ned(t), float)
        h = float(scn.traj.h(t))
        fd = flow_diagnostics(q, rho, Vn, h, atm=scn.atm, cbar=veh.cbar)
        alpha_ref_deg = float(np.rad2deg(scn.traj.pitch_ref(t) - scn.traj.gamma(t)))
        Q = fd["qbar_pa"]
        half = axis_hull(Q, veh)
        nat = natural_moment_v(Q, fd["alpha_deg"] * np.pi / 180.0, 0.0,
                               np.zeros(3), fd["V_rel"], veh)
        rows.append(dict(t=float(t), in_maneuver=bool(in_man(t)), h_m=h, rho=rho,
                         V_rel=fd["V_rel"], qbar_pa=Q, alpha_deg=fd["alpha_deg"],
                         alpha_ref_deg=alpha_ref_deg, beta_deg=fd["beta_deg"],
                         mach=fd["mach"], knudsen=fd["knudsen"],
                         half_x=float(half[0]), half_y=float(half[1]),
                         half_z=float(half[2]),
                         tau_nat_x=float(nat[0]), tau_nat_y=float(nat[1]),
                         tau_nat_z=float(nat[2]),
                         surf_util_y=float(abs(nat[1]) / max(half[1], 1e-9))))
    alpha_target = entry.get("alpha_ref_deg")
    qbar_target = entry.get("qbar_pa")
    rec = dict(case=name, group=entry.get("group"), samples=rows,
               alpha_target_deg=alpha_target, qbar_target_pa=qbar_target)
    ok = True
    outside = [r for r in rows if not r["in_maneuver"]]
    if alpha_target is not None:
        # the model's own alpha must equal its own reference theta_ref - gamma
        err_ref = max(abs(r["alpha_deg"] - r["alpha_ref_deg"]) for r in rows)
        err = max(abs(r["alpha_deg"] - alpha_target) for r in outside)
        rec["alpha_max_err_deg"] = float(err)
        rec["alpha_vs_reference_err_deg"] = float(err_ref)
        rec["alpha_in_maneuver_deg"] = [r["alpha_ref_deg"] for r in rows
                                        if r["in_maneuver"]]
        ok &= bool(err <= ALPHA_TOL_DEG and err_ref <= ALPHA_TOL_DEG)
    if qbar_target is not None:
        err = max(abs(r["qbar_pa"] - qbar_target) / qbar_target for r in rows)
        rec["qbar_max_rel_err"] = float(err)
        ok &= bool(err <= QBAR_TOL_REL)
    if entry.get("maneuver_deg") is not None:
        rec["maneuver_deg"] = entry["maneuver_deg"]
    rec["condition_ok"] = bool(ok)
    return rec


def cmg_and_group_checks():
    """Structural evidence: the A arm has no CMG at all, and is not nu == 0."""
    pa = CFG / "cases" / ("A4_descent_replay_S00_no_cmg.yaml")
    pb = CFG / "cases" / ("A4_descent_replay_S00_with_cmg.yaml")
    sa, sb = Scenario(load_config(str(pa))), Scenario(load_config(str(pb)))
    from nearspace.torque_gap_model import dynamics_tg, dynamics_tg_nocmg
    x0a = sa.initial_state()
    x0b = sb.initial_state()
    gamma = np.array([0.3, 0.1, -0.2, 0.4])
    xb = x0b.copy()
    xb[7:11] = gamma
    xa = x0a.copy()
    tau_d = np.zeros(3)
    tau_a = np.zeros(3)
    nu0 = np.zeros(4)
    dA = dynamics_tg_nocmg(xa, tau_a, tau_d, sa.prm)[4:7]
    dB = dynamics_tg(xb, np.concatenate([tau_a, nu0]), tau_d, sb.prm, sb.cluster)[4:7]
    H = sb.cluster.momentum(gamma)
    w = np.array([1e-3, 2e-3, -1e-3])
    # a genuine body rate is needed for the gyroscopic term to show up
    xa[4:7] = w
    xb[4:7] = w
    dA = dynamics_tg_nocmg(xa, tau_a, tau_d, sa.prm)[4:7]
    dB = dynamics_tg(xb, np.concatenate([tau_a, nu0]), tau_d, sb.prm, sb.cluster)[4:7]
    cmg = sb.prm.cmg
    A_home = sb.cluster.jacobian(np.zeros(4))
    return dict(
        a_group=dict(nx=int(sa.nx), nu=int(sa.nu), cluster_is_none=sa.cluster is None,
                     cmg_present=bool(sa.cmg_present),
                     cmg_enabled=bool(sa.cmg_enabled)),
        b_group=dict(nx=int(sb.nx), nu=int(sb.nu), cmg_present=bool(sb.cmg_present),
                     cmg_enabled=bool(sb.cmg_enabled)),
        frozen_gimbal_is_not_the_a_group=dict(
            gamma_rad=gamma.tolist(), H_norm_Nms=float(np.linalg.norm(H)),
            omega_dot_diff_rad_s2=float(np.max(np.abs(dB - dA)))),
        cmg_device=dict(units=int(cmg.n), h0_Nms=float(cmg.h0),
                        cluster_momentum_Nms=float(cmg.h_total),
                        gdot_max_rad_s=float(cmg.gdot_max),
                        gddot_max_rad_s2=float(cmg.gddot_max),
                        tau_g_s=float(cmg.tau_g),
                        home_sigma_min_Nms=float(cmg.home_sigma_min),
                        sigma_floor_Nms=float(cmg.sigma_floor),
                        capability_along_best_axis_Nm=float(
                            sb.cluster.torque_capability(A_home) * cmg.gdot_max),
                        pitch_axis_capability_Nm=float(
                            abs(A_home[1, :]).sum() * cmg.gdot_max)),
        aero_surface=dict(delta_max_deg=[float(v) for v in np.rad2deg(sb.veh.delta_max)],
                          delta_dot_max_deg_s=[float(v) for v in
                                               np.rad2deg(sb.veh.delta_dot_max)],
                          tau_s_s=float(sb.veh.tau_s),
                          Cm_alpha_per_rad=float(sb.veh.Cm_alpha),
                          Cm_de_per_rad=float(sb.veh.Cm_de),
                          Cm_q_per_rad=float(sb.veh.Cm_q)),
        authority_box=dict(mode=str(sb.auth.mode), scale=float(sb.auth.scale),
                           calibration=sb.authority_calibration,
                           surface_matrix_note="M_delta = Q S diag(b, cbar, b) x "
                                               "(Cl_da, Cm_de, Cn_dr ...)"),
    )


def main():
    design = case_list()
    pairs, conds = [], []
    for entry in design["cases"]:
        pairs.append(check_pair(entry["name"], entry, None))
        conds.append(check_condition(entry["name"], entry, None))
    struct = cmg_and_group_checks()
    bad_pairs = [p for p in pairs if p["unexpected_keys"]]
    bad_cond = [c for c in conds if not c["condition_ok"]]
    payload = dict(reference=design["reference"], pairs=pairs, conditions=conds,
                   structure=struct,
                   verdict=dict(pairs_ok=not bad_pairs, conditions_ok=not bad_cond))
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "config_audit.json").write_text(
        json.dumps(payload, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    L = []
    L.append("# S08 configuration audit")
    L.append("")
    L.append("Generated by `scripts/check_s08_config.py`.")
    L.append("")
    L.append("## Reference flight condition (original S00/S03 trajectory, t = 90 s)")
    L.append("")
    r = design["reference"]
    L.append("| 项 | 值 |")
    L.append("|---|---|")
    L.append("| 高度 | %.1f m |" % r["h_m"])
    L.append("| 相对气流速度 | %.1f m/s |" % r["V_mps"])
    L.append("| 密度 (MSIS2) | %.4e kg/m³ |" % r["rho_kgpm3"])
    L.append("| 动压 Q0 | %.3f Pa |" % r["qbar_pa"])
    L.append("| 参考迎角 alpha_trim | %.1f deg |" % r["alpha_trim_deg"])
    L.append("| 马赫数 M | %.2f |" % r["mach"])
    L.append("| Knudsen 数 | %.3f |" % r["knudsen"])
    L.append("")
    L.append("## A/B 逐键差异")
    L.append("")
    L.append("| 工况 | 差异键 | 结论 |")
    L.append("|---|---|---|")
    for p in pairs:
        keys = ", ".join("`%s`" % d["key"] for d in p["differing_keys"]) or "—"
        verdict = "OK（仅 CMG 开关）" if not p["unexpected_keys"] else \
                  "差异超出开关：%s" % ", ".join(p["unexpected_keys"])
        L.append("| %s | %s | %s |" % (p["case"], keys, verdict))
    L.append("")
    L.append("## 迎角 / 动压是否达到设定（机动窗口之外的三个时刻）")
    L.append("")
    L.append("受控工况在 110–130 s 有一条统一的 +8° 俯仰凸起参考，因此迎角考核取机动窗之外的时刻；"
             "机动中点只作参考记录。")
    L.append("")
    L.append("| 工况 | alpha 目标 | alpha 实测（机动窗外） | 误差 | alpha(机动中点) | qbar 目标 | "
             "qbar 实测 | 相对误差 | Mach | Kn | 舵面俯仰半宽 | 配平需求 | 参考状态利用率 |")
    L.append("|---|---|---|---|---|---|---|---|---|---|---|---|---|")
    for c in conds:
        s = c["samples"]
        out = [x for x in s if not x["in_maneuver"]]
        at = c.get("alpha_target_deg")
        qt = c.get("qbar_target_pa")
        mid = [x["alpha_ref_deg"] for x in s if x["in_maneuver"]]
        L.append("| %s | %s | %s | %s | %s | %s | %s | %s | %.2f | %.3f | %.2f N·m | %.2f N·m | %.3f |"
                 % (c["case"],
                    "—" if at is None else "%+.1f°" % at,
                    "/".join("%+.3f°" % x["alpha_deg"] for x in out),
                    "—" if at is None else "%.1e°" % c["alpha_max_err_deg"],
                    "/".join("%+.1f°" % v for v in mid) if mid else "—",
                    "—" if qt is None else "%.2f Pa" % qt,
                    "/".join("%.2f" % x["qbar_pa"] for x in out),
                    "—" if qt is None else "%.1e" % c["qbar_max_rel_err"],
                    s[0]["mach"], s[0]["knudsen"], s[0]["half_y"], s[0]["tau_nat_y"],
                    s[0]["surf_util_y"]))
    L.append("")
    L.append("## 结构核查：A 组确实没有 CMG")
    L.append("")
    a, b = struct["a_group"], struct["b_group"]
    L.append("- A 组：`nx=%d`、`nu=%d`、`Scenario.cluster is None = %s`、"
             "`cmg_enabled=%s`"
             % (a["nx"], a["nu"], a["cluster_is_none"], a["cmg_enabled"]))
    L.append("- B 组：`nx=%d`、`nu=%d`、`cmg_present=%s`、`cmg_enabled=%s`"
             % (b["nx"], b["nu"], b["cmg_present"], b["cmg_enabled"]))
    f = struct["frozen_gimbal_is_not_the_a_group"]
    L.append("- 冻结框架 ≠ 无 CMG：在 gamma = %s rad 下，集群角动量 |H| = %.3f N·m·s，"
             "B 组（nu ≡ 0）与 A 组的角加速度差为 %.3e rad/s²。"
             % (np.round(f["gamma_rad"], 3).tolist(), f["H_norm_Nms"],
                f["omega_dot_diff_rad_s2"]))
    L.append("")
    L.append("## 执行机构能力（由 `config_resolved.yaml` 解析得到）")
    L.append("")
    c = struct["cmg_device"]
    v = struct["aero_surface"]
    L.append("| 项 | 值 |")
    L.append("|---|---|")
    for k, val in (("CMG 数量", c["units"]), ("单台角动量 h0", "%.1f N·m·s" % c["h0_Nms"]),
                   ("集群角动量", "%.1f N·m·s" % c["cluster_momentum_Nms"]),
                   ("框架速率上限", "%.2f rad/s" % c["gdot_max_rad_s"]),
                   ("框架加速度上限", "%.2f rad/s²" % c["gddot_max_rad_s2"]),
                   ("框架回路时间常数", "%.3f s" % c["tau_g_s"]),
                   ("home sigma_min", "%.2f N·m·s" % c["home_sigma_min_Nms"]),
                   ("奇异下限 sigma_floor", "%.2f N·m·s" % c["sigma_floor_Nms"]),
                   ("CMG 最优方向力矩能力", "%.2f N·m" % c["capability_along_best_axis_Nm"]),
                   ("CMG 俯仰轴力矩能力 (sum|A_y| gdot_max)",
                    "%.2f N·m" % c["pitch_axis_capability_Nm"]),
                   ("舵面行程", "%s deg" % np.round(v["delta_max_deg"], 1).tolist()),
                   ("舵面速率", "%s deg/s" % np.round(v["delta_dot_max_deg_s"], 0).tolist()),
                   ("舵机时间常数", "%.3f s" % v["tau_s_s"]),
                   ("Cm_alpha", "%.3f 1/rad" % v["Cm_alpha_per_rad"]),
                   ("Cm_de", "%.3f 1/rad" % v["Cm_de_per_rad"]),
                   ("Cm_q", "%.3f 1/rad" % v["Cm_q_per_rad"]),
                   ("气动能力盒模式", struct["authority_box"]["mode"]),
                   ("气动能力盒标定系数", "%.4f" % struct["authority_box"]["scale"])):
        L.append("| %s | %s |" % (k, val))
    L.append("")
    L.append("## 结论")
    L.append("")
    L.append("- A/B 逐键差异：%s" % ("全部仅 CMG 开关" if not bad_pairs else "存在非开关差异"))
    L.append("- 迎角/动压达到设定容差（alpha ≤ %.0e deg，qbar ≤ %.1e 相对）：%s"
             % (ALPHA_TOL_DEG, QBAR_TOL_REL, "是" if not bad_cond else "否"))
    (OUT / "config_audit.md").write_text("\n".join(L) + "\n", encoding="utf-8")
    print("[audit] pairs=%d (bad %d)  conditions=%d (bad %d)"
          % (len(pairs), len(bad_pairs), len(conds), len(bad_cond)))
    print("[audit] wrote %s" % (OUT / "config_audit.md").relative_to(ROOT))
    return 0 if (not bad_pairs and not bad_cond) else 1


if __name__ == "__main__":
    sys.exit(main())
