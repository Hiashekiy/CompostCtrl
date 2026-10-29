"""方案 §6-C：超高空姿态机动的正式 A/B 运行入口。

    python scripts/run_cmg_maneuver_ab.py --case M1_pitch_5deg M2_pitch_10deg
    python scripts/run_cmg_maneuver_ab.py --config configs/cmg_maneuver/M3_pitch_15deg.yaml
    python scripts/run_cmg_maneuver_ab.py --check            # 只做一致性检查

A/B 只差执行机构（与 C1 正式实验相同的 modes 定义）。运行前写
``results/cmg_maneuver/<case>/task_parity.json``，记录：

* 解析配置的 A/B 差异集合（必须只有执行机构三项）；
* 参考轨迹（五次多项式）在 t0 / t0+Tm/2 / t0+Tm / t0+Tm+5 的取值与导数；
* 外部扰动力矩全程为 0（主机动实验不允许恒定偏置）；
* 初值一致；
* 机动完成判据（1° / 2 s）与评价窗口，运行前固定。
"""
from __future__ import annotations

import argparse
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
from nearspace import sim                                            # noqa: E402
from nearspace.config import dump, read_yaml                          # noqa: E402
from nearspace.model6c import VehicleCMGParams                        # noqa: E402

CONFIG = os.path.join(ROOT, "configs", "cmg_maneuver", "common.yaml")
OUT = os.path.join(ROOT, "results", "cmg_maneuver")
CACHE = os.path.join(ROOT, "results", "cmg_ab", "_cache")

ALLOWED_DIFFS = {"/actuators/mode", "/mpc_cmg/M_cmg_box", "/label",
                 "/scenario/mode_label"}
# 机动完成判据（方案 §5.1，运行前固定，不随结果调整）
COMPLETION_TOL_DEG = 1.0
COMPLETION_HOLD_S = 2.0


def _walk(a, b, path="", out=None):
    out = [] if out is None else out
    for k in sorted(set(a) | set(b)):
        pa, pb = a.get(k, "<missing>"), b.get(k, "<missing>")
        if isinstance(pa, dict) and isinstance(pb, dict):
            _walk(pa, pb, path + "/" + k, out)
        elif pa != pb:
            out.append("%s/%s: %r -> %r" % (path, k, pa, pb))
    return out


def parity(case, config=CONFIG):
    cfgA = ab.build_cfg(case, "A", config)
    cfgB = ab.build_cfg(case, "B", config)
    diffs = _walk(dump(cfgA), dump(cfgB))
    unexpected = [d for d in diffs if d.split(":")[0] not in ALLOWED_DIFFS]

    bA, bB = sim.build(cfgA), sim.build(cfgB)
    traj = bA["traj"]
    T = float(cfgA.simulation.T)
    mv = traj.maneuver
    t0, Tm = mv.t0, mv.Tm
    t = np.arange(0.0, T + 1e-9, float(cfgA.simulation.dt_ctrl))

    def ref_profile(tr):
        return np.array([np.rad2deg(tr.pitch_ref(x)) for x in t])

    refA, refB = ref_profile(traj), ref_profile(bB["traj"])
    md = np.array([traj.disturbance(x, cfgA, bA["prm"].veh) for x in t])
    xA = sim._initial_state(traj, bA["prm"])
    xB = sim._initial_state(bB["traj"], bB["prm"])
    man = refA - np.rad2deg(np.array([traj.gamma(x) for x in t])) - float(
        cfgA.trajectory.alpha_trim_deg)
    i0 = int(round(t0 / 0.2))
    i_mid = int(round((t0 + 0.5 * Tm) / 0.2))
    i_end = int(round((t0 + Tm) / 0.2))
    i_after = int(round((t0 + Tm + 5.0) / 0.2))
    delta_deg = float(np.rad2deg(mv.delta))
    if mv.shape == "step":
        ref_ok = bool(abs(man[i0 - 1]) < 1e-9 and abs(man[i0] - delta_deg) < 1e-6
                      and abs(man[i_after] - delta_deg) < 1e-6)
        ref_vals = [float(man[i0 - 1]), float(man[i0]), float(man[i_after])]
        ref_label = "阶跃参考：t0 前为 0、t0 起为 Δθ 并保持"
    else:
        ref_ok = bool(abs(man[i0]) < 1e-9 and abs(man[i_end] - delta_deg) < 1e-6
                      and abs(man[i_after] - delta_deg) < 1e-6)
        ref_vals = [float(man[i0]), float(man[i_end]), float(man[i_after])]
        ref_label = "机动参考在 t0 处为 0、在 t0+Tm 处达到 Δθ 并保持"
    w_ref0 = float(np.linalg.norm(traj.omega_ref(t0)))
    checks = [
        ("A/B 配置差异仅限执行机构块", not unexpected, unexpected or diffs),
        ("A/B 参考姿态逐点相同", float(np.max(np.abs(refA - refB))) == 0.0,
         float(np.max(np.abs(refA - refB)))),
        ("外部扰动力矩全程为 0", float(np.max(np.abs(md))) == 0.0,
         float(np.max(np.abs(md)))),
        ("A/B 初值相同", bool(np.allclose(xA, xB)), None),
        ("A 组控制器没有 CMG 权限", float(cfgA.mpc_cmg.M_cmg_box) == 0.0,
         float(cfgA.mpc_cmg.M_cmg_box)),
        ("B 组 CMG 参与", float(cfgB.mpc_cmg.M_cmg_box) > 0.0,
         float(cfgB.mpc_cmg.M_cmg_box)),
        (ref_label, ref_ok, ref_vals),
        ("t0 处参考角速度有限（阶跃偏置不进入导数）[rad/s]", bool(np.isfinite(w_ref0)),
         w_ref0),
        ("参考角速度峰值 [deg/s]（阶跃为 inf，见上一条）", True,
         float(np.rad2deg(mv.peak_rate)) if np.isfinite(mv.peak_rate) else "inf (step)"),
        ("机动时间窗 [s]", True, [float(t0), float(t0 + Tm)]),
        ("完成判据 [deg, s]", True, [COMPLETION_TOL_DEG, COMPLETION_HOLD_S]),
    ]
    out = dict(case=case, config=os.path.relpath(config, ROOT),
               mode_A=dict(mode=bA["mode"], M_cmg_box_Nm=float(cfgA.mpc_cmg.M_cmg_box)),
               mode_B=dict(mode=bB["mode"], M_cmg_box_Nm=float(cfgB.mpc_cmg.M_cmg_box)),
               maneuver=dict(axis=mv.axis, t0=float(t0), Tm=float(Tm),
                             delta_deg=float(np.rad2deg(mv.delta)),
                             shape=mv.shape, t_end=float(t0 + Tm)),
               reference=dict(t=t[::10].tolist(), pitch_ref_deg=refA[::10].tolist(),
                              maneuver_deg=man[::10].tolist()),
               initial_state=dict(
                   zip(["q0", "q1", "q2", "q3", "p", "q", "r", "de", "da", "dr",
                        "g1", "g2", "g3", "g4", "gd1", "gd2", "gd3", "gd4"],
                       [float(v) for v in xA])),
               checks=[dict(check=c, ok=bool(ok), value=v) for c, ok, v in checks],
               passed=all(bool(ok) for _, ok, _ in checks))
    return out


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--case", nargs="*", default=None)
    ap.add_argument("--config", default=CONFIG)
    ap.add_argument("--modes", nargs="*", default=["A", "B"])
    ap.add_argument("--tag", default=None)
    ap.add_argument("--check", action="store_true")
    a = ap.parse_args(argv)

    config = os.path.abspath(a.config)
    raw = read_yaml(config)
    cases = a.case or ([raw["selected_case"]] if "selected_case" in raw else None)
    if not cases:
        cases = sorted(ab.load_matrix(config)[1].get("cases", {}))
    os.makedirs(OUT, exist_ok=True)

    ok = True
    for case in cases:
        par = parity(case, config)
        d = os.path.join(OUT, case)
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, "task_parity.json"), "w", encoding="utf-8") as fh:
            json.dump(par, fh, indent=2, ensure_ascii=False)
        for c in par["checks"]:
            print("  %-48s %s  %s" % (c["check"], "PASS" if c["ok"] else "FAIL",
                                      "" if c["value"] is None else c["value"]))
        print("  -> %s : %s" % (case, "PASSED" if par["passed"] else "FAILED"))
        ok = ok and par["passed"]
    if not ok:
        print("PARITY FAILED - refusing to run")
        return 2
    if a.check:
        return 0

    res = []
    for case in cases:
        for mode in a.modes:
            res.append(ab.run_one(case, mode, verbose=True, tag=a.tag or "maneuver",
                                  config_path=config, outroot=OUT, cache_dir=CACHE))
    for case in cases:
        ab.update_tables([r for r in res if r["case"] == case], OUT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
