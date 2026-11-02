#!/usr/bin/env python
"""Export the dataset for the *visible* block-diagram Simulink model.

    python simulink/py/export_visible_model.py --tag A4_descent_replay_S00_with_cmg_P0

The S-function model (`s08_plant_sfun`) keeps the state inside M-code.  That is
fine for equivalence testing but it is not a model anybody can read.  The visible
model needs the same physics *as blocks*:

    Clock -> 飞行条件(Lookup) -> 气动/CMG/刚体姿态(Integrator, Sum, Gain, Cross)
          <- 执行机构(Saturation, Rate Limiter) <- 控制器(discrete 0.2 s)

so it needs the flight condition and the external torque on the *solver* grid
(0.005 s), not on the 0.2 s control grid.  Everything is sampled once here from
the frozen Python scenario, so the blocks never re-implement the MSIS2
atmosphere:

    t_fine, rho(t), V_ned(t)   -> 攻角/动压 lookups
    tau_d(t)                   -> 外部力矩
    u_k (0.2 s grid)           -> 控制器回放表
    params                     -> J, 气动系数, CMG 几何, 限幅

Output: results/simulink_port/data/<tag>_visible.json
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from nearspace.config import _to_ns, deep_merge, read_yaml            # noqa: E402
from nearspace.torque_gap_scenario import Scenario                    # noqa: E402
from nearspace.torque_gap_sim import ClosedLoop                       # noqa: E402

DATA = ROOT / "results" / "simulink_port" / "data"
CASES = {
    "A4_descent_replay_S00_with_cmg_P0": "A4_descent_replay_S00_with_cmg",
    "A4_descent_replay_S00_no_cmg_P0": "A4_descent_replay_S00_no_cmg",
    "A4_descent_replay_S03_with_cmg_P0": "A4_descent_replay_S03_with_cmg",
    "A1_alpha_p20_with_cmg_P0": "A1_alpha_p20_with_cmg",
}


class _Stub:
    n_vars = 0
    n_cons = 0


def build(tag, dt_fine=0.005):
    case = CASES[tag]
    path = ROOT / "configs" / "torque_gap_s08" / "cases" / ("%s.yaml" % case)
    raw = read_yaml(str(path))
    raw = deep_merge(raw, {"torque_gap": {"plant": "P0", "seed": 0,
                                          "seed_randomize": False}})
    cfg = _to_ns(raw)
    scn = Scenario(cfg)
    scn.case = case
    cl = ClosedLoop(scn, ocp=_Stub(), plant="P0")
    veh = scn.veh

    t0, t1 = float(scn.t_start), float(scn.t_end)
    n_fine = int(round((t1 - t0) / dt_fine))
    t_fine = t0 + dt_fine * np.arange(n_fine + 1)

    rho = np.array([scn.traj.rho(t) for t in t_fine])
    V_ned = np.array([scn.traj.V_ned(t) for t in t_fine])
    tau_d = np.array([cl._tau_d(t) for t in t_fine])
    q_ref = np.array([scn.traj.q_ref(t) for t in t_fine])
    t_ctrl = t0 + float(cl.dt) * np.arange(int(cl.n_steps))
    q_ref_c = np.array([scn.traj.q_ref(t) for t in t_ctrl])
    w_ref_c = np.array([scn.traj.omega_ref(t) for t in t_ctrl])

    rec = CASES[tag]
    u = []
    run = ROOT / "results" / "torque_gap_s08" / (
        "A4_descent_replay" if case.startswith("A4") else "A1_alpha") / case
    import csv
    with open(run / "timeseries.csv", encoding="utf-8", newline="") as fh:
        rows = list(csv.DictReader(fh))
    for r in rows:
        cmd = [float(r["tauc_cmd_%s" % ax]) for ax in "xyz"]
        if scn.cmg_present:
            cmd += [float(r["nuc%d" % (i + 1)]) for i in range(4)]
        u.append(cmd)

    payload = dict(
        tag=tag, case=case, nx=int(scn.nx), nu=int(scn.nu),
        cmg_present=bool(scn.cmg_present),
        t_start=t0, t_end=t1, dt_fine=float(dt_fine), dt_ctrl=float(cl.dt),
        n_ctrl=int(cl.n_steps),
        J=[float(veh.Jx), float(veh.Jy), float(veh.Jz)],
        S=float(veh.S), b=float(veh.b), cbar=float(veh.cbar),
        Cm0=float(veh.Cm0), Cm_alpha=float(veh.Cm_alpha), Cm_q=float(veh.Cm_q),
        Cm_de=float(veh.Cm_de),
        delta_max=[float(v) for v in veh.delta_max],
        gdot_max=float(scn.prm.cmg.gdot_max),
        gddot_max=float(scn.prm.cmg.gddot_max),
        cmg=None if not scn.cmg_present else dict(
            h0=float(scn.prm.cmg.h0), g_axis=scn.cluster.g_axis.tolist(),
            s0=scn.cluster.s0.tolist(), gxs0=scn.cluster.gxs0.tolist()),
        x0=[float(v) for v in cl._init_plant()],
        t_fine=t_fine.tolist(), rho=rho.tolist(), V_ned=V_ned.tolist(),
        tau_d=tau_d.tolist(), q_ref=q_ref.tolist(),
        t_ctrl=t_ctrl.tolist(), q_ref_ctrl=q_ref_c.tolist(),
        omega_ref_ctrl=w_ref_c.tolist(), u=u,
        ref_att_err_deg=np.array([float(r["att_err_deg"]) for r in rows]).tolist(),
    )
    DATA.mkdir(parents=True, exist_ok=True)
    out = DATA / ("%s_visible.json" % tag)
    out.write_text(json.dumps(payload) + "\n", encoding="utf-8")
    return out, n_fine


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", action="append", default=None)
    args = ap.parse_args()
    tags = args.tag or list(CASES)
    for tag in tags:
        out, n = build(tag)
        print("[visible] %-42s %d samples -> %s" % (tag, n + 1, out.name))
    return 0


if __name__ == "__main__":
    sys.exit(main())
