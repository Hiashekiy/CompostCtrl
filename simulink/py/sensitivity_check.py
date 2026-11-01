#!/usr/bin/env python
"""Is the closed-loop difference a porting error or the model's own sensitivity?

    python simulink/py/sensitivity_check.py --tag A4_descent_replay_S00_with_cmg_P0

The ported controller is the reference controller (same code), so the only
difference between the Simulink closed loop and the Python one is a ~1e-10 seed:
the plant states agree to that level after every step.  This script seeds the
*same* magnitude into the reference itself -- it perturbs the initial state by
1e-9 rad and re-runs the Python closed loop -- and reports how quickly the
command sequence and the attitude error separate.  If a 1e-9 perturbation
already produces the separation observed in the Simulink run, then the
closed-loop difference is the model's sensitivity (the CMG is driven to
sigma_min ~ 0.6 in this window), not a porting defect.

Writes results/simulink_port/sensitivity_<tag>.json
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from nearspace.config import _to_ns, deep_merge, read_yaml                 # noqa: E402
from nearspace.torque_gap_rti import TorqueGapRTI                          # noqa: E402
from nearspace.torque_gap_scenario import Scenario                         # noqa: E402
from nearspace.torque_gap_sim import ClosedLoop                            # noqa: E402


class _Stub:
    n_vars = 0
    n_cons = 0


def run(case_file, quat_eps=0.0, plant="P0"):
    raw = read_yaml(str(case_file))
    raw = deep_merge(raw, {"torque_gap": {"plant": plant, "seed": 0,
                                          "seed_randomize": False}})
    scn = Scenario(_to_ns(raw))
    ocp = TorqueGapRTI.from_config(scn)
    cl = ClosedLoop(scn, ocp=ocp, plant=plant)
    x0 = cl._init_plant()
    if quat_eps:
        x0 = x0.copy()
        x0[1] += quat_eps
        x0[0:4] /= np.linalg.norm(x0[0:4])
    cl._init_plant = lambda: x0.copy()
    m = cl.run()
    u = np.array([[r["tauc_cmd_x"], r["tauc_cmd_y"], r["tauc_cmd_z"]]
                  + [r.get("nuc%d" % (i + 1), 0.0) for i in range(4)]
                  for r in cl.rows], float)
    e = np.array([r["att_err_deg"] for r in cl.rows], float)
    return u, e, m


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", default="A4_descent_replay_S00_with_cmg_P0")
    ap.add_argument("--eps", type=float, default=1e-9)
    args = ap.parse_args()
    data = json.loads((ROOT / "results" / "simulink_port" / "data" /
                       ("%s_case.json" % args.tag)).read_text("utf-8"))
    case_file = ROOT / data["case_file"]
    u_ref = np.asarray(data["ref_run"]["u"], float)
    e_ref = np.asarray(data["ref_run"]["att_err_deg"], float)

    u0, e0, _ = run(case_file, 0.0)
    u1, e1, m1 = run(case_file, args.eps)
    k = u_ref.shape[1]                 # 3 for the A group, 7 for the B group
    du0 = np.max(np.abs(u0[:, :k] - u_ref), axis=1)
    du1 = np.max(np.abs(u1[:, :k] - u0[:, :k]), axis=1)
    de0 = np.abs(e0 - e_ref)
    de1 = np.abs(e1 - e0)
    k1 = int(np.argmax(du1 > 1e-3)) if np.any(du1 > 1e-3) else -1
    out = dict(tag=args.tag, eps=args.eps,
               replicate_max_du=float(du0.max()),
               replicate_max_d_err_deg=float(de0.max()),
               perturbed_first_separation_step=k1,
               perturbed_first_separation_t=(float(data["t_start"]) + 0.2 * k1
                                             if k1 >= 0 else None),
               perturbed_median_du=float(np.median(du1)),
               perturbed_max_du=float(du1.max()),
               perturbed_max_d_err_deg=float(de1.max()),
               rmse_ref=float(np.sqrt(np.mean(e_ref ** 2))),
               rmse_unperturbed=float(np.sqrt(np.mean(e0 ** 2))),
               rmse_perturbed=float(np.sqrt(np.mean(e1 ** 2))),
               rmse_perturbed_meta=m1["rmse_deg"])
    p = ROOT / "results" / "simulink_port" / ("sensitivity_%s.json" % args.tag)
    p.write_text(json.dumps(out, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(out, indent=2))
    return 0


if __name__ == "__main__":
    sys.exit(main())
