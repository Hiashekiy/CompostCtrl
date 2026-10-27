"""§5.2 feasibility check for the high-altitude attitude manoeuvre.

    python scripts/check_maneuver_feasibility.py [--heights 90 100 115]

The quintic reference of §5.3 demands a known peak rate, peak acceleration and
half-cycle angular impulse.  Before any A/B run we compare that demand with

    A : the aero control moment available at the manoeuvre altitude
    B : the same aero moment plus the CMG torque box, and the impulse the
        controller momentum budget can actually supply

This is a *model-based admissibility* check, not a simulation: it fixes the
candidate amplitudes / times so that the experiment measures a task that is
physically meaningful instead of an impossible one.

Writes results/cmg_ab/maneuver_feasibility.csv and prints a markdown table.
"""
from __future__ import annotations

import argparse
import csv
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
sys.path.insert(0, ROOT)
sys.path.insert(0, HERE)

from nearspace.atmosphere_factory import make_atmosphere          # noqa: E402
from nearspace.model6c import VehicleCMGParams                    # noqa: E402
from nearspace.scenarios import maneuver_feasibility, vertex_time  # noqa: E402
from nearspace.trajectory3 import Trajectory3D                    # noqa: E402
from run_cmg_ab import OUTROOT, build_cfg                         # noqa: E402

AMPS = (10.0, 20.0)
TMS = (2.0, 3.0, 5.0, 8.0, 10.0, 15.0, 20.0, 25.0, 30.0, 40.0, 60.0)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--heights", nargs="*", type=float, default=[90.0, 100.0, 115.0])
    ap.add_argument("--amps", nargs="*", type=float, default=list(AMPS))
    ap.add_argument("--tms", nargs="*", type=float, default=list(TMS))
    a = ap.parse_args(argv)

    cfg = build_cfg("maneuver10", "B")
    atm = make_atmosphere(cfg, cache_dir=os.path.join(OUTROOT, "_cache"),
                          backend=getattr(cfg.environment, "backend", "msis2"))
    prm = VehicleCMGParams.from_config(cfg)
    rows = []
    for h in a.heights:
        cfg2 = build_cfg("maneuver10", "B")
        cfg2.trajectory.amp = h * 1000.0 - float(cfg2.trajectory.h0)
        traj = Trajectory3D(cfg2, atm)
        t0 = vertex_time(cfg2) - 2.0
        for amp in a.amps:
            for Tm in a.tms:
                r = maneuver_feasibility(cfg2, prm, traj, t0, amp, Tm)
                d, ae = r["demand"], r["aero"]
                rows.append(dict(
                    h_km=h, delta_deg=amp, Tm_s=Tm,
                    aero_Mmax_Nm=ae["M_aero_max_Nm"][1],
                    q_inf_Pa=ae["q_inf_Pa"],
                    peak_rate_deg_s=np.rad2deg(d["peak_rate_rad_s"]),
                    peak_accel_deg_s2=np.rad2deg(d["peak_accel_rad_s2"]),
                    peak_torque_Nm=d["peak_pitch_torque_Nm"],
                    half_impulse_Nms=d["impulse_half_Nms"],
                    A_cap_Nm=r["mode_A"]["peak_torque_Nm"],
                    A_imp_Nms=r["mode_A"]["impulse_capacity_Nms"],
                    A_verdict=r["mode_A"]["verdict"],
                    B_cap_Nm=r["mode_B"]["peak_torque_Nm"],
                    B_imp_Nms=r["mode_B"]["impulse_capacity_Nms"],
                    B_verdict=r["mode_B"]["verdict"]))

    path = os.path.join(OUTROOT, "maneuver_feasibility.csv")
    os.makedirs(OUTROOT, exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0]))
        w.writeheader()
        for r in rows:
            w.writerow({k: ("%.4g" % v if isinstance(v, float) else v)
                        for k, v in r.items()})

    for h in a.heights:
        print("\n### h = %.0f km 顶点（t = %.0f s）\n" % (h, vertex_time(cfg) - 2.0))
        print("| Δθ | Tm | 峰值角速度 | 峰值角加速度 | 需求峰值力矩 | 需求半程冲量 | "
              "A 气动上限 | A 判定 | B 力矩上限 | B 冲量上限 | B 判定 |")
        print("|---|---|---|---|---|---|---|---|---|---|---|")
        for r in rows:
            if r["h_km"] != h:
                continue
            print("| %2.0f° | %2.0f s | %6.3f °/s | %7.4f °/s² | %7.2f N·m | "
                  "%7.1f N·m·s | %7.2f N·m | %s | %7.2f N·m | %7.1f N·m·s | %s |"
                  % (r["delta_deg"], r["Tm_s"], r["peak_rate_deg_s"],
                     r["peak_accel_deg_s2"], r["peak_torque_Nm"],
                     r["half_impulse_Nms"], r["A_cap_Nm"], r["A_verdict"],
                     r["B_cap_Nm"], r["B_imp_Nms"], r["B_verdict"]))
    print("\nwrote", path)
    print("\n判定口径：需求 > 85% 能力 -> marginal；需求 > 115% 能力 -> infeasible。")
    print("A 冲量能力 = 气动最大力矩 × Tm/2（气动可以直接与大气交换动量，"
          "不需要储存）；B 额外加上控制器动量预算。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
