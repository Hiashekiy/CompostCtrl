"""Phase 5: the formal A/B run of the frozen C1 task on the matched three-axis
model (``configs/c1_cmg_reproduction.yaml``).

    python scripts/run_c1_cmg_ab.py --check      # task/fairness parity only
    python scripts/run_c1_cmg_ab.py --modes A B  # the two formal runs (~2-5 min each)

The task, the reference, the disturbance, the initial state, the time steps, the
actuator limits and the scoring constants are identical in A and B; the only
intentional difference is the actuator set.  ``--check`` writes
``results/c1_cmg_reproduction/cmg/task_parity.json`` *before* any simulation is
started, so the parity claim is not derived from the results afterwards.
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

import run_cmg_ab as base                                        # noqa: E402
from nearspace.config import dump                                # noqa: E402
from nearspace.metrics import SCORING                            # noqa: E402
from nearspace.model6c import VehicleCMGParams                    # noqa: E402
from nearspace import sim                                        # noqa: E402

CONFIG = os.path.join(ROOT, "configs", "c1_cmg_reproduction.yaml")
OUT = os.path.join(ROOT, "results", "c1_cmg_reproduction", "cmg")
CACHE = os.path.join(ROOT, "results", "cmg_ab", "_cache")
CASE = "c1"

ALLOWED_DIFFS = {"/actuators/mode", "/mpc_cmg/M_cmg_box", "/label",
                 "/scenario/mode_label"}


def _fields(obj):
    """Plain, comparable snapshot of a (possibly array-carrying) dataclass."""
    out = {}
    for k, v in vars(obj).items():
        out[k] = np.asarray(v).tolist() if isinstance(v, np.ndarray) else v
    return out


def _walk(a, b, path="", out=None):
    out = [] if out is None else out
    for k in sorted(set(a) | set(b)):
        pa, pb = a.get(k, "<missing>"), b.get(k, "<missing>")
        if isinstance(pa, dict) and isinstance(pb, dict):
            _walk(pa, pb, path + "/" + k, out)
        elif pa != pb:
            out.append("%s/%s: %r -> %r" % (path, k, pa, pb))
    return out


def parity():
    cfgA = base.build_cfg(CASE, "A", CONFIG)
    cfgB = base.build_cfg(CASE, "B", CONFIG)
    dA, dB = dump(cfgA), dump(cfgB)
    diffs = _walk(dA, dB)
    unexpected = [d for d in diffs if d.split(":")[0] not in ALLOWED_DIFFS]

    bA, bB = sim.build(cfgA), sim.build(cfgB)
    traj = bA["traj"]
    t = np.arange(0.0, float(cfgA.simulation.T) + 1e-9, 0.2)

    # 1. the disturbance is the legacy C1 pitch profile (20 + 15 sin(2pi 0.05 t))
    legacy = 20.0 + 15.0 * np.sin(2.0 * np.pi * 0.05 * t)
    mdA = np.array([traj.disturbance(x, cfgA, bA["prm"].veh)[1] for x in t])
    mdB = np.array([traj.disturbance(x, cfgB, bB["prm"].veh)[1] for x in t])
    roll = np.array([traj.disturbance(x, cfgA, bA["prm"].veh)[0] for x in t])
    yaw = np.array([traj.disturbance(x, cfgA, bA["prm"].veh)[2] for x in t])

    # 2. the reference: gamma + alpha_trim + 5 deg raised cosine at 105 s / 10 s
    th_ref = np.array([np.rad2deg(traj.pitch_ref(x)) for x in t])
    man = th_ref - np.rad2deg(np.array([traj.gamma(x) for x in t])) - float(
        cfgA.trajectory.alpha_trim_deg)
    i105 = int(round(105.0 / 0.2))
    i110 = int(round(110.0 / 0.2))
    i115 = int(round(115.0 / 0.2))

    # 3. the high-altitude window is fixed by the trajectory, identical in A and B
    winA = traj.windows(float(cfgA.simulation.T)).get("high")
    winB = bB["traj"].windows(float(cfgB.simulation.T)).get("high")
    frozen = None
    mp = os.path.join(ROOT, "results", "c1_cmg_reproduction", "baseline", "meta.json")
    if os.path.exists(mp):
        with open(mp, encoding="utf-8") as fh:
            frozen = json.load(fh).get("frozen_profile", {}).get("high_altitude_window_s")

    # 4. identical initial state, identical time steps / limits / scoring
    xA = sim._initial_state(traj, bA["prm"])
    xB = sim._initial_state(bB["traj"], bB["prm"])
    checks = [
        ("config diff limited to the actuator block", not unexpected, unexpected or diffs),
        ("disturbance identical in A and B", float(np.max(np.abs(mdA - mdB))) == 0.0,
         float(np.max(np.abs(mdA - mdB)))),
        ("pitch disturbance equals legacy 20+15sin(2pi0.05t)",
         float(np.max(np.abs(mdA - legacy))) < 1e-12, float(np.max(np.abs(mdA - legacy)))),
        ("roll/yaw disturbance are exactly zero",
         bool(np.max(np.abs(roll)) == 0.0 and np.max(np.abs(yaw)) == 0.0),
         [float(np.max(np.abs(roll))), float(np.max(np.abs(yaw)))]),
        ("pitch reference pulse = 5 deg raised cosine at 105 s / 10 s",
         bool(abs(man[i105]) < 1e-12 and abs(man[i110] - 5.0) < 1e-6 and abs(man[i115]) < 1e-6),
         [float(man[i105]), float(man[i110]), float(man[i115])]),
        ("high-altitude window identical in A and B", winA == winB, [winA, winB]),
        ("high-altitude window matches the frozen legacy profile",
         frozen is None or [float(winA[0]), float(winA[1])] == [float(frozen[0]), float(frozen[1])],
         [winA, frozen]),
        ("identical initial state", bool(np.allclose(xA, xB)), None),
        ("identical time steps",
         (float(cfgA.simulation.dt_ctrl), float(cfgA.simulation.dt_plant),
          float(cfgA.simulation.T)) == (float(cfgB.simulation.dt_ctrl),
                                        float(cfgB.simulation.dt_plant),
                                        float(cfgB.simulation.T)),
         [float(cfgA.simulation.dt_ctrl), float(cfgA.simulation.dt_plant),
          float(cfgA.simulation.T)]),
        ("identical vehicle / aero / servo parameters",
         _fields(bA["prm"].veh) == _fields(bB["prm"].veh), None),
        ("identical CMG hardware",
         _fields(bA["prm"].cmg) == _fields(bB["prm"].cmg), None),
        ("identical scoring constants", SCORING == SCORING, SCORING),
    ]
    out = dict(
        case=CASE, config=os.path.relpath(CONFIG, ROOT),
        mode_A=dict(mode=bA["mode"], M_cmg_box_Nm=float(cfgA.mpc_cmg.M_cmg_box)),
        mode_B=dict(mode=bB["mode"], M_cmg_box_Nm=float(cfgB.mpc_cmg.M_cmg_box)),
        config_differences=diffs, unexpected_differences=unexpected,
        high_window_s=winA, initial_state=dict(
            zipped=dict(zip(["q0", "q1", "q2", "q3", "p", "q", "r", "de", "da", "dr",
                             "g1", "g2", "g3", "g4", "gd1", "gd2", "gd3", "gd4"],
                            [float(v) for v in xA]))),
        disturbance_pitch=dict(min=float(mdA.min()), max=float(mdA.max()),
                               t_of_max=float(t[int(np.argmax(mdA))]),
                               impulse_over_run_Nms=float(np.sum(mdA) * 0.2)),
        checks=[dict(check=c, ok=bool(ok), value=v) for c, ok, v in checks],
        passed=all(bool(ok) for _, ok, _ in checks),
    )
    return out


def main(argv=None):
    global CASE
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--modes", nargs="*", default=["A", "B"])
    ap.add_argument("--check", action="store_true", help="parity checks only")
    ap.add_argument("--case", default=CASE, help="case block in the experiment YAML")
    ap.add_argument("--tag", default=None)
    a = ap.parse_args(argv)
    CASE = a.case

    os.makedirs(OUT, exist_ok=True)
    par = parity()
    name = "task_parity.json" if CASE == "c1" else "task_parity_%s.json" % CASE
    with open(os.path.join(OUT, name), "w", encoding="utf-8") as fh:
        json.dump(par, fh, indent=2, ensure_ascii=False)
    for c in par["checks"]:
        print("  %-52s %s  %s" % (c["check"], "PASS" if c["ok"] else "FAIL",
                                  "" if c["value"] is None else c["value"]))
    print("  pitch disturbance impulse over the run: %.0f N·m·s"
          % par["disturbance_pitch"]["impulse_over_run_Nms"])
    if not par["passed"]:
        print("PARITY FAILED - not running the formal A/B")
        return 2
    if a.check:
        return 0

    ok = []
    for mode in a.modes:
        r = base.run_one(CASE, mode, verbose=True, tag=a.tag, config_path=CONFIG,
                         outroot=OUT, cache_dir=CACHE)
        ok.append(r)
    base.update_tables(ok, OUT)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
