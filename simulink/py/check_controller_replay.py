#!/usr/bin/env python
"""C2 acceptance: is the server's control law the same as ClosedLoop.run()?

    python simulink/py/check_controller_replay.py --tag A4_descent_replay_S00_with_cmg_P0

The server keeps the controller of the reference model and only replaces the
plant by an external measurement.  So if the *recorded* states of the reference
run are fed back into it one by one, the sequence of commands it produces must
be the recorded command sequence.  Any mismatch here would show up later as a
closed-loop difference, so this test isolates the controller before any MATLAB
is involved.

Writes results/simulink_port/controller_check.json
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "simulink" / "py"))

from ctrl_server import Controller                                   # noqa: E402

DATA = ROOT / "results" / "simulink_port" / "data"
OUT = ROOT / "results" / "simulink_port"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", action="append", default=None)
    ap.add_argument("--max-steps", type=int, default=0, help="0 = all steps")
    args = ap.parse_args()
    tags = args.tag or ["A4_descent_replay_S00_with_cmg_P0",
                        "A4_descent_replay_S00_no_cmg_P0"]
    results = []
    for tag in tags:
        case_json = json.loads((DATA / ("%s_case.json" % tag)).read_text("utf-8"))
        ctrl = Controller(ROOT / case_json["case_file"], plant=case_json["plant"],
                          seed=int(case_json.get("seed", 0)))
        x0 = np.asarray(case_json["x0"], float)
        x_ref = np.asarray(case_json["ref_run"]["x"], float)
        u_ref = np.asarray(case_json["ref_run"]["u"], float)
        t0 = float(case_json["t_start"])
        dt = float(case_json["dt"])
        n = len(x_ref) if args.max_steps <= 0 else min(args.max_steps, len(x_ref))
        du = np.zeros((n, u_ref.shape[1]))
        fb = 0
        for k in range(n):
            # the controller at t_k sees the state at t_k: x0 for k = 1, and the
            # recorded state of the previous interval afterwards
            xk = x0 if k == 0 else x_ref[k - 1]
            out = ctrl.step(t0 + k * dt, xk)
            u = np.asarray(out["u"], float)[:u_ref.shape[1]]
            du[k] = u - u_ref[k]
            fb += int(out["fallback"])
        worst = float(np.max(np.abs(du)))
        rec = dict(tag=tag, steps=int(n), fallback_steps=int(fb),
                   max_abs_du=worst,
                   max_rel_du=worst / max(1.0, float(np.max(np.abs(u_ref[:n])))),
                   per_channel_max=[float(v) for v in np.max(np.abs(du), axis=0)],
                   pass_=bool(worst <= 1e-9))
        results.append(rec)
        print("[C2] %-40s steps=%3d  max|du|=%.3e  fallback=%d  %s"
              % (tag, n, worst, fb, "PASS" if rec["pass_"] else "FAIL"))
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "controller_check.json").write_text(
        json.dumps(results, indent=2) + "\n", encoding="utf-8")
    ok = all(r["pass_"] for r in results)
    print("---- C2 %s ----" % ("PASSED" if ok else "FAILED"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
