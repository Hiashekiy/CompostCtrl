"""Micro-benchmark: identical 10-step warm-started solves, for one code version.

Used only to measure the effect of removing the modulation probe variables on the
NLP size / iteration count / wall clock under identical conditions.

python bench_ocp_modulation.py <results_run_dir>
"""
from __future__ import annotations

import csv
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from nearspace.config import load_config                     # noqa: E402
from nearspace.torque_gap_scenario import Scenario           # noqa: E402
from nearspace.torque_gap_ocp import TorqueGapOCP            # noqa: E402


def main():
    run = Path(sys.argv[1]) if len(sys.argv) > 1 else \
        ROOT / "results" / "torque_gap_unlimited" / "C1_B0_P0"
    with open(run / "timeseries.csv", encoding="utf-8") as fh:
        rows = list(csv.DictReader(fh))
    d = {k: np.array([float(r[k]) for r in rows]) for k in rows[0]
         if k != "status"}
    cfg = load_config(str(ROOT / "configs" / "torque_gap_unlimited" / "cases"
                          / "c1_pitch30.yaml"))
    scn = Scenario(cfg)
    t0 = time.perf_counter()
    ocp = TorqueGapOCP(scn)
    build = time.perf_counter() - t0
    print("vars=%d cons=%d build=%.2fs" % (ocp.n_vars, ocp.n_cons, build))

    # cold start only: the same deterministic NLP each repetition, so the spread
    # reflects IPOPT, not the warm-start drift of a closed loop
    idx = [int(np.argmin(np.abs(d["t"] - tt))) for tt in (100.0, 112.0)]
    for tag, k in zip(("t=100", "t=112"), idx):
        x = np.array([d[k2][k] for k2 in
                      ("q0", "q1", "q2", "q3", "wx", "wy", "wz",
                       "g1", "g2", "g3", "g4")], dtype=float)
        plan = scn.plan(float(d["t"][k]), x)
        nu_prev = np.array([d["nu%d" % (i + 1)][k] for i in range(4)])
        tau_prev = np.array([d["tau_act_x"][k], d["tau_act_y"][k], d["tau_act_z"][k]])
        cpu, its, ok = [], [], []
        for _ in range(10):
            res = ocp.solve(plan, x, nu_prev, tau_prev, None)      # cold start
            cpu.append(res.cpu_s)
            its.append(res.iter_count)
            ok.append(res.ok)
        print("%s cold solves: cpu median=%.3fs min=%.3fs max=%.3fs | "
              "iters median=%.0f | ok=%d/10"
              % (tag, float(np.median(cpu)), float(np.min(cpu)), float(np.max(cpu)),
                 float(np.median(its)), int(sum(ok))))
    return 0


if __name__ == "__main__":
    sys.exit(main())
