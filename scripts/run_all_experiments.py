"""End-to-end reproduction of the whole A/B study (implementation plan §11).

    python scripts/run_all_experiments.py              # everything
    python scripts/run_all_experiments.py --skip-checks  # only the runs

Order (do not change it — §11 asks for the frozen order):
    1. §7.1/§7.2  CMG weight + torque-chain verification
    2. §5.2       manoeuvre feasibility check
    3. §4/§5/§6   all A/B closed-loop runs (parallel processes)
    4. §10        metrics, improvement rates, §10.4 verdict
    5. §9         figures
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)


def call(args, title):
    print("\n" + "#" * 78)
    print("# " + title)
    print("#" * 78, flush=True)
    t0 = time.time()
    rc = subprocess.call([sys.executable] + args, cwd=ROOT)
    print("# -> rc=%d  (%.0f s)" % (rc, time.time() - t0), flush=True)
    return rc


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--jobs", type=int, default=5)
    ap.add_argument("--skip-checks", action="store_true")
    ap.add_argument("--cases", nargs="*", default=None)
    a = ap.parse_args(argv)

    rc = 0
    if not a.skip_checks:
        rc |= call([os.path.join("scripts", "verify_cmg_weights.py")],
                   "1. CMG weight / torque-chain verification (§7.1, §7.2)")
        rc |= call([os.path.join("scripts", "check_maneuver_feasibility.py")],
                   "2. Manoeuvre feasibility check (§5.2)")

    args = [os.path.join("scripts", "run_cmg_ab.py"), "--jobs", str(a.jobs)]
    if a.cases:
        args += ["--case"] + list(a.cases)
    else:
        args += ["--all"]
    rc |= call(args, "3. A/B closed-loop runs (§4, §5, §6)")

    rc |= call([os.path.join("scripts", "evaluate_cmg_ab.py")],
               "4. Metrics, improvement rates and §10.4 verdict")
    rc |= call([os.path.join("scripts", "plot_cmg_ab.py")],
               "5. Figures (§9)")
    print("\nall stages finished, worst rc=%d" % rc)
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
