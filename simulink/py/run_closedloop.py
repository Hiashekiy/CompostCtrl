#!/usr/bin/env python
"""Driver for the Simulink closed-loop runs (server lifecycle + MATLAB batch).

    python simulink/py/run_closedloop.py --tag A4_descent_replay_S00_with_cmg_P0
    python simulink/py/run_closedloop.py --all

For each case:
  1. start simulink/py/ctrl_server.py (Python RTI controller) on a free port,
  2. launch MATLAB through simulink/mlab.cmd and run run_case(tag,'closedloop'),
  3. stop the server and collect the comparison metrics.

The Simulink plant, the interface and the metrics all live on the MATLAB side;
this driver only owns the process lifecycle, so that the non-ASCII workspace path
never has to pass through cmd.exe quoting.
"""
from __future__ import annotations

import argparse
import json
import os
import socket
import subprocess
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
DATA = ROOT / "results" / "simulink_port" / "data"
RUNS = ROOT / "results" / "simulink_port" / "runs"
MATLAB_HOME = Path(os.environ.get("MATLAB_HOME", r"D:\Program Files\MATLAB\R2024a"))
MLAB = ROOT / "simulink" / "mlab.cmd"

CASES = [
    ("A4_descent_replay_S00_with_cmg_P0", "with_cmg"),
    ("A4_descent_replay_S00_no_cmg_P0", "no_cmg"),
    ("A4_descent_replay_S03_with_cmg_P0", "with_cmg"),
    ("A4_descent_replay_S03_no_cmg_P0", "no_cmg"),
    ("A1_alpha_p20_with_cmg_P0", "with_cmg"),
    ("A1_alpha_p20_no_cmg_P0", "no_cmg"),
]


def free_port(start=55911):
    p = start
    while p < start + 200:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            try:
                s.bind(("127.0.0.1", p))
                return p
            except OSError:
                p += 1
    raise SystemExit("no free port")


def wait_port(port, timeout=180.0):
    t0 = time.time()
    while time.time() - t0 < timeout:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.settimeout(1.0)
            try:
                s.connect(("127.0.0.1", port))
                s.sendall(b'{"cmd":"ping"}\n')
                data = s.recv(4096)
                if b"ok" in data:
                    return True
            except OSError:
                time.sleep(0.5)
    return False


def run_one(tag, arm, matlab_extra=None):
    case_yaml = ROOT / "configs" / "torque_gap_s08" / "cases" / ("%s.yaml" % (
        tag.rsplit("_P0", 1)[0]))
    if not case_yaml.exists():
        raise SystemExit("case yaml missing: %s" % case_yaml)
    port = free_port()
    RUNS.mkdir(parents=True, exist_ok=True)
    log = RUNS / ("ctrl_server_%s.log" % tag)
    env = dict(os.environ)
    env["PYTHONPATH"] = str(ROOT) + os.pathsep + str(ROOT / ".pydeps")
    env["PYTHONIOENCODING"] = "utf-8"
    srv = subprocess.Popen(
        [sys.executable, str(ROOT / "simulink" / "py" / "ctrl_server.py"),
         "--case", str(case_yaml), "--port", str(port), "--plant", "P0"],
        cwd=str(ROOT), env=env, stdout=open(log, "w", encoding="utf-8"),
        stderr=subprocess.STDOUT)
    try:
        if not wait_port(port):
            raise SystemExit("controller server did not start (see %s)" % log)
        expr = ("addpath('simulink'); run_case('%s','closedloop','Port',%d)"
                % (tag, port))
        env2 = dict(os.environ)
        env2["MATLAB_HOME"] = str(MATLAB_HOME)
        t0 = time.time()
        proc = subprocess.run(["cmd", "/c", str(MLAB), expr],
                              cwd=str(ROOT), env=env2, capture_output=True,
                              text=True, encoding="utf-8", errors="replace",
                              timeout=3600)
        out = (proc.stdout or "") + (proc.stderr or "")
        print(out[-4000:])
        met = RUNS / ("%s_closedloop_metrics.json" % tag)
        rec = json.loads(met.read_text("utf-8")) if met.exists() else None
        return dict(tag=tag, arm=arm, wall_s=time.time() - t0, rc=proc.returncode,
                    metrics=rec)
    finally:
        try:
            srv.terminate()
            srv.wait(timeout=20)
        except Exception:                                     # noqa: BLE001
            srv.kill()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", action="append", default=None)
    ap.add_argument("--all", action="store_true")
    args = ap.parse_args()
    jobs = CASES if args.all else [
        (t, "with_cmg" if "with_cmg" in t else "no_cmg") for t in (args.tag or [
            "A4_descent_replay_S00_with_cmg_P0"])]
    results = []
    for tag, arm in jobs:
        print("=" * 70)
        print("[closedloop] %s" % tag, flush=True)
        try:
            results.append(run_one(tag, arm))
        except SystemExit as exc:
            print("[FAIL] %s: %s" % (tag, exc))
            results.append(dict(tag=tag, error=str(exc)))
    RUNS.mkdir(parents=True, exist_ok=True)
    (RUNS / "closedloop_summary.json").write_text(
        json.dumps(results, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print("=" * 70)
    for r in results:
        m = r.get("metrics") or {}
        if m:
            print("%-40s state_rel=%.2e  err_max=%.2e deg  C3=%s" % (
                r["tag"], m.get("state_max_rel_diff", float("nan")),
                m.get("att_err_max_abs_diff_deg", float("nan")),
                "PASS" if m.get("C3_state_pass") and m.get("C3_err_pass") else "FAIL"))
        else:
            print("%-40s %s" % (r["tag"], r.get("error", "no metrics")))
    return 0


if __name__ == "__main__":
    sys.exit(main())
