#!/usr/bin/env python
"""TCP controller server for the Simulink closed-loop port.

    python simulink/py/ctrl_server.py --case configs/.../A4_descent_replay_S00_with_cmg.yaml \
                                      --port 55901 --plant P0

The server owns *exactly* the Python controller of the reference model: the same
Scenario, the same TorqueGapRTI solver, the same acceptance/fallback policy and
the same warm start.  The plant stays in Simulink, so each request hands one
measured state over and gets one control action back.

Protocol (newline-delimited JSON, one request -> one response)
    {"cmd":"ping"}                     -> {"ok":true}
    {"cmd":"reset"}                    -> {"ok":true}
    {"cmd":"quit"}                     -> {"ok":true} (then the server exits)
    {"cmd":"step","t":90.2,"x":[...]}  -> {"u":[...7...],"status":"...",
                                           "fallback":0,"iter":3,"cpu_s":0.08}

The per-step body is a line-by-line copy of
nearspace.torque_gap_sim.ClosedLoop.run() with the plant step replaced by the
external measurement, so that Simulink and Python run the identical control law.
"""
from __future__ import annotations

import argparse
import json
import socket
import sys
import time
from pathlib import Path

import numpy as np

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))

from nearspace.config import _to_ns, deep_merge, read_yaml               # noqa: E402
from nearspace.torque_gap_rti import TorqueGapRTI                        # noqa: E402
from nearspace.torque_gap_scenario import Scenario                       # noqa: E402
from nearspace.torque_gap_sim import ClosedLoop                          # noqa: E402


class Controller:
    """The control law of ClosedLoop.run(), driven from outside."""

    def __init__(self, case_path, plant="P0", seed=0):
        raw = read_yaml(str(case_path))
        raw = deep_merge(raw, {"torque_gap": {"plant": plant, "seed": int(seed),
                                              "seed_randomize": False}})
        self.cfg = _to_ns(raw)
        self.scn = Scenario(self.cfg)
        self.scn.case = Path(case_path).stem
        self.ocp = TorqueGapRTI.from_config(self.scn)
        self.cl = ClosedLoop(self.scn, ocp=self.ocp, plant=plant)
        self.solver_cfg = self.cl.solver_cfg
        self.dt = float(self.scn.dt_ctrl)
        self.reset()

    # -- state kept across steps, exactly like ClosedLoop.run() -------------
    def reset(self):
        self.nu_prev = np.zeros(4)
        self.tau_prev = np.zeros(3)
        self.warm = None
        self._last_action = None
        self.steps = 0

    def step(self, t, x):
        scn, ocp = self.scn, self.ocp
        x = np.asarray(x, float).ravel()
        t = float(t)
        x_opt = self.cl._state_opt(x)
        if scn.cmg_present and not scn.gamma_unbounded:
            x_opt = x_opt.copy()
            x_opt[7:11] = np.clip(x_opt[7:11], scn.gamma_lim[0], scn.gamma_lim[1])
        plan = scn.plan(t, x_opt)
        res = ocp.solve(plan, x_opt, self.nu_prev, self.tau_prev, self.warm)

        fallback = 0
        accepted = 0
        feasible = (res.max_violation <= self.solver_cfg.accept_feasible_viol
                    and np.isfinite(res.objective))
        in_time = res.cpu_s <= self.solver_cfg.max_cpu_s
        ok = bool(res.ok and feasible and in_time)
        if not ok and self.solver_cfg.accept_feasible and feasible and in_time:
            ok = True
            accepted = 1
        if ok:
            u_cmd = res.u0.copy()
            status = res.status if not accepted else "%s|accepted" % res.status
            self.warm = (res.X, res.U)
        else:
            u_cmd, why = self.cl._fallback(plan, x_opt, self.nu_prev, self.tau_prev)
            status = "%s|%s" % (res.status, why)
            fallback = 1
            if self.solver_cfg.warm_shift_on_fallback and self.warm is not None:
                self.warm = self.cl._shift_warm(self.warm)

        # execution-side actuator limits, identical to ClosedLoop.run()
        lo, hi = plan["box"]["tau_min"][0], plan["box"]["tau_max"][0]
        tau_a_cmd = np.clip(u_cmd[0:3], lo, hi)
        gdot = float(scn.prm.cmg.gdot_max)
        gddot = float(scn.prm.cmg.gddot_max)
        if scn.cmg_present and scn.cmg_enabled:
            nu_cmd = np.clip(u_cmd[3:7], -gdot, gdot)
            nu_cmd = np.clip(nu_cmd, self.nu_prev - gddot * self.dt,
                             self.nu_prev + gddot * self.dt)
        else:
            nu_cmd = np.zeros(4)

        self._last_action = np.concatenate([tau_a_cmd, nu_cmd])
        self.nu_prev = np.asarray(nu_cmd, float)      # P0: achieved == commanded
        self.tau_prev = np.asarray(tau_a_cmd, float)
        self.steps += 1
        return dict(u=[float(v) for v in np.concatenate([tau_a_cmd, nu_cmd])],
                    status=str(status), fallback=int(fallback),
                    accepted=int(accepted), iter=int(res.iter_count),
                    cpu_s=float(res.cpu_s), viol=float(res.max_violation))


def serve(ctrl, port, timeout_s=600.0):
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("127.0.0.1", port))
    srv.listen(4)
    srv.settimeout(timeout_s)
    print("[ctrl] listening on %d" % port, flush=True)
    t_end = time.time() + timeout_s
    try:
        while time.time() < t_end:
            conn, _addr = srv.accept()
            conn.settimeout(120.0)
            fin = conn.makefile("rwb")
            while True:
                line = fin.readline()
                if not line:
                    break
                try:
                    req = json.loads(line.decode("utf-8"))
                except json.JSONDecodeError:
                    fin.write(b'{"error":"bad json"}\n'); fin.flush(); continue
                cmd = req.get("cmd", "step")
                if cmd == "ping":
                    resp = {"ok": True}
                elif cmd == "reset":
                    ctrl.reset(); resp = {"ok": True}
                elif cmd == "quit":
                    fin.write(b'{"ok":true}\n'); fin.flush()
                    print("[ctrl] quit", flush=True)
                    conn.close(); srv.close(); return
                elif cmd == "step":
                    try:
                        resp = ctrl.step(req["t"], req["x"])
                    except Exception as exc:                       # noqa: BLE001
                        resp = {"error": "%s: %s" % (type(exc).__name__, exc)}
                else:
                    resp = {"error": "unknown cmd"}
                fin.write((json.dumps(resp) + "\n").encode("utf-8"))
                fin.flush()
            conn.close()
    finally:
        try:
            srv.close()
        except OSError:
            pass


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--case", required=True, help="A/B case yaml of the run")
    ap.add_argument("--port", type=int, default=55901)
    ap.add_argument("--plant", default="P0", choices=["P0", "P1"])
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--timeout", type=float, default=1800.0)
    args = ap.parse_args()
    ctrl = Controller(Path(args.case), plant=args.plant, seed=args.seed)
    print("[ctrl] case=%s plant=%s nx=%d nu=%d steps=%d"
          % (args.case, args.plant, ctrl.scn.nx, ctrl.scn.nu,
             int(round((ctrl.scn.t_end - ctrl.scn.t_start) / ctrl.dt))), flush=True)
    serve(ctrl, args.port, args.timeout)
    return 0


if __name__ == "__main__":
    sys.exit(main())
