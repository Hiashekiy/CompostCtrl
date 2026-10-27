"""Aero-only vs aero+CMG under a transient 80 N.m disturbance at 100 km.

    python run_cmg_compare.py <A|B> [T]
    A = aero surfaces only (CMG disabled)
    B = aero surfaces + CMG (hierarchical unified MPC + momentum manager)
Both cases use the *same* controller and the same planner; only the actuator
set differs, so the comparison isolates the CMG contribution.
"""
from __future__ import annotations

import csv
import os
import sys

import numpy as np

from nearspace.atmosphere_factory import make_atmosphere
from nearspace.cmg_control import LTVMPCCMG2
from legacy.cmg_hier import CMGHierController
from nearspace.config import load_config
from nearspace.model6c import VehicleCMGParams, make_cluster, plant_step6c
from nearspace.trajectory3 import Trajectory3D

HERE = os.path.dirname(os.path.abspath(__file__))
OUT = os.path.join(HERE, "results")




class _Hier:
    def __init__(self, c):
        self.c = c
    def _att_error(self, qr, q):
        return self.c.planner._attitude_error(qr, q)
    def act(self, x, t, traj):
        return self.c.act(x, t, traj)


class PDCompare:
    """Classical PD + (optional) CMG steering law + momentum manager.

    Case A: CMG box = 0  -> everything on the aero surfaces (same PD).
    Case B: CMG takes the transient (up to 61 N.m), aero takes the residual and
            unloads the CMG momentum.  Identical gains in both cases, so the
            difference isolates the actuator.
    """

    def __init__(self, prm, cfg, cl, cmg_on=True):
        from nearspace.cmg_control import steering_law
        self.prm, self.cfg, self.cluster = prm, cfg, cl
        self.steering_law = steering_law
        self.cmg_on = cmg_on
        self.M_box = float(cfg.mpc_cmg.M_cmg_box) if cmg_on else 0.0
        self.H_lim = float(cfg.mpc_cmg.H_lim_frac) * prm.cmg.h_total
        self.k_att, self.k_rate, self.k_unload = 4.0, 4.0, 0.05
        self.gamma_home = np.zeros(prm.cmg.n)

    def _att_error(self, qr, q):
        from nearspace.model6 import quat_conj, quat_mul
        dq = quat_mul(quat_conj(qr), q)
        if dq[0] < 0.0:
            dq = -dq
        return 2.0 * dq[1:4]

    def act(self, x, t, traj):
        v = self.prm.veh
        e = self._att_error(traj.q_ref(t), x[0:4])
        w_err = x[4:7] - traj.omega_ref(t)
        M_des = np.diag([v.Jx, v.Jy, v.Jz]) @ (self.k_att * e + self.k_rate * w_err)
        if self.cmg_on:
            M_cmd = np.clip(M_des, -self.M_box, self.M_box)
            H = self.cluster.momentum(x[10:14])
            nH = float(np.linalg.norm(H))
            if nH > self.H_lim and nH > 1e-9:
                u_hat = H / nH
                comp = float(np.dot(M_cmd, u_hat))
                if comp < 0.0:
                    M_cmd = M_cmd - comp * u_hat
            gdot = self.steering_law(self.cluster, x[10:14], M_cmd, self.gamma_home)
            M_cmg_actual = -self.cluster.jacobian(x[10:14]) @ gdot
            M_res = M_des - M_cmg_actual + self.k_unload * H
        else:
            gdot = np.zeros(self.prm.cmg.n)
            M_res = M_des
        Q = 0.5 * traj.rho(t) * traj.V(t) ** 2
        alpha_ref = traj.pitch_ref(t) - traj.gamma(t)
        cap = Q * v.S * np.array([v.b * v.Cl_da, v.cbar * v.Cm_de, v.b * v.Cn_dr])
        rest = np.array([0.0, Q * v.S * v.cbar * (v.Cm0 + v.Cm_alpha * alpha_ref), 0.0])
        den = np.where(np.abs(cap) > 1e-9, cap, 1e-9)
        d_m = (M_res - rest) / den
        d_state = np.clip(np.array([d_m[1], d_m[0], d_m[2]]), -v.delta_max, v.delta_max)
        u = np.concatenate([d_state, gdot])
        from nearspace.model6c import limit_commands6c
        return limit_commands6c(x, u, self.prm), dict(qp_status="pd", solve_time=0.0)


def main():
    case = sys.argv[1] if len(sys.argv) > 1 else "B"
    T = float(sys.argv[2]) if len(sys.argv) > 2 else 140.0
    cfg = load_config(os.path.join(HERE, "config.yaml"))
    atm = make_atmosphere(cfg, cache_dir=OUT, backend="msis2")
    traj = Trajectory3D(cfg, atm)
    prm = VehicleCMGParams.from_config(cfg)
    cl = make_cluster(prm)
    if case.upper() == "A":
        cfg.mpc_cmg.M_cmg_box = 1e-6          # CMG unavailable
        cfg.mpc_cmg.H_lim_frac = 1.0
        label = "A aero only"
    else:
        label = "B aero + CMG"

    class _Wrap:
        def __init__(self, mpc):
            self.mpc = mpc
        def _att_error(self, qr, q):
            return self.mpc._att_error(qr, q)
        def act(self, x, t, traj):
            return self.mpc.solve(x, t, traj)
    ctrl = _Wrap(LTVMPCCMG2(prm, cfg, cl))
    de = -prm.veh.Cm_alpha / prm.veh.Cm_de * (traj.pitch_ref(0.0) - traj.gamma(0.0))
    x = np.concatenate([traj.q_ref(0.0), np.zeros(3), [de, 0, 0], np.zeros(4), np.zeros(4)])
    dtc = float(cfg.simulation.dt_ctrl)
    dtp = float(cfg.simulation.dt_plant)
    nsub = max(1, int(round(dtc / dtp)))
    t = 0.0
    rows = []
    nf = 0
    while t < T - 1e-9:
        u, info = ctrl.act(x, t, traj)
        if "solved" not in info.get("qp_status", ""):
            nf += 1
        for _ in range(nsub):
            p = traj.p_array(t, prm.veh)
            x, _ = plant_step6c(x, u, p, dtp, prm, cl, substeps=1)
            t += dtp
        e = ctrl._att_error(traj.q_ref(t), x[0:4])
        H = cl.momentum(x[10:14])
        sig = cl.sigma_min(cl.jacobian(x[10:14]))
        Mcmg = -cl.jacobian(x[10:14]) @ x[14:18]
        rows.append([t, e[0], e[1], e[2], np.linalg.norm(e), np.linalg.norm(H), sig,
                     np.rad2deg(u[0]), np.rad2deg(u[1]), np.rad2deg(u[2]),
                     Mcmg[0], Mcmg[1], Mcmg[2]] + list(x[10:14]) + list(x[14:18])
                    + list(u[3:7]) + [0.0 if "solved" in info.get("qp_status", "") else 1.0,
                                      float(info.get("viol", 0.0))])

    with open(os.path.join(OUT, "cmp_%s.csv" % case.upper()), "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["t", "e_roll", "e_pitch", "e_yaw", "e_norm", "H_norm", "sigma",
                    "de", "da", "dr", "Mx", "My", "Mz",
                    "g1", "g2", "g3", "g4", "gd1", "gd2", "gd3", "gd4",
                    "gc1", "gc2", "gc3", "gc4", "qp_fail", "viol"])
        w.writerows(rows)
    a = np.array(rows)
    tt = a[:, 0]
    en = np.rad2deg(a[:, 4])
    win = (tt > 112.0) & (tt < 130.0)
    print("%-14s RMSE=%7.4f deg  max=%8.3f deg  pitch_pulse_peak=%8.3f deg  "
          "max|H|=%6.1f  min_sigma=%5.2f  qp_fail=%d"
          % (label, np.sqrt(np.mean(en ** 2)), en.max(), en[win].max(),
             a[:, 5].max(), a[:, 6].min(), nf))
    print("wrote", os.path.join(OUT, "cmp_%s.csv" % case.upper()))


if __name__ == "__main__":
    main()
