"""Hierarchical CMG control: momentum-level MPC + inner steering law.

Planning model (proven 16-state momentum-exchange model)
    x = [q(4), omega(3), delta_aero(3), M_cmg(3), H_cmg(3)]
    u = [delta_c(3), M_cmg_cmd(3)]
    M_cmg_dot = (M_cmg_cmd - M_cmg)/tau_g,   H_cmg_dot = -M_cmg
    limits: |M_cmg| <= M_guaranteed (61 N m),  |H_cmg| <= 200 N m s
This is exactly the reaction-wheel MPC with the wheel replaced by the CMG
cluster's guaranteed torque/momentum, i.e. the planner never needs the gimbal
configuration. The inner loop realises M_cmg_cmd with the damped pseudo-inverse
steering law (+ null motion) on the true 18-state plant.
"""
from __future__ import annotations

import numpy as np

from nearspace.cmg import CMGCluster
from nearspace.cmg_control import steering_law
from nearspace.model6 import VehicleParams
from legacy.mpc3 import LTVMPC3


def planner_params(prm, cfg) -> VehicleParams:
    """VehicleParams for the momentum-level planner (CMG as torque/momentum box)."""
    v = prm.veh
    c = prm.cmg
    M_guar = c.sigma_min_home * c.gdot_max          # guaranteed torque, N m
    M_peak = c.h0 * c.gdot_max * 3.0                # optimistic peak, not used
    return VehicleParams(
        mass=v.mass, Jx=v.Jx, Jy=v.Jy, Jz=v.Jz, S=v.S, b=v.b, cbar=v.cbar,
        Cm0=v.Cm0, Cm_alpha=v.Cm_alpha, Cm_q=v.Cm_q, Cm_de=v.Cm_de,
        Cl_beta=v.Cl_beta, Cl_p=v.Cl_p, Cl_r=v.Cl_r, Cl_da=v.Cl_da, Cl_dr=v.Cl_dr,
        Cn_beta=v.Cn_beta, Cn_p=v.Cn_p, Cn_r=v.Cn_r, Cn_da=v.Cn_da, Cn_dr=v.Cn_dr,
        delta_max=v.delta_max.copy(), delta_dot_max=v.delta_dot_max.copy(),
        tau_s=v.tau_s,
        Jw=1.0, Omega_max=float(getattr(cfg.mpc_cmg, "H_planner_max", 50.0)),  # hard budget
        M_stall=M_guar, P_max=1.0e12, tau_w=c.tau_g,
        tau_cmd_max=M_guar, dtau_cmd_max=c.h0 * c.gddot_max,
        Md0=v.Md0.copy(), Md_amp=v.Md_amp.copy(), Md_freq=v.Md_freq.copy(),
        wheel_on=True, ideal=False)


class CMGHierController:
    name = "aero + CMG (hierarchical)"

    def __init__(self, prm, cfg):
        self.prm = prm
        self.cfg = cfg
        self.cluster = CMGCluster(prm.cmg)
        self.planner = LTVMPC3(planner_params(prm, cfg), cfg)
        self.gamma_home = np.zeros(prm.cmg.n)
        self.M_box = float(getattr(cfg.mpc_cmg, "M_cmg_box", 61.0))
        self.H_lim = float(getattr(cfg.mpc_cmg, "H_planner_max", 50.0))
        self.q_unload_min = float(getattr(cfg.mpc_cmg, "q_unload_min", 40.0))
        self.last_info = {}

    def _planner_state(self, x):
        M_cmg = -self.cluster.jacobian(x[10:14]) @ x[14:18]
        H_cmg = self.cluster.momentum(x[10:14])
        return np.concatenate([x[0:10], M_cmg, H_cmg])

    def act(self, x, t, traj):
        xp = self._planner_state(x)
        u_pl, info = self.planner.solve(xp, t, traj)
        M_cmd = np.clip(u_pl[3:6], -self.M_box, self.M_box)
        # momentum manager: above the budget only allow commands that do NOT
        # increase |H| (dH/dt = -M, so growing requires dot(M,H) < 0)
        H = self.cluster.momentum(x[10:14])
        nH = float(np.linalg.norm(H))
        if nH > 1e-6:
            u_hat = H / nH
            comp = float(np.dot(M_cmd, u_hat))
            if nH > self.H_lim and comp < 0.0:        # over budget: stop charging
                M_cmd = M_cmd - comp * u_hat
            qk = 0.5 * traj.rho(t) * traj.V(t) ** 2
            if qk < self.q_unload_min and comp > 0.0:  # thin air: defer unloading
                M_cmd = M_cmd - comp * u_hat
        gdot = steering_law(self.cluster, x[10:14], M_cmd, self.gamma_home)
        u = np.concatenate([u_pl[0:3], gdot])
        sig = self.cluster.sigma_min(self.cluster.jacobian(x[10:14]))
        self.last_info = dict(info)
        self.last_info.update(M_cmg_cmd=M_cmd, gdot_cmd=gdot,
                              min_sigma=float(sig),
                              H_cmg=float(np.linalg.norm(self.cluster.momentum(x[10:14]))))
        return u, self.last_info
