"""Controllers used in the comparison study."""
from __future__ import annotations

import numpy as np

from legacy.model import ModelParams, aero_Q, limit_commands
from legacy.mpc import LTVMPC


class UnifiedMPCController:
    """Proposed controller: MPC directly commands delta_c and M_rw_c."""

    name = "C3 unified MPC"

    def __init__(self, prm: ModelParams, cfg):
        self.prm = prm
        self.cfg = cfg
        self.mpc = LTVMPC(prm, cfg)

    def act(self, x, t, traj, dist_cfg):
        u, info = self.mpc.solve(x, t, traj, dist_cfg)
        return u, info


class AllocationController:
    """Baseline: reference-model torque demand + chained allocation.

    Aero surface takes the primary share (inverted from the aerodynamic
    moment), the wheel covers the residual and a slow momentum-unloading
    term. This mimics the conventional 'MPC torque + allocation layer'
    architecture.
    """

    name = "C2 torque allocation"

    def __init__(self, prm: ModelParams, cfg, k_unload: float = 0.05):
        m = cfg.mpc
        self.prm = prm
        self.k_theta = float(getattr(m, "k_theta", 16.0))
        self.k_q = float(getattr(m, "k_q", 8.0))
        self.k_unload = float(k_unload)
        self.eta = np.nan
        self.gamma_min = np.nan

    def act(self, x, t, traj, dist_cfg):
        prm = self.prm
        p = traj.p_array(t, dist_cfg)
        qddot_ref = (self.k_theta * (traj.theta_ref(t) - x[0])
                     + self.k_q * (traj.q_ref(t) - x[1]) + traj.qdot_ref(t))
        M_cmd = prm.Iy * qddot_ref - p[3]
        Q = aero_Q(p, prm)
        gain = max(Q * prm.cm_scale * prm.Cm_delta, 1e-9)
        # invert the full pitching-moment model (trim + damping + control)
        alpha = x[0] - p[2]
        Cm_rest = (prm.Cm0 + prm.Cm_alpha * alpha
                   + prm.Cm_q * x[1] * prm.cbar / (2.0 * max(p[1], 1.0)))
        delta_c = float(np.clip((M_cmd - Q * prm.cm_scale * Cm_rest) / gain,
                                -prm.delta_max, prm.delta_max))
        M_a_alloc = Q * prm.cm_scale * (Cm_rest + prm.Cm_delta * delta_c)
        residual = M_cmd - M_a_alloc
        M_unload = self.k_unload * x[4]
        if prm.wheel_on:
            M_rw_c = float(np.clip(residual + M_unload, -prm.M_cmd_max, prm.M_cmd_max))
            M_av = min(prm.M_stall, prm.P_max * prm.Jw / max(abs(x[4]), 1e-6))
            self.gamma_min = M_av
        else:
            M_rw_c = 0.0
            self.gamma_min = 0.0
        u = limit_commands(x, np.array([delta_c, M_rw_c]), prm)
        info = dict(eta=np.nan, gamma_min=self.gamma_min, solve_time=0.0,
                    qp_status="allocation", viol=0.0)
        return u, info
