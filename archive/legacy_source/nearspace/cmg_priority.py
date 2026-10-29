"""E2 diagnostic arm B1: authority-aware surface priority + CMG momentum management.

This is the *same* unified MPC (`nearspace.cmg_control.LTVMPCCMG2`), with exactly
two cost-shaping mechanisms replaced.  No constraint is added, removed, relaxed or
re-tightened: the rows, bounds and limits of the QP are bit-identical to the
factory controller (checked by `scripts/e2_actuator_allocation.py --check`).

1. `cmg_effort_weight` - the factory schedule makes the CMG cheap whenever the
   attitude error exceeds `e_thresh_deg` (0.5 deg).  On this task the error is
   above 0.5 deg for most of the rarefied arc, so the CMG is cheap exactly while
   it is being asked to carry a *persistent* 20 N m bias.  B1 replaces the
   error test with an **aero-authority test**: the CMG is cheap only when the
   surfaces physically cannot cover the persistent demand,
   `max_i(|M_d,i| - eta * M_aero_max,i) > 0`.  On top of that, once |H| passes a
   soft knee (`H_soft_frac * H_lim`) the CMG cost is ramped up linearly to
   `1 + r_growth_scale` at the budget, so the optimiser prefers to spend the
   remaining aero authority first and keeps the momentum in reserve.

2. `cmg_extra_cost` - an explicit momentum-*growth* penalty
   `k * (H_hat . M_cmg)^2` with `k` ramping from 0 at the knee to `r_growth` at
   the budget.  It does not pull |H| toward zero (which the plan explicitly
   rejects): below the knee the CMG is untouched, above it the cluster is
   discouraged from charging further, which is what preserves gimbal authority.
   Unloading stays governed by the existing radial budget row and the thin-air
   rule, so this term cannot block an unload the constraints demand.

Everything else - decision vector, prediction model, horizon, terminal cost,
singularity constraint, momentum budget, gimbal limits, servo model - is the
factory implementation.
"""
from __future__ import annotations

import numpy as np

from .cmg_control import LTVMPCCMG2


class LTVMPCPriority(LTVMPCCMG2):
    """Unified MPC with surface-priority allocation (E2 arm B1)."""

    def __init__(self, prm, cfg, cluster=None):
        super().__init__(prm, cfg, cluster)
        p = getattr(cfg, "mpc_priority", None)
        self.prio_enabled = bool(getattr(p, "enabled", False))
        self.eta_aero = float(getattr(p, "aero_authority_frac", 0.9))
        self.H_soft = float(getattr(p, "H_soft_frac", 0.75)) * self.H_lim
        self.r_growth_scale = float(getattr(p, "r_growth_scale", 1.0))
        self.r_growth = float(getattr(p, "r_growth", 0.5))
        self._log = []

    # ---- mechanism 1: who should carry the persistent demand ---------------
    def aero_authority(self, p):
        """Full-deflection aero moment per axis at this prediction node [N m]."""
        v = self.prm.veh
        Q = 0.5 * float(p[0]) * float(np.dot(p[1:4], p[1:4]))
        return Q * v.S * np.array([v.b * abs(v.Cl_da), v.cbar * abs(v.Cm_de),
                                   v.b * abs(v.Cn_dr)])

    def authority_deficit(self, p):
        """Part of the persistent demand the surfaces cannot carry [N m]."""
        cap = self.eta_aero * self.aero_authority(p)
        md = np.abs(np.asarray(p[4:7], dtype=float))
        return float(max(0.0, np.max(md - cap)))

    def momentum_ramp(self, gamma):
        nH = float(np.linalg.norm(self.cluster.momentum(gamma)))
        span = max(self.H_lim - self.H_soft, 1e-9)
        return float(np.clip((nH - self.H_soft) / span, 0.0, 1.0)), nH

    def cmg_effort_weight(self, k, x_seq, t_seq, p_seq, traj):
        if not self.prio_enabled:
            return super().cmg_effort_weight(k, x_seq, t_seq, p_seq, traj)
        ek = float(np.linalg.norm(self._att_error(traj.q_ref(t_seq[k]), x_seq[k][0:4])))
        deficit = self.authority_deficit(p_seq[k])
        # the CMG is cheap only when it must fire: real transient AND no aero
        # authority left for the persistent demand
        base = (self.r_cmg_tr if (ek > self.e_thresh and deficit > 1e-6)
                else self.r_cmg_trim)
        ramp, nH = self.momentum_ramp(x_seq[k][10:14])
        w = base * (1.0 + self.r_growth_scale * ramp)
        self._log.append(dict(k=int(k), t=float(t_seq[k]), ek_deg=np.rad2deg(ek),
                              deficit_Nm=deficit, H_soft_ramp=ramp, H_norm=nH,
                              base=base, weight=float(w)))
        return float(w)

    # ---- mechanism 2: do not spend the momentum reserve -------------------
    def cmg_extra_cost(self, k, gamma_k, dt):
        if not self.prio_enabled:
            return None
        ramp, nH = self.momentum_ramp(gamma_k)
        if ramp <= 0.0:
            return None
        H = self.cluster.momentum(gamma_k)
        u = H / max(nH, 1e-9)
        return (self.r_growth * ramp) * np.outer(u, u)


__all__ = ["LTVMPCPriority"]
